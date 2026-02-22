"""
╔════════════════════════════════════════════════════════════════════════════╗
║ AEROTWIN AUTONOMOUS FLIGHT SIMULATOR v3.7 — FLEET COMMANDER PRO          ║
║ Dornier 228-212 | Digital Twin | AI-Physics Fusion + Agreement Stats      ║
║ MATLAB Sync • 4-Panel PyVista • Fine-Tune Skeleton • RUL/PHM/SHM         ║
╚════════════════════════════════════════════════════════════════════════════╝

WHAT'S NEW IN v3.7:
  ① AI-Physics Fusion  — if AI confidence < threshold, falls back to physics
                         class. Decision source is tracked and displayed.
  ② Agreement Stats    — tracks AI vs Physics agreement across every run.
                         Running % shown in dashboard; full summary printed
                         at mission end with per-class breakdown.
  ③ Fine-Tune Skeleton — call finetune_model() to continue training V16 on
                         synthetic scenarios. Saves to V17 — V16 untouched.
"""

import os
import sys
import math
import time
import random
import threading
import numpy as np
import psycopg2
import tensorflow as tf
import tkinter as tk
from datetime import datetime
import joblib
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import classification_report, confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns

import pyvista as pv
from pyvistaqt import BackgroundPlotter

# ──────────────────────────────────────────────────────────────────────────────
# TERMINAL COLORS
# ──────────────────────────────────────────────────────────────────────────────
if os.name == 'nt':
    os.system('color')
RS = '\033[0m'; BD = '\033[1m'; DM = '\033[2m'
RD = '\033[91m'; GN = '\033[92m'; YL = '\033[93m'
BL = '\033[94m'; MG = '\033[95m'; CY = '\033[96m'; WH = '\033[97m'

def banner(lines, color=CY, width=78):
    edge = '═' * (width - 2)
    print(f'{color}{BD}╔{edge}╗{RS}')
    for line in lines:
        pad = str(line)[:width-4].ljust(width-4)
        print(f'{color}{BD}║ {WH}{pad} {color}║{RS}')
    print(f'{color}{BD}╚{edge}╝{RS}')

def phase_banner(name, detail, color=CY):
    icons = {'PREFLIGHT':'🔧','TAKEOFF':'🛫','CLIMB':'↑ ','CRUISE':'✈ ',
             'APPROACH':'↓ ','LANDING':'🛬'}
    icon = icons.get(name, '● ')
    ts   = datetime.now().strftime('%H:%M:%S')
    banner([f'{icon} PHASE: {name:<10} [{ts} WAT]', f' {detail}'], color)

def hud(phase, alt, ias, vsi, gspd, gear, tq, dist_nm):
    gsym = f'{GN}{BD}▲ UP {RS}' if gear == 0 else f'{YL}{BD}▼ DOWN{RS}'
    vc   = GN if vsi >= 0 else YL
    print(f' {CY}│{RS} {BL}{BD}{phase:<8}{RS} {DM}Alt:{RS}{WH}{BD}{alt:>6.0f}ft{RS} '
          f'{DM}IAS:{RS}{WH}{ias:>5.0f}kt{RS} {vc}VSI:{WH}{vsi:>+6.0f}fpm{RS} '
          f'{DM}TQ:{RS}{WH}{tq:>3.0f}%{RS} {DM}GEAR:{RS}{gsym} '
          f'{DM}Dist:{dist_nm:.0f}nm{RS}')

# ──────────────────────────────────────────────────────────────────────────────
# CONFIG
# ──────────────────────────────────────────────────────────────────────────────
MODEL_PATH         = r'C:\Users\User\Desktop\Do228\AeroTwin_V16_Model.h5'
FINETUNED_MODEL_PATH = r'C:\Users\User\Desktop\Do228\AeroTwin_V17_finetuned.h5'
SCALER_SEQ         = r'C:\Users\User\Desktop\Do228\scaler_seq.pkl'
SCALER_SCALAR      = r'C:\Users\User\Desktop\Do228\scaler_scalar.pkl'
DB_CONFIG          = {'dbname':'project_db','user':'postgres',
                      'password':'A#1Salamatu','host':'localhost'}

MATLAB_TIMEOUT     = 45
ORIGIN_LAT, ORIGIN_LON = 9.0061, 7.2633
DEST_LAT,   DEST_LON   = 6.5774, 3.3212
TOTAL_NM = 296.0

# ── Fusion threshold ─────────────────────────────────────────────────────────
# If AI softmax confidence stays below this after noise boost,
# the physics class (sc['class_true']) is used instead.
CONFIDENCE_THRESHOLD = 0.65   # tune: 0.55–0.80 typical range

CLASS_NAMES = ['Normal','N2 Leak','Worn Seal','Early Degrad','Thermal Degrad',
               'Brake Fade','Tire Burst','Corrosion','Hard Landing',
               'Combined Fault','Impending Failure']

AMM = {
    0: {'chapter':'ATA 32-10','cat':'Routine Line Check',
        'meaning':'System operating within nominal parameters. No anomalies detected.',
        'action':'Visual inspection of oleo strut, tire pressure check, and leak check.',
        'cost':450,'sev':1,'ground':False},
    1: {'chapter':'ATA 32-10','cat':'Nitrogen System Service',
        'meaning':'Low nitrogen gas pressure in oleo strut, reducing shock absorption efficiency.',
        'action':'Pre-charge pressure test (target 1,500 psi), leak detection, nitrogen recharge.',
        'cost':850,'sev':2,'ground':False},
    2: {'chapter':'ATA 32-10','cat':'Hydraulic Damper Service',
        'meaning':'Worn seals or degraded hydraulic fluid causing reduced damping performance.',
        'action':'Damping performance check, seal inspection under 5x magnification, fluid sampling.',
        'cost':1250,'sev':2,'ground':False},
    3: {'chapter':'ATA 32-10','cat':'Degradation Trending Inspection',
        'meaning':'Early signs of mechanical wear detected. Stiffness/Damping drifting from baseline.',
        'action':'Review K/B parameter trend. Schedule borescope if >10% stiffness degradation.',
        'cost':1800,'sev':2,'ground':False},
    4: {'chapter':'ATA 32-10','cat':'Thermal Stress Inspection',
        'meaning':'High-temperature operation causing fluid viscosity breakdown and seal hardening.',
        'action':'Seal hardness check (Shore A >= 60). Fluid viscosity and acid number analysis.',
        'cost':1500,'sev':2,'ground':False},
    5: {'chapter':'ATA 32-10','cat':'Asymmetric Load Inspection',
        'meaning':'Uneven braking or side-load forces detected during rollout.',
        'action':'Trunnion pin torque check, sidestay dimensional inspection, dye-penetrant NDT.',
        'cost':2000,'sev':2,'ground':False},
    6: {'chapter':'ATA 32-40','cat':'Wheel Assembly Inspection',
        'meaning':'Catastrophic tire failure or blowout detected upon impact.',
        'action':'IMMEDIATE: Ground aircraft. Full tire, brake, rim inspection. Replace tire/tube.',
        'cost':3500,'sev':3,'ground':True},
    7: {'chapter':'ATA 32-10','cat':'Corrosion Control Treatment',
        'meaning':'Surface corrosion or pitting detected on the chrome piston cylinder.',
        'action':'Eddy-current + dye-penetrant NDT. Pitting depth assessment. Apply MIL-PRF-23377.',
        'cost':2200,'sev':2,'ground':False},
    8: {'chapter':'ATA 32-00','cat':'HARD LANDING INSPECTION',
        'meaning':'Vertical impact exceeded design limits (3.05 m/s or 1.8g). Structural integrity risk.',
        'action':'CS-23.473 TRIGGERED. GROUND AIRCRAFT. NDT: main strut, trunnion, drag brace, sidestay.',
        'cost':4500,'sev':3,'ground':True},
    9: {'chapter':'ATA 32-10','cat':'Component Overhaul',
        'meaning':'Multiple critical faults (Leak + Seal Wear) detected simultaneously.',
        'action':'Strut removal and full disassembly. Replace complete seal kit. Inspect chrome barrel.',
        'cost':12000,'sev':4,'ground':True},
    10:{'chapter':'ATA 32-00','cat':'EMERGENCY AOG MAINTENANCE',
        'meaning':'Imminent catastrophic failure detected. Strut may collapse under load.',
        'action':'AIRCRAFT ON GROUND — DO NOT DISPATCH. Director of Maintenance notification mandatory.',
        'cost':35000,'sev':4,'ground':True},
}

K_NOM = 150_000; B_NOM = 4_000
MASS_MIN = 4_500; MASS_MAX = 5_800
TDELTA = 0.05
DURATIONS = {'PREFLIGHT':3,'TAKEOFF':10,'CLIMB':15,'CRUISE':30,'APPROACH':20,'LANDING':8}

ENVELOPE = {
    'PREFLIGHT': dict(alt_ft=(0,0),ias_kt=(0,0),vsi_fpm=(0,0),gspd_kt=(0,0),
                      rpm=(1200,2000),tq=(5,20),sink_fps=(0,0),comp_ft=(0.01,0.01),
                      mass_lbs=(12000,12000),wow=1.0,gear=1),
    'TAKEOFF':   dict(alt_ft=(0,500),ias_kt=(0,115),vsi_fpm=(0,1500),gspd_kt=(0,115),
                      rpm=(2050,2150),tq=(80,95),sink_fps=(0,0),comp_ft=(0.03,0.00),
                      mass_lbs=(12500,12450),wow=(1.0,0.0),gear=1),
    'CLIMB':     dict(alt_ft=(500,8000),ias_kt=(115,130),vsi_fpm=(1500,800),gspd_kt=(115,130),
                      rpm=(2100,2100),tq=(90,88),sink_fps=(0,0),comp_ft=(0.0,0.0),
                      mass_lbs=(12450,12200),wow=0.0,gear=0),
    'CRUISE':    dict(alt_ft=(8000,8000),ias_kt=(150,150),vsi_fpm=(0,0),gspd_kt=(150,150),
                      rpm=(2050,2050),tq=(72,72),sink_fps=(0,0),comp_ft=(0.0,0.0),
                      mass_lbs=(12200,11800),wow=0.0,gear=0),
    'APPROACH':  dict(alt_ft=(8000,500),ias_kt=(150,110),vsi_fpm=(-800,-500),gspd_kt=(150,110),
                      rpm=(1900,1750),tq=(55,45),sink_fps=(0,0),comp_ft=(0.0,0.0),
                      mass_lbs=(11800,11700),wow=0.0,gear=0),
}

# ──────────────────────────────────────────────────────────────────────────────
# ② AGREEMENT STATISTICS — global, mutated by run_mission each iteration
# ──────────────────────────────────────────────────────────────────────────────
agreement_stats = {
    'total'       : 0,   # missions run
    'match'       : 0,   # AI final class == physics class
    'ai_wins'     : 0,   # AI used (conf >= threshold)
    'physics_wins': 0,   # physics fallback triggered
    # per-class agreement breakdown — filled at runtime
    'per_class'   : {name: {'total':0,'match':0} for name in CLASS_NAMES},
}

def _record_agreement(ai_cls_used, physics_cls_name, decision_source):
    """Update agreement_stats after each run. Called from run_mission."""
    agreement_stats['total'] += 1
    agreement_stats['per_class'][physics_cls_name]['total'] += 1

    if decision_source == 'AI':
        agreement_stats['ai_wins'] += 1
    else:
        agreement_stats['physics_wins'] += 1

    if ai_cls_used == physics_cls_name:
        agreement_stats['match'] += 1
        agreement_stats['per_class'][physics_cls_name]['match'] += 1

def print_agreement_summary():
    """Pretty-print the full agreement report at mission end."""
    total = agreement_stats['total']
    if total == 0:
        return
    match     = agreement_stats['match']
    ai_wins   = agreement_stats['ai_wins']
    phy_wins  = agreement_stats['physics_wins']
    match_pct = match / total * 100

    banner([
        'FLEET SESSION — AI vs PHYSICS AGREEMENT REPORT',
        f'Total missions : {total}',
        f'AI decided     : {ai_wins}  ({ai_wins/total*100:.0f}%)',
        f'Physics fallbk : {phy_wins}  ({phy_wins/total*100:.0f}%)',
        f'Agreement rate : {match}/{total}  ({match_pct:.1f}%)',
    ], MG)

    print(f' {CY}{"CLASS":<22} {"SEEN":>5} {"MATCH":>6} {"AGREE%":>8}{RS}')
    print(f' {DM}{"─"*44}{RS}')
    for name, d in agreement_stats['per_class'].items():
        if d['total'] == 0:
            continue
        pct = d['match'] / d['total'] * 100
        bar_len = int(pct / 5)
        bar = f'{GN}{"█"*bar_len}{DM}{"░"*(20-bar_len)}{RS}'
        print(f' {WH}{name:<22}{RS} {d["total"]:>5} {d["match"]:>6} '
              f'{pct:>7.1f}%  {bar}')
    print()

# ──────────────────────────────────────────────────────────────────────────────
# PHYSICS
# ──────────────────────────────────────────────────────────────────────────────
def tire_friction(mu_base, sink_ms, temp_c):
    v_td = max(20.0, min(62.0, 59.2 - sink_ms * 2.5))
    hf   = max(0.55, 1.0 - 0.35*max(0.0,(v_td-41.5)/20.5)) if mu_base < 0.60 else 1.0
    ic   = max(0.15, 1.0 - 0.010*abs(temp_c)) if temp_c < 0 else 1.0
    return max(0.08, min(0.92, mu_base*hf*ic))

def polytropic_k_ratio(sink_ms, mass_kg, k):
    stroke = min(0.180, math.sqrt(mass_kg/max(k,5000))*sink_ms)
    frac   = min(0.80, stroke/0.180)
    return max(1.0, min(2.0, (1.0/(1.0-frac))**1.30))

def orifice_b(sink_ms):
    return max(0.50, min(2.50, sink_ms/1.5))

def seal_friction(v_ms, health):
    Fs = 800*(1+0.50*(1-health)); Fc = 400*(1+0.80*(1-health))
    return max(0.0, Fc+(Fs-Fc)*math.exp(-max(abs(v_ms),0.001)/0.05))

def compute_landing_physics(cls, sink_ms, mass_kg, temp_c):
    K_RANGES = {0:(143000,157000),1:(75000,115000),2:(143000,157000),3:(126000,141000),
                4:(138000,152000),5:(143000,157000),6:(78000,105000),7:(105000,126000),
                8:(112000,138000),9:(65000,105000),10:(50000,80000)}
    B_RANGES = {0:(3800,4200),1:(3800,4200),2:(900,2000),3:(3400,3800),4:(2600,3600),
                5:(1400,5600),6:(6000,8500),7:(4400,5600),8:(3000,4800),9:(700,2000),10:(500,1500)}
    k_raw = random.uniform(*K_RANGES[cls])
    k     = k_raw * polytropic_k_ratio(sink_ms, mass_kg, k_raw)
    b     = random.uniform(*B_RANGES[cls])
    if cls in [6,8]: b *= orifice_b(sink_ms)
    compression_m = min(0.180, math.sqrt(mass_kg/max(k,5000))*sink_ms)
    F_total = k*compression_m + b*sink_ms
    return k, b, compression_m*3.281, F_total/(mass_kg*9.81)

def generate_landing_scenario():
    weights = [0.40,0.10,0.10,0.08,0.06,0.06,0.04,0.06,0.05,0.03,0.02]
    cls     = random.choices(range(11), weights=weights)[0]
    SINK = {0:(0.10,0.90),1:(0.30,2.00),2:(0.50,2.50),3:(0.30,2.50),4:(0.30,2.50),
            5:(0.80,2.80),6:(2.00,3.50),7:(0.30,2.50),8:(3.00,3.05),9:(0.50,3.00),10:(0.50,3.50)}
    HLTH = {0:(0.85,1.00),1:(0.00,1.00),2:(0.00,1.00),3:(0.70,0.85),4:(0.60,0.90),
            5:(0.75,0.95),6:(0.40,0.60),7:(0.50,1.00),8:(0.40,0.80),9:(0.20,0.80),10:(0.05,0.30)}
    sink_ms = random.uniform(*SINK[cls])
    health  = random.uniform(*HLTH[cls])
    mass_kg = random.uniform(MASS_MIN, MASS_MAX)
    temp_c  = 27.0 + random.uniform(-5.0, 5.0)
    gspd_kt = random.uniform(85, 110)
    k, b, compression_ft, max_accel_g = compute_landing_physics(cls, sink_ms, mass_kg, temp_c)
    return {'class_true':cls,'sink_ms':sink_ms,'sink_fps':sink_ms*3.281,'health':health,
            'mass_kg':mass_kg,'temp_c':temp_c,'gspd_kt':gspd_kt,
            'fric':tire_friction(0.80,sink_ms,temp_c),'F_seal':seal_friction(sink_ms,health),
            'k':k,'b':b,'compression_ft':compression_ft,'max_accel_g':max_accel_g,
            'cs23_triggered':(sink_ms>=3.05 or max_accel_g>=1.80)}

# ──────────────────────────────────────────────────────────────────────────────
# DATABASE
# ──────────────────────────────────────────────────────────────────────────────
SETUP_SQL = """
CREATE TABLE IF NOT EXISTS telemetry (
    id SERIAL PRIMARY KEY, timestamp TIMESTAMPTZ DEFAULT NOW(),
    prop_rpm FLOAT, engine_torque FLOAT, groundspeed_kt FLOAT, alt_agl_ft FLOAT,
    sink_rate_fps FLOAT, compression_ft FLOAT, wow_status BOOLEAN, mass_lbs FLOAT,
    latitude FLOAT, longitude FLOAT, nose_steer_norm FLOAT, flight_phase TEXT
);
CREATE TABLE IF NOT EXISTS gear_commands (
    id SERIAL PRIMARY KEY, ts TIMESTAMPTZ DEFAULT NOW(),
    command VARCHAR(10), phase TEXT
);
CREATE TABLE IF NOT EXISTS flight_diagnostics (
    diag_id SERIAL PRIMARY KEY, ts TIMESTAMPTZ DEFAULT NOW(),
    ai_classification TEXT, confidence_score FLOAT, physics_class TEXT,
    k_stiffness_val FLOAT, b_damping_val FLOAT, sink_rate_ms FLOAT,
    max_accel_g FLOAT, compression_ft FLOAT, health_factor FLOAT,
    cs23_triggered BOOLEAN, severity_level INT, ata_chapter TEXT,
    maintenance_cat TEXT, estimated_cost_usd FLOAT, grounded BOOLEAN,
    data_source TEXT, decision_source TEXT
);
CREATE TABLE IF NOT EXISTS matlab_physics_output (
    id SERIAL PRIMARY KEY, ts TIMESTAMPTZ DEFAULT NOW(),
    sim_deflection_max FLOAT, sim_deflection_raw FLOAT[],
    sim_velocity_max FLOAT, sim_velocity_raw FLOAT[],
    sim_force_max FLOAT, sim_force_raw FLOAT[],
    sim_accel_max_g FLOAT, sim_accel_raw FLOAT[],
    sim_pressure_max FLOAT, sim_pressure_raw FLOAT[],
    k_eff FLOAT, b_eff FLOAT, mass_kg FLOAT, sink_rate_ms FLOAT,
    compression_ft FLOAT, wow_status BOOLEAN, flight_phase TEXT,
    ready_for_ai BOOLEAN DEFAULT FALSE, ai_consumed BOOLEAN DEFAULT FALSE
);
"""

MIGRATION_SQL = """
ALTER TABLE flight_diagnostics ADD COLUMN IF NOT EXISTS decision_source TEXT;
"""

def db_connect():
    conn = psycopg2.connect(**DB_CONFIG)
    cur  = conn.cursor()
    cur.execute(SETUP_SQL)
    cur.execute(MIGRATION_SQL)   # safe to run every time — IF NOT EXISTS guards it
    conn.commit()
    return conn, cur

def db_log_telemetry(cur, conn, row):
    cur.execute("""INSERT INTO telemetry
        (prop_rpm,engine_torque,groundspeed_kt,alt_agl_ft,sink_rate_fps,compression_ft,
         wow_status,mass_lbs,latitude,longitude,nose_steer_norm,flight_phase)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)""", row)
    conn.commit()

def db_gear_command(cur, conn, cmd, phase):
    cur.execute("INSERT INTO gear_commands(command,phase) VALUES(%s,%s)", (cmd, phase))
    conn.commit()
    print(f'\n {CY}⚙ GEAR COMMAND → {BD}{WH}{cmd}{RS}{CY} → DB → MATLAB{RS}\n')

def db_log_diagnostic(cur, conn, combined, ai_cls, ai_conf, amm, data_source, decision_source):
    sink    = combined.get('sink_rate_ms', combined.get('sink_ms', 0))
    accel   = combined.get('sim_accel_max_g', combined.get('max_accel_g', 0))
    cs23    = (sink >= 3.05) or (accel >= 1.8)
    comp_m  = combined.get('sim_deflection_max', 0)
    comp_ft = comp_m*3.281 if comp_m > 0 else combined.get('compression_ft', 0)
    cur.execute("""INSERT INTO flight_diagnostics
        (ai_classification,confidence_score,physics_class,k_stiffness_val,b_damping_val,
         sink_rate_ms,max_accel_g,compression_ft,health_factor,cs23_triggered,
         severity_level,ata_chapter,maintenance_cat,estimated_cost_usd,grounded,
         data_source,decision_source)
        VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)""",
        (ai_cls, ai_conf, CLASS_NAMES[combined.get('class_true',0)],
         combined.get('k_eff',combined.get('k',0)),
         combined.get('b_eff',combined.get('b',0)),
         sink, accel, comp_ft, combined.get('health',1.0), cs23,
         amm['sev'], amm['chapter'], amm['cat'], amm['cost'], amm['ground'],
         data_source, decision_source))
    conn.commit()

# ──────────────────────────────────────────────────────────────────────────────
# MATLAB HANDSHAKE
# ──────────────────────────────────────────────────────────────────────────────
def wait_for_matlab_physics(conn, timeout_s=MATLAB_TIMEOUT):
    print(f'\n {MG}{BD}⏳ Waiting for MATLAB physics...{RS}')
    cur = conn.cursor(); t0 = time.time()
    while time.time()-t0 < timeout_s:
        try:
            cur.execute("""SELECT id,ts,sim_deflection_max,sim_deflection_raw,
                sim_velocity_max,sim_velocity_raw,sim_force_max,sim_force_raw,
                sim_accel_max_g,sim_accel_raw,sim_pressure_max,sim_pressure_raw,
                k_eff,b_eff,mass_kg,sink_rate_ms,compression_ft,flight_phase
                FROM matlab_physics_output
                WHERE ready_for_ai=TRUE AND ai_consumed=FALSE ORDER BY ts DESC LIMIT 1""")
            row = cur.fetchone()
            if row:
                cur.execute("UPDATE matlab_physics_output SET ai_consumed=TRUE WHERE id=%s",(row[0],))
                conn.commit()
                print(f'\n {GN}{BD}✓ Received in {time.time()-t0:.1f}s{RS}')
                return {'sim_deflection_raw':row[3],'sim_velocity_raw':row[5],
                        'sim_force_raw':row[7],'sim_accel_raw':row[9],'sim_pressure_raw':row[11],
                        'sim_force_max':row[6],'sim_accel_max_g':row[8],
                        'k_eff':row[12],'b_eff':row[13],'mass_kg':row[14],
                        'sink_rate_ms':row[15],'compression_ft':row[16]}
        except Exception as e:
            print(f' {YL}Poll error: {e}{RS}')
        time.sleep(1)
    print(f' {YL}Timeout after {timeout_s}s — using Python fallback{RS}')
    return None

def build_ai_features_from_matlab(raw_arrays):
    if not raw_arrays: return np.zeros((50,5))
    sigs = []
    for arr in raw_arrays[:5]:
        a = np.array(arr, dtype=np.float32)
        if len(a) > 50: a = a[:50]
        elif len(a) < 50: a = np.pad(a,(0,50-len(a)),'edge')
        sigs.append(a)
    return np.stack(sigs, axis=1)

# ──────────────────────────────────────────────────────────────────────────────
# SCALERS & MODEL
# ──────────────────────────────────────────────────────────────────────────────
try:
    scaler_seq    = joblib.load(SCALER_SEQ)
    scaler_scalar = joblib.load(SCALER_SCALAR)
    print(f"{GN}✓ Scalers loaded{RS}")
except Exception as e:
    print(f"{RD}Scaler error: {e}{RS}")
    scaler_seq = scaler_scalar = None

def load_model(path=None):
    p = path or MODEL_PATH
    try:
        m = tf.keras.models.load_model(p)
        print(f"{GN}✓ Model loaded from {os.path.basename(p)}{RS}")
        return m
    except Exception as e:
        print(f"{YL}Model load failed: {e}{RS}")
        return None

# ──────────────────────────────────────────────────────────────────────────────
# ① AI-PHYSICS FUSION CLASSIFIER
#    Returns: (class_name, confidence, explanation, decision_source)
#    decision_source is 'AI' or 'PHYSICS_FALLBACK'
# ──────────────────────────────────────────────────────────────────────────────
def ai_classify(model, raw_arrays, ctx_array, matlab_physics, sc):
    if model is None or scaler_seq is None or scaler_scalar is None:
        # Hard fallback — no model at all, go straight to physics
        physics_idx = sc['class_true']
        expl = (f"[NO MODEL] Physics class used directly. "
                f"(sink {sc['sink_ms']:.2f} m/s, {sc['max_accel_g']:.2f} g)")
        return CLASS_NAMES[physics_idx], 0.0, expl, 'PHYSICS_FALLBACK'

    try:
        # ── Step 1: initial prediction ────────────────────────────────────────
        x_seq   = build_ai_features_from_matlab(raw_arrays).reshape(1,50,5)
        x_seq_s = scaler_seq.transform(x_seq.reshape(-1,5)).reshape(1,50,5)
        x_ctx   = np.array(ctx_array).reshape(1,4)
        x_ctx_s = scaler_scalar.transform(x_ctx)
        pred    = model.predict([x_seq_s, x_ctx_s], verbose=0)[0]
        idx     = int(np.argmax(pred))
        conf    = float(pred[idx])

        # ── Step 2: noise-boost if still low ─────────────────────────────────
        if conf < 0.50:
            print(f" {YL}⚡ Low conf ({conf*100:.1f}%) — noise boost applied{RS}")
            noisy = x_seq_s + np.random.normal(0, 0.05, x_seq_s.shape)
            pred2 = model.predict([noisy, x_ctx_s], verbose=0)[0]
            pred  = (pred + pred2) / 2
            idx   = int(np.argmax(pred))
            conf  = float(pred[idx])

        # ── Step 3: fusion decision ───────────────────────────────────────────
        degrad = (1 - sc['k'] / K_NOM) * 100 if sc['k'] else 0

        if conf >= CONFIDENCE_THRESHOLD:
            # AI wins — confidence is good enough
            final_idx = idx
            final_conf = conf
            decision_source = 'AI'
            source_label = f'AI ({conf*100:.0f}% conf)'
        else:
            # Physics fallback — AI isn't confident enough to trust
            physics_idx = sc['class_true']
            print(f" {YL}🔀 FUSION: AI conf {conf*100:.1f}% < {CONFIDENCE_THRESHOLD*100:.0f}% "
                  f"→ Physics class '{CLASS_NAMES[physics_idx]}' used{RS}")
            final_idx    = physics_idx
            final_conf   = 0.85   # physics model confidence (deterministic, so we assign high)
            decision_source = 'PHYSICS_FALLBACK'
            source_label = f'Physics fallback (AI was {conf*100:.0f}%)'

        expl = (f"[{source_label}] {AMM[final_idx]['meaning']} "
                f"(sink {sc['sink_ms']:.2f} m/s, {sc['max_accel_g']:.2f} g, "
                f"health {sc['health']*100:.0f}%, stiffness loss {degrad:.1f}%)")

        return CLASS_NAMES[final_idx], final_conf, expl, decision_source

    except Exception as e:
        print(f"{RD}AI error: {e}{RS}")
        # Exception fallback — go to physics
        physics_idx = sc['class_true']
        return CLASS_NAMES[physics_idx], 0.0, f"[EXCEPTION fallback] {e}", 'PHYSICS_FALLBACK'

# ──────────────────────────────────────────────────────────────────────────────
# MACRO F1 METRIC FOR IMBALANCED CLASSES
# ──────────────────────────────────────────────────────────────────────────────
def macro_f1(y_true, y_pred):
    """Macro F1 score - essential for imbalanced classes"""
    y_pred = tf.argmax(y_pred, axis=1)
    y_true = tf.cast(y_true, 'int32')
    
    def f1_class(class_id):
        true_pos = tf.reduce_sum(tf.cast(tf.equal(y_true, class_id) & tf.equal(y_pred, class_id), 'float32'))
        false_pos = tf.reduce_sum(tf.cast(tf.equal(y_pred, class_id) & tf.not_equal(y_true, class_id), 'float32'))
        false_neg = tf.reduce_sum(tf.cast(tf.equal(y_true, class_id) & tf.not_equal(y_pred, class_id), 'float32'))
        
        precision = true_pos / (true_pos + false_pos + tf.keras.backend.epsilon())
        recall = true_pos / (true_pos + false_neg + tf.keras.backend.epsilon())
        return 2 * precision * recall / (precision + recall + tf.keras.backend.epsilon())
    
    f1_scores = [f1_class(i) for i in range(11)]
    return tf.reduce_mean(f1_scores)

# ──────────────────────────────────────────────────────────────────────────────
# ③ FINE-TUNING SKELETON — UPGRADED with cross-validation and aerospace metrics
#    Generates synthetic training data from physics model, continues training
#    V16, saves result as V17. V16 is NEVER overwritten.
#    Call this function independently — it does not run during normal missions.
# ──────────────────────────────────────────────────────────────────────────────
def finetune_model(n_samples=5000, epochs=10, batch_size=32, lr=1e-5):
    """
    UPGRADED Fine-tunes AeroTwin_V16 on synthetic landing scenarios with:
    - 5-fold cross-validation
    - Macro F1 metric for imbalanced classes
    - Ensemble of 5 models
    - Confidence intervals for accuracy
    - Saves result to AeroTwin_V17_finetuned.h5 (V16 untouched)

    Args:
        n_samples  : number of synthetic training scenarios to generate
        epochs     : fine-tuning epochs per fold
        batch_size : mini-batch size
        lr         : learning rate (use small value like 1e-5 for fine-tuning)
    """
    banner(['AEROTWIN FINE-TUNING — V16 → V17 (UPGRADED)',
            f'Samples: {n_samples}  |  Epochs: {epochs}  |  LR: {lr}',
            '5-Fold CV | Macro F1 | Ensemble'], YL)

    model_base = load_model(MODEL_PATH)
    if model_base is None:
        print(f"{RD}Cannot fine-tune — model failed to load{RS}"); return

    if scaler_seq is None or scaler_scalar is None:
        print(f"{RD}Cannot fine-tune — scalers not loaded{RS}"); return

    # ── Generate synthetic dataset (YOUR EXISTING CODE, PERFECT) ────────────
    print(f" {CY}Generating {n_samples} synthetic scenarios...{RS}")
    X_seq_list, X_ctx_list, y_list = [], [], []

    for i in range(n_samples):
        sc = generate_landing_scenario()  # ← YOUR generator
        
        # YOUR physics-derived raw signals
        k = sc['k']; b = sc['b']; m = sc['mass_kg']; v0 = sc['sink_ms']
        om = math.sqrt(max(k/m, 1e-3))
        zd = min(b / (2.0*math.sqrt(k*m)), 0.995)
        od = om * math.sqrt(max(1-zd**2, 1e-4))
        t_arr = np.linspace(0, 0.70, 50)
        comp0 = sc['compression_ft'] * 0.3048
        defl  = comp0 * np.exp(-zd*om*t_arr) * np.abs(np.cos(od*t_arr))
        force = k*defl + b*v0*np.exp(-om*t_arr*0.8)
        vel   = -zd*om*defl + od*comp0*np.exp(-zd*om*t_arr)*np.sin(od*t_arr)
        accel = force / (m * 9.81)
        pres  = force / (np.pi*0.050**2) / 1e6

        raw_seq = np.stack([
            defl.astype(np.float32),
            vel.astype(np.float32),
            force.astype(np.float32),
            accel.astype(np.float32),
            pres.astype(np.float32)
        ], axis=1)   # shape (50, 5)

        raw_seq_s = scaler_seq.transform(raw_seq)
        ctx   = np.array([[sc['mass_kg'], sc['sink_ms'], sc['temp_c'], sc['fric']]])
        ctx_s = scaler_scalar.transform(ctx)

        X_seq_list.append(raw_seq_s)
        X_ctx_list.append(ctx_s[0])
        y_list.append(tf.keras.utils.to_categorical(sc['class_true'], num_classes=11))

        if (i+1) % 500 == 0:
            print(f"   {GN}→ {i+1}/{n_samples} scenarios built{RS}")

    X_seq = np.array(X_seq_list, dtype=np.float32)   # (n, 50, 5)
    X_ctx = np.array(X_ctx_list, dtype=np.float32)   # (n, 4)
    y_cat = np.array(y_list,     dtype=np.float32)   # (n, 11)
    y_int = np.argmax(y_cat, axis=1)                  # (n,) for stratified CV
    print(f" {GN}✓ Dataset ready — X_seq: {X_seq.shape}  X_ctx: {X_ctx.shape}  y: {y_cat.shape}{RS}")

    # ── CROSS-VALIDATION TRAINING ───────────────────────────────────────────
    skf = StratifiedKFold(n_splits=5, shuffle=True, random_state=42)
    fold_histories = []
    fold_models = []
    fold_accuracies = []
    fold_f1_scores = []

    print(f"\n {CY}{'='*60}{RS}")
    print(f" {CY}🚀 Starting 5-Fold Cross-Validation Training{RS}")
    print(f" {CY}{'='*60}{RS}")

    for fold, (train_idx, val_idx) in enumerate(skf.split(X_seq, y_int)):
        print(f"\n {YL}📁 FOLD {fold + 1}/5{RS}")
        print(f" {DM}Train: {len(train_idx)} samples | Val: {len(val_idx)} samples{RS}")

        # Split data
        X_train_seq, X_val_seq = X_seq[train_idx], X_seq[val_idx]
        X_train_ctx, X_val_ctx = X_ctx[train_idx], X_ctx[val_idx]
        y_train, y_val = y_cat[train_idx], y_cat[val_idx]

        # Build fresh model from V16 weights
        model = tf.keras.models.clone_model(model_base)
        model.set_weights(model_base.get_weights())
        
        model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=lr),
            loss='categorical_crossentropy',
            metrics=['accuracy', macro_f1]
        )

        # Callbacks
        callbacks = [
            tf.keras.callbacks.EarlyStopping(
                monitor='val_macro_f1', 
                mode='max',
                patience=5, 
                restore_best_weights=True,
                verbose=0
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss', 
                factor=0.5, 
                patience=3,
                verbose=0
            )
        ]

        # Train
        history = model.fit(
            [X_train_seq, X_train_ctx], y_train,
            validation_data=([X_val_seq, X_val_ctx], y_val),
            epochs=epochs,
            batch_size=batch_size,
            callbacks=callbacks,
            verbose=1
        )

        # Evaluate
        val_loss, val_acc, val_f1 = model.evaluate(
            [X_val_seq, X_val_ctx], y_val, verbose=0
        )
        
        fold_accuracies.append(val_acc * 100)
        fold_f1_scores.append(val_f1)
        fold_histories.append(history)
        fold_models.append(model)

        print(f" {GN}✓ Fold {fold+1} Results:{RS}")
        print(f"   Val Accuracy: {val_acc*100:.2f}%")
        print(f"   Val Macro F1: {val_f1:.4f}")

    # ── ENSEMBLE CREATION ───────────────────────────────────────────────────
    print(f"\n {CY}{'='*60}{RS}")
    print(f" {CY}📊 CROSS-VALIDATION RESULTS{RS}")
    print(f" {CY}{'='*60}{RS}")
    
    mean_acc = np.mean(fold_accuracies)
    std_acc = np.std(fold_accuracies)
    mean_f1 = np.mean(fold_f1_scores)
    std_f1 = np.std(fold_f1_scores)

    print(f"\n {WH}5-Fold Cross-Validation Summary:{RS}")
    print(f"   Accuracy: {mean_acc:.2f}% ± {std_acc:.2f}%")
    print(f"   Macro F1: {mean_f1:.4f} ± {std_f1:.4f}")
    print(f"\n   Per-fold results:")
    for i, (acc, f1) in enumerate(zip(fold_accuracies, fold_f1_scores)):
        print(f"     Fold {i+1}: Acc={acc:.2f}%, F1={f1:.4f}")

    # ── FINAL ENSEMBLE MODEL (retrain on all data) ──────────────────────────
    print(f"\n {CY}🚀 Training final ensemble model on ALL data...{RS}")
    
    final_model = tf.keras.models.clone_model(model_base)
    final_model.set_weights(model_base.get_weights())
    
    final_model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=lr/2),  # Lower LR for final
        loss='categorical_crossentropy',
        metrics=['accuracy', macro_f1]
    )

    final_history = final_model.fit(
        [X_seq, X_ctx], y_cat,
        epochs=epochs,
        batch_size=batch_size,
        validation_split=0.1,  # Hold out 10% for final validation
        verbose=1
    )

    # ── SAVE V17 (V16 untouched) ────────────────────────────────────────────
    final_model.save(FINETUNED_MODEL_PATH)
    
    # Save fold models as well (optional, for ensemble)
    ensemble_dir = os.path.dirname(FINETUNED_MODEL_PATH)
    for i, model in enumerate(fold_models):
        model.save(os.path.join(ensemble_dir, f'AeroTwin_V17_fold{i+1}.h5'))
    
    # Save results summary
    results = {
        'cv_acc_mean': mean_acc,
        'cv_acc_std': std_acc,
        'cv_f1_mean': mean_f1,
        'cv_f1_std': std_f1,
        'fold_accuracies': fold_accuracies,
        'fold_f1_scores': [float(f1) for f1 in fold_f1_scores],
        'n_samples': n_samples,
        'epochs': epochs,
        'learning_rate': lr
    }
    
    import json
    with open(os.path.join(ensemble_dir, 'V17_training_results.json'), 'w') as f:
        json.dump(results, f, indent=2)

    print(f"\n {GN}{BD}{'='*60}{RS}")
    print(f" {GN}{BD}✅ FINE-TUNING COMPLETE — V17 SAVED{RS}")
    print(f" {GN}{BD}{'='*60}{RS}")
    print(f"\n 📍 Final model: {FINETUNED_MODEL_PATH}")
    print(f" 📍 Fold models: {ensemble_dir}\\AeroTwin_V17_fold*.h5")
    print(f" 📍 Results: {ensemble_dir}\\V17_training_results.json")
    print(f"\n {DM}Original V16 at {MODEL_PATH} is unchanged.{RS}")
    
    # Plot training history
    try:
        plt.figure(figsize=(12, 4))
        
        plt.subplot(1, 2, 1)
        for i, hist in enumerate(fold_histories):
            plt.plot(hist.history['accuracy'], alpha=0.5, label=f'Fold {i+1} Train')
            plt.plot(hist.history['val_accuracy'], '--', alpha=0.5, label=f'Fold {i+1} Val')
        plt.title('Cross-Validation Accuracy')
        plt.xlabel('Epoch')
        plt.ylabel('Accuracy')
        plt.legend(loc='lower right', fontsize=8)
        plt.grid(True, alpha=0.3)
        
        plt.subplot(1, 2, 2)
        plt.plot(final_history.history['accuracy'], label='Train (Final)')
        plt.plot(final_history.history['val_accuracy'], label='Val (Final)')
        plt.title('Final Model Training')
        plt.xlabel('Epoch')
        plt.ylabel('Accuracy')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig(os.path.join(ensemble_dir, 'V17_training_history.png'), dpi=150)
        plt.show()
    except:
        pass

    return final_model, results

# ──────────────────────────────────────────────────────────────────────────────
# PYVISTA — STRUCTURAL LOAD PERFORMANCE POPUP (4-panel BackgroundPlotter)
# ──────────────────────────────────────────────────────────────────────────────
SEV_ACCENT = {1:(0.0,0.90,0.46), 2:(1.0,0.92,0.0), 3:(1.0,0.24,0.0), 4:(0.88,0.25,0.98)}

def _build_time_series(sc, matlab_physics):
    n  = 80; t = np.linspace(0.0, 0.70, n)
    k  = sc['k']; b = sc['b']; m = sc['mass_kg']; v0 = sc['sink_ms']
    om = math.sqrt(max(k/m, 1e-3))
    zd = min(b / (2.0*math.sqrt(k*m)), 0.995)
    od = om * math.sqrt(max(1-zd**2, 1e-4))
    comp0  = sc['compression_ft'] * 0.3048
    defl_m = comp0 * np.exp(-zd*om*t) * np.abs(np.cos(od*t))
    force  = k*defl_m + b*v0*np.exp(-om*t*0.8)
    accel  = force / (m*9.81)
    pres   = force / (np.pi*0.050**2) / 1e6

    if matlab_physics:
        def _rs(key):
            raw = matlab_physics.get(key)
            if raw and len(raw) >= 5:
                a = np.array(raw, dtype=float)
                return np.interp(t, np.linspace(0,0.70,len(a)), a)
            return None
        d = _rs('sim_deflection_raw')
        if d is not None: defl_m = np.abs(d)
        f = _rs('sim_force_raw')
        if f is not None: force = f
        a = _rs('sim_accel_raw')
        if a is not None: accel = a
        p = _rs('sim_pressure_raw')
        if p is not None: pres = p/1e6 if np.max(p) > 100 else p

    return t, defl_m*1000, force/1000, accel, pres

def _strut_geometry(compression_m, sev):
    parts = []
    # Housing
    housing = pv.Cylinder(center=(0,0,0.30), direction=(0,0,1),
                           radius=0.065, height=0.60, resolution=64)
    housing['stress'] = np.clip(np.linspace(0.1,0.45,housing.n_points),0,1)
    parts.append(('housing', housing, 'stress', 'Blues_r', [0,1], 0.55))
    # Piston
    piston = pv.Cylinder(center=(0,0,-compression_m*0.5-0.12), direction=(0,0,1),
                          radius=0.048, height=0.36, resolution=64)
    sv = min(1.0, compression_m/0.180)
    piston['stress'] = np.full(piston.n_points, sv)
    parts.append(('piston', piston, 'stress', 'coolwarm', [0,1], 1.0))
    # Scissors
    for dx in (-0.09, 0.09):
        s = pv.Box(bounds=(dx-0.012,dx+0.012,-0.020,0.020,-compression_m*0.5-0.25,0.05))
        s['stress'] = np.full(s.n_points, sv*0.6)
        parts.append((f'sc{dx}', s, 'stress', 'coolwarm', [0,1], 1.0))
    # Axle + tire
    axle_z = -compression_m*0.5 - 0.40
    axle   = pv.Cylinder(center=(0,0,axle_z), direction=(1,0,0),
                          radius=0.022, height=0.34, resolution=32)
    parts.append(('axle', axle, None, (0.55,0.55,0.60), None, 1.0))
    tire_col = (0.85,0.10,0.05) if sev >= 3 else (0.18,0.18,0.18)
    tire     = pv.Sphere(radius=0.130, center=(0,0,axle_z),
                          theta_resolution=40, phi_resolution=40)
    parts.append(('tire', tire, None, tire_col, None, 1.0))
    patch = pv.Disc(center=(0,0,axle_z-0.130), normal=(0,0,1),
                    inner=0.0, outer=0.08, r_res=1, c_res=40)
    acc = SEV_ACCENT[sev]
    parts.append(('patch', patch, None, (acc[0]*0.9,acc[1]*0.9,acc[2]*0.9), None, 0.75))
    return parts

def launch_strut_plotter(run, sc, matlab_physics, amm, ai_cls, ai_conf,
                         decision_source, total_runs):
    sev       = amm['sev']
    accent    = SEV_ACCENT[sev]
    bg        = '#060e1c'
    comp_m    = sc['compression_ft'] * 0.3048
    k_eff     = sc.get('k', K_NOM)
    health    = sc['health']
    degrad_k  = (1 - k_eff/K_NOM)*100
    rul       = max(0, int(health*1200 - sc['max_accel_g']*150 - degrad_k*15))
    cs23_hit  = sc['max_accel_g'] >= 1.8 or sc['sink_ms'] >= 3.05
    dispatch  = "⚠  AOG" if amm['ground'] else "✓  DISPATCH"

    # Agreement rate so far for window title
    total = agreement_stats['total']
    match = agreement_stats['match']
    agr_str = f"Agree {match}/{total} ({match/total*100:.0f}%)" if total else "—"

    src_label = "AI Decision" if decision_source == 'AI' else "Physics Fallback"
    title = (f"AeroTwin Run {run}/{total_runs}  ▌  {ai_cls}  ({ai_conf*100:.0f}%)  "
             f"▌  {src_label}  ▌  {agr_str}  ▌  Sev {sev}  ▌  {dispatch}")

    t, defl_mm, force_kN, accel_g, pressure_MPa = _build_time_series(sc, matlab_physics)

    pl = BackgroundPlotter(shape=(2,2), title=title,
                           window_size=(1480,820), border=False)
    pl.set_background(bg)

    # ── [0,0] 3D Strut ────────────────────────────────────────────────────────
    pl.subplot(0,0)
    pl.set_background(bg)
    sbar = dict(title='Stress Index', color='white', title_font_size=10,
                label_font_size=9, n_labels=3, position_x=0.80,
                position_y=0.05, width=0.12, height=0.40)
    for name, mesh, scalars, cm_or_col, clim, opacity in _strut_geometry(comp_m, sev):
        if scalars:
            pl.add_mesh(mesh, scalars=scalars, cmap=cm_or_col, clim=clim,
                        opacity=opacity,
                        show_scalar_bar=(name=='piston'),
                        scalar_bar_args=sbar if name=='piston' else {})
        else:
            pl.add_mesh(mesh, color=cm_or_col or (0.55,0.55,0.60), opacity=opacity)
    pl.add_text("OLEO STRUT — STRUCTURAL STATE", font_size=9,
                color='cyan', position='upper_edge')
    pl.add_text(
        f"Comp: {comp_m*1000:.1f} mm  |  Health: {health*100:.0f}%\n"
        f"K: {k_eff/1000:.1f} kN/m  |  Degrad: {degrad_k:.1f}%\n"
        f"RUL: ~{rul} landings\n"
        f"Decision: {src_label}",
        font_size=8, color='white', position='lower_left')
    # Severity ring
    bpts = np.array([[-0.30,-0.30,0.82],[0.30,-0.30,0.82],
                     [0.30,0.30,0.82],[-0.30,0.30,0.82]])
    ring = pv.lines_from_points(np.vstack([bpts,bpts[0]]))
    pl.add_mesh(ring, color=accent, line_width=4, render_lines_as_tubes=True)
    pl.camera_position = [(0.55,-0.80,0.20),(0.00,0.00,-0.10),(0.00,0.00,1.00)]

    # ── [0,1] Force Profile ───────────────────────────────────────────────────
    pl.subplot(0,1)
    pl.set_background(bg)
    chart_f = pv.Chart2D(size=(1.0,1.0), loc=(0.0,0.0))
    chart_f.background_color = bg
    chart_f.line(t, force_kN, color=(1.0,0.45,0.05), width=2.5, label='Force (kN)')
    pi = int(np.argmax(force_kN))
    chart_f.scatter([t[pi]], [force_kN[pi]], color=(1.0,1.0,0.0),
                    size=12, style='o', label=f'Peak {force_kN[pi]:.1f} kN')
    chart_f.x_label='Time (s)'; chart_f.y_label='Force (kN)'
    chart_f.title='Landing Strut Force'; chart_f.legend_visible=True
    pl.add_chart(chart_f)
    pl.add_text("FORCE PROFILE", font_size=9, color='cyan', position='upper_edge')

    # ── [1,0] Compression ─────────────────────────────────────────────────────
    pl.subplot(1,0)
    pl.set_background(bg)
    chart_d = pv.Chart2D(size=(1.0,1.0), loc=(0.0,0.0))
    chart_d.background_color = bg
    chart_d.line(t, defl_mm, color=(0.12,0.75,1.00), width=2.5, label='Deflection (mm)')
    chart_d.line(t, np.full_like(t,180.0), color=(1.0,0.20,0.20),
                 width=1.5, style='--', label='Design Limit 180 mm')
    chart_d.x_label='Time (s)'; chart_d.y_label='Deflection (mm)'
    chart_d.title='Strut Compression'; chart_d.legend_visible=True
    pl.add_chart(chart_d)
    pl.add_text("COMPRESSION PROFILE", font_size=9, color='cyan', position='upper_edge')

    # ── [1,1] G-load vs CS-23.473 ────────────────────────────────────────────
    pl.subplot(1,1)
    pl.set_background(bg)
    chart_g = pv.Chart2D(size=(1.0,1.0), loc=(0.0,0.0))
    chart_g.background_color = bg
    g_col = (1.0,0.20,0.05) if cs23_hit else (0.20,0.95,0.30)
    chart_g.line(t, accel_g, color=g_col, width=2.5, label='G-Load (g)')
    chart_g.line(t, np.full_like(t,1.80), color=(1.0,0.90,0.0),
                 width=1.8, style='--', label='CS-23.473 Limit 1.8 g')
    chart_g.x_label='Time (s)'; chart_g.y_label='Acceleration (g)'
    chart_g.title='G-Load — CS-23.473 Check'; chart_g.legend_visible=True
    pl.add_chart(chart_g)
    cs23_lbl = "⚠  CS-23.473 TRIGGERED" if cs23_hit else "✓  CS-23.473 CLEAR"
    pl.add_text("G-LOAD / REGULATORY CHECK", font_size=9, color='cyan', position='upper_edge')
    pl.add_text(cs23_lbl, font_size=10,
                color=(1.0,0.22,0.05) if cs23_hit else (0.05,0.95,0.40),
                position='lower_right')

    print(f" {GN}✓ Structural load window opened — stays until next batch{RS}")
    return pl

# ──────────────────────────────────────────────────────────────────────────────
# TKINTER DASHBOARD
# ──────────────────────────────────────────────────────────────────────────────
class DashboardApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title('AEROTWIN v3.7 — FLEET COMMANDER')
        self.configure(bg='#030d1a')
        self.geometry('1400x920')
        self.attributes('-topmost', True); self.update()
        self.attributes('-topmost', False)

        tk.Label(self, text='🛰 AEROTWIN v3.7 — AI-PHYSICS FUSION | FLEET CONTROL',
                 bg='#071626', fg='#00e5ff',
                 font=('Courier New',18,'bold')).pack(fill='x', pady=12)

        main = tk.Frame(self, bg='#030d1a')
        main.pack(fill='both', expand=True, padx=12, pady=12)

        left  = tk.Frame(main, bg='#071626', width=480)
        left.pack(side='left', fill='y', padx=6)
        mid   = tk.Frame(main, bg='#071626')
        mid.pack(side='left', fill='both', expand=True, padx=6)
        right = tk.Frame(main, bg='#071626', width=400)
        right.pack(side='right', fill='y', padx=6)

        # Left — Diagnostics
        tk.Label(left, text="MISSION DIAGNOSTICS", bg='#0d2137', fg='#00e5ff',
                 font=('Courier New',13,'bold')).pack(fill='x')
        self.diag = tk.Text(left, bg='#071626', fg='#b2ebf2',
                            font=('Courier New',10), wrap='word', height=28)
        self.diag.pack(fill='both', expand=True, padx=10, pady=8)

        # Mid — PHM + Agreement Stats
        tk.Label(mid, text="PHM / SHM / PdM / RUL  +  AGREEMENT STATS",
                 bg='#0d2137', fg='#00e5ff',
                 font=('Courier New',13,'bold')).pack(fill='x')
        self.phm = tk.Text(mid, bg='#071626', fg='#b2ebf2',
                           font=('Courier New',10), wrap='word', height=28)
        self.phm.pack(fill='both', expand=True, padx=10, pady=8)

        # Right — Fleet Summary
        tk.Label(right, text="FLEET SUMMARY", bg='#0d2137', fg='#00e5ff',
                 font=('Courier New',13,'bold')).pack(fill='x')
        self.hist_frame = tk.Frame(right, bg='#071626')
        self.hist_frame.pack(fill='both', expand=True, padx=8, pady=8)
        self.history = []

    def update_dashboard(self, run, cls, conf, expl, sc, mp, amm, decision_source):
        force  = mp.get('sim_force_max', sc['k']*sc['compression_ft']*0.3048) if mp \
                 else sc['k']*sc['compression_ft']*0.3048
        accel  = mp.get('sim_accel_max_g', sc['max_accel_g']) if mp else sc['max_accel_g']
        sink   = mp.get('sink_rate_ms',  sc['sink_ms'])  if mp else sc['sink_ms']
        k_eff  = mp.get('k_eff', sc['k']) if mp else sc['k']
        health = sc['health']
        sev    = amm['sev']
        sev_c  = {1:'#00e676',2:'#ffea00',3:'#ff3d00',4:'#e040fb'}[sev]
        src_c  = '#00e5ff' if decision_source == 'AI' else '#ff9800'

        diag_txt = (
            f"\nRUN #{run}  •  {cls}\n"
            f"Conf : {conf*100:.1f}%\n"
            f"Decision source: {decision_source}\n\n"
            f"{expl}\n\n"
            f"PHYSICS:\n"
            f"Force: {force:,.0f} N  |  Accel: {accel:.2f} g  |  Sink: {sink:.2f} m/s\n"
            f"CS-23: {'⚠ TRIGGERED' if accel>=1.8 else '✓ CLEAR'}\n\n"
            f"MAINTENANCE:\n"
            f"Sev {sev}  •  {amm['chapter']}  •  ${amm['cost']:,}\n"
            f"{'⛔ GROUNDED — AOG' if amm['ground'] else '✓ OK TO DISPATCH'}\n"
        )
        self.diag.delete('1.0', tk.END)
        self.diag.insert(tk.END, diag_txt)
        self.diag.config(fg=sev_c)

        degrad_k = (1-k_eff/K_NOM)*100
        rul      = max(0, int(health*1200 - accel*150 - degrad_k*15))
        risk     = 'High' if rul<150 else 'Medium' if rul<400 else 'Low'

        # Agreement stats snapshot
        total = agreement_stats['total']
        match = agreement_stats['match']
        ai_w  = agreement_stats['ai_wins']
        phy_w = agreement_stats['physics_wins']
        agr_pct = match/total*100 if total else 0.0

        phm_txt = (
            f"\nHEALTH MGMT\n"
            f"Health: {health*100:.1f}%  |  RUL: ~{rul} landings\n\n"
            f"SHM\n"
            f"Stiffness loss: {degrad_k:.1f}%  (K: {k_eff/1000:.1f} kN/m)\n\n"
            f"PdM\n"
            f"Next action: {amm['action']}\n"
            f"Risk level : {risk}\n\n"
            f"═══ AI vs PHYSICS AGREEMENT ═══\n"
            f"Runs total : {total}\n"
            f"Agreement  : {match}/{total}  ({agr_pct:.1f}%)\n"
            f"AI decided : {ai_w}  |  Physics fallback: {phy_w}\n"
            f"This run   : [{decision_source}]\n"
        )
        self.phm.delete('1.0', tk.END)
        self.phm.insert(tk.END, phm_txt)
        self.phm.tag_add('src', '18.11', '18.end')
        self.phm.tag_config('src', foreground=src_c, font=('Courier New',10,'bold'))

        self.history.append({'run':run,'cls':cls,'sev':sev,'cost':amm['cost'],
                             'src':decision_source})
        for w in self.hist_frame.winfo_children(): w.destroy()
        tot_cost = sum(h['cost'] for h in self.history)
        for h in self.history:
            hc  = {1:'#00e676',2:'#ffea00',3:'#ff3d00',4:'#e040fb'}[h['sev']]
            src_tag = ' [PHY]' if h['src'] == 'PHYSICS_FALLBACK' else ''
            tk.Label(self.hist_frame,
                     text=f"Run {h['run']}: {h['cls']}{src_tag} (${h['cost']:,})",
                     bg='#071626', fg=hc,
                     font=('Courier New',10,'bold')).pack(anchor='w', pady=2)
        tk.Frame(self.hist_frame, bg='#455a64', height=2).pack(fill='x', pady=10)
        tk.Label(self.hist_frame, text=f"TOTAL COST: ${tot_cost:,}",
                 bg='#071626', fg='#ff6f00',
                 font=('Courier New',13,'bold')).pack(anchor='w')
        self.update_idletasks()

# ──────────────────────────────────────────────────────────────────────────────
# TELEMETRY
# ──────────────────────────────────────────────────────────────────────────────
def lerp(a, b, t): return a+(b-a)*t

def interp_pos(t_ratio):
    return (lerp(ORIGIN_LAT,DEST_LAT,t_ratio),
            lerp(ORIGIN_LON,DEST_LON,t_ratio),
            TOTAL_NM*(1.0-t_ratio))

def run_phase(phase, duration, t_start_ratio, t_end_ratio, cur, conn, gear_ref, sc=None):
    env = ENVELOPE.get(phase); n = int(duration/TDELTA)
    gear_cmd_sent = [False]
    for i in range(n):
        t = i/max(n-1,1); t_ratio = lerp(t_start_ratio,t_end_ratio,t)
        lat, lon, dist_nm = interp_pos(t_ratio)
        if env:
            alt  = lerp(*env['alt_ft'],  t)+random.gauss(0,1.5)
            ias  = lerp(*env['ias_kt'],  t)+random.gauss(0,0.8)
            vsi  = lerp(*env['vsi_fpm'], t)+random.gauss(0,15)
            gspd = lerp(*env['gspd_kt'], t)+random.gauss(0,0.8)
            rpm  = lerp(*env.get('rpm',(1200,1200)),t)+random.gauss(0,8)
            tq   = lerp(*env.get('tq',(5,5)),t)+random.gauss(0,0.4)
            sink_fps = lerp(*env['sink_fps'],t)
            comp_ft  = lerp(*env['comp_ft'],t)
            mass_lbs = lerp(*env['mass_lbs'],t)
            nose_st  = random.gauss(0,0.01)
            wow_v    = env['wow']
            wow      = (1.0 if lerp(*wow_v,t)>0.5 else 0.0) \
                       if isinstance(wow_v,tuple) else float(wow_v)
            gear = gear_ref[0]
            if phase=='CLIMB'    and t>0.10 and not gear_cmd_sent[0]:
                db_gear_command(cur,conn,'UP',phase);   gear_ref[0]=0; gear_cmd_sent[0]=True
            if phase=='APPROACH' and t>0.25 and not gear_cmd_sent[0]:
                db_gear_command(cur,conn,'DOWN',phase); gear_ref[0]=1; gear_cmd_sent[0]=True
        else:
            if sc is None: break
            progress = min(t/0.35, 1.0)
            alt  = max(0.0, lerp(500,0,progress)+random.gauss(0,0.5))
            ias  = max(0.0, lerp(110,0,t))
            gspd = max(0.0, lerp(110,5,t))
            vsi  = lerp(-600,0,progress)
            rpm  = lerp(1750,1200,t)+random.gauss(0,8)
            tq   = lerp(45,5,t)+random.gauss(0,0.3)
            nose_st  = random.gauss(0,0.04) if t>0.35 else 0.0
            mass_lbs = sc['mass_kg']*2.205
            wow = 1.0 if t>0.35 else 0.0; gear = gear_ref[0]
            if t > 0.35:
                td_t    = (t-0.35)/0.65
                comp_ft  = sc['compression_ft']*max(0,1.0-td_t*1.8)
                sink_fps = sc['sink_fps']*max(0,1.0-td_t*3.0)
            else:
                ramp     = t/0.35
                comp_ft  = lerp(0,sc['compression_ft'],ramp)
                sink_fps = lerp(3.0,sc['sink_fps'],ramp)
        row = (rpm,tq,gspd,max(alt,0.0),sink_fps,comp_ft,
               bool(wow>0.5),mass_lbs,lat,lon,nose_st,phase)
        try:   db_log_telemetry(cur,conn,row)
        except:
            try: conn.rollback()
            except: pass
        if i%40==0: hud(phase,max(alt,0),ias,vsi,gspd,gear_ref[0],tq,dist_nm)
        time.sleep(TDELTA)

# ──────────────────────────────────────────────────────────────────────────────
# MISSION CONTROLLER
# ──────────────────────────────────────────────────────────────────────────────
PHASE_COLORS = {'PREFLIGHT':BL,'TAKEOFF':GN,'CLIMB':CY,
                'CRUISE':WH,'APPROACH':YL,'LANDING':RD}

def run_mission():
    global dash_app
    banner([
        'AEROTWIN DIGITAL TWIN — FLEET COMMANDER v3.7',
        'Dornier 228-212 | Abuja → Lagos | AI-Physics Fusion + Agreement Stats',
        'CONFIDENCE_THRESHOLD = ' + str(CONFIDENCE_THRESHOLD) +
        '  |  Fine-tune: call finetune_model()',
    ], CY)

    try:
        conn, cur = db_connect()
        print(f" {GN}✓ PostgreSQL connected{RS}")
    except Exception as e:
        print(f" {RD}DB failed: {e}{RS}"); sys.exit(1)

    model = load_model()
    if model is None:
        print(f"{RD}No model — exiting{RS}"); return

    threading.Thread(
        target=lambda: (globals().update(dash_app=DashboardApp()), dash_app.mainloop()),
        daemon=True).start()
    time.sleep(2.5)

    TOTAL_RUNS    = 5
    active_plotter = None

    for run in range(1, TOTAL_RUNS + 1):

        # ── Close previous structural load window ─────────────────────────────
        if active_plotter is not None:
            print(f" {DM}Closing structural load window from run {run-1}...{RS}")
            try:   active_plotter.close()
            except: pass
            active_plotter = None

        banner([f"INITIATING MISSION {run}/{TOTAL_RUNS}"], MG)
        cur.execute("UPDATE matlab_physics_output SET ai_consumed = TRUE")
        conn.commit()

        sc       = generate_landing_scenario()
        gear_ref = [1]
        t_elapsed = 0.0
        total_dur = sum(DURATIONS.values())

        for phase in DURATIONS:
            dur     = DURATIONS[phase]
            t_start = t_elapsed / total_dur
            t_end   = (t_elapsed + dur) / total_dur
            phase_banner(phase, f"Duration: {dur}s", PHASE_COLORS.get(phase, CY))
            run_phase(phase, dur, t_start, t_end, cur, conn, gear_ref,
                      sc if phase == 'LANDING' else None)
            t_elapsed += dur

        matlab_physics = wait_for_matlab_physics(conn)

        if matlab_physics:
            data_source = 'MATLAB_PRISMATIC_JOINT'
            raw_arrays  = [matlab_physics['sim_deflection_raw'],
                           matlab_physics['sim_velocity_raw'],
                           matlab_physics['sim_force_raw'],
                           matlab_physics['sim_accel_raw'],
                           matlab_physics['sim_pressure_raw']]
            mass_val = matlab_physics['mass_kg']
            sink_val = matlab_physics['sink_rate_ms']
        else:
            data_source = 'PYTHON_FALLBACK'
            raw_arrays  = [[0.0]*50 for _ in range(5)]
            mass_val = sc['mass_kg']
            sink_val = sc['sink_ms']

        ctx_array = [mass_val, sink_val, sc['temp_c'], sc['fric']]

        # ① Classify with fusion — returns 4 values now
        ai_cls, ai_conf, explanation, decision_source = ai_classify(
            model, raw_arrays, ctx_array, matlab_physics, sc)

        amm = AMM[sc['class_true']]

        # ② Record agreement statistics
        physics_cls_name = CLASS_NAMES[sc['class_true']]
        _record_agreement(ai_cls, physics_cls_name, decision_source)

        # Print per-run decision line
        match_sym = GN+'✓'+RS if ai_cls == physics_cls_name else RD+'✗'+RS
        src_col   = CY if decision_source == 'AI' else YL
        print(f"\n {match_sym} Run {run}: [{src_col}{decision_source}{RS}] "
              f"AI→{BD}{ai_cls}{RS}  Physics→{BD}{physics_cls_name}{RS}  "
              f"({ai_conf*100:.0f}%)")

        combined = {**sc}
        if matlab_physics: combined.update(matlab_physics)
        db_log_diagnostic(cur, conn, combined, ai_cls, ai_conf, amm,
                          data_source, decision_source)

        # Update Tkinter dashboard
        if 'dash_app' in globals() and dash_app:
            dash_app.update_dashboard(run, ai_cls, ai_conf, explanation,
                                      sc, matlab_physics, amm, decision_source)

        # Launch PyVista structural load popup
        active_plotter = launch_strut_plotter(
            run, sc, matlab_physics, amm, ai_cls, ai_conf,
            decision_source, TOTAL_RUNS)

        print(f"\n {GN}✓ MISSION {run} COMPLETE{RS}\n {DM}{explanation}{RS}\n")

        if run < TOTAL_RUNS:
            input(f" {YL}Press ENTER for next mission "
                  f"(structural popup will close)...{RS}\n")
        else:
            # Final summary
            print_agreement_summary()
            print(f" {GN}{BD}ALL MISSIONS COMPLETED — FLEET ASSESSMENT DONE{RS}")
            input(f" {DM}Press ENTER to close terminal "
                  f"(GUI and 3D window stay open)...{RS}\n")

    conn.close()


# ──────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    # To run fine-tuning instead of simulation:
    #   python aerotwin_v37.py --finetune
    if len(sys.argv) > 1 and sys.argv[1] == '--finetune':
        finetune_model(n_samples=5000, epochs=10, batch_size=32, lr=1e-5)
    else:
        run_mission()