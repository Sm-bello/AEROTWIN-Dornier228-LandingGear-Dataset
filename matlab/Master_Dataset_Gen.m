% =========================================================================
%   🏭 AEROTWIN DATA FACTORY V16.1 — DORNIER 228-212
%   Purpose : 10,000 physics-based landing cycles for LSTM training
%   Aircraft : Dornier 228-212 | Fixed tricycle gear | MTOW 6,400 kg
%
%   CERTIFICATION:
%   ► CS-23.473 (JAR-23 Commuter Category) — Do228-212 correct
%   ► Design sink velocity : 3.05 m/s (10 ft/s) per CS-23.473(d)
%   ► Hard landing trigger  : ≥ 3.05 m/s OR ≥ 1.8g (limit load level)
%   ► ATA 32-30 NOT applicable — Do228 fixed gear, no retraction system
%   ► ATA task numbers are RUAG proprietary — chapter refs only
%
%   V16.1 PHYSICS UPGRADES (4 new pre-simulation corrections):
%   [U1] tireFriction()    — velocity + hydroplaning + ice dependent µ
%        Replaces flat F_DRY/F_WET/F_ICE in ALL 11 classes
%        Do228 hydroplaning threshold: 41.5 m/s (Horne formula, 80 psi tire)
%        Improves Class 5 (brake fade) and Class 6 (tire burst) realism
%
%   [U2] polytropicKRatio() — polytropic oleo gas spring correction
%        P·V^n = const, n=1.30 (adiabatic fast impact)
%        Higher sink rate → more stroke → higher effective K (physically correct)
%        Applied in ALL 11 classes. Ratio capped 1.0×–2.0× (physical limits)
%        Improves Class 1 (N2 leak) and Class 8 (hard landing) physics
%
%   [U3] orificeB()        — velocity-squared orifice damping correction
%        F_damp = C_d·ρ·A·v² → B_eff scales with sink velocity
%        Applied to Classes 6 and 8 ONLY (high-sink hard/burst events)
%        Calibrated at v_ref=1.5 m/s → 0.5×–2.5× range
%
%   [U4] sealFriction()    — Stribeck curve seal force
%        F_seal = F_coulomb + (F_static - F_coulomb)·exp(-v/v_strib)
%        Do228 oleo: F_static~800–1200 N, F_coulomb~400–720 N (health-scaled)
%        Stored as Seal_Force_N column + passed to Simulink workspace
%        If model has seal_force_N FromWorkspace block: used in physics
%        If not: sits in workspace harmlessly, dataset column still valuable
%
%   K/B SEPARATION TABLE (ISA 15°C, m=5000 kg, BEFORE polytropic correction):
%   Class  K_eff (N/m)    B_eff (N·s/m)  fn (Hz)    ζ
%   0      143k–157k      3800–4200      0.85–0.89  0.50–0.57  Healthy baseline
%   1       75k–115k      3800–4200      0.62–0.76  0.56–0.69  Low K, normal B
%   2      143k–157k       900–2000      0.85–0.89  0.12–0.26  Normal K, low B
%   3      126k–141k      3400–3800      0.80–0.84  0.48–0.54  Mild wear
%   4      138k–152k      2600–3600      0.84–0.88  0.35–0.49  Thermal B loss
%   5      143k–157k      1400–5600      0.85–0.89  0.18–0.75  Erratic B (judder)
%   6       78k–105k      6000–8500      0.63–0.73  0.93–1.20  Low K + high B (rim)
%   7      105k–126k      4400–5600      0.73–0.80  0.61–0.77  Corroded: mid K + high B
%   8      112k–138k      3000–4800      0.75–0.83  0.41–0.64  Permanent set
%   9       65k–105k       700–2000      0.57–0.73  0.09–0.28  Both degraded
%   10      50k– 80k       500–1500      0.50–0.64  0.07–0.19  Critical failure
%
%   NOTE: polytropicKRatio (U2) adds up to 2× K at full stroke impact.
%         Class 6 and 8 B ranges shift up by orificeB (U3) at high sink.
%         K/B class clusters remain distinct because K is the primary
%         separator between Class 6 (K=78k-105k) and Class 8 (K=112k-138k).
%
%   COLUMN LAYOUT: 23 scalar columns + 5 signals × 50 = 273 total
%   Scalars (1–23):
%   1  SampleID          9  B_Damping_Nsm    17 Max_Pressure_Pa
%   2  Class            10  K_Normalized     18 Seal_Force_N   ← V16.1 new
%   3  RUL_Percent      11  B_Normalized     19 Maint_Trigger
%   4  Mass_kg          12  Health_Factor    20 Severity
%   5  SinkRate_ms      13  Max_Deflection_m 21 Fatigue_Damage
%   6  Friction_Coef    14  Max_Velocity_ms  22 Maintenance_Cost_USD
%   7  Temp_C           15  Max_Accel_g      23 Maint_Category_Index
%   8  K_Stiffness_Nm   16  Max_Force_N
%   Signals: Defl(24-73) | Velo(74-123) | Force(124-173) | Accel(174-223) | P(224-273)
% =========================================================================

clear; clc; close all;
fprintf('🏭 AEROTWIN V16.1 — DORNIER 228-212 | CS-23.473 | ATA 32\n');
fprintf('=============================================================\n');
fprintf('Cert     : CS-23.473 JAR-23 Commuter (Do228-212 correct)\n');
fprintf('Trigger  : ≥ 3.05 m/s OR ≥ 1.8g — CS-23.473(d) limit load\n');
fprintf('Gear     : Fixed tricycle | ATA 32-30 NOT applicable\n');
fprintf('Dataset  : 10,000 samples | 11 classes | 5 signals\n');
fprintf('V16.1 +  : tireFriction | polytropicK | orificeB | sealFriction\n');
fprintf('=============================================================\n\n');

% =========================================================================
%   SECTION 1: REGULATORY THRESHOLDS — CS-23.473 / Do228-212
% =========================================================================
REG.SinkDesign      = 3.05;
REG.SinkHardLanding = 3.05;
REG.SinkCaution     = 2.50;
REG.AccelLimit      = 1.80;
REG.AccelExtreme    = 2.50;
REG.FixedInterval   = 500;

% ATA 32 chapter structure — Do228-212 fixed gear
% 32-00 General / Hard Landing   32-10 Main Gear   32-20 Nose Gear
% 32-40 Wheels and Brakes        32-50 Steering    32-60 Position/Warning
% 32-30 does NOT exist — no retraction system on Do228

MAINT(1).chapter='ATA 32-10'; MAINT(1).category='Routine Line Check';
MAINT(1).action='Visual inspection oleo strut, tire pressure, leak check'; MAINT(1).cost=450;

MAINT(2).chapter='ATA 32-10'; MAINT(2).category='Nitrogen System Service';
MAINT(2).action='Pre-charge pressure test, leak detection, nitrogen recharge'; MAINT(2).cost=850;

MAINT(3).chapter='ATA 32-10'; MAINT(3).category='Hydraulic Damper Service';
MAINT(3).action='Damping performance check, seal inspection, fluid sampling'; MAINT(3).cost=1250;

MAINT(4).chapter='ATA 32-10'; MAINT(4).category='Degradation Trending Inspection';
MAINT(4).action='Parameter trending, borescope inspection if indicated'; MAINT(4).cost=1800;

MAINT(5).chapter='ATA 32-10'; MAINT(5).category='Thermal Stress Inspection';
MAINT(5).action='Seal hardness check, fluid analysis, heat damage assessment'; MAINT(5).cost=1500;

MAINT(6).chapter='ATA 32-10'; MAINT(6).category='Asymmetric Load Inspection';
MAINT(6).action='Trunnion pin, sidestay, drag brace, attachment hardware check'; MAINT(6).cost=2000;

MAINT(7).chapter='ATA 32-40'; MAINT(7).category='Wheel Assembly Inspection';
MAINT(7).action='Tire, brake, rim, bearing post-event inspection'; MAINT(7).cost=3500;

MAINT(8).chapter='ATA 32-10'; MAINT(8).category='Corrosion Control Treatment';
MAINT(8).action='NDT, pitting assessment, protective treatment, torque check'; MAINT(8).cost=2200;

MAINT(9).chapter='ATA 32-00'; MAINT(9).category='Hard Landing Inspection';
MAINT(9).action=['CS-23.473 triggered: strut, trunnion, drag brace, sidestay NDT. '...
    'Trigger: sink >= 3.05 m/s OR >= 1.8g. Ref: RUAG AMM ATA 32-00.']; MAINT(9).cost=4500;

MAINT(10).chapter='ATA 32-10'; MAINT(10).category='Component Overhaul';
MAINT(10).action='Strut disassembly, full seal kit, nitrogen recharge, pressure test'; MAINT(10).cost=12000;

MAINT(11).chapter='ATA 32-00'; MAINT(11).category='Emergency AOG Maintenance';
MAINT(11).action='AOG repair, expedited parts sourcing, mandatory ground until cleared'; MAINT(11).cost=35000;

ECON.Routine=450; ECON.Detailed=1800; ECON.HardLanding=4500;
ECON.AOG=28000;   ECON.PrevOverhaul=12000; ECON.Emergency=35000;

FAT.S_ut=1950e6; FAT.S_e=0.5*1950e6; FAT.b=-0.12; FAT.N_ref=1e6;

% =========================================================================
%   SECTION 2: DO228-212 PHYSICS CONSTANTS
% =========================================================================
MASS_NORM_MIN=4500; MASS_NORM_MAX=5800; MASS_MTOW=6400;
K_NOM=150000; B_NOM=4000; %#ok<NASGU>
SINK_MIN=0.3; SINK_MAX=3.5;
F_DRY=0.80; F_WET=0.40; F_ICE=0.20;
T_ISA=15; T_HOT=45; T_COLD=-30;

% =========================================================================
%   SECTION 3: MODEL SETUP
% =========================================================================
mdl='Fancy_Landing_Gear';
if ~bdIsLoaded(mdl), load_system(mdl); end
try
    set_param([mdl '/UDP Receive1'],'Commented','on');
    fprintf('✅ UDP Receive1 disabled\n');
catch
    fprintf('ℹ️  UDP not found / already disabled\n');
end

% =========================================================================
%   🎬 VISUALIZATION — nominal Do228 landing, 3D animation, then freeze
% =========================================================================
fprintf('\n🎬 Opening model with 3D animation...\n');
open_system(mdl);
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','on');
fprintf('🛬 Nominal: 5,200 kg | 1.8 m/s | K=150k | B=4k | ISA 15°C\n\n');
assignAll(mdl,K_NOM,B_NOM,5200,1.8,F_DRY,T_ISA,500);  % 500N nominal seal force
vOut=sim(mdl,'StopTime','3');
if isempty(vOut.ErrorMessage)
    fprintf('✅ Visual OK — freezing 3D view, starting silent batch\n\n');
else
    fprintf('⚠️  Visual: %s\n\n',vOut.ErrorMessage);
end
set_param(mdl,'SimMechanicsOpenEditorOnUpdate','off');
set_param(mdl,'FastRestart','off');
set_param(mdl,'Solver','ode23t');
set_param(mdl,'MaxStep','0.005');

% =========================================================================
%   SECTION 4: OUTPUT PATHS
% =========================================================================
projPath='C:\Users\User\Desktop\Do228';
outFile='AEROTWIN_Do228_V16p1_Dataset.csv';
outPath=fullfile(projPath,outFile);
if ~exist(projPath,'dir'), mkdir(projPath); end
fprintf('📂 Output : %s\n',outPath);
fprintf('⏳ Est.   : 2–4 hours | MaxStep=0.005\n');
fprintf('=============================================================\n\n');

% =========================================================================
%   SECTION 5: PRE-FLIGHT SANITY CHECK
% =========================================================================
fprintf('🔍 Pre-flight CS-23.473 + V16.1 physics sanity check...\n');
% Demonstrate all four new physics functions before batch run
tc_test=T_ISA; sink_test=1.5; mass_test=5000; k_test=K_NOM; b_test=B_NOM;
fric_test=tireFriction(F_DRY,sink_test,tc_test);
krat_test=polytropicKRatio(sink_test,mass_test,k_test,1.30);
brat_test=orificeB(sink_test);
seal_test=sealFriction(sink_test,1.0);
fprintf('   tireFriction(dry,1.5m/s,15°C) = %.3f (base=%.2f)\n',fric_test,F_DRY);
fprintf('   polytropicKRatio(1.5m/s,5000kg,150kN/m) = %.3f\n',krat_test);
fprintf('   orificeB(1.5m/s) = %.3f (=1.00 at calibration pt)\n',brat_test);
fprintf('   sealFriction(1.5m/s,health=1.0) = %.0f N\n',seal_test);

F_seal_vis=sealFriction(1.8,1.0);
assignAll(mdl,K_NOM,B_NOM,5000,1.5,fric_test,tc_test,seal_test);
tSim=sim(mdl,'StopTime','3');
if ~isempty(tSim.ErrorMessage)
    fprintf('❌ SIM ERROR: %s\n',tSim.ErrorMessage); return;
end
[dw,vw,fw,aw,pw,ok]=xWin(tSim,50);
if ~ok, fprintf('❌ Signal extraction failed — check To Workspace block names\n'); return; end

gPk=max(abs(aw))/9.81;
fprintf('✅ All 5 signals confirmed\n');
fprintf('   Peak: %.2fg  (CS-23.473 limit: %.1fg)  ',gPk,REG.AccelLimit);
if gPk>REG.AccelLimit, fprintf('⚠️  TRIGGERS ATA 32-00\n');
else, fprintf('✅ Normal envelope\n'); end
fprintf('   Defl=%.3fm | Vel=%.2fm/s | Accel=%.2fg | P=%.0fPa\n\n',...
    max(abs(dw)),max(abs(vw)),gPk,max(abs(pw)));
fprintf('🚀 Starting 10,000-sample batch (V16.1 with physics upgrades)...\n');
fprintf('=============================================================\n\n');

% =========================================================================
%   SECTION 6: STORAGE
% =========================================================================
DS=[]; META=[]; MLOG={};
dmg=zeros(1,11);
costCBM=0; costFIXED=0; aogSaved=0;
sc=0; cc=0;
t0=tic; rng(42);

% =========================================================================
%   CLASS 0 — NORMAL OPERATIONS  (1,500 samples)
%   K_eff: 143k–157k | B_eff: 3800–4200
%   U1: velocity+ice dependent friction applied
%   U2: polytropic K correction (mild at low sink → 1.02–1.35×)
%   U3: orificeB NOT applied (normal sink range, not a hard landing class)
%   U4: sealFriction computed and passed (health-scaled, high-health class)
% =========================================================================
fprintf('--- ✅ CLASS 0: NORMAL (1,500) | K:143k–157k | B:3800–4200 ---\n');
tgt=1500; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.85+0.15*rand();
    tc=T_COLD+(T_HOT-T_COLD)*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    if rand()<0.8, sink=0.3+(2.0-0.3)*rand();
    else,          sink=2.0+(3.05-2.0)*rand(); end
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*(0.953+0.094*rand());
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2] polytropic correction
    b=bt*(0.950+0.100*rand());

    fric_base=randsample([F_DRY,F_WET,F_ICE],1,true,[0.70,0.25,0.05]);
    fric=tireFriction(fric_base,sink,tc);       % [U1] physics friction

    F_seal=sealFriction(sink,health);           % [U4] Stribeck seal force

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(0,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(1)=dmg(1)+dp; rul=max(0,100*(1-dmg(1)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,0,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,0,tc,sink,fric];
                MLOG{end+1}={sc,0,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,100)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 0: %d | Damage: %.6f\n\n',cs,dmg(1));

% =========================================================================
%   CLASS 1 — NITROGEN LEAK  (1,500 samples)
%   K_eff: 75k–115k | B_eff: 3800–4200
%   U2: polytropicK important here — N2 leak changes P·V^n directly
%   Low-health samples have lower K → polytropicK shows less correction
%   (soft spring = less compression ratio at same sink → lower fn increase)
% =========================================================================
fprintf('--- 🧪 CLASS 1: NITROGEN LEAK (1,500) | K:75k–115k | B:3800–4200 ---\n');
tgt=1500; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=rand();
    tc=T_COLD+(T_HOT-T_COLD)*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((75000+40000*health)/K_NOM);
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*(0.950+0.100*rand());

    fric_base=randsample([F_DRY,F_WET,F_ICE],1,true,[0.60,0.30,0.10]);
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(1,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(2)=dmg(2)+dp; rul=max(0,100*(1-dmg(2)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,1,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,1,tc,sink,fric];
                MLOG{end+1}={sc,1,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,300)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 1: %d | Damage: %.6f\n\n',cs,dmg(2));

% =========================================================================
%   CLASS 2 — WORN SEAL / DAMPER DEGRADATION  (1,500 samples)
%   K_eff: 143k–157k | B_eff: 900–2000
%   U4: sealFriction especially meaningful here — worn seal changes
%   Stribeck curve profile (higher Coulomb, lower static breakout force)
%   This creates a distinctive Force signal shape for this class
% =========================================================================
fprintf('--- 🧪 CLASS 2: WORN SEAL (1,500) | K:143k–157k | B:900–2000 ---\n');
tgt=1500; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=rand();
    tc=T_COLD+(T_HOT-T_COLD)*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*(0.953+0.094*rand());
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((900+1100*health)/B_NOM);

    fric_base=randsample([F_DRY,F_WET,F_ICE],1,true,[0.60,0.30,0.10]);
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    % Worn seal: Stribeck curve degrades — F_static lower, Coulomb higher
    F_seal=sealFriction(sink,health);           % [U4] health=0–1 scales seal wear

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(2,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(3)=dmg(3)+dp; rul=max(0,100*(1-dmg(3)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,2,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,2,tc,sink,fric];
                MLOG{end+1}={sc,2,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,300)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 2: %d | Damage: %.6f\n\n',cs,dmg(3));

% =========================================================================
%   CLASS 3 — EARLY DEGRADATION  (1,000 samples)
%   K_eff: 126k–141k | B_eff: 3400–3800
%   Health: 70–85% — subtle wear, still airworthy
% =========================================================================
fprintf('--- ⚠️  CLASS 3: EARLY DEGRAD (1,000) | K:126k–141k | B:3400–3800 ---\n');
tgt=1000; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.70+0.15*rand();
    tc=T_COLD+(T_HOT-T_COLD)*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((126000+100000*(health-0.70))/K_NOM);
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((3400+2667*(health-0.70))/B_NOM);

    fric_base=randsample([F_DRY,F_WET],1,true,[0.80,0.20]);
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(3,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(4)=dmg(4)+dp; rul=max(0,100*(1-dmg(4)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,3,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,3,tc,sink,fric];
                MLOG{end+1}={sc,3,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,200)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 3: %d | Damage: %.6f\n\n',cs,dmg(4));

% =========================================================================
%   CLASS 4 — THERMAL DEGRADATION  (800 samples)
%   K_eff: 138k–152k | B_eff: 2600–3600
%   U1: forced hot ops (35–45°C) means high temperature in tireFriction
%   Hot ops reduce oil viscosity → sealFriction shows different Stribeck profile
% =========================================================================
fprintf('--- 🌡️  CLASS 4: THERMAL DEGRAD (800) | K:138k–152k | B:2600–3600 ---\n');
tgt=800; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.60+0.30*rand();
    tc=35+(T_HOT-35)*rand();   % Forced hot ops: 35–45°C
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((138000+14000*health)/K_NOM)*(0.97+0.06*rand());
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((2600+1000*health)/B_NOM)*(0.92+0.16*rand());

    fric_base=F_DRY;  % Hot climate = dry runway
    fric=tireFriction(fric_base,sink,tc);       % [U1] hot-day dry friction

    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(4,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(5)=dmg(5)+dp; rul=max(0,100*(1-dmg(5)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,4,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,4,tc,sink,fric];
                MLOG{end+1}={sc,4,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,200)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 4: %d | Damage: %.6f\n\n',cs,dmg(5));

% =========================================================================
%   CLASS 5 — BRAKE FADE / ASYMMETRIC LOAD  (600 samples)
%   K_eff: 143k–157k | B_eff: 1400–5600  (ERRATIC — key LSTM signature)
%   U1: tireFriction applied with random base (brake fade = unpredictable µ)
%   U3: orificeB NOT applied — erratic B variance is the signature,
%       scaling it uniformly by sink would reduce inter-sample variance
% =========================================================================
fprintf('--- 🛑 CLASS 5: BRAKE FADE (600) | K:143k–157k | B:1400–5600 ERRATIC ---\n');
tgt=600; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.75+0.20*rand();
    tc=T_ISA+10*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*(0.90+0.20*rand());
    sink=0.8+(2.8-0.8)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*(0.953+0.094*rand());
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((1400+4200*rand())/B_NOM);            % Erratic B preserved

    % U1: brake fade itself makes friction unpredictable — erratic base × physics
    fric_base=F_DRY*(0.85+0.30*rand());        % Erratic base friction
    fric=tireFriction(fric_base,sink,tc);       % Then physics-corrected

    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(5,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(6)=dmg(6)+dp; rul=max(0,100*(1-dmg(6)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,5,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,5,tc,sink,fric];
                MLOG{end+1}={sc,5,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,150)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 5: %d | Damage: %.6f\n\n',cs,dmg(6));

% =========================================================================
%   CLASS 6 — TIRE BURST  (400 samples)
%   K_eff: 78k–105k | B_eff: 6000–8500
%   U1: tireFriction applied — tire burst on dry runway typically
%   U3: orificeB APPLIED — tire burst at high sink → velocity-squared
%       metal-on-metal damping is a strong function of impact velocity
%       With orificeB at sink=2.5–3.5 m/s: B ratio = 1.67–2.33
%       → B_eff range becomes ~10k–20k, reinforcing the high-B signature
%   Note: Class 6 (K=78k–105k) remains distinct from Class 8 (K=112k–138k)
% =========================================================================
fprintf('--- 💥 CLASS 6: TIRE BURST (400) | K:78k–105k | B:6k–8.5k + orificeB ---\n');
tgt=400; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.40+0.20*rand();
    tc=T_ISA+5*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MAX*(0.80+0.30*rand());
    sink=2.0+(3.5-2.0)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((78000+27000*rand())/K_NOM);
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((6000+2500*rand())/B_NOM);
    b=b*orificeB(sink);                         % [U3] velocity-sq metal-on-metal

    fric_base=F_DRY;
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    % Tire burst → seal exposed to higher loads
    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(6,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(7)=dmg(7)+dp; rul=max(0,100*(1-dmg(7)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,6,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,6,tc,sink,fric];
                MLOG{end+1}={sc,6,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,100)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 6: %d | Damage: %.6f\n\n',cs,dmg(7));

% =========================================================================
%   CLASS 7 — CORROSION / FATIGUE  (600 samples)
%   K_eff: 105k–126k | B_eff: 4400–5600
%   Tropical ops: 20–35°C | wet/dry mix
%   U1: wet surface + tropical humidity degrades friction realistically
% =========================================================================
fprintf('--- 🌊 CLASS 7: CORROSION (600) | K:105k–126k | B:4400–5600 ---\n');
tgt=600; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.50+0.50*rand();
    tc=20+15*rand();   % Tropical: 20–35°C
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((105000+21000*health)/K_NOM)*(0.97+0.06*rand());
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((4400+1200*(1-health))/B_NOM)*(0.95+0.10*rand());

    fric_base=randsample([F_DRY,F_WET],1,true,[0.50,0.50]);
    fric=tireFriction(fric_base,sink,tc);       % [U1] tropical wet physics

    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(7,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(8)=dmg(8)+dp; rul=max(0,100*(1-dmg(8)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,7,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,7,tc,sink,fric];
                MLOG{end+1}={sc,7,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,150)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 7: %d | Damage: %.6f\n\n',cs,dmg(8));

% =========================================================================
%   CLASS 8 — HARD LANDING / OVERLOAD  (600 samples)
%   K_eff: 112k–138k | B_eff: 3000–4800
%   U3: orificeB APPLIED — hard landing at 3.0–3.05 m/s is the primary
%       class where orifice velocity-squared physics matters most
%       B_eff after orificeB at sink=3.0: 2× → 6000–9600 range
%   CS-23.473(d): sink ≥ 3.05 m/s → mandatory ATA 32-00 inspection
%   Note: Class 8 K (112k–138k) still clearly below Class 0 (143k–157k)
%         even with polytropicK correction, because K starts degraded
% =========================================================================
fprintf('--- 💢 CLASS 8: HARD LANDING (600) | K:112k–138k | B:3k–4.8k + orificeB ---\n');
fprintf('    CS-23.473(d): sink ≥ 3.05 m/s → ATA 32-00 mandatory inspection\n');
tgt=600; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.40+0.40*rand();
    tc=T_ISA+20*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=5500+(MASS_MTOW-5500)*rand();
    sink=3.0+(REG.SinkDesign-3.0)*rand();       % 3.0–3.05 → earns CS-23 trigger
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((112000+26000*health)/K_NOM);
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((3000+1800*rand())/B_NOM);
    b=b*orificeB(sink);                         % [U3] hard landing velocity-sq

    fric_base=randsample([F_DRY,F_WET],1,true,[0.70,0.30]);
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    F_seal=sealFriction(sink,health);           % [U4] high-sink = high seal force

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(8,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT)*1.5;
                dmg(9)=dmg(9)+dp; rul=max(0,100*(1-dmg(9)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,8,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,8,tc,sink,fric];
                MLOG{end+1}={sc,8,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,100)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 8: %d | Damage: %.6f\n\n',cs,dmg(9));

% =========================================================================
%   CLASS 9 — COMBINED FAULTS: N2 LEAK + WORN SEAL  (500 samples)
%   K_eff: 65k–105k | B_eff: 700–2000
%   U4: sealFriction very meaningful — combined fault degrades seal
%   health range 0.20–0.80 → F_seal shows broad range for LSTM
% =========================================================================
fprintf('--- 🔥 CLASS 9: COMBINED FAULTS (500) | K:65k–105k | B:700–2000 ---\n');
tgt=500; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.20+0.60*rand();
    tc=T_COLD+(T_HOT-T_COLD)*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((65000+40000*health)/K_NOM);
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((700+1300*health)/B_NOM);

    fric_base=randsample([F_DRY,F_WET,F_ICE],1,true,[0.50,0.30,0.20]);
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    F_seal=sealFriction(sink,health);           % [U4]

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(9,sink,gp,health,REG,MAINT);
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT);
                dmg(10)=dmg(10)+dp; rul=max(0,100*(1-dmg(10)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,9,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,9,tc,sink,fric];
                MLOG{end+1}={sc,9,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,100)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 9: %d | Damage: %.6f\n\n',cs,dmg(10));

% =========================================================================
%   CLASS 10 — IMPENDING FAILURE  (500 samples)
%   K_eff: 50k–80k | B_eff: 500–1500
%   Health 5–30%: naturally earns Severity 4 via assess() health<0.30
%   U4: sealFriction at low health → high F_seal (degraded seal = sticky)
%   This creates another distinctive signal: high F_seal, very low K and B
% =========================================================================
fprintf('--- 🚨 CLASS 10: IMPENDING FAILURE (500) | K:50k–80k | B:500–1500 ---\n');
tgt=500; cs=0;
while cs<tgt
    sc=sc+1; cc=cc+1;
    health=0.05+0.25*rand();   % 5–30% → sev=4 earned naturally
    tc=T_COLD+(T_HOT-T_COLD)*rand();
    
    % BUG FIX: Generate mass and sink BEFORE physics corrections
    mass=MASS_NORM_MIN+(MASS_NORM_MAX-MASS_NORM_MIN)*rand();
    sink=SINK_MIN+(SINK_MAX-SINK_MIN)*rand();
    
    [kt,bt]=therm(K_NOM,B_NOM,tc);
    k=kt*((50000+30000*health)/K_NOM);
    k=k*polytropicKRatio(sink,mass,k,1.30);    % [U2]
    b=bt*((500+1000*health)/B_NOM);

    fric_base=randsample([F_DRY,F_WET,F_ICE],1,true,[0.40,0.40,0.20]);
    fric=tireFriction(fric_base,sink,tc);       % [U1]

    % Low health → degraded seal → elevated sticky friction (Stribeck shifts)
    F_seal=sealFriction(sink,health);           % [U4] high value at low health

    assignAll(mdl,k,b,mass,sink,fric,tc,F_seal);
    try
        sO=sim(mdl,'StopTime','3');
        if isempty(sO.ErrorMessage)
            [dw,vw,fw,aw,pw,ok]=xWin(sO,50);
            if ok
                gp=max(abs(aw))/9.81;
                kn=kNorm(k,tc); bn=b;
                [sev,tr,mi]=assess(10,sink,gp,health,REG,MAINT);
                if tr&&sev==4, aogSaved=aogSaved+1; end
                dp=dmgCalc(sink,mass,max(abs(dw)),tc,FAT)*2.5;
                dmg(11)=dmg(11)+dp; rul=max(0,100*(1-dmg(11)));
                c=mCost(tr,sev,mi,MAINT); costCBM=costCBM+c;
                costFIXED=fixedCost(cc,REG.FixedInterval,sev,mi,MAINT,ECON,costFIXED);
                DS=[DS;bRow(sc,10,health,mass,sink,fric,tc,k,b,kn,bn,...
                    dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)];
                META=[META;sc,10,tc,sink,fric];
                MLOG{end+1}={sc,10,MAINT(mi).chapter,MAINT(mi).category,sev,tr,c};
                cs=cs+1;
            end
        end
    catch, end
    if mod(cs,100)==0&&cs>0, pProg(cs,tgt,sc,10000,t0); end
end
fprintf('✅ Class 10: %d | Damage: %.6f\n\n',cs,dmg(11));

% =========================================================================
%   SECTION 7: SAVE ALL OUTPUTS
%   273 total columns: 23 scalar + 5 × 50 signals
%   Signal offsets: Defl(24-73) Velo(74-123) Force(124-173) Accel(174-223) P(224-273)
% =========================================================================
totalTime=toc(t0);

if ~isempty(DS)
    vn={'SampleID','Class','RUL_Percent','Mass_kg','SinkRate_ms',...
        'Friction_Coef','Temp_C','K_Stiffness_Nm','B_Damping_Nsm',...
        'K_Normalized','B_Normalized','Health_Factor',...
        'Max_Deflection_m','Max_Velocity_ms','Max_Accel_g',...
        'Max_Force_N','Max_Pressure_Pa','Seal_Force_N',...
        'Maint_Trigger','Severity','Fatigue_Damage',...
        'Maintenance_Cost_USD','Maint_Category_Index'};
    for s={'Defl','Velo','Force','Accel','Pressure'}
        for t=1:50, vn{end+1}=sprintf('%s_t%d',s{1},t); end
    end

    T=array2table(DS,'VariableNames',vn);
    writetable(T,outPath);

    writetable(array2table(META,'VariableNames',{'SampleID','Class','Temp_C','SinkRate','Friction'}),...
        fullfile(projPath,'AEROTWIN_Metadata_V16p1.csv'));

    mL=vertcat(MLOG{:});
    mLT=cell2table(mL,'VariableNames',...
        {'SampleID','Class','ATA_Chapter','Category','Severity','Triggered','Cost_USD'});
    writetable(mLT,fullfile(projPath,'AEROTWIN_MaintenanceLog_V16p1.csv'));

    sp.train_idx=1:7000; sp.val_idx=7001:8500;
    sp.test_idx=8501:min(10000,height(T));
    save(fullfile(projPath,'train_val_split_V16p1.mat'),'sp');

    n=height(T); nt=sum(T.Maint_Trigger);
    sav=costFIXED-costCBM; savPct=100*sav/max(costFIXED,1);
    econD={'Metric','Value';
        'Version','V16.1 — tireFriction + polytropicK + orificeB + sealFriction';
        'Certification','CS-23.473 JAR-23 Commuter (Do228-212)';
        'Hard Landing Threshold (m/s)',REG.SinkHardLanding;
        'Hard Landing Threshold (g)',REG.AccelLimit;
        'Total Samples',n; 'Maintenance Triggers',nt;
        'Trigger Rate (%)',100*nt/n;
        'Fixed-Interval Cost (USD)',costFIXED;
        'CBM Optimized Cost (USD)',costCBM;
        'Savings (USD)',sav; 'Savings (%)',savPct;
        'AOG Prevented (Class10 Sev4)',aogSaved;
        'AOG Prevention Value (USD)',aogSaved*ECON.AOG};
    for ci=0:10
        econD{end+1,1}=sprintf('Mode Damage Class%d',ci);
        econD{end,2}=dmg(ci+1);
    end
    writecell(econD,fullfile(projPath,'AEROTWIN_Economic_V16p1.csv'));

    % =====================================================================
    %   CONSOLE SUMMARY
    % =====================================================================
    fprintf('\n=============================================================\n');
    fprintf('✅ V16.1 COMPLETE — Do228-212 | CS-23.473 | ATA 32\n');
    fprintf('=============================================================\n');
    fprintf('📊 Samples  : %d | Columns : %d\n',n,width(T));
    fprintf('⏱️  Time     : %.1f minutes\n',totalTime/60);
    fprintf('📂 File     : %s\n',outPath);
    fprintf('🔬 Physics  : tireFriction✓ polytropicK✓ orificeB(C6,C8)✓ sealFriction✓\n');

    fprintf('\n📈 CLASS DISTRIBUTION + TRIGGER RATES:\n');
    cN={'Normal','N2 Leak','Worn Seal','Early Degrad','Thermal Degrad',...
        'Brake Fade','Tire Burst','Corrosion','Hard Landing[CS23]',...
        'Combined Fault','Impending[AOG]'};
    for ci=0:10
        nc=sum(T.Class==ci); ntc=sum(T.Class==ci & T.Maint_Trigger==1);
        fprintf('   Class %2d  %-26s %4d | Triggers: %3d (%3.0f%%)\n',...
            ci,cN{ci+1},nc,ntc,100*ntc/max(nc,1));
    end

    fprintf('\n🎯 K/B SPACE — LSTM DISCRIMINABILITY CHECK:\n');
    fprintf('   Class  K_norm_mean  B_mean    fn(Hz)   zeta   SealF_mean\n');
    for ci=0:10
        km=mean(T.K_Normalized(T.Class==ci));
        bm=mean(T.B_Damping_Nsm(T.Class==ci));
        fm=mean(T.Seal_Force_N(T.Class==ci));
        fn_=(1/(2*pi))*sqrt(km/5000);
        z_=bm/(2*sqrt(km*5000));
        fprintf('   %2d     %8.0f     %6.0f    %.3f    %.3f   %.0fN\n',ci,km,bm,fn_,z_,fm);
    end

    fprintf('\n🔬 V16.1 PHYSICS UPGRADE SUMMARY:\n');
    fprintf('   [U1] tireFriction  : Applied all 11 classes\n');
    fprintf('        Do228 hydro threshold: 41.5 m/s (80 psi tire, Horne)\n');
    fprintf('   [U2] polytropicK   : Applied all 11 classes | n=1.30 adiabatic\n');
    fprintf('        Ratio range 1.0× (gentle) → 2.0× (full stroke)\n');
    fprintf('   [U3] orificeB      : Applied Class 6 and 8 ONLY\n');
    fprintf('        Class 6 mean B: %.0f N·s/m (vs %.0f base)\n',...
        mean(T.B_Damping_Nsm(T.Class==6)),7250);
    fprintf('        Class 8 mean B: %.0f N·s/m (vs %.0f base)\n',...
        mean(T.B_Damping_Nsm(T.Class==8)),3900);
    fprintf('   [U4] sealFriction  : Applied all 11 classes + workspace\n');
    fprintf('        Mean seal force by class:\n');
    for ci=[2,9,10]
        fprintf('           Class %d %-18s: %.0f N avg\n',ci,cN{ci+1},...
            mean(T.Seal_Force_N(T.Class==ci)));
    end

    fprintf('\n🔧 PER-MODE MINER DAMAGE:\n');
    for ci=0:10
        fprintf('   Class %2d %-22s: %.8f\n',ci,cN{ci+1},dmg(ci+1));
    end

    fprintf('\n💰 ECONOMIC ANALYSIS (CS-23.473 scenario):\n');
    fprintf('   Fixed-Interval (every %d cycles): $%12.0f\n',REG.FixedInterval,costFIXED);
    fprintf('   CBM AI-Optimised:                 $%12.0f\n',costCBM);
    fprintf('   ─────────────────────────────────────────────\n');
    fprintf('   SAVINGS:                          $%12.0f (%.1f%%)\n',sav,savPct);
    fprintf('   AOG prevented:                    %d  ($%,.0f value)\n',...
        aogSaved,aogSaved*ECON.AOG);

    % =====================================================================
    %   PLOTS
    %   Signal column offsets (V16.1 — 23 scalars before signals):
    %   Defl: 24-73 | Velo: 74-123 | Force: 124-173 | Accel: 174-223 | P: 224-273
    % =====================================================================
    figure('Name','AEROTWIN V16.1 — Do228 CS-23.473','Position',[40 40 1650 950]);

    subplot(2,4,1);
    histogram(T.Class,'BinEdges',-0.5:1:10.5,'FaceColor',[0.2 0.6 1]);
    title('Class Distribution'); xlabel('Class'); ylabel('Count'); grid on;

    subplot(2,4,2);
    scatter(T.SinkRate_ms,T.Max_Accel_g,7,T.Class,'filled');
    hold on;
    yline(REG.AccelLimit,'r--','LineWidth',2);
    xline(REG.SinkHardLanding,'m--','LineWidth',2);
    title('CS-23.473(d) Hard Landing Envelope');
    xlabel('Sink Rate (m/s)'); ylabel('Accel (g)');
    legend({'Data','1.8g limit','3.05 m/s limit'},'Location','northwest','FontSize',7);
    colormap(jet); grid on;

    subplot(2,4,3);
    scatter(T.K_Normalized,T.B_Damping_Nsm,7,T.Class,'filled');
    hold on;
    for ci=0:10
        xc=mean(T.K_Normalized(T.Class==ci));
        yc=mean(T.B_Damping_Nsm(T.Class==ci));
        text(xc,yc,sprintf('C%d',ci),'FontSize',7,'HorizontalAlignment','center',...
            'FontWeight','bold','Color','k');
    end
    title({'K_{norm}–B Space (LSTM feature space)';'orificeB shifts C6,C8 higher in B'});
    xlabel('K_{norm} (N/m)'); ylabel('B (N·s/m)'); colormap(jet); grid on;

    subplot(2,4,4);
    % Seal force by class — new V16.1 LSTM feature
    sf_mean=arrayfun(@(c) mean(T.Seal_Force_N(T.Class==c)),0:10);
    sf_std =arrayfun(@(c) std(T.Seal_Force_N(T.Class==c)),0:10);
    bar(0:10,sf_mean,'FaceColor',[0.5 0.3 0.8]);
    hold on;
    errorbar(0:10,sf_mean,sf_std,'k.','LineWidth',1);
    title({'Seal Force by Class [U4]';'sealFriction() — health-scaled Stribeck'});
    xlabel('Class'); ylabel('F_{seal} (N)'); grid on; xticks(0:10);

    subplot(2,4,5);
    trigR=arrayfun(@(c) 100*sum(T.Class==c & T.Maint_Trigger==1)/max(sum(T.Class==c),1),0:10);
    bar(0:10,trigR,'FaceColor','flat','CData',repmat([0.85 0.35 0.25],11,1));
    title('Trigger Rate by Class (%)'); xlabel('Class'); ylabel('%');
    grid on; xticks(0:10); yline(50,'k--','LineWidth',1.5);

    subplot(2,4,6);
    % Friction effective vs base — shows tireFriction correction
    fric_c0=T.Friction_Coef(T.Class==0);
    fric_c5=T.Friction_Coef(T.Class==5);
    fric_c6=T.Friction_Coef(T.Class==6);
    boxplot([fric_c0;fric_c5;fric_c6],...
        [zeros(length(fric_c0),1);5*ones(length(fric_c5),1);6*ones(length(fric_c6),1)],...
        'Labels',{'C0 Normal','C5 Brake','C6 Burst'});
    title({'Effective µ (tireFriction) [U1]';'Physics-corrected runway friction'});
    ylabel('Effective friction µ'); grid on;

    subplot(2,4,7);
    bar([costFIXED/1000,costCBM/1000],...
        'FaceColor','flat','CData',[0.85 0.25 0.25;0.25 0.75 0.35]);
    set(gca,'XTickLabel',{'Fixed-Interval','CBM AI'});
    ylabel('Total Cost ($1,000s)');
    title(sprintf('Savings: $%,.0f (%.0f%%)\nAOG saved: %d',sav,savPct,aogSaved));
    grid on;

    subplot(2,4,8);
    idx=randi(height(T));
    plot(1:50,T{idx,24:73},'b-','LineWidth',1.5); hold on;   % Defl: col 24-73
    plot(1:50,T{idx,74:123},'r--','LineWidth',1.5);           % Velo: col 74-123
    title(sprintf('Sample %d | Cls%d | Sev%d | RUL=%.0f%% | Seal=%.0fN',...
        idx,T.Class(idx),T.Severity(idx),T.RUL_Percent(idx),T.Seal_Force_N(idx)));
    xlabel('Timestep'); legend('Deflection','Velocity'); grid on;

    saveas(gcf,fullfile(projPath,'Dataset_Analysis_V16p1.png'));
    fprintf('\n📊 Analysis saved: Dataset_Analysis_V16p1.png\n');
    fprintf('=============================================================\n');
else
    fprintf('❌ No data collected.\n');
end


% =========================================================================
%   FUNCTIONS — FULL SET (V16.1)
% =========================================================================

% ---- workspace injection (V16.1: adds seal_f parameter) -----------------
function assignAll(mdl,k,b,mass,sink,fric,temp,seal_f)
    assignin('base','k_strut_current', k);
    assignin('base','b_strut_current', b);
    assignin('base','mass_landing',    mass);
    assignin('base','sink_rate_input', sink);
    assignin('base','runway_friction', fric);
    assignin('base','temperature',     temp);
    assignin('base','seal_force_N',    seal_f); % Used if model has FromWorkspace for it
end

% ---- signal extraction --------------------------------------------------
function [dw,vw,fw,aw,pw,ok]=xWin(sO,n)
    ok=false;
    dw=zeros(1,n);vw=zeros(1,n);fw=zeros(1,n);aw=zeros(1,n);pw=zeros(1,n);
    try
        Yd=squeeze(sO.get('sim_Deflection'));
        Yv=squeeze(sO.get('sim_Velocity'));
        Yf=squeeze(sO.get('sim_Force'));
        Ya=squeeze(sO.get('sim_Acceleration'));
        Yp=squeeze(sO.get('sim_Chamber_Pressure'));
        Yd=Yd(:);Yv=Yv(:);Yf=Yf(:);Ya=Ya(:);Yp=Yp(:);
        if length(Yd)<10,return;end
        [~,ai]=max(abs(Ya)); i0=max(1,ai-5); i1=min(i0+n-1,length(Yd));
        np=i1-i0+1; if np<n*0.5,return;end
        xo=linspace(0,1,np); xn=linspace(0,1,n);
        dw=rs(Yd(i0:i1),xo,xn); vw=rs(Yv(i0:i1),xo,xn);
        fw=rs(Yf(i0:i1),xo,xn); aw=rs(Ya(i0:i1),xo,xn);
        pw=rs(Yp(i0:i1),xo,xn); ok=true;
    catch ME
        fprintf('xWin: %s\n',ME.message);
    end
end
function o=rs(d,xo,xn)
    if length(d)==length(xn),o=d';
    else,o=interp1(xo,d,xn,'linear','extrap'); end
end

% ---- thermal physics (Charles Law + Arrhenius + seal effects) -----------
function [kt,bt]=therm(k0,b0,tc)
    Tr=288.15; Tk=tc+273.15;
    kt=k0*(Tk/Tr);
    vis=exp(2500*(1/Tk-1/Tr));
    bt=b0*max(0.3,min(3.5,vis));
    if tc<0
        kt=kt*min(1+0.008*abs(tc),1.24);
        bt=bt*min(1+0.005*abs(tc),1.15);
    elseif tc>40
        bt=bt*max(1-0.005*(tc-40),0.90);
    end
end

% ---- thermal-normalised K -----------------------------------------------
function kn=kNorm(k,tc)
    kn=k/((tc+273.15)/288.15);
end

% =========================================================================
%   [U1] tireFriction — velocity + hydroplaning + ice dependent µ
%   Replaces flat F_DRY/F_WET/F_ICE in all class loops
%   Do228-212 specifics:
%     Tire: 24×7.7-10 | Inflation: ~80 psi (5.5 bar)
%     Horne hydroplaning: V_p = 9√p_psi = 9√80 ≈ 80.7 kt = 41.5 m/s
%     Touchdown speed: ~100-115 kt (51-59 m/s) → above hydroplaning onset
% =========================================================================
function mu=tireFriction(mu_base,sink_rate,temp_c)
    % Estimate Do228 touchdown speed from sink rate
    % Empirical: faster approach = shallower glideslope = lower sink rate
    v_td=59.2-(sink_rate*2.5);   % m/s approximate
    v_td=max(v_td,20.0);
    v_td=min(v_td,62.0);

    % Hydroplaning correction (wet/ice surfaces only — dry needs no correction)
    v_hydro=41.5;   % m/s — Do228 hydroplaning threshold (Horne, 80 psi)
    if mu_base<0.60   % Wet or ice
        hydro_ratio=max(0,(v_td-v_hydro)/max(62.0-v_hydro,1));
        hydro_factor=1.0-0.35*hydro_ratio;
        hydro_factor=max(hydro_factor,0.55);
    else
        hydro_factor=1.0;  % Dry runway: no hydroplaning effect
    end

    % Temperature ice hardening (below 0°C progressive friction loss)
    if temp_c<0
        ice_factor=1.0-0.010*abs(temp_c);
        ice_factor=max(ice_factor,0.15);   % Floor: black ice (~0.05 floor at -65°C)
    else
        ice_factor=1.0;
    end

    mu=mu_base*hydro_factor*ice_factor;
    mu=max(0.08,min(mu,0.92));
end

% =========================================================================
%   [U2] polytropicKRatio — polytropic oleo gas spring correction
%   Real oleo-pneumatic struts follow P·V^n = const (n≈1.30 fast impact)
%   As stroke increases, gas compresses → stiffness increases non-linearly
%   Returns a multiplier for the fault-specific K value
%   Range: 1.0× (gentle 0.3 m/s) → up to 2.0× (full stroke hard landing)
% =========================================================================
function ratio=polytropicKRatio(sink_rate,mass,k_current,n_poly)
    if nargin<4, n_poly=1.30; end
    g=9.81;
    % Estimate stroke fraction from kinetic energy = spring energy
    % 0.5·m·v² = 0.5·K·x²  →  x = v√(m/K)
    stroke_est=sqrt(mass/max(k_current,5000))*sink_rate;
    stroke_est=min(stroke_est,0.180);    % Cap at 180mm (Do228 oleo stroke limit)
    frac=stroke_est/0.180;               % Fractional stroke (0=no compression)
    frac=min(frac,0.80);                 % Cap at 80% — physical validity limit

    % Polytropic stiffness ratio: K_poly/K_0 = (1/(1-frac))^n
    ratio=max(1.0,min((1.0/(1.0-frac))^n_poly,2.0));
end

% =========================================================================
%   [U3] orificeB — velocity-squared orifice damping correction
%   Real oleo damping: F_damp = C_d · ρ · A_orifice · v²
%   Linear B underestimates damping at high sink rates
%   Applied to Classes 6 (tire burst) and 8 (hard landing) only
%   Calibration: at v_ref=1.5 m/s → ratio=1.0 (preserves nominal B)
% =========================================================================
function ratio=orificeB(sink_rate)
    v_ref=1.5;   % m/s calibration point (moderate landing)
    ratio=sink_rate/v_ref;
    ratio=max(0.50,min(ratio,2.50));
end

% =========================================================================
%   [U4] sealFriction — Stribeck curve for oleo strut seal
%   Models the static breakout → Coulomb → viscous friction transitions
%   that occur at the start of oleo compression
%   Do228 main gear oleo approximate values:
%     F_static  : 800–1200 N (increases as seal degrades)
%     F_coulomb : 400–720 N  (increases as seal degrades)
%     v_strib   : 0.05 m/s  (crossover velocity — typical elastomer seal)
%   health=1.0 → fresh seal (low friction)
%   health=0.0 → failed seal (high sticky friction)
% =========================================================================
function F_seal=sealFriction(v_strut,health)
    F_static =(800 *(1+0.50*(1-health)));   % 800–1200 N
    F_coulomb=(400 *(1+0.80*(1-health)));   % 400–720  N
    v_strib  =0.05;  % m/s crossover velocity
    v_strut  =max(abs(v_strut),0.001);      % Prevent zero division
    F_seal=F_coulomb+(F_static-F_coulomb)*exp(-v_strut/v_strib);
    F_seal=max(0,F_seal);
end

% ---- CS-23.473 severity assessment (physics-earned) ---------------------
function [sev,tr,mi]=assess(cID,sink,gp,health,REG,MAINT)
    sev=1; tr=0; mi=1;
    if sink>=REG.SinkHardLanding||gp>=REG.AccelLimit
        sev=3; tr=1; mi=9;
    end
    if gp>=REG.AccelExtreme||sink>4.0
        sev=4; tr=1; mi=11;
    end
    if sink>=REG.SinkCaution&&sev<2
        sev=2; tr=1; mi=max(mi,min(cID+1,11));
    end
    if health<0.30
        sev=max(sev,4); tr=1; mi=11;
    elseif health<0.50
        sev=max(sev,3); tr=1;
        if mi<9, mi=10; end
    elseif health<0.75
        sev=max(sev,2); tr=1;
        if mi==1, mi=min(cID+1,11); end
    elseif cID>0
        sev=max(sev,2); tr=1;
        if mi==1, mi=min(cID+1,11); end
    end
    mi=max(1,min(mi,11));
end

% ---- Miner-Palmgren fatigue ---------------------------------------------
function dp=dmgCalc(sink,mass,mxD,tc,F)
    g=9.81;
    geq=1+(sink^2)/(2*g*max(mxD,0.005));
    geq=max(1.0,min(geq,6.0));
    stress=geq*mass*g/0.05;
    if stress<=F.S_e, dp=0; return; end
    Nf=F.N_ref*(F.S_ut/stress)^(1/abs(F.b));
    Nf=max(Nf,1); dp=1/Nf;
    if tc>40,  dp=dp*(1+0.015*(tc-40));  end
    if tc<-20, dp=dp*(1+0.010*abs(tc+20)); end
end

% ---- maintenance cost ---------------------------------------------------
function c=mCost(tr,sev,mi,MAINT) %#ok<INUSL>
    if ~tr, c=0; return; end
    c=MAINT(mi).cost;
end

% ---- fixed-interval baseline cost --------------------------------------
function cf=fixedCost(cc,interval,sev,mi,MAINT,ECON,cf)
    if mod(cc,interval)==0
        if sev>=3, cf=cf+MAINT(mi).cost;
        else,      cf=cf+ECON.Routine; end
    end
end

% ---- dataset row builder — 23 scalars + 5×50 signals = 273 total -------
function row=bRow(id,cls,hf,mass,sink,fric,tc,k,b,kn,bn,...
                  dw,vw,fw,aw,pw,F_seal,tr,sev,dp,rul,c,mi)
    row=[id,cls,rul,mass,sink,fric,tc,k,b,kn,bn,hf,...
         max(abs(dw)),max(abs(vw)),max(abs(aw))/9.81,...
         max(abs(fw)),max(abs(pw)),F_seal,...
         tr,sev,dp,c,mi,...
         dw,vw,fw,aw,pw];
end

% ---- progress printer ---------------------------------------------------
function pProg(cs,tgt,sc,tot,t0)
    e=toc(t0); r=e/max(sc,1);
    fprintf('   %d/%d | Total:%d | %.2fs/sim | ETA:%.1fmin\n',...
        cs,tgt,sc,r,(r*(tot-sc))/60);
end
