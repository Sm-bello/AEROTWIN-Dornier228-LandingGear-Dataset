%% AeroTwin Digital Twin v2.0 — launch_twin.m
%  Reads live telemetry from PostgreSQL, drives Simulink gear model,
%  and on EVERY touchdown writes real Prismatic Joint physics back to DB
%  so Python AI can predict from MATLAB data — not Python-generated data.
%
%  Requires: Database Toolbox, Simscape Multibody (R2023a+)

clc; clear; close all;
fprintf('╔══════════════════════════════════════════════════════════╗\n');
fprintf('║  AeroTwin Digital Twin v2.0  |  Dornier 228-212          ║\n');
fprintf('║  PostgreSQL ↔ Simulink  |  Prismatic Joint → AI         ║\n');
fprintf('╚══════════════════════════════════════════════════════════╝\n\n');

MDL       = 'Fancy_Landing_Gear';
POLL_HZ   = 10;
LOG_EVERY = 50;

% ── Step 1: Connect to PostgreSQL ────────────────────────────────────
fprintf('[1/3] Connecting to PostgreSQL...\n');
conn = database('project_db', 'postgres', 'A#1Salamatu', ...
    'org.postgresql.Driver', ...
    'jdbc:postgresql://localhost:5432/project_db');
if ~isopen(conn)
    error('Cannot connect to PostgreSQL. Is the service running?');
end
fprintf('      Connected ✓\n\n');

% ── Step 2: Initialise workspace variables ────────────────────────────
fprintf('[2/3] Loading Simulink model: %s\n', MDL);
assignin('base', 'k_strut_current',   150000);
assignin('base', 'b_strut_current',   4000);
assignin('base', 'mass_kg',           5000);
assignin('base', 'sink_rate_mps',     0);
assignin('base', 'hard_landing_flag', 0);
assignin('base', 'gear_down_cmd',     1);      % default DOWN
assignin('base', 'step_time_var',     0);      
assignin('base', 'seal_force_N',      500);    

if ~bdIsLoaded(MDL), load_system(MDL); end
open_system(MDL);

set_param(MDL, 'SimulationMode',    'normal');
set_param(MDL, 'StopTime',          'inf');
set_param(MDL, 'SimulationCommand', 'start');
fprintf('      Simulink running ✓\n\n');

% ── Step 3: Query strings ─────────────────────────────────────────────
query = ['SELECT sink_rate_fps, compression_ft, wow_status, ' ...
         'groundspeed_kt, alt_agl_ft, mass_lbs, '            ...
         'engine_torque, prop_rpm, flight_phase '             ...
         'FROM telemetry ORDER BY timestamp DESC LIMIT 1'];

gear_query = 'SELECT command FROM gear_commands ORDER BY ts DESC LIMIT 1';
fprintf('[3/3] Polling at %dHz | Touchdown sim will write to matlab_physics_output\n\n', POLL_HZ);

% ── State tracking ────────────────────────────────────────────────────
gear_down       = 1;
last_gear       = 1;
last_wow        = 0;        
wow_debounce    = 0;        
cycle           = 0;

% ═════════════════════════════════════════════════════════════════════
%  MAIN POLLING LOOP
% ═════════════════════════════════════════════════════════════════════
while true
    try
        %% ── 1. Read latest telemetry ─────────────────────────────────
        result = fetch(conn, query);
        if isempty(result)
            fprintf('  Waiting for telemetry...\n'); pause(2); continue;
        end
        
        sink_fps    = result.sink_rate_fps(1);
        compression = result.compression_ft(1);
        wow         = double(result.wow_status(1));
        gspeed_kt   = result.groundspeed_kt(1);
        alt_agl     = result.alt_agl_ft(1);
        mass_lbs    = result.mass_lbs(1);
        phase       = result.flight_phase{1};

        %% ── 2. Read gear command from Python ─────────────────────────
        try
            r = fetch(conn, gear_query);
            if ~isempty(r)
                if iscell(r.command)
                    cmd_received = r.command{1};
                else
                    cmd_received = r.command;
                end
                gear_down = strcmp(strtrim(cmd_received), 'DOWN');
            end
        catch
            % Maintain state on DB lock
        end
        
        assignin('base', 'gear_down_cmd', double(gear_down));
        
        if gear_down ~= last_gear
            fprintf('  [GEAR EVENT] → %s\n', cmd_received);
            last_gear = gear_down;
            % Force running Simulink model to re-read the workspace variable
            try
                set_param(MDL, 'SimulationCommand', 'update');
            catch
            end
        end

        %% ── 3. Update K / B / Mass / Sink from live telemetry ────────
        if compression > 0.05
            k_new = 150000 + (compression * 50000);
            b_new = 4000   + (compression * 1000);
        else
            k_new = 150000; b_new = 4000;
        end
        mass_kg_live = mass_lbs * 0.453592;
        sink_ms_live = sink_fps * 0.3048;
        
        assignin('base', 'k_strut_current', k_new);
        assignin('base', 'b_strut_current', b_new);
        assignin('base', 'mass_kg',         mass_kg_live);
        assignin('base', 'sink_rate_mps',   sink_ms_live);

        %% ── 4. Hard landing flag ─────────────────────────────────────
        hard = double(wow > 0.5 && abs(sink_fps) > 10);
        assignin('base', 'hard_landing_flag', hard);

        %% ── 5. TOUCHDOWN DETECTION → Trigger Prismatic Joint sim ─────
        wow_rising = (wow > 0.5) && (last_wow < 0.5);
        if wow_rising && (wow_debounce == 0) && (abs(sink_fps) > 1.0)
            fprintf('\n  ╔══════════════════════════════════════════╗\n');
            fprintf('  ║  TOUCHDOWN DETECTED  |  Triggering sim   ║\n');
            fprintf('  ║  Sink: %.2f m/s  |  Mass: %.0f kg       ║\n', ...
                    sink_ms_live, mass_kg_live);
            fprintf('  ╚══════════════════════════════════════════╝\n\n');
            
            run_touchdown_sim(MDL, conn, ...
                k_new, b_new, mass_kg_live, sink_ms_live, ...
                compression, true, phase);
            
            wow_debounce = POLL_HZ * 5;  
        end

        %% ── 6. Debounce counter ──────────────────────────────────────
        if wow_debounce > 0
            wow_debounce = wow_debounce - 1;
        end
        last_wow = wow;

        %% ── 7. Periodic heartbeat ────────────────────────────────────
        cycle = cycle + 1;
        if mod(cycle, LOG_EVERY) == 0
            g_str = 'DOWN';
            if ~gear_down, g_str = 'UP  '; end
            fprintf('[%s] Alt:%6.0fft | GS:%5.1fkt | WOW:%d | k:%.0f | b:%.0f | GEAR:%s\n', ...
                    phase, alt_agl, gspeed_kt, wow, k_new, b_new, g_str);
        end

    catch db_err
        fprintf('DB Error: %s  Retrying...\n', db_err.message);
        pause(3);
        try
            conn = database('project_db', 'postgres', 'A#1Salamatu', ...
                'org.postgresql.Driver', ...
                'jdbc:postgresql://localhost:5432/project_db');
        catch, end
    end
    pause(1 / POLL_HZ);
end

close(conn);
set_param(MDL, 'SimulationCommand', 'stop');

% ═════════════════════════════════════════════════════════════════════
%  FUNCTION: run_touchdown_sim
% ═════════════════════════════════════════════════════════════════════
function run_touchdown_sim(MDL, conn, k, b, mass_kg, sink_ms, comp_ft, wow, phase)
    fprintf('  [SIM] Configuring touchdown sim...\n');
    fprintf('  [SIM] K=%.0f N/m | B=%.0f Ns/m | mass=%.0f kg | sink=%.3f m/s\n', k, b, mass_kg, sink_ms);

    % ── STOP the live model completely before changing parameters ──
    try
        set_param(MDL, 'SimulationCommand', 'stop');
        % FIX: Wait until the simulation status actually reports as 'stopped'
        while ~strcmp(get_param(MDL, 'SimulationStatus'), 'stopped')
            pause(0.1);
        end
    catch
    end

    % Safe to push parameters
    assignin('base', 'k_strut_current',   k);
    assignin('base', 'b_strut_current',   b);
    assignin('base', 'mass_kg',           mass_kg);
    assignin('base', 'sink_rate_mps',     sink_ms);
    assignin('base', 'gear_down_cmd',     1);
    assignin('base', 'step_time_var',     0);
    assignin('base', 'seal_force_N',      500);
    assignin('base', 'hard_landing_flag', 0);

    % Safe to change solver settings
    set_param(MDL, 'Solver',   'ode23t');
    set_param(MDL, 'MaxStep',  '0.005');
    
    sim_out = [];
    try
        sim_out = sim(MDL, 'StopTime', '3');
        fprintf('  [SIM] ✓ Impact complete\n');
    catch sim_err
        fprintf('  [SIM] ERROR: %s\n', sim_err.message);
    end

    % ── Extract Prismatic Joint signals & Write to DB ────────────────
    if ~isempty(sim_out)
        [dw, vw, fw, aw, pw, ok] = extract_prismatic_signals(sim_out, 50);
        if ok
            defl_max  = max(abs(dw));               % m
            velo_max  = max(abs(vw));               % m/s
            force_max = max(abs(fw));               % N
            accel_g   = max(abs(aw)) / 9.81;        % g  
            pres_max  = k * defl_max;               % Pa 

            fprintf('  [SIM] Deflection: %.4f m | Velocity: %.3f m/s\n', defl_max, velo_max);
            fprintf('  [SIM] Force: %.0f N | Accel: %.3f g | Pressure: %.0f Pa\n', ...
                    force_max, accel_g, pres_max);

            cs23 = (sink_ms >= 3.05) || (accel_g >= 1.80);
            if cs23
                fprintf('  [SIM] ⚠  CS-23.473(d) TRIGGERED — sink=%.3f m/s, accel=%.3f g\n', ...
                        sink_ms, accel_g);
            end

            write_physics_to_db(conn, ...
                dw, vw, fw, aw, pw, ...
                defl_max, velo_max, force_max, accel_g, pres_max, ...
                k, b, mass_kg, sink_ms, comp_ft, wow, phase);
        else
            fprintf('  [SIM] WARNING: Could not extract Prismatic Joint signals\n');
            fprintf('  [SIM] Check To Workspace block names in model\n');
        end
    end

    % ── Restore live model settings and restart ────────────────────────
    set_param(MDL, 'StopTime', 'inf');
    set_param(MDL, 'MaxStep',  'auto');
    set_param(MDL, 'SimulationCommand', 'start');
    fprintf('  [SIM] Live model restarted\n');
end

% ═════════════════════════════════════════════════════════════════════
%  FUNCTION: extract_prismatic_signals
% ═════════════════════════════════════════════════════════════════════
function [dw, vw, fw, aw, pw, ok] = extract_prismatic_signals(sO, n)
    ok = false;
    dw = zeros(1,n); vw = zeros(1,n); fw = zeros(1,n);
    aw = zeros(1,n); pw = zeros(1,n);
    try
        Yd = squeeze(sO.get('sim_Deflection'));
        Yv = squeeze(sO.get('sim_Velocity'));
        Yf = squeeze(sO.get('sim_Force'));
        Ya = squeeze(sO.get('sim_Acceleration'));
        Yp = squeeze(sO.get('sim_Chamber_Pressure'));
        Yd = Yd(:); Yv = Yv(:); Yf = Yf(:);
        Ya = Ya(:); Yp = Yp(:);
        
        if length(Yd) < 10
            fprintf('  [EXTRACT] Signal too short (%d samples)\n', length(Yd));
            return;
        end
        
        [~, ai] = max(abs(Ya));
        i0 = max(1, ai - 5);
        i1 = min(i0 + n - 1, length(Yd));
        np = i1 - i0 + 1;
        
        if np < n * 0.5
            fprintf('  [EXTRACT] Window too short (%d samples)\n', np);
            return;
        end
        
        xo = linspace(0, 1, np);
        xn = linspace(0, 1, n);
        dw = interp1(xo, Yd(i0:i1), xn, 'linear', 'extrap');
        vw = interp1(xo, Yv(i0:i1), xn, 'linear', 'extrap');
        fw = interp1(xo, Yf(i0:i1), xn, 'linear', 'extrap');
        aw = interp1(xo, Ya(i0:i1), xn, 'linear', 'extrap');
        pw = interp1(xo, Yp(i0:i1), xn, 'linear', 'extrap');
        ok = true;
        fprintf('  [EXTRACT] ✓ All 5 Prismatic Joint signals extracted (%d pts)\n', n);
    catch ME
        fprintf('  [EXTRACT] ERROR: %s\n', ME.message);
    end
end

% ═════════════════════════════════════════════════════════════════════
%  FUNCTION: write_physics_to_db
% ═════════════════════════════════════════════════════════════════════
function write_physics_to_db(conn, ...
    dw, vw, fw, aw, pw, ...
    defl_max, velo_max, force_max, accel_g, pres_max, ...
    k, b, mass_kg, sink_ms, comp_ft, wow, phase)
    
    dw_str = array_to_pg(dw);
    vw_str = array_to_pg(vw);
    fw_str = array_to_pg(fw);
    aw_str = array_to_pg(aw);
    pw_str = array_to_pg(pw);
    
    insert_sql = sprintf(['INSERT INTO matlab_physics_output '                  ...
        '(sim_deflection_max, sim_deflection_raw, '                             ...
        ' sim_velocity_max,   sim_velocity_raw,   '                             ...
        ' sim_force_max,      sim_force_raw,       '                            ...
        ' sim_accel_max_g,    sim_accel_raw,       '                            ...
        ' sim_pressure_max,   sim_pressure_raw,    '                            ...
        ' k_eff, b_eff, mass_kg, sink_rate_ms, '                                ...
        ' compression_ft, wow_status, flight_phase, ready_for_ai) '             ...
        'VALUES (%.6f, %s, %.6f, %s, %.6f, %s, '                                ...
        '        %.6f, %s, %.6f, %s, '                                          ...
        '        %.2f, %.2f, %.2f, %.6f, '                                      ...
        '        %.6f, %s, ''%s'', TRUE)'], ...
        defl_max, dw_str, ...
        velo_max, vw_str, ...
        force_max, fw_str, ...
        accel_g,  aw_str, ...
        pres_max, pw_str, ...
        k, b, mass_kg, sink_ms, ...
        comp_ft, bool_to_pg(wow), phase);
        
    try
        exec(conn, insert_sql);
        fprintf('  [DB] ✓ Prismatic Joint physics written → ready_for_ai=TRUE\n');
        fprintf('  [DB]   Python AI can now classify this touchdown\n\n');
    catch db_err
        fprintf('  [DB] ERROR writing physics: %s\n', db_err.message);
    end
end

% ═════════════════════════════════════════════════════════════════════
%  HELPER: array_to_pg
% ═════════════════════════════════════════════════════════════════════
function s = array_to_pg(v)
    strs = arrayfun(@(x) sprintf('%.6f', x), v(:)', 'UniformOutput', false);
    s = ['ARRAY[' strjoin(strs, ',') ']::FLOAT[]'];
end

% ═════════════════════════════════════════════════════════════════════
%  HELPER: bool_to_pg
% ═════════════════════════════════════════════════════════════════════
function s = bool_to_pg(v)
    if v, s = 'TRUE'; else, s = 'FALSE'; end
end