-- =========================================================
-- AEROTWIN V16.3 — MATLAB PHYSICS HANDSHAKE TABLE
-- Add this to your existing V16.3 schema
-- Purpose: MATLAB writes real Prismatic Joint readings here
--          after each touchdown sim. Python AI reads from here.
-- =========================================================

-- Drop and recreate handshake table
DROP TABLE IF EXISTS matlab_physics_output CASCADE;

CREATE TABLE matlab_physics_output (
    id                  SERIAL PRIMARY KEY,
    ts                  TIMESTAMPTZ DEFAULT NOW(),

    -- ── Prismatic Joint Direct Readings (from Simscape sensing) ──
    sim_deflection_max  FLOAT,      -- metres  — Prismatic Joint q
    sim_deflection_raw  FLOAT[],    -- 50-point time series (for LSTM)
    sim_velocity_max    FLOAT,      -- m/s     — Prismatic Joint v
    sim_velocity_raw    FLOAT[],    -- 50-point time series
    sim_force_max       FLOAT,      -- N       — Prismatic Joint f
    sim_force_raw       FLOAT[],    -- 50-point time series
    sim_accel_max_g     FLOAT,      -- g       — Prismatic Joint a / 9.81
    sim_accel_raw       FLOAT[],    -- 50-point time series
    sim_pressure_max    FLOAT,      -- Pa      — K * deflection
    sim_pressure_raw    FLOAT[],    -- 50-point time series

    -- ── Parameters that drove the sim (from DB telemetry at touchdown) ──
    k_eff               FLOAT,      -- N/m  — effective stiffness used
    b_eff               FLOAT,      -- N·s/m — effective damping used
    mass_kg             FLOAT,      -- kg
    sink_rate_ms        FLOAT,      -- m/s  — from telemetry sink_rate_fps * 0.3048
    compression_ft      FLOAT,      -- ft   — from telemetry
    wow_status          BOOLEAN,    -- should always be TRUE here
    flight_phase        VARCHAR(20),

    -- ── Handshake flag — Python polls this ──────────────────────────
    ready_for_ai        BOOLEAN DEFAULT FALSE,
    ai_consumed         BOOLEAN DEFAULT FALSE   -- Python sets TRUE after reading
);

CREATE INDEX idx_matlab_physics_ts      ON matlab_physics_output (ts DESC);
CREATE INDEX idx_matlab_physics_ready   ON matlab_physics_output (ready_for_ai, ai_consumed);

-- ── Also add compression_ft column to flight_diagnostics if missing ──
ALTER TABLE flight_diagnostics
    ADD COLUMN IF NOT EXISTS compression_ft FLOAT,
    ADD COLUMN IF NOT EXISTS ata_chapter    TEXT;

-- ── Confirmation ────────────────────────────────────────────────────
SELECT '✅ matlab_physics_output table created — handshake ready' AS status;