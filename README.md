<div align="center">
  
# 🛩️ AeroTwin: Physics-AI Digital Twin for Dornier 228 Landing Gear

**Certification-Compliant Predictive Maintenance | Real-Time Health Monitoring | AI-Physics Fusion**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![DOI](https://zenodo.org/badge/DOI/10.34740/KAGGLE/DSV/14910841.svg)](https://doi.org/10.34740/KAGGLE/DSV/14910841)
[![AIAA Journal](https://img.shields.io/badge/AIAA-JAIS-important)](https://arc.aiaa.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-Simscape-orange)](https://www.mathworks.com/products/simscape.html)
[![PostgreSQL](https://img.shields.io/badge/PostgreSQL-16+-blue)](https://www.postgresql.org/)
[![PyVista](https://img.shields.io/badge/3D-PyVista-green)](https://docs.pyvista.org/)

---

**Welcome to the future of aerospace predictive maintenance.**  
AeroTwin is a hybrid physics-AI digital twin that achieves **certification-compliant** landing gear health monitoring through a novel **confidence-weighted fusion architecture** – the first system of its kind to explicitly satisfy **CS-23.473** regulatory requirements while leveraging state-of-the-art deep learning.

</div>

---

## 🔭 What is AeroTwin?

AeroTwin bridges the critical gap between **high-fidelity physics simulation** and **adaptive AI diagnostics**. By integrating a MATLAB Simscape Multibody plant with a dual-input Long Short-Term Memory (LSTM) network via a PostgreSQL backbone, the system enables:

- **Deterministic physics fallback** when AI uncertainty exceeds $\tau = 0.65$
- **Explicit CS-23.473 certification linkage** – mandatory ATA 32-00 inspections triggered at sink rate $\geq$ 3.05 m/s OR acceleration $\geq$ 1.8g
- **Decision provenance logging** for complete audit trails
- **Real-time 3D stress visualization** with PyVista
- **350× cost reduction** compared to commercial HUMS

> 📄 **Associated Publication:**  
> Mohammed Bello Sani, O. Uhiah, "Certification-Compliant Digital Twin for Landing Gear Health Monitoring: A Physics-AI Fusion Approach on the Dornier 228," *AIAA Journal of Aerospace Information Systems*, 2026.

---

## 🏗️ System Architecture

![AeroTwin Architecture](docs/architecture.png)

The system operates through four interconnected nodes:
1. **Python Autonomous FMS & AI Core** – orchestrates missions, hosts AI-physics fusion
2. **MATLAB Simscape Plant** – provides high-fidelity Prismatic Joint physics
3. **PostgreSQL Digital Flight Recorder** – persistent telemetry and diagnostics
4. **Visualization Suite** – PyVista 3D + Tkinter Fleet Commander dashboard

---

## ✨ Key Features

| Component | Capability |
|-----------|------------|
| **AI-Physics Fusion** | Confidence-weighted decision layer with 65% threshold |
| **Fault Classification** | 94.2% accuracy across 11 fault-severity classes |
| **RUL Prediction** | MAE 2.68%, $R^2 = 0.96$ |
| **CS-23.473 Compliance** | Automatic hard landing detection & ATA chapter logging |
| **Real-Time Visualization** | 4-panel PyVista structural stress + Tkinter dashboard |
| **Database Backbone** | PostgreSQL with telemetry, commands, diagnostics tables |
| **Open Architecture** | Fully open-source except optional MATLAB license |

---

## 📊 Dataset: 9,500 Physics-Based Landing Cycles

The included dataset ([Kaggle DOI](https://doi.org/10.34740/KAGGLE/DSV/14910841)) features:

- **11 fault classes** aligned with ATA-32 maintenance actions
- **273 columns**: 23 scalar features + 5 time-series signals × 50 timesteps
- **Physics upgrades**:
  - `tireFriction()` – velocity + hydroplaning + ice-dependent µ
  - `polytropicKRatio()` – adiabatic gas spring correction
  - `orificeB()` – velocity-squared damping (Class 6 & 8)
  - `sealFriction()` – Stribeck curve seal dynamics

| Class | Fault Description | Samples |
|-------|-------------------|---------|
| 0 | Normal | 1,500 |
| 1 | Nitrogen Leak | 1,500 |
| 2 | Worn Seal | 1,500 |
| 3 | Early Degradation | 1,000 |
| 4 | Thermal Degradation | 800 |
| 5 | Brake Fade | 600 |
| 6 | Tire Burst | 400 |
| 7 | Corrosion | 600 |
| 8 | Hard Landing [CS-23.473] | 600 |
| 9 | Combined Faults | 500 |
| 10 | Impending Failure | 500 |

---

## 🚀 Quick Start

### Prerequisites
- Python 3.8+ with packages in `requirements.txt`
- MATLAB R2023a+ with Simscape Multibody *(optional – Python fallback provided)*
- PostgreSQL 13+

### Installation

```bash
# Clone the repository
git clone https://github.com/Sm-bello/AEROTWIN-Dornier228-LandingGear-Dataset.git
cd AEROTWIN-Dornier228-LandingGear-Dataset

# Install Python dependencies
pip install -r requirements.txt

# Set up PostgreSQL database
psql -U postgres -f sql/Postgresql_Database_Script.sql

# Download trained model (optional)
# Place AeroTwin_V16_Model.h5 in the root directory
