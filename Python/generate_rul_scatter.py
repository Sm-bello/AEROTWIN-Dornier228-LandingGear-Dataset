"""
Generate RUL scatter plot for AIAA paper
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.metrics import r2_score, mean_absolute_error
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import train_test_split
from tensorflow.keras.models import load_model, Model  # ← FIXED: Added load_model
import joblib

# Configuration
DATA_PATH = r'C:\Users\User\Desktop\Do228\AEROTWIN_Do228_V16p1_Dataset.csv'
MODEL_PATH = r'C:\Users\User\Desktop\Do228\AeroTwin_V16_Model.h5'
SCALER_SEQ = r'C:\Users\User\Desktop\Do228\scaler_seq.pkl'
SCALER_SCALAR = r'C:\Users\User\Desktop\Do228\scaler_scalar.pkl'
OUTPUT_PATH = r'C:\Users\User\Desktop\Do228\rul_scatter.png'

# Class names
CLASS_NAMES = ['Normal', 'N2 Leak', 'Worn Seal', 'Early Degrad', 'Thermal Degrad',
               'Brake Fade', 'Tire Burst', 'Corrosion', 'Hard Landing',
               'Combined Fault', 'Impending Failure']

print("📊 Loading dataset...")
df = pd.read_csv(DATA_PATH)

# Extract sequences and scalars
seq_cols = df.columns[23:273]
X_seq = df[seq_cols].values.reshape(-1, 5, 50).transpose(0, 2, 1)

scalar_cols = ['Mass_kg', 'SinkRate_ms', 'Temp_C', 'Friction_Coef']
X_scalar = df[scalar_cols].values

# Get RUL from dataset (column index 2 is RUL_Percent)
y_rul_true = df['RUL_Percent'].values

print("📦 Loading scalers and model...")
scaler_seq = joblib.load(SCALER_SEQ)
scaler_scalar = joblib.load(SCALER_SCALAR)
model = load_model(MODEL_PATH, compile=False)  # ← Now this works

# Scale the data
print("🔄 Scaling data...")
X_seq_scaled = X_seq.reshape(-1, 5)
X_seq_scaled = scaler_seq.transform(X_seq_scaled).reshape(-1, 50, 5)
X_scalar_scaled = scaler_scalar.transform(X_scalar)

# Create a feature extractor model (get features from second-to-last layer)
print("🤖 Creating feature extractor...")
feature_extractor = Model(inputs=model.inputs, 
                         outputs=model.layers[-2].output)  # Second last layer

print("🤖 Extracting features...")
features = feature_extractor.predict([X_seq_scaled, X_scalar_scaled], verbose=1)

print("📊 Training RUL regressor...")
# Train a simple regressor on features to predict RUL
X_train, X_test, y_train, y_test = train_test_split(
    features, y_rul_true, test_size=0.2, random_state=42
)

regressor = RandomForestRegressor(n_estimators=100, random_state=42, n_jobs=-1)
regressor.fit(X_train, y_train)
y_rul_pred = regressor.predict(X_test)

# Calculate metrics
mae = mean_absolute_error(y_test, y_rul_pred)
r2 = r2_score(y_test, y_rul_pred)

print(f"\n📊 RUL Prediction Performance:")
print(f"   MAE: {mae:.2f}%")
print(f"   R²:  {r2:.4f}")
print(f"   Samples: {len(y_test)}")

# Create scatter plot
plt.figure(figsize=(12, 10))
plt.scatter(y_test, y_rul_pred, alpha=0.5, c='steelblue', s=40, 
            edgecolors='white', linewidth=0.5)

# Add perfect prediction line
min_val = 0
max_val = 100
plt.plot([min_val, max_val], [min_val, max_val], 'r-', linewidth=2, 
         label='Perfect Prediction', alpha=0.7)

# Add ±5% error bands
x_line = np.linspace(0, 100, 100)
plt.fill_between(x_line, x_line-5, x_line+5, alpha=0.1, color='gray', 
                 label='±5% Error Band')

plt.title(f'Remaining Useful Life Prediction Performance\nMAE = {mae:.2f}%, $R^2$ = {r2:.4f}', 
          fontsize=18, fontweight='bold', pad=20)
plt.xlabel('Actual RUL (%)', fontsize=14)
plt.ylabel('Predicted RUL (%)', fontsize=14)
plt.grid(True, alpha=0.2, linestyle='--')
plt.xlim(0, 100)
plt.ylim(0, 100)
plt.legend(loc='lower right', fontsize=12)
plt.tight_layout()
plt.savefig(OUTPUT_PATH, dpi=300, bbox_inches='tight')
print(f"✅ RUL scatter plot saved to: {OUTPUT_PATH}")
plt.show()