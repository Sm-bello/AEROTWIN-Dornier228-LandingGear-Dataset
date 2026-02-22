"""
Generate confusion matrix figure for AIAA paper from your trained model and dataset
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.metrics import confusion_matrix, classification_report
from tensorflow.keras.models import load_model
import joblib

# Configuration
DATA_PATH = r'C:\Users\User\Desktop\Do228\AEROTWIN_Do228_V16p1_Dataset.csv'
MODEL_PATH = r'C:\Users\User\Desktop\Do228\AeroTwin_V16_Model.h5'
SCALER_SEQ = r'C:\Users\User\Desktop\Do228\scaler_seq.pkl'
SCALER_SCALAR = r'C:\Users\User\Desktop\Do228\scaler_scalar.pkl'
OUTPUT_PATH = r'C:\Users\User\Desktop\Do228\confusion_matrix.png'

# Class names (from your script)
CLASS_NAMES = ['Normal', 'N2 Leak', 'Worn Seal', 'Early Degrad', 'Thermal Degrad',
               'Brake Fade', 'Tire Burst', 'Corrosion', 'Hard Landing',
               'Combined Fault', 'Impending Failure']

print("📊 Loading dataset...")
df = pd.read_csv(DATA_PATH)

# Extract sequences and scalars (same as your training code)
seq_cols = df.columns[23:273]  # Defl(24-73), Velo(74-123), Force(124-173), Accel(174-223), P(224-273)
X_seq = df[seq_cols].values.reshape(-1, 5, 50).transpose(0, 2, 1)

scalar_cols = ['Mass_kg', 'SinkRate_ms', 'Temp_C', 'Friction_Coef']
X_scalar = df[scalar_cols].values

y_true = df['Class'].values

print("📦 Loading scalers and model...")
scaler_seq = joblib.load(SCALER_SEQ)
scaler_scalar = joblib.load(SCALER_SCALAR)
model = load_model(MODEL_PATH, compile=False)

# Scale the data
X_seq_scaled = X_seq.reshape(-1, 5)
X_seq_scaled = scaler_seq.transform(X_seq_scaled).reshape(-1, 50, 5)
X_scalar_scaled = scaler_scalar.transform(X_scalar)

# Make predictions
print("🤖 Running predictions...")
predictions = model.predict([X_seq_scaled, X_scalar_scaled], verbose=1)
y_pred = np.argmax(predictions, axis=1)

# Calculate confusion matrix
cm = confusion_matrix(y_true, y_pred)

# Print classification report
print("\n📋 Classification Report:")
print(classification_report(y_true, y_pred, target_names=CLASS_NAMES))

# Plot confusion matrix
plt.figure(figsize=(14, 12))
sns.heatmap(cm, annot=True, fmt='d', cmap='Blues', 
            xticklabels=CLASS_NAMES, yticklabels=CLASS_NAMES,
            annot_kws={'size': 10})
plt.title('Confusion Matrix - 11-Class Fault Classification', fontsize=16, fontweight='bold')
plt.xlabel('Predicted Label', fontsize=14)
plt.ylabel('True Label', fontsize=14)
plt.xticks(rotation=45, ha='right', fontsize=10)
plt.yticks(fontsize=10)
plt.tight_layout()
plt.savefig(OUTPUT_PATH, dpi=300, bbox_inches='tight')
print(f"✅ Confusion matrix saved to: {OUTPUT_PATH}")
plt.show()