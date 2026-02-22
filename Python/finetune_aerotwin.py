"""
╔════════════════════════════════════════════════════════════════════════════╗
║ AEROTWIN V16 → V17 ROBUST FINE-TUNING PIPELINE                            ║
║ Dornier 228-212 | Physics-Based Fine-Tuning | Cross-Validation            ║
║ Saves: V17_finetuned.h5 + 5-fold models + training plots + JSON report   ║
╚════════════════════════════════════════════════════════════════════════════╝

FIXED: Macro F1 dimension error - now handles (batch_size, 11) vs (batch_size,) correctly
"""

import os
import sys
import math
import time
import random
import json
import numpy as np
import tensorflow as tf
import joblib
import matplotlib.pyplot as plt
from datetime import datetime
from sklearn.model_selection import StratifiedKFold
from sklearn.metrics import classification_report, confusion_matrix, f1_score
import seaborn as sns

# ──────────────────────────────────────────────────────────────────────────────
# TERMINAL COLORS
# ──────────────────────────────────────────────────────────────────────────────
if os.name == 'nt':
    os.system('color')
RS = '\033[0m'; BD = '\033[1m'; DM = '\033[2m'
RD = '\033[91m'; GN = '\033[92m'; YL = '\033[93m'
BL = '\033[94m'; MG = '\033[95m'; CY = '\033[96m'; WH = '\033[97m'

def banner(title, subtitle=None, color=CY):
    width = 78
    edge = '═' * (width - 2)
    print(f'{color}{BD}╔{edge}╗{RS}')
    print(f'{color}{BD}║ {WH}{title:<{width-4}} {color}║{RS}')
    if subtitle:
        print(f'{color}{BD}║ {YL}{subtitle:<{width-4}} {color}║{RS}')
    print(f'{color}{BD}╚{edge}╝{RS}')

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION
# ──────────────────────────────────────────────────────────────────────────────
BASE_DIR = r'C:\Users\User\Desktop\Do228'
MODEL_PATH = os.path.join(BASE_DIR, 'AeroTwin_V16_Model.h5')
SCALER_SEQ_PATH = os.path.join(BASE_DIR, 'scaler_seq.pkl')
SCALER_SCALAR_PATH = os.path.join(BASE_DIR, 'scaler_scalar.pkl')
OUTPUT_DIR = os.path.join(BASE_DIR, 'V17_FineTune_Output')

# Create output directory
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Fine-tuning parameters
N_SAMPLES = 10000          # Number of synthetic scenarios
N_FOLDS = 5                 # Cross-validation folds
EPOCHS_PER_FOLD = 15        # Training epochs per fold
BATCH_SIZE = 64             # Batch size
LEARNING_RATE = 1e-5        # Low LR for fine-tuning
VALIDATION_SPLIT = 0.15     # Validation split for final model

# Physics constants (from your MATLAB script)
K_NOM = 150000
MASS_MIN = 4500
MASS_MAX = 5800
CLASS_NAMES = ['Normal', 'N2 Leak', 'Worn Seal', 'Early Degrad', 'Thermal Degrad',
               'Brake Fade', 'Tire Burst', 'Corrosion', 'Hard Landing',
               'Combined Fault', 'Impending Failure']

# Class distributions (match your MATLAB script)
CLASS_WEIGHTS = [0.40, 0.10, 0.10, 0.08, 0.06, 0.06, 0.04, 0.06, 0.05, 0.03, 0.02]
SINK_RANGES = {
    0: (0.10, 0.90), 1: (0.30, 2.00), 2: (0.50, 2.50), 3: (0.30, 2.50),
    4: (0.30, 2.50), 5: (0.80, 2.80), 6: (2.00, 3.50), 7: (0.30, 2.50),
    8: (3.00, 3.05), 9: (0.50, 3.00), 10: (0.50, 3.50)
}
HEALTH_RANGES = {
    0: (0.85, 1.00), 1: (0.00, 1.00), 2: (0.00, 1.00), 3: (0.70, 0.85),
    4: (0.60, 0.90), 5: (0.75, 0.95), 6: (0.40, 0.60), 7: (0.50, 1.00),
    8: (0.40, 0.80), 9: (0.20, 0.80), 10: (0.05, 0.30)
}
K_RANGES = {
    0: (143000, 157000), 1: (75000, 115000), 2: (143000, 157000), 3: (126000, 141000),
    4: (138000, 152000), 5: (143000, 157000), 6: (78000, 105000), 7: (105000, 126000),
    8: (112000, 138000), 9: (65000, 105000), 10: (50000, 80000)
}
B_RANGES = {
    0: (3800, 4200), 1: (3800, 4200), 2: (900, 2000), 3: (3400, 3800), 4: (2600, 3600),
    5: (1400, 5600), 6: (6000, 8500), 7: (4400, 5600), 8: (3000, 4800), 9: (700, 2000), 10: (500, 1500)
}

# ──────────────────────────────────────────────────────────────────────────────
# PHYSICS FUNCTIONS (from your robot aircraft script)
# ──────────────────────────────────────────────────────────────────────────────
def tire_friction(mu_base, sink_ms, temp_c):
    v_td = max(20.0, min(62.0, 59.2 - sink_ms * 2.5))
    hf = max(0.55, 1.0 - 0.35*max(0.0,(v_td-41.5)/20.5)) if mu_base < 0.60 else 1.0
    ic = max(0.15, 1.0 - 0.010*abs(temp_c)) if temp_c < 0 else 1.0
    return max(0.08, min(0.92, mu_base*hf*ic))

def polytropic_k_ratio(sink_ms, mass_kg, k):
    stroke = min(0.180, math.sqrt(mass_kg/max(k,5000))*sink_ms)
    frac = min(0.80, stroke/0.180)
    return max(1.0, min(2.0, (1.0/(1.0-frac))**1.30))

def orifice_b(sink_ms):
    return max(0.50, min(2.50, sink_ms/1.5))

def seal_friction(v_ms, health):
    Fs = 800*(1+0.50*(1-health))
    Fc = 400*(1+0.80*(1-health))
    return max(0.0, Fc+(Fs-Fc)*math.exp(-max(abs(v_ms),0.001)/0.05))

def compute_landing_physics(cls, sink_ms, mass_kg, temp_c):
    k_raw = random.uniform(*K_RANGES[cls])
    k = k_raw * polytropic_k_ratio(sink_ms, mass_kg, k_raw)
    b = random.uniform(*B_RANGES[cls])
    if cls in [6, 8]:
        b *= orifice_b(sink_ms)
    compression_m = min(0.180, math.sqrt(mass_kg/max(k,5000))*sink_ms)
    F_total = k*compression_m + b*sink_ms
    return k, b, compression_m*3.281, F_total/(mass_kg*9.81)

def generate_landing_scenario():
    """Generate a single landing scenario with physics-based parameters"""
    cls = random.choices(range(11), weights=CLASS_WEIGHTS)[0]
    
    sink_ms = random.uniform(*SINK_RANGES[cls])
    health = random.uniform(*HEALTH_RANGES[cls])
    mass_kg = random.uniform(MASS_MIN, MASS_MAX)
    temp_c = 27.0 + random.uniform(-15.0, 15.0)  # Wider temp range for robustness
    
    k, b, compression_ft, max_accel_g = compute_landing_physics(cls, sink_ms, mass_kg, temp_c)
    
    return {
        'class_true': cls,
        'sink_ms': sink_ms,
        'health': health,
        'mass_kg': mass_kg,
        'temp_c': temp_c,
        'fric': tire_friction(0.80, sink_ms, temp_c),
        'F_seal': seal_friction(sink_ms, health),
        'k': k,
        'b': b,
        'compression_ft': compression_ft,
        'max_accel_g': max_accel_g
    }

def generate_time_series(sc):
    """Generate 50-step time series from physics parameters"""
    k = sc['k']
    b = sc['b']
    m = sc['mass_kg']
    v0 = sc['sink_ms']
    
    # Damped oscillator physics
    om = math.sqrt(max(k/m, 1e-3))
    zd = min(b / (2.0*math.sqrt(k*m)), 0.995)
    od = om * math.sqrt(max(1-zd**2, 1e-4))
    
    t_arr = np.linspace(0, 0.70, 50)
    comp0 = sc['compression_ft'] * 0.3048  # Convert to meters
    
    # Generate 5 signals
    defl = comp0 * np.exp(-zd*om*t_arr) * np.abs(np.cos(od*t_arr))
    force = k*defl + b*v0*np.exp(-om*t_arr*0.8)
    vel = -zd*om*defl + od*comp0*np.exp(-zd*om*t_arr)*np.sin(od*t_arr)
    accel = force / (m * 9.81)
    pres = force / (np.pi*0.050**2) / 1e6
    
    return np.stack([
        defl.astype(np.float32),
        vel.astype(np.float32),
        force.astype(np.float32),
        accel.astype(np.float32),
        pres.astype(np.float32)
    ], axis=1)  # Shape: (50, 5)

# ──────────────────────────────────────────────────────────────────────────────
# FIXED MACRO F1 METRIC - Now handles dimensions correctly
# ──────────────────────────────────────────────────────────────────────────────
def macro_f1(y_true, y_pred):
    """
    FIXED: Macro F1 score for imbalanced classes
    y_true: shape (batch_size,) - integer labels
    y_pred: shape (batch_size, 11) - softmax probabilities
    """
    # Convert predictions to class indices
    y_pred_classes = tf.argmax(y_pred, axis=1)
    y_true = tf.cast(y_true, tf.int32)
    
    # Create confusion matrix components for each class
    def f1_class(class_id):
        # True positives: y_true == class_id AND y_pred == class_id
        true_pos = tf.reduce_sum(
            tf.cast(
                tf.logical_and(
                    tf.equal(y_true, class_id),
                    tf.equal(y_pred_classes, class_id)
                ),
                tf.float32
            )
        )
        
        # False positives: y_pred == class_id but y_true != class_id
        false_pos = tf.reduce_sum(
            tf.cast(
                tf.logical_and(
                    tf.equal(y_pred_classes, class_id),
                    tf.not_equal(y_true, class_id)
                ),
                tf.float32
            )
        )
        
        # False negatives: y_true == class_id but y_pred != class_id
        false_neg = tf.reduce_sum(
            tf.cast(
                tf.logical_and(
                    tf.equal(y_true, class_id),
                    tf.not_equal(y_pred_classes, class_id)
                ),
                tf.float32
            )
        )
        
        precision = true_pos / (true_pos + false_pos + tf.keras.backend.epsilon())
        recall = true_pos / (true_pos + false_neg + tf.keras.backend.epsilon())
        f1 = 2 * precision * recall / (precision + recall + tf.keras.backend.epsilon())
        return f1
    
    # Calculate F1 for each class
    f1_scores = [f1_class(i) for i in range(11)]
    
    # Return macro average (mean across all classes)
    return tf.reduce_mean(f1_scores)

# ──────────────────────────────────────────────────────────────────────────────
# LOAD MODEL AND SCALERS
# ──────────────────────────────────────────────────────────────────────────────
def load_model_and_scalers():
    """Load V16 model and scalers"""
    banner("LOADING V16 MODEL & SCALERS", "Original model will remain untouched")
    
    if not os.path.exists(MODEL_PATH):
        print(f"{RD}❌ Model not found: {MODEL_PATH}{RS}")
        return None, None, None
    
    try:
        model = tf.keras.models.load_model(MODEL_PATH, compile=False)
        print(f"{GN}✓ Model loaded: {os.path.basename(MODEL_PATH)}{RS}")
    except Exception as e:
        print(f"{RD}❌ Failed to load model: {e}{RS}")
        return None, None, None
    
    try:
        scaler_seq = joblib.load(SCALER_SEQ_PATH)
        scaler_scalar = joblib.load(SCALER_SCALAR_PATH)
        print(f"{GN}✓ Scalers loaded{RS}")
    except Exception as e:
        print(f"{RD}❌ Failed to load scalers: {e}{RS}")
        return None, None, None
    
    return model, scaler_seq, scaler_scalar

# ──────────────────────────────────────────────────────────────────────────────
# GENERATE SYNTHETIC DATASET
# ──────────────────────────────────────────────────────────────────────────────
def generate_dataset(n_samples, scaler_seq, scaler_scalar):
    """Generate synthetic dataset with physics-based signals"""
    banner(f"GENERATING {n_samples} SYNTHETIC SCENARIOS", 
           "Using physics models from your MATLAB script")
    
    X_seq_list = []
    X_ctx_list = []
    y_list = []
    metadata = []
    
    start_time = time.time()
    
    for i in range(n_samples):
        # Generate scenario
        sc = generate_landing_scenario()
        
        # Generate time series
        raw_seq = generate_time_series(sc)
        
        # Scale sequence
        raw_seq_s = scaler_seq.transform(raw_seq)
        
        # Context vector: [mass, sink_rate, temp, friction]
        ctx = np.array([[sc['mass_kg'], sc['sink_ms'], sc['temp_c'], sc['fric']]])
        ctx_s = scaler_scalar.transform(ctx)
        
        # Store
        X_seq_list.append(raw_seq_s)
        X_ctx_list.append(ctx_s[0])
        y_list.append(sc['class_true'])
        metadata.append(sc)
        
        # Progress
        if (i + 1) % 1000 == 0:
            elapsed = time.time() - start_time
            rate = (i + 1) / elapsed
            print(f"   {GN}→ {i+1}/{n_samples} scenarios generated ({rate:.1f} samples/sec){RS}")
    
    # Convert to numpy arrays
    X_seq = np.array(X_seq_list, dtype=np.float32)
    X_ctx = np.array(X_ctx_list, dtype=np.float32)
    y = np.array(y_list, dtype=np.int32)
    y_cat = tf.keras.utils.to_categorical(y, num_classes=11)
    
    print(f"\n{GN}✓ Dataset generated:{RS}")
    print(f"   X_seq: {X_seq.shape} (samples, timesteps, features)")
    print(f"   X_ctx: {X_ctx.shape} (samples, context_features)")
    print(f"   y: {y_cat.shape} (samples, classes)")
    
    # Class distribution
    print(f"\n{YL}Class distribution:{RS}")
    for i, name in enumerate(CLASS_NAMES):
        count = np.sum(y == i)
        pct = 100 * count / n_samples
        bar = '█' * int(pct / 2)
        print(f"   {i:2d} {name:<20}: {count:5d} ({pct:5.1f}%) {bar}")
    
    return X_seq, X_ctx, y, y_cat, metadata

# ──────────────────────────────────────────────────────────────────────────────
# CROSS-VALIDATION TRAINING
# ──────────────────────────────────────────────────────────────────────────────
def train_cross_validation(model_base, X_seq, X_ctx, y, y_cat, n_folds=5):
    """Perform k-fold cross-validation"""
    banner(f"{n_folds}-FOLD CROSS-VALIDATION", "Training on folds to measure robustness")
    
    skf = StratifiedKFold(n_splits=n_folds, shuffle=True, random_state=42)
    fold_histories = []
    fold_models = []
    fold_results = []
    
    for fold, (train_idx, val_idx) in enumerate(skf.split(X_seq, y)):
        print(f"\n{YL}{'='*60}{RS}")
        print(f"{YL}📁 FOLD {fold + 1}/{n_folds}{RS}")
        print(f"{YL}{'='*60}{RS}")
        
        # Split data
        X_train_seq, X_val_seq = X_seq[train_idx], X_seq[val_idx]
        X_train_ctx, X_val_ctx = X_ctx[train_idx], X_ctx[val_idx]
        y_train, y_val = y_cat[train_idx], y_cat[val_idx]
        y_train_int, y_val_int = y[train_idx], y[val_idx]  # For metrics
        
        print(f"   Train: {len(train_idx)} samples")
        print(f"   Val:   {len(val_idx)} samples")
        
        # Clone model
        model = tf.keras.models.clone_model(model_base)
        model.set_weights(model_base.get_weights())
        
        model.compile(
            optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE),
            loss='categorical_crossentropy',
            metrics=['accuracy', macro_f1]
        )
        
        # Callbacks
        callbacks = [
            tf.keras.callbacks.EarlyStopping(
                monitor='val_macro_f1', mode='max',
                patience=5, restore_best_weights=True, verbose=0
            ),
            tf.keras.callbacks.ReduceLROnPlateau(
                monitor='val_loss', factor=0.5, patience=3, verbose=0
            )
        ]
        
        # Train
        history = model.fit(
            [X_train_seq, X_train_ctx], y_train,
            validation_data=([X_val_seq, X_val_ctx], y_val),
            epochs=EPOCHS_PER_FOLD,
            batch_size=BATCH_SIZE,
            callbacks=callbacks,
            verbose=1
        )
        
        # Evaluate
        val_loss, val_acc, val_f1 = model.evaluate(
            [X_val_seq, X_val_ctx], y_val, verbose=0
        )
        
        fold_results.append({
            'fold': fold + 1,
            'val_loss': float(val_loss),
            'val_accuracy': float(val_acc * 100),
            'val_f1': float(val_f1)
        })
        
        fold_histories.append(history)
        fold_models.append(model)
        
        print(f"\n{GN}✓ Fold {fold + 1} Results:{RS}")
        print(f"   Val Accuracy: {val_acc*100:.2f}%")
        print(f"   Val Macro F1: {val_f1:.4f}")
    
    # Summary
    print(f"\n{CY}{'='*60}{RS}")
    print(f"{CY}📊 CROSS-VALIDATION SUMMARY{RS}")
    print(f"{CY}{'='*60}{RS}")
    
    accuracies = [r['val_accuracy'] for r in fold_results]
    f1_scores = [r['val_f1'] for r in fold_results]
    
    mean_acc = np.mean(accuracies)
    std_acc = np.std(accuracies)
    mean_f1 = np.mean(f1_scores)
    std_f1 = np.std(f1_scores)
    
    print(f"\n{WH}5-Fold Cross-Validation Results:{RS}")
    print(f"   Accuracy: {mean_acc:.2f}% ± {std_acc:.2f}%")
    print(f"   Macro F1: {mean_f1:.4f} ± {std_f1:.4f}")
    print(f"\n   Per-fold results:")
    for r in fold_results:
        print(f"     Fold {r['fold']}: Acc={r['val_accuracy']:.2f}%, F1={r['val_f1']:.4f}")
    
    return fold_models, fold_histories, fold_results, mean_acc, std_acc, mean_f1, std_f1

# ──────────────────────────────────────────────────────────────────────────────
# TRAIN FINAL MODEL
# ──────────────────────────────────────────────────────────────────────────────
def train_final_model(model_base, X_seq, X_ctx, y_cat):
    """Train final ensemble model on all data"""
    banner("TRAINING FINAL ENSEMBLE MODEL", "Using all data with lower learning rate")
    
    final_model = tf.keras.models.clone_model(model_base)
    final_model.set_weights(model_base.get_weights())
    
    final_model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=LEARNING_RATE / 2),
        loss='categorical_crossentropy',
        metrics=['accuracy', macro_f1]
    )
    
    # Split for final validation
    val_split = VALIDATION_SPLIT
    n_val = int(len(X_seq) * val_split)
    indices = np.random.permutation(len(X_seq))
    val_indices = indices[:n_val]
    train_indices = indices[n_val:]
    
    X_train_seq, X_val_seq = X_seq[train_indices], X_seq[val_indices]
    X_train_ctx, X_val_ctx = X_ctx[train_indices], X_ctx[val_indices]
    y_train, y_val = y_cat[train_indices], y_cat[val_indices]
    
    print(f"   Training: {len(train_indices)} samples")
    print(f"   Validation: {len(val_indices)} samples")
    
    # Train
    history = final_model.fit(
        [X_train_seq, X_train_ctx], y_train,
        validation_data=([X_val_seq, X_val_ctx], y_val),
        epochs=EPOCHS_PER_FOLD * 2,  # Train longer on full dataset
        batch_size=BATCH_SIZE,
        verbose=1
    )
    
    # Final evaluation
    val_loss, val_acc, val_f1 = final_model.evaluate(
        [X_val_seq, X_val_ctx], y_val, verbose=0
    )
    
    print(f"\n{GN}✓ Final Model Results:{RS}")
    print(f"   Validation Accuracy: {val_acc*100:.2f}%")
    print(f"   Validation Macro F1: {val_f1:.4f}")
    
    return final_model, history, val_acc, val_f1

# ──────────────────────────────────────────────────────────────────────────────
# PLOT TRAINING HISTORY
# ──────────────────────────────────────────────────────────────────────────────
def plot_training_history(fold_histories, final_history, output_dir):
    """Plot cross-validation and final training history"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('AeroTwin V17 Fine-Tuning History', fontsize=16, fontweight='bold')
    
    # Plot 1: Cross-validation accuracy
    ax = axes[0, 0]
    for i, hist in enumerate(fold_histories):
        ax.plot(hist.history['accuracy'], alpha=0.5, label=f'Fold {i+1} Train')
        ax.plot(hist.history['val_accuracy'], '--', alpha=0.5, label=f'Fold {i+1} Val')
    ax.set_title('Cross-Validation Accuracy')
    ax.set_xlabel('Epoch')
    ax.set_ylabel('Accuracy')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Plot 2: Cross-validation F1 score
    ax = axes[0, 1]
    for i, hist in enumerate(fold_histories):
        ax.plot(hist.history['macro_f1'], alpha=0.5, label=f'Fold {i+1} Train')
        ax.plot(hist.history['val_macro_f1'], '--', alpha=0.5, label=f'Fold {i+1} Val')
    ax.set_title('Cross-Validation Macro F1')
    ax.set_xlabel('Epoch')
    ax.set_ylabel('Macro F1')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # Plot 3: Final model accuracy
    ax = axes[1, 0]
    ax.plot(final_history.history['accuracy'], label='Train')
    ax.plot(final_history.history['val_accuracy'], '--', label='Validation')
    ax.set_title('Final Model Accuracy')
    ax.set_xlabel('Epoch')
    ax.set_ylabel('Accuracy')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Plot 4: Final model F1
    ax = axes[1, 1]
    ax.plot(final_history.history['macro_f1'], label='Train')
    ax.plot(final_history.history['val_macro_f1'], '--', label='Validation')
    ax.set_title('Final Model Macro F1')
    ax.set_xlabel('Epoch')
    ax.set_ylabel('Macro F1')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plot_path = os.path.join(output_dir, 'V17_training_history.png')
    plt.savefig(plot_path, dpi=150, bbox_inches='tight')
    plt.show()
    print(f"{GN}✓ Training plot saved: {plot_path}{RS}")

# ──────────────────────────────────────────────────────────────────────────────
# SAVE ALL MODELS AND REPORTS
# ──────────────────────────────────────────────────────────────────────────────
def save_all_models(final_model, fold_models, results, output_dir):
    """Save all models and generate comprehensive report"""
    
    # Save final model
    final_model_path = os.path.join(output_dir, 'AeroTwin_V17_finetuned.h5')
    final_model.save(final_model_path)
    print(f"{GN}✓ Final model saved: {final_model_path}{RS}")
    
    # Save fold models
    for i, model in enumerate(fold_models):
        fold_path = os.path.join(output_dir, f'AeroTwin_V17_fold{i+1}.h5')
        model.save(fold_path)
    print(f"{GN}✓ {len(fold_models)} fold models saved{RS}")
    
    # Save JSON report
    report = {
        'timestamp': datetime.now().isoformat(),
        'parameters': {
            'n_samples': N_SAMPLES,
            'n_folds': N_FOLDS,
            'epochs_per_fold': EPOCHS_PER_FOLD,
            'batch_size': BATCH_SIZE,
            'learning_rate': LEARNING_RATE,
            'validation_split': VALIDATION_SPLIT
        },
        'cross_validation': results['cv'],
        'final_model': results['final'],
        'class_names': CLASS_NAMES
    }
    
    report_path = os.path.join(output_dir, 'V17_training_results.json')
    with open(report_path, 'w') as f:
        json.dump(report, f, indent=2)
    print(f"{GN}✓ Report saved: {report_path}{RS}")
    
    # Save a summary text file
    summary_path = os.path.join(output_dir, 'V17_SUMMARY.txt')
    with open(summary_path, 'w') as f:
        f.write("="*60 + "\n")
        f.write("AEROTWIN V17 FINE-TUNING SUMMARY\n")
        f.write("="*60 + "\n\n")
        f.write(f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Original V16: {MODEL_PATH}\n\n")
        
        f.write("PARAMETERS:\n")
        f.write(f"  Samples: {N_SAMPLES}\n")
        f.write(f"  Folds: {N_FOLDS}\n")
        f.write(f"  Epochs per fold: {EPOCHS_PER_FOLD}\n")
        f.write(f"  Batch size: {BATCH_SIZE}\n")
        f.write(f"  Learning rate: {LEARNING_RATE}\n\n")
        
        f.write("CROSS-VALIDATION RESULTS:\n")
        f.write(f"  Accuracy: {results['cv']['mean_acc']:.2f}% ± {results['cv']['std_acc']:.2f}%\n")
        f.write(f"  Macro F1: {results['cv']['mean_f1']:.4f} ± {results['cv']['std_f1']:.4f}\n")
        f.write("  Per fold:\n")
        for r in results['cv']['folds']:
            f.write(f"    Fold {r['fold']}: Acc={r['accuracy']:.2f}%, F1={r['f1']:.4f}\n")
        
        f.write("\nFINAL MODEL:\n")
        f.write(f"  Validation Accuracy: {results['final']['accuracy']*100:.2f}%\n")
        f.write(f"  Validation Macro F1: {results['final']['f1']:.4f}\n\n")
        
        f.write("OUTPUT FILES:\n")
        f.write(f"  {final_model_path}\n")
        for i in range(len(fold_models)):
            f.write(f"  {os.path.join(output_dir, f'AeroTwin_V17_fold{i+1}.h5')}\n")
        f.write(f"  {report_path}\n")
        f.write(f"  {summary_path}\n")
    
    print(f"{GN}✓ Summary saved: {summary_path}{RS}")

# ──────────────────────────────────────────────────────────────────────────────
# MAIN PIPELINE
# ──────────────────────────────────────────────────────────────────────────────
def main():
    """Main fine-tuning pipeline"""
    banner("AEROTWIN V16 → V17 ROBUST FINE-TUNING",
           f"Output directory: {OUTPUT_DIR}", CY)
    
    # 1. Load model and scalers
    model_base, scaler_seq, scaler_scalar = load_model_and_scalers()
    if model_base is None:
        return
    
    # 2. Generate dataset
    X_seq, X_ctx, y, y_cat, metadata = generate_dataset(
        N_SAMPLES, scaler_seq, scaler_scalar
    )
    
    # 3. Cross-validation training
    fold_models, fold_histories, fold_results, mean_acc, std_acc, mean_f1, std_f1 = \
        train_cross_validation(model_base, X_seq, X_ctx, y, y_cat, N_FOLDS)
    
    # 4. Train final model
    final_model, final_history, final_acc, final_f1 = train_final_model(
        model_base, X_seq, X_ctx, y_cat
    )
    
    # 5. Compile results
    results = {
        'cv': {
            'mean_acc': float(mean_acc),
            'std_acc': float(std_acc),
            'mean_f1': float(mean_f1),
            'std_f1': float(std_f1),
            'folds': fold_results
        },
        'final': {
            'accuracy': float(final_acc),
            'f1': float(final_f1)
        }
    }
    
    # 6. Plot training history
    plot_training_history(fold_histories, final_history, OUTPUT_DIR)
    
    # 7. Save everything
    save_all_models(final_model, fold_models, results, OUTPUT_DIR)
    
    # 8. Final summary
    print(f"\n{GN}{BD}{'='*60}{RS}")
    print(f"{GN}{BD}✅ FINE-TUNING COMPLETE!{RS}")
    print(f"{GN}{BD}{'='*60}{RS}")
    print(f"\n{WH}📍 Output directory:{RS} {OUTPUT_DIR}")
    print(f"{WH}📍 Final model:{RS} AeroTwin_V17_finetuned.h5")
    print(f"{WH}📍 V16 untouched at:{RS} {MODEL_PATH}")
    print(f"\n{CY}Cross-validation accuracy: {mean_acc:.2f}% ± {std_acc:.2f}%{RS}")
    print(f"{CY}Final model accuracy: {final_acc*100:.2f}%{RS}")
    print(f"\n{YL}To use V17 in your robot aircraft:{RS}")
    print(f"   Change MODEL_PATH to: {os.path.join(OUTPUT_DIR, 'AeroTwin_V17_finetuned.h5')}")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(f"\n{YL}⚠ Fine-tuning interrupted by user{RS}")
    except Exception as e:
        print(f"\n{RD}❌ Error: {e}{RS}")
        import traceback
        traceback.print_exc()
    finally:
        print(f"\n{DM}Press Enter to exit...{RS}")
        input()