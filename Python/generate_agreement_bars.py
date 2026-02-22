"""
Generate agreement statistics bar chart from your mission logs
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Configuration - point this to your mission log CSV if you have it
# Or use the data from your terminal output
LOG_PATH = r'C:\Users\User\Desktop\Do228\AEROTWIN_MaintenanceLog_V16p1.csv'
OUTPUT_PATH = r'C:\Users\User\Desktop\Do228\agreement_bars.png'

# From your terminal output - mission results
mission_data = [
    {'class': 'Brake Fade', 'ai_class': 'Brake Fade', 'physics_class': 'Normal', 'agree': False},
    {'class': 'Tire Burst', 'ai_class': 'Tire Burst', 'physics_class': 'Tire Burst', 'agree': True},
    {'class': 'Worn Seal', 'ai_class': 'Worn Seal', 'physics_class': 'Worn Seal', 'agree': True},
    {'class': 'Worn Seal', 'ai_class': 'Normal', 'physics_class': 'Worn Seal', 'agree': False},
    {'class': 'N2 Leak', 'ai_class': 'Normal', 'physics_class': 'N2 Leak', 'agree': False},
]

# Calculate per-class agreement
class_names = ['Normal', 'N2 Leak', 'Worn Seal', 'Brake Fade', 'Tire Burst']
class_counts = {'Normal': 0, 'N2 Leak': 0, 'Worn Seal': 0, 'Brake Fade': 0, 'Tire Burst': 0}
class_agreements = {'Normal': 0, 'N2 Leak': 0, 'Worn Seal': 0, 'Brake Fade': 0, 'Tire Burst': 0}

for mission in mission_data:
    physics_class = mission['physics_class']
    class_counts[physics_class] = class_counts.get(physics_class, 0) + 1
    if mission['agree']:
        class_agreements[physics_class] = class_agreements.get(physics_class, 0) + 1

# Calculate agreement percentages
classes = []
agreement_pcts = []
counts = []

for class_name in class_names:
    if class_counts.get(class_name, 0) > 0:
        classes.append(class_name)
        pct = 100 * class_agreements.get(class_name, 0) / class_counts[class_name]
        agreement_pcts.append(pct)
        counts.append(class_counts[class_name])

# Create bar chart
fig, ax = plt.subplots(figsize=(10, 6))

bars = ax.bar(classes, agreement_pcts, color=['#2ecc71' if p == 100 else '#e74c3c' for p in agreement_pcts])

# Add value labels on bars
for bar, pct, count in zip(bars, agreement_pcts, counts):
    height = bar.get_height()
    ax.text(bar.get_x() + bar.get_width()/2., height + 1,
            f'{pct:.0f}%\n({count} mission{"s" if count>1 else ""})',
            ha='center', va='bottom', fontsize=11)

ax.set_ylim(0, 110)
ax.set_ylabel('Agreement (%)', fontsize=14)
ax.set_xlabel('Fault Class', fontsize=14)
ax.set_title('AI vs. Physics Agreement by Class\n(5 Simulated Missions)', 
             fontsize=16, fontweight='bold')
ax.grid(axis='y', alpha=0.3)

# Add a horizontal line at 65% threshold
ax.axhline(y=65, color='blue', linestyle='--', linewidth=2, alpha=0.7, label='65% Confidence Threshold')
ax.legend()

plt.tight_layout()
plt.savefig(OUTPUT_PATH, dpi=300, bbox_inches='tight')
print(f"✅ Agreement chart saved to: {OUTPUT_PATH}")
plt.show()