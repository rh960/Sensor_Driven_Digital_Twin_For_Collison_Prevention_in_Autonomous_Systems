import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import os
import glob

# ── File paths ────────────────────────────────────────────────
BASE_PATH = r"C:\Sensor Driven Digital Twin For Collision Prevention in Autonomous Systems\Phase2 Hardware Python Scripts\Hardware Tests"

# Auto-find all CSV log files in the folder
csv_files = glob.glob(os.path.join(BASE_PATH, "avoidance_log_*.csv"))
csv_files.sort()

if not csv_files:
    print(f"No avoidance_log_*.csv files found in:\n{BASE_PATH}")
    print("Place your CSV log files there and run again.")
    exit()

print(f"Found {len(csv_files)} log file(s):")
for f in csv_files:
    print(f"  {os.path.basename(f)}")

# ── Load and label files ──────────────────────────────────────
def load(path):
    df = pd.read_csv(path, parse_dates=['timestamp'])
    for col in ['lidar_fl_m','lidar_fc_m','lidar_fr_m',
                'radar_range_m','radar_velocity_mps','radar_ttc_s','retry_count']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df['elapsed'] = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
    return df

datasets = []
for path in csv_files:
    df   = load(path)
    name = os.path.basename(path).replace('avoidance_log_','').replace('.csv','')
    # Label as single or multiple based on avoidance event count
    braking = (df['motor_state'] == 'BRAKING').sum()
    label   = f"Test {name}\n({'Multiple Objects' if braking > 10 else 'Single Object'}, {braking} avoidance events)"
    datasets.append((df, label, name))

# ── Colours ───────────────────────────────────────────────────
STATE_COLOURS = {
    'NORMAL':    '#2ecc71',
    'BRAKING':   '#e67e22',
    'REVERSING': '#e74c3c',
    'TURNING':   '#9b59b6',
    'ESCAPING':  '#3498db',
}
LEVEL_COLOURS = {
    'SAFE':     '#2ecc71',
    'CAUTION':  '#f39c12',
    'IMMINENT': '#e74c3c',
}

out = BASE_PATH  # save plots next to the CSV files

# ═════════════════════════════════════════════════════════════
# PLOT 1 — LiDAR distances over time
# ═════════════════════════════════════════════════════════════
n = len(datasets)
fig1, axes = plt.subplots(n, 1, figsize=(14, 5*n), sharex=False)
if n == 1:
    axes = [axes]
fig1.suptitle('LiDAR Zone Distances Over Time', fontsize=14, fontweight='bold', y=1.0)

for ax, (df, label, name) in zip(axes, datasets):
    ax.plot(df['elapsed'], df['lidar_fc_m'], color='#e74c3c', lw=1.2, label='FC Centre', alpha=0.9)
    ax.plot(df['elapsed'], df['lidar_fl_m'], color='#3498db', lw=1.0, label='FL Left',   alpha=0.7)
    ax.plot(df['elapsed'], df['lidar_fr_m'], color='#2ecc71', lw=1.0, label='FR Right',  alpha=0.7)
    ax.axhline(0.6, color='#e74c3c', lw=0.8, ls='--', alpha=0.6)
    ax.axhline(0.8, color='#f39c12', lw=0.8, ls='--', alpha=0.6)
    ax.text(0.01, 0.62, 'STOP 0.6m', transform=ax.get_yaxis_transform(), fontsize=7, color='#e74c3c')
    ax.text(0.01, 0.82, 'STEER 0.8m', transform=ax.get_yaxis_transform(), fontsize=7, color='#f39c12')

    # shade motor states
    prev_t = df['elapsed'].iloc[0]
    prev_s = df['motor_state'].iloc[0]
    for _, row in df.iterrows():
        if row['motor_state'] != prev_s:
            ax.axvspan(prev_t, row['elapsed'], color=STATE_COLOURS.get(prev_s,'#aaa'), alpha=0.13)
            prev_t = row['elapsed']
            prev_s = row['motor_state']
    ax.axvspan(prev_t, df['elapsed'].iloc[-1], color=STATE_COLOURS.get(prev_s,'#aaa'), alpha=0.13)

    state_patches = [mpatches.Patch(color=c, alpha=0.5, label=s) for s,c in STATE_COLOURS.items()]
    line_handles, line_labels = ax.get_legend_handles_labels()
    ax.legend(handles=line_handles+state_patches,
              labels=line_labels+list(STATE_COLOURS.keys()),
              loc='upper right', fontsize=7, ncol=4)
    ax.set_title(label, fontsize=10)
    ax.set_ylabel('Distance (m)')
    ax.set_ylim(0, 3.2)
    ax.grid(True, alpha=0.3)

axes[-1].set_xlabel('Elapsed Time (s)')
plt.tight_layout()
p1 = os.path.join(out, 'plot1_lidar_distances.png')
fig1.savefig(p1, dpi=150, bbox_inches='tight')
plt.close(fig1)
print(f"Saved: {p1}")

# ═════════════════════════════════════════════════════════════
# PLOT 2 — Motor state distribution
# ═════════════════════════════════════════════════════════════
fig2, axes = plt.subplots(1, n, figsize=(6*n, 5))
if n == 1:
    axes = [axes]
fig2.suptitle('Motor State Distribution', fontsize=14, fontweight='bold')

for ax, (df, label, name) in zip(axes, datasets):
    counts  = df['motor_state'].value_counts()
    colours = [STATE_COLOURS.get(s,'#999') for s in counts.index]
    bars    = ax.bar(counts.index, counts.values, color=colours, edgecolor='white')
    total   = len(df)
    for bar, count in zip(bars, counts.values):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+total*0.01,
                f'{count} ({count/total*100:.1f}%)', ha='center', va='bottom', fontsize=8)
    ax.set_title(label, fontsize=9)
    ax.set_ylabel('Event Count')
    ax.set_xlabel('Motor State')
    ax.tick_params(axis='x', rotation=20)
    ax.grid(axis='y', alpha=0.3)

plt.tight_layout()
p2 = os.path.join(out, 'plot2_motor_states.png')
fig2.savefig(p2, dpi=150, bbox_inches='tight')
plt.close(fig2)
print(f"Saved: {p2}")

# ═════════════════════════════════════════════════════════════
# PLOT 3 — Fusion level pie + timeline
# ═════════════════════════════════════════════════════════════
fig3, axes = plt.subplots(2, n, figsize=(6*n, 9))
if n == 1:
    axes = [[axes[0]], [axes[1]]]
fig3.suptitle('Fusion Level Analysis', fontsize=14, fontweight='bold')

for col, (df, label, name) in enumerate(datasets):
    counts  = df['fusion_level'].value_counts()
    colours = [LEVEL_COLOURS.get(l,'#999') for l in counts.index]
    axes[0][col].pie(counts.values, labels=counts.index, colors=colours,
                     autopct='%1.1f%%', startangle=90, textprops={'fontsize':10})
    axes[0][col].set_title(label, fontsize=9)

    level_map   = {'SAFE':0,'CAUTION':1,'IMMINENT':2}
    df['lnum']  = df['fusion_level'].map(level_map)
    axes[1][col].fill_between(df['elapsed'], df['lnum'], step='post', alpha=0.7, color='#e74c3c')
    axes[1][col].set_yticks([0,1,2])
    axes[1][col].set_yticklabels(['SAFE','CAUTION','IMMINENT'], fontsize=9)
    axes[1][col].set_xlabel('Elapsed Time (s)')
    axes[1][col].set_ylabel('Fusion Level')
    axes[1][col].set_title(f'Fusion Timeline — {label}', fontsize=9)
    axes[1][col].grid(True, alpha=0.3)

plt.tight_layout()
p3 = os.path.join(out, 'plot3_fusion_levels.png')
fig3.savefig(p3, dpi=150, bbox_inches='tight')
plt.close(fig3)
print(f"Saved: {p3}")

# ═════════════════════════════════════════════════════════════
# PLOT 4 — YOLO detections
# Note: objects were unknown static objects — YOLO misclassified
# Radar all NaN because static objects suppressed by MTI filter
# ═════════════════════════════════════════════════════════════
fig4, axes = plt.subplots(1, n, figsize=(7*n, 6))
if n == 1:
    axes = [axes]
fig4.suptitle('YOLO Camera Detections\n'
              'Objects were unknown static test items — YOLO assigned nearest known class\n'
              'Radar: all NaN — static objects suppressed by MTI clutter filter (velocity ≈ 0)',
              fontsize=11, fontweight='bold')

for ax, (df, label, name) in zip(axes, datasets):
    cam = df[df['camera_class'] != 'none']['camera_class'].value_counts()
    if len(cam) == 0:
        ax.text(0.5, 0.5, 'No YOLO detections', transform=ax.transAxes,
                ha='center', va='center', fontsize=12)
    else:
        colours = plt.cm.Set3(np.linspace(0, 1, len(cam)))
        bars    = ax.barh(cam.index, cam.values, color=colours, edgecolor='white')
        ax.bar_label(bars, fmt='%d', fontsize=9, padding=3)
        total = cam.sum()
        for bar, count in zip(bars, cam.values):
            ax.text(bar.get_width()+max(total*0.01, 1.0),
                    bar.get_y()+bar.get_height()/2,
                    f'{count/total*100:.1f}%', va='center', fontsize=8, color='#555',
                    clip_on=False)
    ax.set_xlabel('Detection Count')
    ax.set_title(f'{label}\nTotal detections: {(df["camera_class"]!="none").sum()}', fontsize=9)
    ax.grid(axis='x', alpha=0.3)
    if len(cam) > 0:
        ax.set_xlim(0, cam.max() * 1.25)

plt.tight_layout()
p4 = os.path.join(out, 'plot4_yolo_detections.png')
fig4.savefig(p4, dpi=150, bbox_inches='tight')
plt.close(fig4)
print(f"Saved: {p4}")

# ═════════════════════════════════════════════════════════════
# PLOT 5 — LiDAR histograms per zone
# ═════════════════════════════════════════════════════════════
zones = [('lidar_fl_m','FL Left','#3498db'),
         ('lidar_fc_m','FC Centre','#e74c3c'),
         ('lidar_fr_m','FR Right','#2ecc71')]

fig5, axes = plt.subplots(n, 3, figsize=(15, 5*n))
if n == 1:
    axes = [axes]
fig5.suptitle('LiDAR Distance Distribution per Zone', fontsize=14, fontweight='bold')

for row, (df, label, name) in enumerate(datasets):
    for col, (zcol, zlabel, zcolor) in enumerate(zones):
        ax   = axes[row][col]
        vals = df[zcol].dropna()
        ax.hist(vals, bins=40, color=zcolor, alpha=0.75, edgecolor='white', linewidth=0.3)
        ax.axvline(0.6, color='#e74c3c', lw=1.2, ls='--', label='STOP 0.6m')
        ax.axvline(0.8, color='#f39c12', lw=1.2, ls='--', label='STEER 0.8m')
        ax.axvline(vals.mean(), color='#333', lw=1.0, ls='-', label=f'Mean {vals.mean():.2f}m')
        ax.set_title(f'{label}\n{zlabel}', fontsize=9)
        ax.set_xlabel('Distance (m)')
        ax.set_ylabel('Frequency')
        ax.legend(fontsize=7, loc='upper right', framealpha=0.7)
        ax.grid(True, alpha=0.3)

plt.tight_layout()
p5 = os.path.join(out, 'plot5_lidar_histograms.png')
fig5.savefig(p5, dpi=150, bbox_inches='tight')
plt.close(fig5)
print(f"Saved: {p5}")

# ═════════════════════════════════════════════════════════════
# PLOT 6 — Summary metrics table
# ═════════════════════════════════════════════════════════════
fig6, axes = plt.subplots(1, n, figsize=(7*n, 8))
if n == 1:
    axes = [axes]
fig6.suptitle('Avoidance Event Summary', fontsize=14, fontweight='bold')

for ax, (df, label, name) in zip(axes, datasets):
    braking  = (df['motor_state']=='BRAKING').sum()
    rev_rows = (df['motor_state']=='REVERSING').sum()
    trn_rows = (df['motor_state']=='TURNING').sum()
    esc_rows = (df['motor_state']=='ESCAPING').sum()
    total    = len(df)
    avoid_pct = (braking+rev_rows+trn_rows+esc_rows)/total*100
    max_retry = int(df['retry_count'].max()) if df['retry_count'].notna().any() else 0
    retry_rows = int((df['retry_count']>0).sum())
    duration   = df['elapsed'].iloc[-1]

    summary = [
        ['Total log events',           f'{total:,}'],
        ['Duration (s)',                f'{duration:.1f}'],
        ['Avoidance triggers',          f'{braking}'],
        ['Time in avoidance (%)',        f'{avoid_pct:.1f}%'],
        ['REVERSING events',            f'{rev_rows}'],
        ['TURNING events',              f'{trn_rows}'],
        ['ESCAPING events',             f'{esc_rows}'],
        ['Max retry count',             f'{max_retry}'],
        ['Non-zero retry rows',         f'{retry_rows}'],
        ['Fusion IMMINENT rows',        f'{(df["fusion_level"]=="IMMINENT").sum()}'],
        ['Fusion CAUTION rows',         f'{(df["fusion_level"]=="CAUTION").sum()}'],
        ['YOLO detections',             f'{(df["camera_class"]!="none").sum()}'],
        ['Radar detections',            f'{df["radar_range_m"].notna().sum()}'],
    ]

    ax.axis('off')
    tbl = ax.table(cellText=summary,
                   colLabels=['Metric','Value'],
                   cellLoc='left', loc='center',
                   colWidths=[0.68, 0.32])
    tbl.auto_set_font_size(False)
    tbl.set_fontsize(10)
    tbl.scale(1, 1.6)
    for j in range(2):
        tbl[0,j].set_facecolor('#2c3e50')
        tbl[0,j].set_text_props(color='white', fontweight='bold')
    for i in range(1, len(summary)+1):
        for j in range(2):
            tbl[i,j].set_facecolor('#f8f9fa' if i%2==0 else '#ffffff')
    ax.set_title(label, fontsize=9, fontweight='bold', pad=20)

plt.tight_layout()
p6 = os.path.join(out, 'plot6_summary_table.png')
fig6.savefig(p6, dpi=150, bbox_inches='tight')
plt.close(fig6)
print(f"Saved: {p6}")

print(f"\nAll plots saved to:\n{out}")


# ═════════════════════════════════════════════════════════════
# PLOT 7 — System Performance Metrics
# ═════════════════════════════════════════════════════════════
def calc_performance(df):
    total = len(df)

    # Count actual state TRANSITIONS not row counts
    # Each transition = one real event regardless of how long it lasted
    transitions   = df['motor_state'] != df['motor_state'].shift()
    states_seq    = df[transitions]['motor_state'].tolist()
    braking_ev    = states_seq.count('BRAKING')
    escaping_ev   = states_seq.count('ESCAPING')
    reversing_ev  = states_seq.count('REVERSING')
    turning_ev    = states_seq.count('TURNING')

    # Avoidance success rate: how many braking events led to a full escape
    # escaping_ev / braking_ev — if equal = 100%, if some got stuck = lower
    success_rate  = (escaping_ev / braking_ev * 100) if braking_ev > 0 else 0.0
    success_rate  = min(success_rate, 100.0)  # cap at 100%

    # Time efficiency: % of time in useful motion (NORMAL + ESCAPING rows)
    useful_rows   = df['motor_state'].isin(['NORMAL', 'ESCAPING']).sum()
    efficiency    = useful_rows / total * 100

    # LiDAR coverage: % of frames with a valid FC reading
    lidar_cov     = df['lidar_fc_m'].notna().sum() / total * 100

    # YOLO coverage: % of frames with a camera detection
    yolo_cov      = (df['camera_class'] != 'none').sum() / total * 100

    # Radar coverage: 0% expected — static objects suppressed by MTI filter
    radar_cov     = df['radar_range_m'].notna().sum() / total * 100

    # Obstacle response rate: % of braking events that also had a reversing
    # (i.e. the car actually committed to reversing not just braked and recovered)
    response_rate = (reversing_ev / braking_ev * 100) if braking_ev > 0 else 0.0
    response_rate = min(response_rate, 100.0)

    return {
        'Avoidance\nSuccess Rate':  success_rate,
        'Drive Time\nEfficiency':   efficiency,
        'LiDAR\nCoverage':         lidar_cov,
        'YOLO\nCoverage':          yolo_cov,
        'Radar\nCoverage':         radar_cov,
        'Obstacle\nResponse Rate': response_rate,
    }

perf_data = [(calc_performance(df), label, name) for df, label, name in datasets]

fig7, axes = plt.subplots(1, n, figsize=(8*n, 7))
if n == 1:
    axes = [axes]
fig7.suptitle('System Performance Metrics (%)', fontsize=14, fontweight='bold', y=1.01)

BAR_COLOURS = {
    'Avoidance\nSuccess Rate':  '#2ecc71',
    'Drive Time\nEfficiency':   '#3498db',
    'LiDAR\nCoverage':         '#e74c3c',
    'YOLO\nCoverage':          '#9b59b6',
    'Radar\nCoverage':         '#95a5a6',
    'Obstacle\nResponse Rate': '#f39c12',
}

for ax, (metrics, label, name) in zip(axes, perf_data):
    keys   = list(metrics.keys())
    vals   = [metrics[k] for k in keys]
    colours = [BAR_COLOURS[k] for k in keys]
    x      = np.arange(len(keys))
    bars   = ax.bar(x, vals, color=colours, edgecolor='white', linewidth=0.8, width=0.55)

    # Value labels on top of each bar — no overlap
    for bar, val in zip(bars, vals):
        ypos = bar.get_height() + 1.5
        ax.text(bar.get_x() + bar.get_width()/2, ypos,
                f'{val:.1f}%', ha='center', va='bottom',
                fontsize=10, fontweight='bold', color='#222')

    ax.set_xticks(x)
    ax.set_xticklabels(keys, fontsize=9, ha='center')
    ax.set_ylim(0, 115)
    ax.set_ylabel('Percentage (%)', fontsize=10)
    ax.set_title(label, fontsize=9, pad=10)
    ax.axhline(100, color='#bbb', lw=0.8, ls='--')
    ax.grid(axis='y', alpha=0.3)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Annotation box explaining radar 0% and YOLO misclassification
    note = ("Radar 0%: static objects suppressed\nby MTI clutter filter (velocity ≈ 0)\n"
            "YOLO: misclassified unknown objects\nas nearest COCO class")
    ax.text(0.98, 0.97, note, transform=ax.transAxes,
            fontsize=7.5, va='top', ha='right', color='#555',
            bbox=dict(boxstyle='round,pad=0.4', facecolor='#fff9e6',
                      edgecolor='#ddd', alpha=0.9))

plt.tight_layout()
p7 = os.path.join(out, 'plot7_performance_metrics.png')
fig7.savefig(p7, dpi=150, bbox_inches='tight')
plt.close(fig7)
print(f"Saved: {p7}")
print("\nAll 7 plots saved.")