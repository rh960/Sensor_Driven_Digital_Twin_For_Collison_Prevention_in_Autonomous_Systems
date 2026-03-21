"""
ULTRA-FAST PLAYBACK WITH YOLO PERFORMANCE ANALYSIS
Student: Raffay Hassan (M00944822)

Features:
- Maximum speed playback
- YOLO detection on ALL classes (80+ objects)
- Performance metrics CSV export
- Real-time performance graphs
- Detection statistics
"""

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import pandas as pd
import glob
import time

SENTINEL_DIST = 999.0

def load_ply(filepath):
    """Load LiDAR point cloud from PLY file"""
    if not os.path.exists(filepath):
        return None
    
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    header_end = 0
    for i, line in enumerate(lines):
        if line.strip() == 'end_header':
            header_end = i + 1
            break
    
    points = []
    for line in lines[header_end:]:
        parts = line.strip().split()
        if len(parts) >= 3:
            try:
                points.append([float(parts[0]), float(parts[1]), float(parts[2])])
            except ValueError:
                continue
    
    return np.array(points) if points else None

def load_yolo_model():
    """Load YOLOv8 model with smart device selection"""
    try:
        from ultralytics import YOLO
        import torch
        
        print("📦 Loading YOLOv8 model...")
        model = YOLO('yolov8n.pt')  # Nano model (fastest)
        
        # Smart device selection
        if torch.cuda.is_available():
            try:
                model.to('cuda')
                device_name = torch.cuda.get_device_name(0)
                print(f"✅ YOLO loaded on GPU: {device_name}")
                print(f"   CUDA version: {torch.version.cuda}\n")
                return model
            except Exception as e:
                print(f"⚠️  GPU failed: {e}")
                print(f"   Using CPU...\n")
                model.to('cpu')
                return model
        else:
            model.to('cpu')
            print("✅ YOLO loaded on CPU\n")
            return model
            
    except ImportError:
        print("⚠️  Install: pip install ultralytics --break-system-packages\n")
        return None
    except Exception as e:
        print(f"⚠️  YOLO load failed: {e}\n")
        return None

def run_yolo_detection(model, image):
    """Run YOLO on ALL 80 classes and measure performance"""
    if model is None:
        return image, [], 0.0
    
    try:
        start_time = time.time()
        
        # Run inference on ALL classes
        results = model(image, verbose=False)[0]
        
        inference_time = time.time() - start_time
        
        # Get ALL detections (no filtering by class)
        detections = []
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            label = results.names[cls]
            
            detections.append({
                'bbox': (int(x1), int(y1), int(x2), int(y2)),
                'confidence': conf,
                'class': label,
                'class_id': cls
            })
        
        # Draw bounding boxes
        annotated = image.copy()
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class']} {det['confidence']:.2f}"
            
            # Different colors for different classes
            if det['class'] in ['car', 'truck', 'bus']:
                color = (0, 255, 0)  # Green for vehicles
            elif det['class'] in ['person', 'bicycle', 'motorcycle']:
                color = (255, 0, 0)  # Red for people/bikes
            else:
                color = (0, 165, 255)  # Orange for everything else
            
            # Draw box
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
            cv2.putText(annotated, label, (x1, y1 - 3),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return annotated, detections, inference_time
    
    except RuntimeError as e:
        # CUDA error - move model to CPU and retry
        if 'CUDA' in str(e) or 'cuda' in str(e):
            try:
                print(f"\n⚠️  CUDA error detected - switching to CPU mode...")
                model.to('cpu')
                
                start_time = time.time()
                results = model(image, verbose=False)[0]
                inference_time = time.time() - start_time
                
                detections = []
                for box in results.boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    label = results.names[cls]
                    detections.append({
                        'bbox': (int(x1), int(y1), int(x2), int(y2)),
                        'confidence': conf,
                        'class': label,
                        'class_id': cls
                    })
                
                annotated = image.copy()
                for det in detections:
                    x1, y1, x2, y2 = det['bbox']
                    label = f"{det['class']} {det['confidence']:.2f}"
                    if det['class'] in ['car', 'truck', 'bus']:
                        color = (0, 255, 0)
                    elif det['class'] in ['person', 'bicycle', 'motorcycle']:
                        color = (255, 0, 0)
                    else:
                        color = (0, 165, 255)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                    cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
                    cv2.putText(annotated, label, (x1, y1 - 3),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                return annotated, detections, inference_time
            except Exception as e2:
                return image, [], 0.0
        return image, [], 0.0
    except Exception as e:
        return image, [], 0.0

def visualize_lidar_frame(pts, ax, fused_dist, action, ttc, radar_dist):
    """Fast LiDAR visualization with RADAR overlay"""
    ax.clear()
    
    if pts is None or len(pts) == 0:
        ax.text(0.5, 0.5, 'No LiDAR', ha='center', va='center', fontsize=16)
        ax.set_xlim(-5, 50)
        ax.set_ylim(-8, 8)
        return
    
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    
    # Simplified visualization for speed
    ax.scatter(x, y, c='purple', s=1, alpha=0.5, label='LiDAR')
    
    # Ego
    ego_rect = Rectangle((-2, -1), 4, 2, fill=True, color='white', 
                         alpha=0.9, edgecolor='blue', linewidth=2)
    ax.add_patch(ego_rect)
    ax.text(0, 0, 'EGO', ha='center', va='center', fontsize=9, 
            fontweight='bold', color='blue')
    
    # RADAR detection (Red X)
    if radar_dist < SENTINEL_DIST and radar_dist < 50:
        ax.plot(radar_dist, 0, 'rx', markersize=12, markeredgewidth=3, 
               label=f'Radar: {radar_dist:.1f}m', zorder=10)
        ax.plot([0, radar_dist], [0, 0], 'r--', linewidth=2, alpha=0.5)
    
    # Fused detection (Green line)
    if fused_dist < SENTINEL_DIST and fused_dist < 50:
        ax.plot([0, fused_dist], [0, 0], 'lime', linewidth=3, alpha=0.9,
               label=f'Fused: {fused_dist:.1f}m', zorder=9)
        circle = plt.Circle((fused_dist, 0), 1, color='red', alpha=0.6,
                           edgecolor='darkred', linewidth=2)
        ax.add_patch(circle)
        
        # Safety zone indicator
        if fused_dist < 8.0:
            zone_color = 'red'
            zone_label = '⚠ CRITICAL'
        elif fused_dist < 15.0:
            zone_color = 'orange'
            zone_label = '⚠ WARNING'
        else:
            zone_color = 'green'
            zone_label = '✓ SAFE'
        
        ax.text(fused_dist, 2, f'{zone_label}\n{fused_dist:.1f}m',
               ha='center', fontsize=9, color='white', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.4', facecolor=zone_color, 
                        edgecolor='white', alpha=0.9, linewidth=2))
    
    # Action
    action_colors = {
        'EMERGENCY_BRAKE': 'red', 'LANE_CHANGE': 'orange',
        'AVOIDING': 'yellow', 'DRIVING': 'limegreen',
        'STOPPED': 'red', 'STRAIGHTENING': 'cyan', 'RESUMED': 'limegreen'
    }
    
    ax.text(40, -6, action, ha='center', fontsize=11, fontweight='bold', color='white',
           bbox=dict(boxstyle='round,pad=0.4', 
                    facecolor=action_colors.get(action, 'gray'), 
                    alpha=0.9, linewidth=2))
    
    ax.set_xlim(-5, 50)
    ax.set_ylim(-8, 8)
    ax.set_xlabel('Forward (m)', fontsize=9, fontweight='bold')
    ax.set_ylabel('Lateral (m)', fontsize=9, fontweight='bold')
    ax.set_title('Sensor Fusion (LiDAR + Radar)', fontsize=11, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.legend(loc='upper right', fontsize=8, framealpha=0.9)

def create_performance_graphs(perf_df, output_dir):
    """Create comprehensive YOLO performance analysis graphs"""
    print("\n📊 Creating performance graphs...")
    
    # ========== GRAPH SET 1: YOLO Performance (2x2) ==========
    fig1, axes1 = plt.subplots(2, 2, figsize=(16, 10))
    fig1.suptitle('YOLO Detection Performance Analysis', fontsize=16, fontweight='bold')
    
    # 1.1: Inference Time Over Frames
    ax = axes1[0, 0]
    ax.plot(perf_df['frame'], perf_df['inference_time_ms'], 'b-', linewidth=1, alpha=0.7)
    ax.axhline(perf_df['inference_time_ms'].mean(), color='r', linestyle='--', 
                label=f'Mean: {perf_df["inference_time_ms"].mean():.1f}ms')
    ax.axhline(perf_df['inference_time_ms'].median(), color='g', linestyle='--',
                label=f'Median: {perf_df["inference_time_ms"].median():.1f}ms')
    ax.fill_between(perf_df['frame'], 
                     perf_df['inference_time_ms'].min(), 
                     perf_df['inference_time_ms'].max(), 
                     alpha=0.1, color='blue')
    ax.set_xlabel('Frame Number', fontsize=10)
    ax.set_ylabel('Inference Time (ms)', fontsize=10)
    ax.set_title('YOLO Inference Time per Frame', fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 1.2: FPS Over Time
    ax = axes1[0, 1]
    fps = 1000.0 / perf_df['inference_time_ms']
    ax.plot(perf_df['frame'], fps, 'g-', linewidth=1, alpha=0.7)
    ax.axhline(fps.mean(), color='r', linestyle='--', 
                label=f'Mean: {fps.mean():.1f} FPS')
    ax.axhline(fps.min(), color='orange', linestyle=':', 
                label=f'Min: {fps.min():.1f} FPS')
    ax.axhline(fps.max(), color='blue', linestyle=':', 
                label=f'Max: {fps.max():.1f} FPS')
    ax.set_xlabel('Frame Number', fontsize=10)
    ax.set_ylabel('FPS', fontsize=10)
    ax.set_title('YOLO Processing Speed (FPS)', fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 1.3: Number of Detections per Frame
    ax = axes1[1, 0]
    ax.plot(perf_df['frame'], perf_df['num_detections'], 'm-', linewidth=1, alpha=0.7)
    ax.axhline(perf_df['num_detections'].mean(), color='r', linestyle='--',
                label=f'Mean: {perf_df["num_detections"].mean():.2f}')
    ax.fill_between(perf_df['frame'], 0, perf_df['num_detections'], alpha=0.3, color='magenta')
    ax.set_xlabel('Frame Number', fontsize=10)
    ax.set_ylabel('Number of Objects', fontsize=10)
    ax.set_title('Objects Detected per Frame', fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.legend()
    
    # 1.4: Detection Class Distribution
    ax = axes1[1, 1]
    class_counts = {}
    for classes_str in perf_df['detected_classes']:
        if pd.notna(classes_str) and classes_str != '':
            for cls in classes_str.split(';'):
                class_counts[cls] = class_counts.get(cls, 0) + 1
    
    if class_counts:
        sorted_classes = sorted(class_counts.items(), key=lambda x: x[1], reverse=True)[:10]
        classes, counts = zip(*sorted_classes)
        colors = plt.cm.Set3(np.linspace(0, 1, len(classes)))
        ax.barh(classes, counts, color=colors)
        ax.set_xlabel('Frame Count', fontsize=10)
        ax.set_title('Top 10 Detected Object Classes', fontweight='bold')
        ax.grid(True, alpha=0.3, axis='x')
        
        # Add count labels
        for i, (cls, count) in enumerate(zip(classes, counts)):
            ax.text(count, i, f' {count}', va='center', fontsize=9)
    else:
        ax.text(0.5, 0.5, 'No detections', ha='center', va='center', fontsize=14)
    
    plt.tight_layout()
    graph1_path = os.path.join(output_dir, 'yolo_performance.png')
    plt.savefig(graph1_path, dpi=150, bbox_inches='tight')
    print(f"  ✅ Performance graphs: {graph1_path}")
    plt.close()
    
    # ========== GRAPH SET 2: Statistical Comparisons (2x2) ==========
    fig2, axes2 = plt.subplots(2, 2, figsize=(16, 10))
    fig2.suptitle('YOLO Statistical Analysis & Comparisons', fontsize=16, fontweight='bold')
    
    # 2.1: Inference Time Distribution (Histogram)
    ax = axes2[0, 0]
    ax.hist(perf_df['inference_time_ms'], bins=30, color='skyblue', edgecolor='black', alpha=0.7)
    ax.axvline(perf_df['inference_time_ms'].mean(), color='red', linestyle='--', 
               linewidth=2, label=f'Mean: {perf_df["inference_time_ms"].mean():.1f}ms')
    ax.axvline(perf_df['inference_time_ms'].median(), color='green', linestyle='--', 
               linewidth=2, label=f'Median: {perf_df["inference_time_ms"].median():.1f}ms')
    ax.set_xlabel('Inference Time (ms)', fontsize=10)
    ax.set_ylabel('Frequency (frames)', fontsize=10)
    ax.set_title('Inference Time Distribution', fontweight='bold')
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')
    
    # 2.2: Detections vs Inference Time (Scatter)
    ax = axes2[0, 1]
    scatter = ax.scatter(perf_df['num_detections'], perf_df['inference_time_ms'], 
                        c=perf_df['frame'], cmap='viridis', alpha=0.6, s=20)
    ax.set_xlabel('Number of Detections', fontsize=10)
    ax.set_ylabel('Inference Time (ms)', fontsize=10)
    ax.set_title('Detections vs Processing Time', fontweight='bold')
    ax.grid(True, alpha=0.3)
    cbar = plt.colorbar(scatter, ax=ax)
    cbar.set_label('Frame Number', fontsize=9)
    
    # Add trend line
    if len(perf_df) > 1:
        z = np.polyfit(perf_df['num_detections'], perf_df['inference_time_ms'], 1)
        p = np.poly1d(z)
        ax.plot(perf_df['num_detections'], p(perf_df['num_detections']), 
               "r--", alpha=0.8, linewidth=2, label=f'Trend: y={z[0]:.2f}x+{z[1]:.2f}')
        ax.legend()
    
    # 2.3: Cumulative Detections Over Time
    ax = axes2[1, 0]
    cumulative_detections = perf_df['num_detections'].cumsum()
    ax.plot(perf_df['frame'], cumulative_detections, 'purple', linewidth=2)
    ax.fill_between(perf_df['frame'], 0, cumulative_detections, alpha=0.3, color='purple')
    ax.set_xlabel('Frame Number', fontsize=10)
    ax.set_ylabel('Cumulative Detections', fontsize=10)
    ax.set_title(f'Total Objects Detected: {cumulative_detections.iloc[-1]}', fontweight='bold')
    ax.grid(True, alpha=0.3)
    
    # Add milestone markers
    total = cumulative_detections.iloc[-1]
    for pct in [25, 50, 75]:
        milestone = total * pct / 100
        frame_idx = (cumulative_detections >= milestone).idxmax()
        ax.axhline(milestone, color='red', linestyle=':', alpha=0.5)
        ax.text(len(perf_df)*0.02, milestone, f'{pct}%', fontsize=9, color='red')
    
    # 2.4: Performance Summary Box
    ax = axes2[1, 1]
    ax.axis('off')
    
    # Calculate statistics
    stats_text = f"""
    YOLO PERFORMANCE SUMMARY
    {'='*40}
    
    Inference Time:
      • Mean:     {perf_df['inference_time_ms'].mean():.2f} ms
      • Median:   {perf_df['inference_time_ms'].median():.2f} ms
      • Std Dev:  {perf_df['inference_time_ms'].std():.2f} ms
      • Min:      {perf_df['inference_time_ms'].min():.2f} ms
      • Max:      {perf_df['inference_time_ms'].max():.2f} ms
    
    Processing Speed:
      • Mean FPS: {fps.mean():.1f}
      • Min FPS:  {fps.min():.1f}
      • Max FPS:  {fps.max():.1f}
    
    Detections:
      • Total Objects:     {perf_df['num_detections'].sum()}
      • Avg per Frame:     {perf_df['num_detections'].mean():.2f}
      • Frames with Detections: {(perf_df['num_detections'] > 0).sum()}
      • Detection Rate:    {(perf_df['num_detections'] > 0).sum() / len(perf_df) * 100:.1f}%
    
    Unique Classes: {len(class_counts)}
    Total Frames:   {len(perf_df)}
    """
    
    ax.text(0.1, 0.9, stats_text, transform=ax.transAxes, fontsize=10,
           verticalalignment='top', fontfamily='monospace',
           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    graph2_path = os.path.join(output_dir, 'yolo_statistics.png')
    plt.savefig(graph2_path, dpi=150, bbox_inches='tight')
    print(f"  ✅ Statistics graphs: {graph2_path}")
    plt.close()
    
    # ========== GRAPH SET 3: Sensor Comparison (if data available) ==========
    # Try to load sensor fusion data for comparison
    try:
        parent_dir = os.path.dirname(output_dir)
        fusion_csv = os.path.join(parent_dir, 'sensor_fusion.csv')
        
        if os.path.exists(fusion_csv):
            fusion_df = pd.read_csv(fusion_csv)
            
            fig3, axes3 = plt.subplots(2, 2, figsize=(16, 10))
            fig3.suptitle('Sensor Fusion Comparison Analysis', fontsize=16, fontweight='bold')
            
            # 3.1: Radar vs LiDAR vs Fused Distance
            ax = axes3[0, 0]
            valid_fusion = fusion_df[fusion_df['fused_dist_m'] < 999]
            if len(valid_fusion) > 0:
                ax.plot(valid_fusion.index, valid_fusion['radar_dist_m'], 
                       'r-', linewidth=1, alpha=0.6, label='Radar')
                ax.plot(valid_fusion.index, valid_fusion['lidar_dist_m'], 
                       'b-', linewidth=1, alpha=0.6, label='LiDAR')
                ax.plot(valid_fusion.index, valid_fusion['fused_dist_m'], 
                       'g-', linewidth=2, alpha=0.8, label='Fused')
                ax.set_xlabel('Frame', fontsize=10)
                ax.set_ylabel('Distance (m)', fontsize=10)
                ax.set_title('Sensor Distance Measurements Comparison', fontweight='bold')
                ax.legend()
                ax.grid(True, alpha=0.3)
            
            # 3.2: Sensor Agreement (Difference between Radar and LiDAR)
            ax = axes3[0, 1]
            valid_both = fusion_df[(fusion_df['radar_dist_m'] < 999) & 
                                   (fusion_df['lidar_dist_m'] < 999)]
            if len(valid_both) > 0:
                diff = valid_both['radar_dist_m'] - valid_both['lidar_dist_m']
                ax.hist(diff, bins=30, color='orange', edgecolor='black', alpha=0.7)
                ax.axvline(0, color='red', linestyle='--', linewidth=2, label='Perfect Agreement')
                ax.axvline(diff.mean(), color='blue', linestyle='--', 
                          linewidth=2, label=f'Mean: {diff.mean():.2f}m')
                ax.set_xlabel('Radar - LiDAR Distance (m)', fontsize=10)
                ax.set_ylabel('Frequency', fontsize=10)
                ax.set_title('Sensor Agreement Distribution', fontweight='bold')
                ax.legend()
                ax.grid(True, alpha=0.3, axis='y')
            
            # 3.3: TTC Distribution
            ax = axes3[1, 0]
            valid_ttc = fusion_df[(fusion_df['ttc_s'] < 999) & (fusion_df['ttc_s'] > 0)]
            if len(valid_ttc) > 0:
                ax.hist(valid_ttc['ttc_s'], bins=30, color='red', edgecolor='black', alpha=0.7)
                ax.axvline(2.0, color='orange', linestyle='--', linewidth=2, 
                          label='Critical Threshold (2s)')
                ax.set_xlabel('Time to Collision (s)', fontsize=10)
                ax.set_ylabel('Frequency', fontsize=10)
                ax.set_title('TTC Distribution', fontweight='bold')
                ax.legend()
                ax.grid(True, alpha=0.3, axis='y')
            
            # 3.4: Safety Level Distribution
            ax = axes3[1, 1]
            if 'safety_level' in fusion_df.columns:
                safety_counts = fusion_df['safety_level'].value_counts()
                colors_map = {'SAFE': 'green', 'WARNING': 'orange', 'CRITICAL': 'red'}
                colors = [colors_map.get(s, 'gray') for s in safety_counts.index]
                ax.pie(safety_counts.values, labels=safety_counts.index, autopct='%1.1f%%',
                      colors=colors, startangle=90)
                ax.set_title('Safety Level Distribution', fontweight='bold')
            
            plt.tight_layout()
            graph3_path = os.path.join(output_dir, 'sensor_comparison.png')
            plt.savefig(graph3_path, dpi=150, bbox_inches='tight')
            print(f"  ✅ Sensor comparison: {graph3_path}")
            plt.close()
    except Exception as e:
        print(f"  ⚠️  Sensor comparison graphs skipped: {e}")
    
    print(f"\n📊 All graphs saved to: {output_dir}\n")

def main():
    print("\n" + "="*70)
    print("🚀 ULTRA-FAST PLAYBACK WITH YOLO ANALYSIS")
    print("="*70 + "\n")
    
    # === MODE SELECTION ===
    print("Select mode:")
    print("1. HEADLESS (no visualization - MAXIMUM SPEED)")
    print("2. FAST DISPLAY (update every 10 frames)")
    print("3. NORMAL (update every frame)")
    mode_choice = input("Enter choice [1/2/3, default=2]: ").strip() or "2"
    
    HEADLESS = (mode_choice == "1")
    DISPLAY_UPDATE_INTERVAL = 1 if mode_choice == "3" else (999999 if HEADLESS else 10)
    
    print(f"\n{'🔥 HEADLESS MODE' if HEADLESS else f'📺 DISPLAY MODE (update every {DISPLAY_UPDATE_INTERVAL} frames)'}\n")
    
    log_base = "/home/rh960/carla_env/dt_logs"
    run_dir = os.path.join(log_base, "20260211_130643_collision_avoidance")
    
    if not os.path.exists(run_dir):
        print(f"❌ Directory not found: {run_dir}")
        return
    
    # Create separate output folder with timestamp
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = os.path.join(run_dir, f"yolo_results_{timestamp}")
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"📁 Input:  {run_dir}")
    print(f"📁 Output: {output_dir}\n")
    
    # Load YOLO
    yolo_model = load_yolo_model()
    
    # Load data
    fusion_df = pd.read_csv(os.path.join(run_dir, "sensor_fusion.csv"))
    state_df = pd.read_csv(os.path.join(run_dir, "vehicle_state.csv"))
    
    camera_frames = sorted(glob.glob(os.path.join(run_dir, "camera", "*.jpg")))
    lidar_files = sorted(glob.glob(os.path.join(run_dir, "lidar", "*.ply")))
    
    print(f"✅ {len(camera_frames)} camera frames")
    print(f"✅ {len(lidar_files)} LiDAR clouds")
    print(f"✅ YOLO: {'GPU' if yolo_model else 'Disabled'}\n")
    
    # Performance tracking
    perf_data = []
    
    # Setup figure (only if not headless)
    if not HEADLESS:
        plt.ion()
        fig = plt.figure(figsize=(18, 8))
        ax_cam = fig.add_subplot(1, 2, 1)
        ax_lidar = fig.add_subplot(1, 2, 2)
        plt.tight_layout(pad=1)
    else:
        fig = ax_cam = ax_lidar = None
    
    print("🎬 Processing...\n")
    
    # SPEED SETTINGS
    num_frames = min(len(camera_frames), len(lidar_files))
    current_frame = 0
    
    def get_data(idx):
        if idx < len(fusion_df):
            f = fusion_df.iloc[idx]
            fused_dist = f.get('fused_dist_m', SENTINEL_DIST)
            ttc = f.get('ttc_s', SENTINEL_DIST)
            radar_dist = f.get('radar_dist_m', SENTINEL_DIST)
        else:
            fused_dist, ttc, radar_dist = SENTINEL_DIST, SENTINEL_DIST, SENTINEL_DIST
        
        if idx < len(state_df):
            s = state_df.iloc[idx]
            action = s.get('action', 'UNKNOWN')
            speed = s.get('ego_speed_mps', 0.0)
            time_s = s.get('time_s', 0.0)
        else:
            action, speed, time_s = 'UNKNOWN', 0.0, 0.0
        
        return fused_dist, ttc, action, speed, time_s, radar_dist
    
    start_playback = time.time()
    
    try:
        while current_frame < num_frames and (HEADLESS or plt.fignum_exists(fig.number)):
            # Load image
            cam_img = cv2.imread(camera_frames[current_frame])
            cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
            
            # YOLO detection with timing (ALWAYS RUN - this is what we're measuring!)
            cam_annotated, detections, inf_time = run_yolo_detection(yolo_model, cam_img)
            
            # Record performance
            detected_classes = ';'.join([d['class'] for d in detections]) if detections else ''
            perf_data.append({
                'frame': current_frame,
                'inference_time_ms': inf_time * 1000,
                'num_detections': len(detections),
                'detected_classes': detected_classes
            })
            
            fused_dist, ttc, action, speed, time_s, radar_dist = get_data(current_frame)
            
            # === UPDATE DISPLAY ONLY EVERY N FRAMES ===
            if current_frame % DISPLAY_UPDATE_INTERVAL == 0:
                # Load LiDAR
                lidar_pts = load_ply(lidar_files[current_frame])
                
                # === CAMERA VIEW ===
                ax_cam.clear()
                ax_cam.imshow(cam_annotated)
                
                info = f'Frame: {current_frame}/{num_frames}\n'
                info += f'Time: {time_s:.1f}s | Speed: {speed:.1f}m/s\n'
                info += f'Distance: {fused_dist:.1f}m | TTC: {ttc:.1f}s'
                
                if yolo_model:
                    info += f'\n\n🎯 Detections: {len(detections)}'
                    if inf_time > 0:
                        info += f'\n⚡ YOLO: {inf_time*1000:.1f}ms ({1/inf_time:.1f} FPS)'
                
                box_color = 'red' if fused_dist < 8 else ('orange' if fused_dist < 15 else 'green')
                
                ax_cam.text(0.02, 0.98, info, transform=ax_cam.transAxes,
                           fontsize=11, fontweight='bold', verticalalignment='top',
                           color='white',
                           bbox=dict(boxstyle='round,pad=0.6', facecolor='black', 
                                    edgecolor=box_color, alpha=0.8, linewidth=2))
                
                ax_cam.set_title(f'Camera + YOLO (All {len(yolo_model.names) if yolo_model else 0} Classes)', 
                               fontsize=12, fontweight='bold')
                ax_cam.axis('off')
                
                # === LIDAR VIEW ===
                visualize_lidar_frame(lidar_pts, ax_lidar, fused_dist, action, ttc, radar_dist)
                
                # Minimal pause for display update
                plt.pause(0.001)
            
            current_frame += 1
            
            if current_frame % 100 == 0:
                elapsed = time.time() - start_playback
                fps = current_frame / elapsed
                print(f"Frame {current_frame}/{num_frames} | Playback: {fps:.1f} FPS | "
                      f"{len(detections)} objects detected")
        
        total_time = time.time() - start_playback
        print(f"\n✅ Playback complete in {total_time:.1f}s")
        print(f"   Average playback speed: {num_frames/total_time:.1f} FPS\n")
        
        # Save performance CSV
        if perf_data:
            perf_df = pd.DataFrame(perf_data)
            csv_path = os.path.join(output_dir, "yolo_performance.csv")
            perf_df.to_csv(csv_path, index=False)
            print(f"✅ Performance CSV: {csv_path}")
            
            # Copy original CSV files to output folder for complete record
            print(f"\n📋 Copying original data files...")
            import shutil
            
            csv_files_to_copy = [
                'sensor_fusion.csv',
                'vehicle_state.csv',
                'radar_data.csv',
                'lidar_data.csv'
            ]
            
            for csv_file in csv_files_to_copy:
                src = os.path.join(run_dir, csv_file)
                dst = os.path.join(output_dir, csv_file)
                if os.path.exists(src):
                    shutil.copy2(src, dst)
                    print(f"  ✅ Copied: {csv_file}")
                else:
                    print(f"  ⚠️  Not found: {csv_file}")
            
            # Create a summary CSV with key metrics
            summary_data = {
                'Metric': [
                    'Total Frames',
                    'Mean Inference Time (ms)',
                    'Median Inference Time (ms)',
                    'Std Dev Inference Time (ms)',
                    'Min Inference Time (ms)',
                    'Max Inference Time (ms)',
                    'Mean FPS',
                    'Min FPS',
                    'Max FPS',
                    'Total Detections',
                    'Mean Detections per Frame',
                    'Frames with Detections',
                    'Detection Rate (%)',
                    'Unique Classes Detected'
                ],
                'Value': [
                    len(perf_df),
                    f"{perf_df['inference_time_ms'].mean():.2f}",
                    f"{perf_df['inference_time_ms'].median():.2f}",
                    f"{perf_df['inference_time_ms'].std():.2f}",
                    f"{perf_df['inference_time_ms'].min():.2f}",
                    f"{perf_df['inference_time_ms'].max():.2f}",
                    f"{(1000/perf_df['inference_time_ms']).mean():.1f}",
                    f"{(1000/perf_df['inference_time_ms']).min():.1f}",
                    f"{(1000/perf_df['inference_time_ms']).max():.1f}",
                    perf_df['num_detections'].sum(),
                    f"{perf_df['num_detections'].mean():.2f}",
                    (perf_df['num_detections'] > 0).sum(),
                    f"{(perf_df['num_detections'] > 0).sum() / len(perf_df) * 100:.1f}",
                    len(set(';'.join(perf_df['detected_classes'].dropna()).split(';')))
                ]
            }
            
            summary_df = pd.DataFrame(summary_data)
            summary_path = os.path.join(output_dir, "yolo_summary.csv")
            summary_df.to_csv(summary_path, index=False)
            print(f"  ✅ Created: yolo_summary.csv\n")
            
            # Print statistics
            print(f"\n📊 YOLO PERFORMANCE STATISTICS:")
            print(f"   Mean inference time: {perf_df['inference_time_ms'].mean():.2f} ms")
            print(f"   Min inference time:  {perf_df['inference_time_ms'].min():.2f} ms")
            print(f"   Max inference time:  {perf_df['inference_time_ms'].max():.2f} ms")
            print(f"   Mean FPS: {(1000/perf_df['inference_time_ms']).mean():.1f}")
            print(f"   Total detections: {perf_df['num_detections'].sum()}")
            print(f"   Avg detections/frame: {perf_df['num_detections'].mean():.2f}")
            
            # Create graphs
            create_performance_graphs(perf_df, output_dir)
        
        # List all files in output directory
        print(f"\n{'='*70}")
        print(f"📁 ALL FILES SAVED TO: {output_dir}")
        print(f"{'='*70}\n")
        
        print("📊 CSV Files:")
        csv_files = sorted([f for f in os.listdir(output_dir) if f.endswith('.csv')])
        for csv_file in csv_files:
            file_path = os.path.join(output_dir, csv_file)
            file_size = os.path.getsize(file_path) / 1024  # KB
            print(f"  • {csv_file:<30} ({file_size:.1f} KB)")
        
        print("\n📈 Graph Files:")
        graph_files = sorted([f for f in os.listdir(output_dir) if f.endswith('.png')])
        for graph_file in graph_files:
            file_path = os.path.join(output_dir, graph_file)
            file_size = os.path.getsize(file_path) / 1024  # KB
            print(f"  • {graph_file:<30} ({file_size:.1f} KB)")
        
        print(f"\n{'='*70}\n")
        
        if not HEADLESS:
            print("\nClose window to exit...")
            plt.ioff()
            plt.show()
        else:
            print("\n✅ Processing complete! Check CSV and graphs.\n")
    
    except KeyboardInterrupt:
        print("\n⏹️  Stopped\n")
    finally:
        plt.close('all')

if __name__ == "__main__":
    main()