"""
IMPROVED PLAYBACK VISUALIZATION
Student: Raffay Hassan (M00944822)

Clean, fast playback with better views!
"""

import os
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyBboxPatch
import pandas as pd
import glob

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

def visualize_lidar_frame(pts, ax, fused_dist, action, ttc):
    """Visualize LiDAR with BETTER ZOOM"""
    ax.clear()
    
    if pts is None or len(pts) == 0:
        ax.text(0.5, 0.5, 'No LiDAR Data', ha='center', va='center', fontsize=20)
        ax.set_xlim(-5, 50)
        ax.set_ylim(-8, 8)
        return
    
    x = pts[:, 0]
    y = pts[:, 1]
    z = pts[:, 2]
    
    # Color by height
    colors = plt.cm.viridis((z - z.min()) / (z.max() - z.min() + 0.001))
    
    # Plot points - larger size for better visibility
    ax.scatter(x, y, c=colors, s=3, alpha=0.7)
    
    # Ego vehicle (larger)
    ego_rect = Rectangle((-2, -1), 4, 2, fill=True, color='white', 
                         alpha=0.9, edgecolor='blue', linewidth=3)
    ax.add_patch(ego_rect)
    ax.text(0, 0, 'EGO', ha='center', va='center', fontsize=10, fontweight='bold', color='blue')
    
    # Detection zone (narrower, more focused)
    roi_rect = Rectangle((2, -1.5), 48, 3, fill=False, edgecolor='yellow',
                         linewidth=2, linestyle='--', alpha=0.6)
    ax.add_patch(roi_rect)
    
    # Detected obstacle
    if fused_dist < SENTINEL_DIST and fused_dist < 50:
        # Distance line
        ax.plot([0, fused_dist], [0, 0], 'r-', linewidth=4, alpha=0.9)
        
        # Obstacle marker (larger)
        circle = plt.Circle((fused_dist, 0), 1.5, color='red', fill=True, alpha=0.5, linewidth=3, edgecolor='darkred')
        ax.add_patch(circle)
        
        # Safety zone
        if fused_dist < 8.0:
            zone_color = 'red'
            zone_label = '⚠ CRITICAL'
        elif fused_dist < 15.0:
            zone_color = 'orange'
            zone_label = '⚠ WARNING'
        else:
            zone_color = 'green'
            zone_label = '✓ SAFE'
        
        # Label above obstacle
        ax.text(fused_dist, 2.5, f'{zone_label}\n{fused_dist:.1f}m',
               ha='center', fontsize=11, color='white', fontweight='bold',
               bbox=dict(boxstyle='round,pad=0.5', facecolor=zone_color, 
                        edgecolor='white', alpha=0.9, linewidth=2))
    
    # Action in bottom right
    action_colors = {
        'EMERGENCY_BRAKE': 'red', 'LANE_CHANGE': 'orange',
        'AVOIDING': 'yellow', 'DRIVING': 'limegreen',
        'STOPPED': 'red', 'STRAIGHTENING': 'cyan', 'RESUMED': 'limegreen'
    }
    
    ax.text(40, -6.5, action, ha='center', fontsize=13,
           fontweight='bold', color='white',
           bbox=dict(boxstyle='round,pad=0.6', 
                    facecolor=action_colors.get(action, 'gray'), 
                    edgecolor='white', alpha=0.95, linewidth=2))
    
    # BETTER ZOOM - focus on relevant area
    ax.set_xlim(-5, 50)   # Was -5 to 120, now focused on first 50m
    ax.set_ylim(-8, 8)    # Was -10 to 10, now -8 to 8
    ax.set_xlabel('Forward Distance (m)', fontsize=11, fontweight='bold')
    ax.set_ylabel('Lateral Offset (m)', fontsize=11, fontweight='bold')
    ax.set_title('LiDAR Point Cloud (Top-Down View)', fontsize=13, fontweight='bold')
    ax.grid(True, alpha=0.4, linewidth=0.8)
    ax.set_aspect('equal')

def main():
    print("\n" + "="*70)
    print("📹 PLAYBACK VISUALIZATION - IMPROVED")
    print("="*70 + "\n")
    
    log_base = "/home/rh960/carla_env/dt_logs"
    run_dir = os.path.join(log_base, "20260211_130643_collision_avoidance")
    
    if not os.path.exists(run_dir):
        print(f"❌ Directory not found: {run_dir}")
        return
    
    print(f"📁 Loading: {run_dir}\n")
    
    # Load CSV
    fusion_df = pd.read_csv(os.path.join(run_dir, "sensor_fusion.csv"))
    state_df = pd.read_csv(os.path.join(run_dir, "vehicle_state.csv"))
    
    # Get files
    camera_frames = sorted(glob.glob(os.path.join(run_dir, "camera", "*.jpg")))
    lidar_files = sorted(glob.glob(os.path.join(run_dir, "lidar", "*.ply")))
    
    print(f"✅ {len(camera_frames)} camera frames")
    print(f"✅ {len(lidar_files)} LiDAR point clouds")
    print(f"✅ {len(fusion_df)} data records\n")
    
    # Setup figure - BETTER LAYOUT
    plt.ion()
    fig = plt.figure(figsize=(18, 8))
    
    # Camera takes more space (70%), LiDAR (30%)
    ax_cam = fig.add_subplot(1, 2, 1)
    ax_lidar = fig.add_subplot(1, 2, 2)
    
    plt.tight_layout(pad=2)
    
    print("🎬 Playing (SPACE=pause, Q=quit, close window to exit)...\n")
    
    num_frames = min(len(camera_frames), len(lidar_files))
    current_frame = 0
    
    def get_data(idx):
        if idx < len(fusion_df):
            f = fusion_df.iloc[idx]
            fused_dist = f.get('fused_dist_m', SENTINEL_DIST)
            ttc = f.get('ttc_s', SENTINEL_DIST)
        else:
            fused_dist, ttc = SENTINEL_DIST, SENTINEL_DIST
        
        if idx < len(state_df):
            s = state_df.iloc[idx]
            action = s.get('action', 'UNKNOWN')
            speed = s.get('ego_speed_mps', 0.0)
            time_s = s.get('time_s', 0.0)
        else:
            action, speed, time_s = 'UNKNOWN', 0.0, 0.0
        
        return fused_dist, ttc, action, speed, time_s
    
    try:
        while current_frame < num_frames and plt.fignum_exists(fig.number):
            # Load data
            cam_img = cv2.imread(camera_frames[current_frame])
            cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
            lidar_pts = load_ply(lidar_files[current_frame])
            fused_dist, ttc, action, speed, time_s = get_data(current_frame)
            
            # === CAMERA VIEW - CLEANER OVERLAY ===
            ax_cam.clear()
            ax_cam.imshow(cam_img)
            
            # Single clean info box in top-left
            info_text = f'Time: {time_s:.1f}s\n'
            info_text += f'Speed: {speed:.1f} m/s\n'
            info_text += f'Distance: {fused_dist:.1f} m\n'
            info_text += f'TTC: {ttc:.1f} s'
            
            # Color based on distance
            if fused_dist < 8:
                box_color = 'red'
            elif fused_dist < 15:
                box_color = 'orange'
            else:
                box_color = 'green'
            
            ax_cam.text(0.02, 0.98, info_text,
                       transform=ax_cam.transAxes,
                       fontsize=13, fontweight='bold',
                       verticalalignment='top',
                       color='white',
                       bbox=dict(boxstyle='round,pad=0.8', 
                                facecolor='black', 
                                edgecolor=box_color,
                                alpha=0.8, linewidth=3))
            
            # Action label - bottom left, large
            action_colors = {
                'EMERGENCY_BRAKE': 'red', 'LANE_CHANGE': 'orange',
                'AVOIDING': 'yellow', 'DRIVING': 'limegreen',
                'STOPPED': 'red', 'STRAIGHTENING': 'cyan', 'RESUMED': 'limegreen'
            }
            
            ax_cam.text(0.02, 0.05, action,
                       transform=ax_cam.transAxes,
                       fontsize=16, fontweight='bold',
                       color='white',
                       bbox=dict(boxstyle='round,pad=0.7',
                                facecolor=action_colors.get(action, 'gray'),
                                edgecolor='white',
                                alpha=0.9, linewidth=3))
            
            ax_cam.set_title(f'Camera Feed - Frame {current_frame+1}/{num_frames}',
                           fontsize=14, fontweight='bold', pad=10)
            ax_cam.axis('off')
            
            # === LIDAR VIEW ===
            visualize_lidar_frame(lidar_pts, ax_lidar, fused_dist, action, ttc)
            
            # EVEN FASTER PLAYBACK - 1.5x speed (75 FPS)
            plt.pause(0.013)
            
            current_frame += 1
            
            if current_frame % 50 == 0:
                print(f"Frame {current_frame}/{num_frames}: {time_s:.1f}s, {speed:.1f}m/s, {fused_dist:.1f}m, {action}")
        
        print(f"\n✅ Playback complete!\n")
        print("Close window to exit...")
        plt.ioff()
        plt.show()
    
    except KeyboardInterrupt:
        print("\n⏹️  Stopped\n")
    finally:
        plt.close('all')

if __name__ == "__main__":
    main()