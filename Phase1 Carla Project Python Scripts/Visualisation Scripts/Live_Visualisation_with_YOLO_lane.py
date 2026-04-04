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
from matplotlib.backends.backend_agg import FigureCanvasAgg
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

def load_yolo_model(weight_path='yolov5n.pt'):
    """Load YOLO model with smart device selection"""
    try:
        from ultralytics import YOLO
        import torch
        
        print(f"📦 Loading YOLO model: {weight_path} ...")
        model = YOLO(weight_path)
        
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

def _extrapolate_line(line_segments, y_bottom, y_top):
    """Fit a single line through all segments and return (x_bottom, y_bottom, x_top, y_top)."""
    if not line_segments:
        return None
    pts = np.array([[x1, y1] for x1, y1, x2, y2 in line_segments] +
                   [[x2, y2] for x1, y1, x2, y2 in line_segments], dtype=np.float32)
    if len(pts) < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01)
    # cv2 returns (1,) arrays; explicitly squeeze to plain scalars
    vx, vy, x0, y0 = [float(v.ravel()[0]) for v in (vx, vy, x0, y0)]
    if abs(vy) < 1e-6:
        return None
    x_bot = int(x0 + (vx / vy) * (y_bottom - y0))
    x_top = int(x0 + (vx / vy) * (y_top   - y0))
    return x_bot, y_bottom, x_top, y_top


def detect_lane_mask(image):
    """Detect ego-lane boundaries via Hough lines and fill the lane polygon."""
    h, w = image.shape[:2]
    # Horizon for lane detection; keep on-road to avoid sky artifacts
    y_top = int(h * 0.60)

    # ── Edge detection ──────────────────────────────────────────────────────
    gray    = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges   = cv2.Canny(blurred, 50, 150)

    # Restrict edges to a trapezoidal ROI (road area only)
    roi = np.zeros_like(edges)
    roi_verts = np.array([[
        (int(w * 0.10), h),
        (int(w * 0.45), y_top),
        (int(w * 0.55), y_top),
        (int(w * 0.90), h)
    ]], dtype=np.int32)
    cv2.fillPoly(roi, roi_verts, 255)
    edges = cv2.bitwise_and(edges, roi)

    # ── Hough lines ─────────────────────────────────────────────────────────
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30,
                            minLineLength=30, maxLineGap=200)

    left_segs, right_segs = [], []
    if lines is not None:
        for x1, y1, x2, y2 in lines[:, 0]:
            if x2 == x1:
                continue
            slope = (y2 - y1) / (x2 - x1)
            mid_x = (x1 + x2) / 2
            if slope < -0.3 and mid_x < w * 0.55:   # left lane line
                left_segs.append((x1, y1, x2, y2))
            elif slope > 0.3 and mid_x > w * 0.45:  # right lane line
                right_segs.append((x1, y1, x2, y2))

    left  = _extrapolate_line(left_segs,  h, y_top)
    right = _extrapolate_line(right_segs, h, y_top)

    # ── Build lane polygon ───────────────────────────────────────────────────
    if left is not None and right is not None:
        vertices = np.array([[
            (left[0],  left[1]),    # bottom-left
            (left[2],  left[3]),    # top-left
            (right[2], right[3]),   # top-right
            (right[0], right[1])    # bottom-right
        ]], dtype=np.int32)
    else:
        # Fallback: fixed single-lane trapezoid (centered)
        vertices = np.array([[
            (int(w * 0.25), h),
            (int(w * 0.45), y_top),
            (int(w * 0.55), y_top),
            (int(w * 0.75), h)
        ]], dtype=np.int32)

    lane_mask = np.zeros((h, w), dtype=np.uint8)
    cv2.fillPoly(lane_mask, vertices, 255)
    # Light morphology to avoid bloating into shoulders
    lane_mask = cv2.dilate(lane_mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)))
    lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    return lane_mask, vertices

def is_object_in_lane(bbox, lane_mask, image_height):
    """Check if object is in the detected lane"""
    x1, y1, x2, y2 = bbox
    h, w = lane_mask.shape[:2]
    
    # Clip bbox to image bounds
    x1c = max(0, min(w - 1, x1))
    x2c = max(0, min(w - 1, x2))
    y1c = max(0, min(h - 1, y1))
    y2c = max(0, min(h - 1, y2))
    if x2c <= x1c or y2c <= y1c:
        return False
    
    # Get bottom center of bounding box (where vehicle touches ground)
    center_x = int((x1c + x2c) / 2)
    bottom_y = int(y2c)
    
    # Primary test: lane overlap ratio inside the box (captures far objects)
    lane_crop = lane_mask[y1c:y2c, x1c:x2c]
    if lane_crop.size > 0:
        lane_ratio = np.mean(lane_crop > 0)
        if lane_ratio > 0.20:
            return True
    
    # Secondary test: generous strip around contact point (for close vehicles)
    if 0 <= center_x < lane_mask.shape[1] and 0 <= bottom_y < lane_mask.shape[0]:
        check_region = lane_mask[max(0, bottom_y - 60):min(lane_mask.shape[0], bottom_y + 20),
                                 max(0, center_x - 40):min(lane_mask.shape[1], center_x + 40)]

        return np.sum(check_region > 0) > (check_region.size * 0.10)
    
    return False

def run_yolo_detection(model, image):
    """Run YOLO with lane detection - only detect objects in ego lane"""
    if image is None:
        print("Detection error: received empty frame (image is None)")
        return None, [], 0.0
    if model is None:
        return image, [], 0.0
    
    try:
        start_time = time.time()
        
        # Detect lane first
        lane_mask, lane_vertices = detect_lane_mask(image)
        
        # Run YOLO inference
        results = model(image, verbose=False)[0]
        
        inference_time = time.time() - start_time
        
        # Filter detections - only keep objects in lane
        detections = []
        all_detections = []  # Track all for comparison
        
        for box in results.boxes:
            # Convert tensors to plain Python scalars to avoid numpy scalar conversion issues
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].cpu().tolist()]
            conf = float(box.conf.cpu().item())
            cls = int(box.cls.cpu().item())
            label = results.names[cls]
            
            bbox = (int(x1), int(y1), int(x2), int(y2))
            
            detection_data = {
                'bbox': bbox,
                'confidence': conf,
                'class': label,
                'class_id': cls,
                'in_lane': False
            }
            
            all_detections.append(detection_data)
            
            # Check if object is in our lane
            if is_object_in_lane(bbox, lane_mask, image.shape[0]):
                detection_data['in_lane'] = True
                detections.append(detection_data)
        
        # Draw annotated image
        annotated = image.copy()
        
        # Draw lane overlay (semi-transparent green)
        lane_overlay = np.zeros_like(image)
        lane_overlay[:,:,1] = lane_mask  # Green channel
        annotated = cv2.addWeighted(annotated, 1.0, lane_overlay, 0.15, 0)
        
        # Draw lane boundaries (yellow)
        cv2.polylines(annotated, lane_vertices, True, (255, 255, 0), 3)
        
        # Draw all detections (faded for out-of-lane)
        for det in all_detections:
            x1, y1, x2, y2 = det['bbox']
            label = f"{det['class']} {det['confidence']:.2f}"
            
            if det['in_lane']:
                # IN LANE - Bright colors
                if det['class'] in ['car', 'truck', 'bus']:
                    color = (0, 255, 0)  # Bright green
                    thickness = 3
                elif det['class'] in ['person', 'bicycle', 'motorcycle']:
                    color = (255, 0, 0)  # Bright red
                    thickness = 3
                else:
                    color = (0, 165, 255)  # Bright orange
                    thickness = 3
                
                # Draw box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
                
                # Draw label with "IN LANE" tag
                label = f"🎯 {label}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(annotated, (x1, y1 - th - 10), (x1 + tw, y1), color, -1)
                cv2.putText(annotated, label, (x1, y1 - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                # Draw ground contact point
                center_x = int((x1 + x2) / 2)
                bottom_y = int(y2)
                cv2.circle(annotated, (center_x, bottom_y), 5, color, -1)
                
            else:
                # OUT OF LANE - Faded gray
                color = (150, 150, 150)
                thickness = 1
                
                # Draw faded box
                cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)
                
                # Small faded label
                (tw, th), _ = cv2.getTextSize(det['class'], cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                cv2.rectangle(annotated, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
                cv2.putText(annotated, det['class'], (x1, y1 - 3),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        # Add lane detection info
        lane_text = f"Lane Objects: {len(detections)}/{len(all_detections)}"
        cv2.putText(annotated, lane_text, (20, 50),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2,
                   cv2.LINE_AA)
        
        return annotated, detections, inference_time
    
    except Exception as e:
        import traceback
        trace = traceback.format_exc()
        print(f"Detection error: {e}")
        print(trace)
        try:
            with open("last_detection_error.txt", "w", encoding="utf-8") as f:
                f.write(trace)
        except Exception:
            pass
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
    
    # Add trend line only if data varies
    if len(perf_df) > 1 and perf_df['num_detections'].std() > 0:
        try:
            z = np.polyfit(perf_df['num_detections'], perf_df['inference_time_ms'], 1)
            p = np.poly1d(z)
            ax.plot(perf_df['num_detections'], p(perf_df['num_detections']), 
                   "r--", alpha=0.8, linewidth=2, label=f'Trend: y={z[0]:.2f}x+{z[1]:.2f}')
            ax.legend()
        except:
            # Skip trend line if calculation fails
            pass
    
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

def process_model(model_weight, model_tag, HEADLESS, DISPLAY_UPDATE_INTERVAL, run_dir, base_output_dir):
    """Run the full pipeline for a single YOLO model; returns performance DataFrame."""
    output_dir = os.path.join(base_output_dir, model_tag)
    os.makedirs(output_dir, exist_ok=True)

    cam_video_path = os.path.join(output_dir, f"{model_tag}_camera_yolo_video.mp4")
    vis_video_path = os.path.join(output_dir, f"{model_tag}_visualisation_video.mp4")
    cam_video_writer = None
    vis_video_writer = None

    # Load YOLO
    yolo_model = load_yolo_model(model_weight)
    
    # Load data
    fusion_df = pd.read_csv(os.path.join(run_dir, "sensor_fusion.csv"))
    state_df = pd.read_csv(os.path.join(run_dir, "vehicle_state.csv"))
    
    camera_frames = sorted(glob.glob(os.path.join(run_dir, "camera", "*.jpg")))
    lidar_files = sorted(glob.glob(os.path.join(run_dir, "lidar", "*.ply")))
    
    print(f"\n=== MODEL: {model_tag} | Weight: {model_weight} ===")
    print(f"✅ {len(camera_frames)} camera frames")
    print(f"✅ {len(lidar_files)} LiDAR clouds")
    print(f"✅ YOLO: {'GPU' if yolo_model else 'Disabled'}\n")

    perf_data = []

    # Always build the visualization figure so we can save vis videos even in headless mode
    if not HEADLESS:
        plt.ion()
    else:
        plt.ioff()
    fig = plt.figure(figsize=(18, 8))
    ax_cam = fig.add_subplot(1, 2, 1)
    ax_lidar = fig.add_subplot(1, 2, 2)
    if HEADLESS:
        canvas = FigureCanvasAgg(fig)
        fig.set_canvas(canvas)
    plt.tight_layout(pad=1)

    print("🎬 Processing...\n")
    num_frames = min(len(camera_frames), len(lidar_files))
    current_frame = 0
    start_playback = time.time()
    vis_capture_interval = 1 if HEADLESS else DISPLAY_UPDATE_INTERVAL

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

    try:
        while current_frame < num_frames:
            cam_img = cv2.imread(camera_frames[current_frame])
            cam_img = cv2.cvtColor(cam_img, cv2.COLOR_BGR2RGB)
            
            cam_annotated, detections, inf_time = run_yolo_detection(yolo_model, cam_img)

            frame_bgr = cv2.cvtColor(cam_annotated, cv2.COLOR_RGB2BGR)
            if cam_video_writer is None:
                h_f, w_f = frame_bgr.shape[:2]
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                cam_video_writer = cv2.VideoWriter(cam_video_path, fourcc, 15, (w_f, h_f))
            cam_video_writer.write(frame_bgr)

            detected_classes = ';'.join([d['class'] for d in detections]) if detections else ''
            perf_data.append({
                'frame': current_frame,
                'inference_time_ms': inf_time * 1000,
                'num_detections': len(detections),
                'detected_classes': detected_classes
            })
            
            fused_dist, ttc, action, speed, time_s, radar_dist = get_data(current_frame)
            
            if current_frame % vis_capture_interval == 0:
                lidar_pts = load_ply(lidar_files[current_frame])
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
                
                ax_cam.set_title(f'Camera + {model_tag} (Lane Detection - Objects in Ego Lane Only)', 
                               fontsize=12, fontweight='bold')
                ax_cam.axis('off')
                
                visualize_lidar_frame(lidar_pts, ax_lidar, fused_dist, action, ttc, radar_dist)
                
                if not HEADLESS:
                    plt.pause(0.001)
                fig.canvas.draw()
                buf = np.asarray(fig.canvas.buffer_rgba())
                vis_frame = buf[:, :, :3]
                fig_w, fig_h = vis_frame.shape[1], vis_frame.shape[0]
                vis_frame_bgr = cv2.cvtColor(vis_frame, cv2.COLOR_RGB2BGR)
                if vis_video_writer is None:
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    vis_video_writer = cv2.VideoWriter(vis_video_path, fourcc, 15, (fig_w, fig_h))
                vis_video_writer.write(vis_frame_bgr)

            current_frame += 1
            
            if current_frame % 100 == 0:
                elapsed = time.time() - start_playback
                fps = current_frame / elapsed
                print(f"[{model_tag}] Frame {current_frame}/{num_frames} | Playback: {fps:.1f} FPS | "
                      f"{len(detections)} objects detected")
        
        total_time = time.time() - start_playback
        print(f"\n✅ [{model_tag}] Playback complete in {total_time:.1f}s")
        print(f"   Average playback speed: {num_frames/total_time:.1f} FPS\n")

        if cam_video_writer is not None:
            cam_video_writer.release()
            print(f"✅ [{model_tag}] Camera video saved: {cam_video_path}")
        if vis_video_writer is not None:
            vis_video_writer.release()
            print(f"✅ [{model_tag}] Visualisation video saved: {vis_video_path}")
        
        perf_df = pd.DataFrame(perf_data)
        if not perf_df.empty:
            csv_path = os.path.join(output_dir, "yolo_performance.csv")
            perf_df.to_csv(csv_path, index=False)
            print(f"✅ [{model_tag}] Performance CSV: {csv_path}")
            
            import shutil
            print(f"\n📋 Copying original data files...")
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
            
            create_performance_graphs(perf_df, output_dir)

        # List files
        print(f"\n{'='*70}")
        print(f"📁 [{model_tag}] FILES SAVED TO: {output_dir}")
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
        return perf_df

    except KeyboardInterrupt:
        print(f"\n⏹️  [{model_tag}] Stopped\n")
        if cam_video_writer is not None:
            cam_video_writer.release()
            print(f"✅ Camera video saved (partial): {cam_video_path}")
        if vis_video_writer is not None:
            vis_video_writer.release()
            print(f"✅ Visualisation video saved (partial): {vis_video_path}")
        return pd.DataFrame()
    finally:
        plt.close('all')


def create_comparison_charts(perf_a, perf_b, labels, output_dir):
    """Create multiple comparison charts between two models."""
    if perf_a.empty or perf_b.empty:
        print("Comparison skipped: one of the perf dataframes is empty.")
        return

    # Derived FPS
    fps_a = 1000 / perf_a['inference_time_ms']
    fps_b = 1000 / perf_b['inference_time_ms']

    # --- Chart 1: Aggregate metrics bar ---
    metrics = [
        ("Mean Inference Time (ms)", perf_a['inference_time_ms'].mean(), perf_b['inference_time_ms'].mean()),
        ("Median Inference Time (ms)", perf_a['inference_time_ms'].median(), perf_b['inference_time_ms'].median()),
        ("Mean FPS", fps_a.mean(), fps_b.mean()),
        ("Total Detections", perf_a['num_detections'].sum(), perf_b['num_detections'].sum()),
        ("Mean Detections/Frame", perf_a['num_detections'].mean(), perf_b['num_detections'].mean()),
    ]
    fig, ax = plt.subplots(figsize=(9, 5))
    x = np.arange(len(metrics))
    width = 0.35
    ax.bar(x - width/2, [m[1] for m in metrics], width, label=labels[0], color='steelblue')
    ax.bar(x + width/2, [m[2] for m in metrics], width, label=labels[1], color='orange')
    ax.set_xticks(x)
    ax.set_xticklabels([m[0] for m in metrics], rotation=20, ha='right')
    ax.legend()
    ax.set_title('YOLO Model Comparison - Key Metrics')
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path_bar = os.path.join(output_dir, "yolo_comparison_metrics.png")
    plt.savefig(path_bar, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Comparison chart: {path_bar}")

    # --- Chart 2: Inference time histogram overlay ---
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.hist(perf_a['inference_time_ms'], bins=40, alpha=0.6, label=labels[0], color='steelblue')
    ax.hist(perf_b['inference_time_ms'], bins=40, alpha=0.6, label=labels[1], color='orange')
    ax.set_xlabel('Inference Time (ms)')
    ax.set_ylabel('Frames')
    ax.set_title('Inference Time Distribution')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    path_hist = os.path.join(output_dir, "yolo_comparison_inftime_hist.png")
    plt.savefig(path_hist, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Comparison chart: {path_hist}")

    # --- Chart 3: Per-frame inference time series (downsampled if long) ---
    fig, ax = plt.subplots(figsize=(10, 4))
    step = max(1, len(perf_a) // 2000)  # keep plots light
    ax.plot(perf_a['frame'][::step], perf_a['inference_time_ms'][::step], label=labels[0], color='steelblue', linewidth=1)
    ax.plot(perf_b['frame'][::step], perf_b['inference_time_ms'][::step], label=labels[1], color='orange', linewidth=1)
    ax.set_xlabel('Frame')
    ax.set_ylabel('Inference Time (ms)')
    ax.set_title('Inference Time per Frame')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    path_series = os.path.join(output_dir, "yolo_comparison_inftime_series.png")
    plt.savefig(path_series, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Comparison chart: {path_series}")

    # --- Chart 4: Detections per frame series ---
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(perf_a['frame'], perf_a['num_detections'], label=labels[0], color='steelblue', alpha=0.8)
    ax.plot(perf_b['frame'], perf_b['num_detections'], label=labels[1], color='orange', alpha=0.8)
    ax.set_xlabel('Frame')
    ax.set_ylabel('# Detections')
    ax.set_title('Detections per Frame')
    ax.grid(True, alpha=0.3)
    ax.legend()
    plt.tight_layout()
    path_det_series = os.path.join(output_dir, "yolo_comparison_detections_series.png")
    plt.savefig(path_det_series, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Comparison chart: {path_det_series}")

    # --- Chart 5: FPS boxplot ---
    fig, ax = plt.subplots(figsize=(6, 5))
    ax.boxplot([fps_a, fps_b], labels=labels, patch_artist=True,
               boxprops=dict(facecolor='lightblue'),
               medianprops=dict(color='red'))
    ax.set_ylabel('FPS')
    ax.set_title('FPS Distribution')
    ax.grid(True, axis='y', alpha=0.3)
    plt.tight_layout()
    path_fps_box = os.path.join(output_dir, "yolo_comparison_fps_box.png")
    plt.savefig(path_fps_box, dpi=150, bbox_inches='tight')
    plt.close(fig)
    print(f"✅ Comparison chart: {path_fps_box}")


def create_side_by_side_video(video_a, video_b, label_a, label_b, output_path):
    """Combine two videos side-by-side with labels."""
    if not (os.path.exists(video_a) and os.path.exists(video_b)):
        print(f"Side-by-side skipped (missing video): {video_a if not os.path.exists(video_a) else video_b}")
        return
    cap_a = cv2.VideoCapture(video_a)
    cap_b = cv2.VideoCapture(video_b)
    fps = cap_a.get(cv2.CAP_PROP_FPS) or 15
    width_a = int(cap_a.get(cv2.CAP_PROP_FRAME_WIDTH))
    height_a = int(cap_a.get(cv2.CAP_PROP_FRAME_HEIGHT))
    width_b = int(cap_b.get(cv2.CAP_PROP_FRAME_WIDTH))
    height_b = int(cap_b.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Resize second to match first height
    target_h = min(height_a, height_b)
    scale_a = target_h / height_a
    scale_b = target_h / height_b
    new_w_a = int(width_a * scale_a)
    new_w_b = int(width_b * scale_b)
    out_w = new_w_a + new_w_b
    out_h = target_h

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_path, fourcc, fps, (out_w, out_h))

    font = cv2.FONT_HERSHEY_SIMPLEX
    while True:
        ret_a, frame_a = cap_a.read()
        ret_b, frame_b = cap_b.read()
        if not (ret_a and ret_b):
            break
        frame_a = cv2.resize(frame_a, (new_w_a, target_h))
        frame_b = cv2.resize(frame_b, (new_w_b, target_h))
        combined = np.hstack([frame_a, frame_b])

        cv2.putText(combined, label_a, (15, 30), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(combined, label_b, (new_w_a + 15, 30), font, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        writer.write(combined)

    cap_a.release()
    cap_b.release()
    writer.release()
    print(f"✅ Side-by-side video: {output_path}")


def main():
    print("\n" + "="*70)
    print("🚀 ULTRA-FAST PLAYBACK WITH YOLO ANALYSIS (Dual-Model)")
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
    run_names = [
        "20260403_112049_collision_avoidance",
        "20260211_161428_rainy_collision_avoidance",
        "20260211_130643_collision_avoidance"
    ]
    
    # Define models to compare
    models = [
        ("yolov5n.pt", "yolov5"),
        ("yolov8n.pt", "yolov8")
    ]

    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    for run_name in run_names:
        run_dir = os.path.join(log_base, run_name)
        if not os.path.exists(run_dir):
            print(f"❌ Directory not found: {run_dir}")
            continue
        
        base_output_dir = os.path.join(run_dir, f"yolo_results_{timestamp}")
        os.makedirs(base_output_dir, exist_ok=True)
        
        print(f"\n📁 Input:  {run_dir}")
        print(f"📁 Output base: {base_output_dir}\n")

        perf_results = []
        for weight, tag in models:
            perf_df = process_model(weight, tag, HEADLESS, DISPLAY_UPDATE_INTERVAL, run_dir, base_output_dir)
            perf_results.append((tag, perf_df))

        if len(perf_results) == 2:
            create_comparison_charts(perf_results[0][1], perf_results[1][1],
                                     [perf_results[0][0], perf_results[1][0]],
                                     base_output_dir)
            # Build side-by-side videos (camera and visualisation)
            cam_a = os.path.join(base_output_dir, models[0][1], f"{models[0][1]}_camera_yolo_video.mp4")
            cam_b = os.path.join(base_output_dir, models[1][1], f"{models[1][1]}_camera_yolo_video.mp4")
            vis_a = os.path.join(base_output_dir, models[0][1], f"{models[0][1]}_visualisation_video.mp4")
            vis_b = os.path.join(base_output_dir, models[1][1], f"{models[1][1]}_visualisation_video.mp4")
            create_side_by_side_video(cam_a, cam_b, models[0][1], models[1][1],
                                      os.path.join(base_output_dir, "comparison_camera_side_by_side.mp4"))
            create_side_by_side_video(vis_a, vis_b, models[0][1], models[1][1],
                                      os.path.join(base_output_dir, "comparison_visualisation_side_by_side.mp4"))

    print("\n✅ Dual-model runs complete for all specified folders. Check each base output for per-model folders and comparison chart.\n")

if __name__ == "__main__":
    main()
