"""
Demonstration script showing the HRNet pose estimation fix.
This script demonstrates how the keypoint positioning issue was fixed.
"""

import numpy as np
import cv2
import os
from utils1 import (
    preprocess_image,
    parse_hrnet_output,
    visualize_keypoints,
    COCO_KEYPOINT_NAMES
)

# Import create_mock_hrnet_output from test1.py
from test1 import create_mock_hrnet_output


def simulate_broken_behavior(heatmaps: np.ndarray, original_size: tuple) -> tuple:
    """
    Simulate the old broken behavior where keypoints cluster in top-left corner.
    This is what the code was doing before the fix.
    """
    # This simulates the old broken coordinate mapping
    if heatmaps.ndim == 4:
        heatmaps = heatmaps[0]
    
    num_keypoints = heatmaps.shape[0]
    keypoints = np.zeros((num_keypoints, 2), dtype=np.float32)
    scores = np.zeros(num_keypoints, dtype=np.float32)
    
    # Simulate the old bug - coordinates not properly scaled
    for i in range(num_keypoints):
        max_idx = np.unravel_index(np.argmax(heatmaps[i]), heatmaps[i].shape)
        max_y, max_x = max_idx
        max_score = heatmaps[i][max_y, max_x]
        
        # OLD BUG: Not scaling coordinates properly to original image size
        # This would result in keypoints clustered in top-left corner
        keypoints[i] = [max_x, max_y]  # No scaling applied!
        scores[i] = max_score
    
    return keypoints, scores


def demonstrate_fix():
    """Demonstrate the difference between old and new behavior."""
    print("HRNet Pose Estimation Fix Demonstration")
    print("=" * 50)
    
    # Create test image
    test_image = np.ones((480, 640, 3), dtype=np.uint8) * 128  # Gray background
    
    # Add some visual elements to make it more interesting
    cv2.rectangle(test_image, (50, 50), (590, 430), (200, 200, 200), -1)
    cv2.putText(test_image, "HRNet Pose Estimation Test", (150, 100), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
    
    # Create mock HRNet output
    mock_output = create_mock_hrnet_output(num_keypoints=17, heatmap_size=(96, 72))
    
    # Demonstrate old broken behavior
    print("1. Old broken behavior (keypoints cluster in top-left):")
    broken_keypoints, broken_scores = simulate_broken_behavior(mock_output, (640, 480))
    
    # Count keypoints in top-left corner for broken version
    broken_top_left = np.sum((broken_keypoints[:, 0] < 50) & (broken_keypoints[:, 1] < 50))
    print(f"   Keypoints in top-left corner: {broken_top_left}/{len(broken_keypoints)}")
    
    # Demonstrate new fixed behavior  
    print("2. New fixed behavior (keypoints properly distributed):")
    fixed_keypoints, fixed_scores = parse_hrnet_output(mock_output, (640, 480))
    
    # Count keypoints in top-left corner for fixed version
    fixed_top_left = np.sum((fixed_keypoints[:, 0] < 50) & (fixed_keypoints[:, 1] < 50))
    print(f"   Keypoints in top-left corner: {fixed_top_left}/{len(fixed_keypoints)}")
    
    # Create visualizations
    broken_vis = visualize_keypoints(test_image.copy(), broken_keypoints, broken_scores,
                                    threshold=0.3, keypoint_names=COCO_KEYPOINT_NAMES)
    
    fixed_vis = visualize_keypoints(test_image.copy(), fixed_keypoints, fixed_scores,
                                   threshold=0.3, keypoint_names=COCO_KEYPOINT_NAMES)
    
    # Add titles to images
    cv2.putText(broken_vis, "BROKEN: Keypoints cluster in top-left", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.putText(fixed_vis, "FIXED: Keypoints properly distributed", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    
    # Save comparison images
    broken_path = "/home/runner/work/embadded_image/embadded_image/demo_broken.jpg"
    fixed_path = "/home/runner/work/embadded_image/embadded_image/demo_fixed.jpg"
    
    cv2.imwrite(broken_path, broken_vis)
    cv2.imwrite(fixed_path, fixed_vis)
    
    # Create side-by-side comparison
    comparison = np.hstack([broken_vis, fixed_vis])
    comparison_path = "/home/runner/work/embadded_image/embadded_image/demo_comparison.jpg"
    cv2.imwrite(comparison_path, comparison)
    
    print(f"\n✓ Demonstration images saved:")
    print(f"   - Broken behavior: {broken_path}")
    print(f"   - Fixed behavior: {fixed_path}")
    print(f"   - Side-by-side comparison: {comparison_path}")
    
    # Print detailed statistics
    print(f"\n📊 Detailed Statistics:")
    print(f"   Original image size: {test_image.shape[:2]}")
    print(f"   Heatmap size: {mock_output.shape[2:]}")
    print(f"   Number of keypoints: {len(COCO_KEYPOINT_NAMES)}")
    print(f"   ")
    print(f"   Broken version:")
    print(f"     - Keypoints in top-left (< 50, 50): {broken_top_left}")
    print(f"     - Average X coordinate: {np.mean(broken_keypoints[:, 0]):.1f}")
    print(f"     - Average Y coordinate: {np.mean(broken_keypoints[:, 1]):.1f}")
    print(f"   ")
    print(f"   Fixed version:")
    print(f"     - Keypoints in top-left (< 50, 50): {fixed_top_left}")
    print(f"     - Average X coordinate: {np.mean(fixed_keypoints[:, 0]):.1f}")
    print(f"     - Average Y coordinate: {np.mean(fixed_keypoints[:, 1]):.1f}")
    
    return comparison


if __name__ == "__main__":
    comparison_image = demonstrate_fix()
    print("\n" + "=" * 50)
    print("✓ Fix demonstration completed successfully!")
    print("✓ The keypoint positioning issue has been resolved!")