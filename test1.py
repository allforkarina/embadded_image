"""
Test script for HRNet pose estimation with keypoint positioning fixes.
"""

import numpy as np
import cv2
import os
from typing import Tuple, Optional
import sys

# Import our utility functions
from utils1 import (
    preprocess_image, 
    parse_hrnet_output, 
    visualize_keypoints,
    COCO_KEYPOINT_NAMES
)


def create_mock_hrnet_output(num_keypoints: int = 17, heatmap_size: Tuple[int, int] = (96, 72)) -> np.ndarray:
    """
    Create mock HRNet output for testing purposes.
    This simulates realistic keypoint heatmaps with proper spatial distribution.
    
    Args:
        num_keypoints: Number of keypoints (default 17 for COCO)
        heatmap_size: Size of heatmap (width, height)
        
    Returns:
        Mock heatmap output of shape (1, num_keypoints, H, W)
    """
    w, h = heatmap_size
    heatmaps = np.zeros((1, num_keypoints, h, w), dtype=np.float32)
    
    # Define realistic keypoint positions for a standing person
    # These are relative positions in heatmap coordinates
    keypoint_positions = [
        (w//2, h//4),      # nose
        (w//2 - 5, h//4 - 3),  # left_eye
        (w//2 + 5, h//4 - 3),  # right_eye
        (w//2 - 8, h//4),      # left_ear
        (w//2 + 8, h//4),      # right_ear
        (w//2 - 15, h//3),     # left_shoulder
        (w//2 + 15, h//3),     # right_shoulder
        (w//2 - 20, h//2),     # left_elbow
        (w//2 + 20, h//2),     # right_elbow
        (w//2 - 25, h//2 + 5), # left_wrist
        (w//2 + 25, h//2 + 5), # right_wrist
        (w//2 - 10, h//2 + 10), # left_hip
        (w//2 + 10, h//2 + 10), # right_hip
        (w//2 - 12, h//3*2),   # left_knee
        (w//2 + 12, h//3*2),   # right_knee
        (w//2 - 10, h//6*5),   # left_ankle
        (w//2 + 10, h//6*5),   # right_ankle
    ]
    
    # Generate gaussian heatmaps for each keypoint
    for i, (cx, cy) in enumerate(keypoint_positions):
        if i < num_keypoints:
            # Create gaussian heatmap
            sigma = 3.0
            for y in range(h):
                for x in range(w):
                    distance = ((x - cx) ** 2 + (y - cy) ** 2) / (2 * sigma ** 2)
                    heatmaps[0, i, y, x] = np.exp(-distance)
    
    return heatmaps


def test_preprocessing():
    """Test image preprocessing function."""
    print("Testing image preprocessing...")
    
    # Create a test image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Test preprocessing
    processed = preprocess_image(test_image, input_size=(384, 288))
    
    print(f"Original image shape: {test_image.shape}")
    print(f"Processed image shape: {processed.shape}")
    
    # Check if format is correct (NCHW)
    assert processed.shape == (1, 3, 288, 384), f"Expected (1, 3, 288, 384), got {processed.shape}"
    
    # Check if values are normalized
    assert 0 <= processed.min() <= 1, f"Min value should be in [0, 1], got {processed.min()}"
    assert 0 <= processed.max() <= 1, f"Max value should be in [0, 1], got {processed.max()}"
    
    print("✓ Preprocessing test passed!")


def test_hrnet_output_parsing():
    """Test HRNet output parsing function."""
    print("\nTesting HRNet output parsing...")
    
    # Create mock output
    mock_output = create_mock_hrnet_output()
    original_size = (640, 480)
    
    # Parse output
    keypoints, scores = parse_hrnet_output(mock_output, original_size)
    
    print(f"Keypoints shape: {keypoints.shape}")
    print(f"Scores shape: {scores.shape}")
    
    # Check shapes
    assert keypoints.shape == (17, 2), f"Expected (17, 2), got {keypoints.shape}"
    assert scores.shape == (17,), f"Expected (17,), got {scores.shape}"
    
    # Check if keypoints are within image bounds
    assert np.all(keypoints[:, 0] >= 0), "X coordinates should be non-negative"
    assert np.all(keypoints[:, 1] >= 0), "Y coordinates should be non-negative"
    assert np.all(keypoints[:, 0] <= original_size[0]), f"X coordinates should be <= {original_size[0]}"
    assert np.all(keypoints[:, 1] <= original_size[1]), f"Y coordinates should be <= {original_size[1]}"
    
    # Check if scores are reasonable
    assert np.all(scores >= 0), "Scores should be non-negative"
    assert np.all(scores <= 1), "Scores should be <= 1"
    
    print("✓ HRNet output parsing test passed!")
    
    # Print some keypoint positions for verification
    print("\nSample keypoint positions:")
    for i in range(min(5, len(keypoints))):
        print(f"  {COCO_KEYPOINT_NAMES[i]}: ({keypoints[i, 0]:.1f}, {keypoints[i, 1]:.1f}), score: {scores[i]:.3f}")


def test_visualization():
    """Test keypoint visualization function."""
    print("\nTesting keypoint visualization...")
    
    # Create test image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Create mock keypoints
    mock_output = create_mock_hrnet_output()
    keypoints, scores = parse_hrnet_output(mock_output, (640, 480))
    
    # Visualize keypoints
    vis_image = visualize_keypoints(test_image, keypoints, scores, 
                                   threshold=0.3, keypoint_names=COCO_KEYPOINT_NAMES)
    
    print(f"Visualization image shape: {vis_image.shape}")
    
    # Check if visualization worked
    assert vis_image.shape == test_image.shape, "Visualization should maintain original image shape"
    
    print("✓ Visualization test passed!")
    
    return vis_image


def test_full_pipeline():
    """Test the complete pose estimation pipeline."""
    print("\nTesting full pipeline...")
    
    # Create test image
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    # Step 1: Preprocess image
    processed = preprocess_image(test_image)
    print(f"1. Preprocessed image shape: {processed.shape}")
    
    # Step 2: Simulate model inference (create mock output)
    mock_output = create_mock_hrnet_output()
    print(f"2. Mock model output shape: {mock_output.shape}")
    
    # Step 3: Parse output
    keypoints, scores = parse_hrnet_output(mock_output, (640, 480))
    print(f"3. Parsed keypoints shape: {keypoints.shape}")
    
    # Step 4: Visualize results
    vis_image = visualize_keypoints(test_image, keypoints, scores, 
                                   threshold=0.3, keypoint_names=COCO_KEYPOINT_NAMES)
    print(f"4. Visualization complete")
    
    # Check that keypoints are not clustered in top-left corner
    # This was the main issue mentioned in the problem statement
    top_left_count = np.sum((keypoints[:, 0] < 50) & (keypoints[:, 1] < 50))
    total_valid_keypoints = np.sum(scores > 0.3)
    
    print(f"Valid keypoints: {total_valid_keypoints}")
    print(f"Keypoints in top-left corner (< 50, 50): {top_left_count}")
    
    if total_valid_keypoints > 0:
        clustering_ratio = top_left_count / total_valid_keypoints
        print(f"Clustering ratio: {clustering_ratio:.2f}")
        
        # The fix should ensure keypoints are distributed across the image
        assert clustering_ratio < 0.5, f"Too many keypoints clustered in top-left corner: {clustering_ratio:.2f}"
    
    print("✓ Full pipeline test passed!")
    
    return vis_image


def save_test_result(image: np.ndarray, filename: str):
    """Save test result image."""
    filepath = os.path.join('/home/runner/work/embadded_image/embadded_image', filename)
    cv2.imwrite(filepath, image)
    print(f"Test result saved to: {filepath}")


def main():
    """Run all tests."""
    print("Running HRNet pose estimation tests...")
    print("=" * 50)
    
    try:
        # Run individual tests
        test_preprocessing()
        test_hrnet_output_parsing()
        vis_image = test_visualization()
        
        # Run full pipeline test
        final_image = test_full_pipeline()
        
        # Save test results
        save_test_result(vis_image, "test_visualization.jpg")
        save_test_result(final_image, "test_full_pipeline.jpg")
        
        print("\n" + "=" * 50)
        print("✓ All tests passed successfully!")
        print("✓ Keypoint positioning issue has been fixed!")
        print("✓ Keypoints are now properly distributed across the image")
        
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()