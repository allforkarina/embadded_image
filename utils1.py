"""
HRNet pose estimation utility functions with fixes for keypoint positioning.
"""

import numpy as np
import cv2
from typing import Tuple, List, Optional


def preprocess_image(image: np.ndarray, input_size: Tuple[int, int] = (384, 288)) -> np.ndarray:
    """
    Preprocess image for HRNet with proper NCHW format conversion.
    
    Args:
        image: Input image in BGR format (H, W, C)
        input_size: Target size (width, height) for model input
        
    Returns:
        Preprocessed image in NCHW format
    """
    # Resize image to target size
    resized = cv2.resize(image, input_size)
    
    # Convert BGR to RGB
    rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
    
    # Normalize to [0, 1]
    normalized = rgb_image.astype(np.float32) / 255.0
    
    # Convert to CHW format (Channel, Height, Width)
    chw_image = np.transpose(normalized, (2, 0, 1))
    
    # Add batch dimension to get NCHW format
    nchw_image = np.expand_dims(chw_image, axis=0)
    
    return nchw_image


def find_keypoints_from_heatmap(heatmap: np.ndarray, threshold: float = 0.1) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract keypoints from heatmap using maximum response locations.
    
    Args:
        heatmap: Heatmap of shape (num_keypoints, H, W)
        threshold: Confidence threshold for valid keypoints
        
    Returns:
        keypoints: Array of shape (num_keypoints, 2) with (x, y) coordinates
        scores: Array of shape (num_keypoints,) with confidence scores
    """
    num_keypoints, h, w = heatmap.shape
    keypoints = np.zeros((num_keypoints, 2), dtype=np.float32)
    scores = np.zeros(num_keypoints, dtype=np.float32)
    
    for i in range(num_keypoints):
        # Find maximum response location
        max_idx = np.unravel_index(np.argmax(heatmap[i]), heatmap[i].shape)
        max_y, max_x = max_idx
        max_score = heatmap[i][max_y, max_x]
        
        # Store keypoint coordinates and score
        keypoints[i] = [max_x, max_y]
        scores[i] = max_score
    
    return keypoints, scores


def parse_hrnet_output(output: np.ndarray, original_size: Tuple[int, int], 
                      input_size: Tuple[int, int] = (384, 288)) -> Tuple[np.ndarray, np.ndarray]:
    """
    Parse HRNet output heatmaps and map coordinates to original image size.
    
    Args:
        output: Model output heatmaps of shape (1, num_keypoints, H, W)
        original_size: Original image size (width, height)
        input_size: Model input size (width, height)
        
    Returns:
        keypoints: Array of shape (num_keypoints, 2) with (x, y) coordinates in original image
        scores: Array of shape (num_keypoints,) with confidence scores
    """
    # Remove batch dimension
    if output.ndim == 4:
        heatmaps = output[0]
    else:
        heatmaps = output
    
    # Get keypoints from heatmaps
    keypoints, scores = find_keypoints_from_heatmap(heatmaps)
    
    # Calculate scale factors for mapping back to original image
    heatmap_h, heatmap_w = heatmaps.shape[-2:]
    scale_x = original_size[0] / heatmap_w
    scale_y = original_size[1] / heatmap_h
    
    # Map keypoints to original image coordinates
    keypoints[:, 0] *= scale_x  # x coordinates
    keypoints[:, 1] *= scale_y  # y coordinates
    
    return keypoints, scores


def visualize_keypoints(image: np.ndarray, keypoints: np.ndarray, scores: np.ndarray, 
                       threshold: float = 0.3, keypoint_names: Optional[List[str]] = None) -> np.ndarray:
    """
    Visualize keypoints on image, showing only valid keypoints above threshold.
    
    Args:
        image: Original image in BGR format
        keypoints: Array of shape (num_keypoints, 2) with (x, y) coordinates
        scores: Array of shape (num_keypoints,) with confidence scores
        threshold: Confidence threshold for displaying keypoints
        keypoint_names: Optional list of keypoint names for labeling
        
    Returns:
        Annotated image with keypoints
    """
    vis_image = image.copy()
    
    # Define colors for different keypoints
    colors = [
        (0, 0, 255),    # Red
        (0, 255, 0),    # Green
        (255, 0, 0),    # Blue
        (0, 255, 255),  # Yellow
        (255, 0, 255),  # Magenta
        (255, 255, 0),  # Cyan
        (128, 0, 128),  # Purple
        (255, 165, 0),  # Orange
        (255, 192, 203), # Pink
        (0, 128, 128),  # Teal
        (128, 128, 0),  # Olive
        (128, 0, 0),    # Maroon
        (0, 128, 0),    # Dark Green
        (0, 0, 128),    # Navy
        (128, 128, 128), # Gray
        (255, 20, 147), # Deep Pink
        (0, 191, 255),  # Deep Sky Blue
    ]
    
    # Draw only valid keypoints
    for i, (keypoint, score) in enumerate(zip(keypoints, scores)):
        if score > threshold:
            x, y = int(keypoint[0]), int(keypoint[1])
            
            # Ensure coordinates are within image bounds
            if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                color = colors[i % len(colors)]
                
                # Draw keypoint circle
                cv2.circle(vis_image, (x, y), 4, color, -1)
                cv2.circle(vis_image, (x, y), 6, (255, 255, 255), 2)
                
                # Add keypoint label if names provided
                if keypoint_names and i < len(keypoint_names):
                    cv2.putText(vis_image, f"{keypoint_names[i]}: {score:.2f}", 
                               (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    
    return vis_image


# COCO keypoint names for reference
COCO_KEYPOINT_NAMES = [
    'nose', 'left_eye', 'right_eye', 'left_ear', 'right_ear',
    'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
    'left_wrist', 'right_wrist', 'left_hip', 'right_hip',
    'left_knee', 'right_knee', 'left_ankle', 'right_ankle'
]


def get_transform_matrix(src_size: Tuple[int, int], dst_size: Tuple[int, int]) -> np.ndarray:
    """
    Get transformation matrix for image preprocessing.
    
    Args:
        src_size: Source image size (width, height)
        dst_size: Destination size (width, height)
        
    Returns:
        Transformation matrix
    """
    src_w, src_h = src_size
    dst_w, dst_h = dst_size
    
    # Calculate scaling factors
    scale_x = dst_w / src_w
    scale_y = dst_h / src_h
    
    # Create transformation matrix
    transform_matrix = np.array([
        [scale_x, 0, 0],
        [0, scale_y, 0],
        [0, 0, 1]
    ], dtype=np.float32)
    
    return transform_matrix


def apply_transform(keypoints: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
    """
    Apply transformation matrix to keypoints.
    
    Args:
        keypoints: Array of shape (num_keypoints, 2) with (x, y) coordinates
        transform_matrix: 3x3 transformation matrix
        
    Returns:
        Transformed keypoints
    """
    # Convert to homogeneous coordinates
    num_keypoints = keypoints.shape[0]
    homogeneous = np.ones((num_keypoints, 3), dtype=np.float32)
    homogeneous[:, :2] = keypoints
    
    # Apply transformation
    transformed = np.dot(homogeneous, transform_matrix.T)
    
    # Convert back to 2D coordinates
    return transformed[:, :2]