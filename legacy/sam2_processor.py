import os
import cv2
import numpy as np
import torch
from ultralytics import SAM
from PIL import Image

# ==============================================================================
# ⚙️ CONFIGURATION SECTION
# ==============================================================================
import config
CONFIG = config.sam2_processor_CONFIG
# ==============================================================================


class SAM2Processor:
    def __init__(self, config: dict):
        """
        SAM 2.1 Segmenter Module using centralized configuration settings.
        """
        self.config = config
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        print(f"[SAM2Processor] Running inference on device: {self.device}")
        
        # Load Ultralytics SAM 2.1 model
        self.model = SAM(self.config["MODEL_PATH"])
        self.pixels_per_cm = self.config["PIXELS_PER_CM"]

    def _denoise_scanlines(self, frame: np.ndarray) -> np.ndarray:
        """Removes horizontal line artifacts caused by ESP32-CAM power noise."""
        return cv2.medianBlur(frame, 3)

    def process_image(self, image_input=None):
        """
        Processes image frame, cleans noise, removes nested sub-masks, and returns structured data.
        
        :param image_input: Optional path, PIL Image, or NumPy array. If None, defaults to CONFIG['INPUT_IMAGE'].
        """
        # Determine image source
        target_input = image_input if image_input is not None else self.config["INPUT_IMAGE"]

        if isinstance(target_input, str):
            if not os.path.exists(target_input):
                raise FileNotFoundError(f"Input image not found: {target_input}")
            frame = cv2.imread(target_input)
        elif isinstance(target_input, Image.Image):
            frame = cv2.cvtColor(np.array(target_input), cv2.COLOR_RGB2BGR)
        else:
            frame = target_input.copy()

        if frame is None:
            raise ValueError("Invalid image input provided.")

        # 1. Hardware Noise Mitigation
        if self.config["FILTER_SCANLINES"]:
            frame = self._denoise_scanlines(frame)

        # 2. Run SAM 2.1 Inference
        results = self.model(
            frame, 
            device=self.device, 
            retina_masks=True, 
            conf=self.config["CONFIDENCE_THRESHOLD"], 
            verbose=False
        )[0]

        detected_objects = []

        if results.masks is not None:
            masks_xy = results.masks.xy
            boxes = results.boxes.xyxy.cpu().numpy() if results.boxes is not None else []
            
            valid_indices = list(range(len(masks_xy)))

            # Option A: Filter out tiny masks contained inside larger masks
            if self.config["FILTER_NESTED"] and len(boxes) > 0:
                valid_indices = []
                for i, box_a in enumerate(boxes):
                    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
                    is_contained = False
                    
                    for j, box_b in enumerate(boxes):
                        if i == j:
                            continue
                        area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
                        
                        # Check if box_a is inside larger box_b
                        if area_a < area_b:
                            inter_x1 = max(box_a[0], box_b[0])
                            inter_y1 = max(box_a[1], box_b[1])
                            inter_x2 = min(box_a[2], box_b[2])
                            inter_y2 = min(box_a[3], box_b[3])
                            
                            inter_area = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
                            overlap_ratio = inter_area / (area_a + 1e-6)
                            
                            if overlap_ratio > self.config["NESTED_OVERLAP_RATIO"]:
                                is_contained = True
                                break
                    if not is_contained:
                        valid_indices.append(i)

            # Build metadata and overlays
            obj_counter = 1
            for i in valid_indices:
                contour = masks_xy[i]
                if len(contour) < 5:
                    continue

                pts = contour.astype(np.int32)

                # Calculate Centroid
                M = cv2.moments(pts)
                if M["m00"] != 0:
                    center_x_px = M["m10"] / M["m00"]
                    center_y_px = M["m01"] / M["m00"]
                else:
                    center_x_px, center_y_px = pts[0][0], pts[0][1]

                center_x_cm = round(center_x_px / self.pixels_per_cm, 2)
                center_y_cm = round(center_y_px / self.pixels_per_cm, 2)
                # Convert bounding box coordinates to integers and calculate dimensions
                if i < len(boxes):
                    x1, y1, x2, y2 = map(int, boxes[i])
                    width_px = x2 - x1
                    height_px = y2 - y1
                    width_cm = round(width_px / self.pixels_per_cm, 2)
                    height_cm = round(height_px / self.pixels_per_cm, 2)
                else:
                    x1 = y1 = x2 = y2 = width_px = height_px = width_cm = height_cm = 0

                obj_data = {
                    "id": obj_counter,
                    "centroid_cm": {"x": center_x_cm, "y": center_y_cm},
                    "centroid_px": {"x": int(center_x_px), "y": int(center_y_px)},
                    "bounding_box_pixels": {
                        "left_x": x1,
                        "top_y": y1,
                        "right_x": x2,
                        "bottom_y": y2,
                        "width_px": width_px,
                        "height_px": height_px
                    },
                    "estimated_size_cm": {
                        "width_cm": width_cm,
                        "height_cm": height_cm
                    }
                }

                detected_objects.append(obj_data)

                # 1. Draw Green SAM Mask Outline
                cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

                # 2. Draw Cyan Bounding Box Rectangle
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)

                # 3. Draw Red Centroid Dot
                cv2.circle(frame, (int(center_x_px), int(center_y_px)), 4, (0, 0, 255), -1)

                # 4. Add Labeled Top Header Above Bounding Box
                box_label = f"ID:{obj_counter} | Box:[{x1},{y1},{x2},{y2}]"
                cv2.putText(frame, box_label, (x1, max(y1 - 8, 15)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1, cv2.LINE_AA)

        # Save output image as configured
        cv2.imwrite(self.config["OUTPUT_IMAGE"], frame)
        return frame, detected_objects


# --- Standalone Test Execution ---
if __name__ == "__main__":
    # Initialize using top-level CONFIG
    processor = SAM2Processor(config=CONFIG)
    
    # Fallback check for test script
    input_file = CONFIG["INPUT_IMAGE"]
    if not os.path.exists(input_file):
        # Default to test_image.jpg if configured image isn't available
        input_file = "test_image.jpg"
    
    if os.path.exists(input_file):
        print(f"\n[Testing] Processing image: {input_file}")
        _, objects = processor.process_image(image_input=input_file)
        
        print(f"\n--- SAM 2.1 RESULTS ({len(objects)} objects found) ---")
        for obj in objects:
            print(
                f"Object #{obj['id']} -> Centroid: {obj['centroid_cm']} cm | "
                f"BBox: [L:{obj['bounding_box_pixels']['left_x']}, T:{obj['bounding_box_pixels']['top_y']}, "
                f"R:{obj['bounding_box_pixels']['right_x']}, B:{obj['bounding_box_pixels']['bottom_y']}]"
            )
        
        print(f"\nAnnotated output saved to: '{CONFIG['OUTPUT_IMAGE']}'")
    else:
        print("No input image found to run test.")