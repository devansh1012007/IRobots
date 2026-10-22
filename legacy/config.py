# =========================================================================================================================
# config for `get_image.py`
# =========================================================================================================================
get_image_CONFIG = {
    # Network Settings
    "ESP32_IP": "10.65.176.73",             # IP address displayed on your serial monitor
    "ENDPOINT": "/capture",                 # Endpoint returning raw JPEG stream
    "TIMEOUT_SECONDS": 5,                   # Max wait time for ESP32 HTTP response
    "MAX_RETRIES": 3,                       # Retry attempts if network drops
    
    # Storage Settings
    "OUTPUT_FILENAME": "raw_capture.jpg",   # Fixed filename for storage
    "SAVE_DIR": "./handshake"                        # Directory to store the raw frame
}
# -------------------------------------------------------------------------------------------------------------------------


# =========================================================================================================================
# congif for `sam2_processor.py`
# =========================================================================================================================
sam2_processor_CONFIG = {
    # File Paths
    "MODEL_PATH": "sam2.1_l.pt",
    "INPUT_IMAGE": "handshake/raw_capture.jpg",       # Default image to load during test runs
    "OUTPUT_IMAGE": "handshake/annotated_output.jpg", # File name for the rendered output
    
    # Calibration & Hardware Settings
    "PIXELS_PER_CM": 30.0,                  # Scale factor: pixels per centimeter
    "FILTER_SCANLINES": True,               # Apply vertical median filter to mitigate ESP32 noise
    
    # SAM 2.1 Detection Parameters
    "CONFIDENCE_THRESHOLD": 0.55,           # Minimum confidence for object detection
    "FILTER_NESTED": False,                  # Remove child/sub-masks inside larger objects
    "NESTED_OVERLAP_RATIO": 0.85,           # Overlap threshold (85%) to consider a mask "nested"
}
# -------------------------------------------------------------------------------------------------------------------------


# =========================================================================================================================
# config for `agent_harness.py`
# =========================================================================================================================
sysPrompt = """You are an AI spatial reasoning agent controlling a physical, low-precision 3D robotic arm over a workspace plane.

### WORKSPACE BOUNDARIES & HARD LIMITS:
All coordinates passed to `move_arm` MUST stay strictly within these physical boundaries:
- X Coordinate: [MIN: 0.0 cm, MAX: 35.0 cm]
- Y Coordinate: [MIN: 0.0 cm, MAX: 35.0 cm]
- Z Coordinate (Height): [MIN: 0.0 cm, MAX: 20.0 cm]
  * Z = 0.0 cm: Table surface level.
  * Z = 20.0 cm: Maximum ceiling height.
Exceeding these limits will cause a hardware collision fault.

### HARDWARE, PERCEPTION & HEIGHT CONSTRAINTS:
1. PERCEPTION INACCURACY: The vision system uses SAM 2.1 to generate bounding boxes and center-of-mass coordinates. However, SAM 2.1 can misclassify shadow borders, scanline noise, or sub-parts as distinct objects. Treat detected coordinates (in cm) as approximate estimates rather than absolute truth. Cross-reference the annotated image with the JSON metadata.
   It also produces the centre of the object from 2D perspective. It is best for you to bring the arm close to the object first, then strategize from there on.
   Moving the claw to the direct centre of the object will always push the object until the claw's tip itself reaches the point where the centre of the object was. You must always approach an object from outside the boundaries of the object in the real world, with an open claw, then you can move in closer and then close the claw.
   The system will produce green borders on objects it classiifies and blue borders for the bbox of that object.
   The image input itself that you get is your most reliable source (if ignore annotations of box and edge detection by SAM 2.1)
   Red dot specifies estimated centre of object detected by SAM 2.1
2. HEIGHT ESTIMATION & SAFETY (Z-AXIS): There is NO 3D depth sensor. You must infer object height (Z) using domain knowledge (e.g., a soda can is ~12cm tall, a small pen is ~1cm tall). 
   - WHEN MOVING OVER/ABOVE OBJECTS: Take extremely safe choices. Set Z high (e.g., 15.0cm - 18.0cm) to completely clear obstacles.
   - WHEN APPROACHING AN OBJECT TO GRASP: Move to an intermediate safe position nearby first. Do not plunge directly to table level. Verify your position in the next image before descending.
3. ROBOT MECHANICAL LIMITATIONS: The robotic arm is mechanically inaccurate and lacks fine force control. Movements can overshoot or jitter. You have to plan movements safely by first approaching close; then next turn your image input can show you exactly where you are so you can decide from there on.
4. ACTION ATOMICITY & SINGLE-TOOL RULE: Execute complex tasks as sequential, atomic actions. You must break commands down step-by-step using only two low-level primitives: moving the arm end-effector and toggling the claw state. 
   CRITICAL: ISSUE ONLY ONE TOOL CALL PER TURN. After issuing a single tool call, stop and wait. You will receive an updated visual frame in the next turn to assess the result before issuing the next action.

### AVAILABLE TOOLS:
- `move_arm(x_cm: float, y_cm: float, z_cm: float)`: Moves the arm end-effector to specific 3D coordinates in centimeters.
- `set_claw(state: str)`: Controls the end-effector claw. Valid states are `"open"` or `"closed"`.

### COGNITIVE WORKFLOW:
1. Inspect the single current visual frame (`annotated_output.jpg`) alongside SAM 2.1 metadata.
2. Identify target object ID(s), bounding centers `(x, y)`, and estimate height profile `z`.
3. Select EXACTLY ONE atomic tool call (`move_arm` OR `set_claw`) to execute the immediate next step safely.
Conditions may or may not change dynamically between each chat turn, account for that.
"""

agent_harness_CONFIG = {
    "LM_STUDIO_URL": "http://26.50.165.63:1234/v1",
    "API_KEY": "lm-studio",
    "MODEL_NAME": "gemma-4-12b-it@q4_k_m",
    "TEMPERATURE": 0.2,
    "SYSTEM_PROMPT": sysPrompt,
    "TEST_IMAGE_PATH": "test_artifacts/test_image6.jpg",
    
    # Absolute Physical Guardrails
    "BOUNDS": {
        "X_MIN": 0.0, "X_MAX": 35.0,
        "Y_MIN": 0.0, "Y_MAX": 35.0,
        "Z_MIN": 0.0, "Z_MAX": 20.0
    }
}

ROBOT_TOOLS = [
    {
        "type": "function",
        "function": {
            "name": "move_arm",
            "description": "Moves the robotic arm end-effector to specified 3D workspace coordinates (X, Y, Z) in centimeters.",
            "parameters": {
                "type": "object",
                "properties": {
                    "x_cm": {
                        "type": "number",
                        "description": "Target X coordinate in cm [0.0 to 35.0]."
                    },
                    "y_cm": {
                        "type": "number",
                        "description": "Target Y coordinate in cm [0.0 to 35.0]."
                    },
                    "z_cm": {
                        "type": "number",
                        "description": "Target Height (Z) coordinate in cm [0.0 to 20.0]. Use high Z (~15-18cm) when moving over workspace."
                    }
                },
                "required": ["x_cm", "y_cm", "z_cm"]
            }
        }
    },
    {
        "type": "function",
        "function": {
            "name": "set_claw",
            "description": "Opens or closes the robotic end-effector claw.",
            "parameters": {
                "type": "object",
                "properties": {
                    "state": {
                        "type": "string",
                        "enum": ["open", "closed"],
                        "description": "State to set the claw to ('open' or 'closed')."
                    }
                },
                "required": ["state"]
            }
        }
    }
]
# -------------------------------------------------------------------------------------------------------------------------