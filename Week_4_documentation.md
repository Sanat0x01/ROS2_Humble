# 🟠 Week 4: Orange Cone Detection using OpenCV and YOLO

## 🔶 Task Overview

Detect an orange cone or box inside a Gazebo world using the robot's camera. The goal is to use OpenCV (and optionally YOLOv8) to detect the object and control robot behavior based on the detection.

---

## 📌 Steps Followed

1. **Updated the Gazebo world**  
   - Modified `worlds/custom_world.sdf` to add an orange cone using cylinder geometry.

2. **Configured Robot Camera**  
   - Verified that the robot’s camera is properly mounted in the URDF/XACRO.  
   - Confirmed image stream is available on `/camera/image_raw`.

3. **Implemented OpenCV Detection Node** (`orange_detector.py`)  
   - Subscribes to the camera feed.  
   - Converts ROS image to OpenCV format using `cv_bridge`.  
   - Converts image from BGR to HSV color space.  
   - Applies orange color mask using HSV thresholds.  
   - Finds contours and draws a bounding box around detected orange region.

4. **Tested Detection in Simulation**  
   - Moved cone to various positions and lighting setups in Gazebo.  
   - Verified bounding box accuracy and detection responsiveness.

5. **(Optional)**: Integrated YOLOv8 (with `yolov8n.pt`)  
   - Used pretrained weights to test object detection pipeline.  
   - Compared accuracy and speed against OpenCV.

---

## 🟠 OpenCV-based Detection

### ✅ Pros

- Fast and lightweight.
- No training or dataset required.
- Easy to implement using HSV color filtering.
- Works well for vibrant and unique colors like orange.

### ❌ Cons

- Sensitive to lighting and shadows.
- Can produce false positives if background has similar colors.
- Cannot distinguish between multiple similar-colored objects.

---

## 🔍 YOLOv8-based Detection

### ✅ Pros

- High accuracy and robustness.
- Can detect and classify multiple object types.
- Pretrained models (e.g., `yolov8n.pt`) available.

### ❌ Cons

- May require training for cones if not present in default classes.
- Heavier compute load; may lag on CPU.
- More complex ROS integration (requires topic remapping, preprocessing, etc.).

---

## 📚 Learnings Summary

- HSV color space is better suited for color detection than RGB.
- `cv_bridge` is critical for converting ROS images into OpenCV format.
- OpenCV is best for lightweight, single-color detection tasks.
- YOLO is better for multi-class, high-accuracy needs.
- Simulation lighting and camera pose/FOV greatly affect detection.
- YOLO inference should be optimized for CPU/GPU to ensure real-time performance.
- ROS 2 nodes must be configured with compatible QoS and topic names for stable operation.
- Always test detection under different conditions to ensure robustness.

---

## 📂 Related Files

- `launch/bringup_launch.py` – Launches Gazebo + robot + camera + detection node.
- `orange_detector.py` – Python ROS 2 node for OpenCV-based detection.
- `worlds/custom_world.sdf` – Custom Gazebo world with cone.

---

## ✅ Future Improvements

- Add YOLO training pipeline for custom cone dataset.
- Combine color + shape filtering for better OpenCV performance.
- Deploy detection-based navigation (e.g., stop at cone, avoid, etc.).

