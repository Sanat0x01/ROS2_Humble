Week 4 Documentation: Orange Cone Detection using OpenCV and YOLO

🔶 Task Overview
================\
Detect an orange cone or box inside a Gazebo world using the robot's camera. The goal is to use OpenCV (and optionally YOLO) to detect the object and potentially control robot behavior based on this detection.

📌 Steps Followed
=================\
1\. Updated the Gazebo world (`custom_world.sdf`) and added an orange cone using cylinder geometry.\
2\. Ensured robot camera is mounted and working (publishing to `/camera/image_raw`).\
3\. Wrote a ROS 2 node (`orange_detector.py`) using OpenCV that:\
- Subscribes to camera feed.\
- Converts ROS image to OpenCV format via `cv_bridge`.\
- Converts BGR to HSV and applies color threshold to detect orange regions.\
- Draws contours and finds bounding box around the detected orange object.\
4\. Tested detection accuracy in Gazebo with varying lighting and cone positions.\
5\. (Optional) Tested YOLOv8 with `yolov8n.pt` weights for object detection (for learning).

🟠 OpenCV-based Detection
=========================\
Pros:\
- Simple and fast for color-based detection.\
- Doesn't require training or labeled data.\
- Works well for objects with unique and vibrant colors like orange.

Cons:\
- Not robust to lighting changes and shadows.\
- May fail if the object's color blends with the environment.\
- Cannot distinguish between multiple similar-colored objects.

🔍 YOLOv8-based Detection
=========================\
Pros:\
- Very accurate and robust.\
- Can detect multiple objects with labels, even in complex environments.\
- Pretrained models available.

Cons:\
- Requires labeled dataset for cone if not present in default classes.\
- Heavier than OpenCV; real-time performance depends on hardware.\
- Integration with ROS requires additional setup (e.g., image conversion, topic handling).

📚 Learnings Summary
====================

-   - HSV Color Space is better for color detection than RGB.

-   - OpenCV is ideal for simple, color-based tasks; no training required.

-   - Gazebo camera needs proper resolution, pose, and FOV for effective detection.

-   - cv_bridge is essential for ROS + OpenCV image processing.

-   - YOLOv8 is powerful but needs training for custom objects like cones.

-   - Real-time detection (YOLO) on CPU may lag; optimize frame size and rate.

-   - Lighting in Gazebo significantly affects detection performance.

-   - ROS 2 nodes must match QoS and topic/message formats for smooth communication.

