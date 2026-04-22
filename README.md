# 857_Final_Project_Code

This repository includes the original ROS/YOLO project plus a standalone `pygame`
robot face demo in `robot_face.py`.

Run the face demo with:

```bash
python robot_face.py
```

The face assets used by the demo live in `assets/kaia_face/`.

To run the ROS face monitor together with YOLO:

```bash
cd /home/nvidia/Milton_Final_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch milton_final_project face_yolo_launch.py
```

To start the terminal input node for destination prompts:

```bash
cd /home/nvidia/Milton_Final_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run milton_final_project wayfinding_input_node
```
