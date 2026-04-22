# 857_Final_Project_Code

This repository contains a ROS 2 wayfinding and object-detection project plus a
standalone `pygame` robot face demo.

## Main Features

- A standalone animated robot face in `robot_face.py`
- A ROS 2 face display node that shows expressions and messages
- A terminal-based wayfinding input node
- A SEIC directory lookup workflow backed by `data/seic_public_directory.xlsx`
- A YOLO object-detection node using `yolov8n.pt`
- A simple web stream for the annotated YOLO output

## Run The Standalone Face Demo

```bash
cd /home/nvidia/Milton_Final_Project
python robot_face.py
```

## Run The ROS 2 Face And YOLO System

```bash
cd /home/nvidia/Milton_Final_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch milton_final_project face_yolo_launch.py
```

## Run The Terminal Wayfinding Input Node

```bash
cd /home/nvidia/Milton_Final_Project
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run milton_final_project wayfinding_input_node
```

## Python Requirements

The pip-based Python dependencies are listed in `requirements.txt`.

ROS-specific packages such as `rclpy`, `cv_bridge`, `sensor_msgs`, `std_msgs`,
`launch`, and `launch_ros` are normally installed through ROS 2 rather than
through `pip`.

## Folder Organization

### Repository Root

- `README.md`: project overview, run commands, and folder guide
- `requirements.txt`: pip-installable Python dependencies used by the project
- `robot_face.py`: standalone non-ROS `pygame` face demo
- `yolov8n.pt`: YOLO model weights used by the detection node

### Assets And Data

- `assets/`: image assets used by the robot face
- `assets/kaia_face/`: grouped facial-expression graphics
- `data/`: project data files
- `data/seic_public_directory.xlsx`: SEIC public directory spreadsheet used for room and person lookup

### ROS 2 Source Package

- `src/milton_final_project/`: ROS 2 Python package root
- `src/milton_final_project/package.xml`: ROS 2 package manifest and runtime dependencies
- `src/milton_final_project/setup.py`: Python packaging and install configuration
- `src/milton_final_project/setup.cfg`: setuptools configuration
- `src/milton_final_project/resource/`: ROS package resource markers
- `src/milton_final_project/launch/`: ROS 2 launch files
- `src/milton_final_project/test/`: package test files

### Main Python Modules

- `src/milton_final_project/milton_final_project/__init__.py`: package initializer
- `src/milton_final_project/milton_final_project/face_display_node.py`: ROS 2 node that renders the robot face, expressions, and text messages on screen
- `src/milton_final_project/milton_final_project/wayfinding_input_node.py`: ROS 2 node that reads terminal input and publishes face messages and expressions
- `src/milton_final_project/milton_final_project/seic_directory.py`: loads the SEIC spreadsheet and finds the best room or person match
- `src/milton_final_project/milton_final_project/robot_interpreter.py`: extracts a destination target from free-form user input
- `src/milton_final_project/milton_final_project/robot_assistant.py`: helper for short robot-style responses using an Ollama model when available
- `src/milton_final_project/milton_final_project/yolo_node.py`: ROS 2 node that runs YOLO on camera images and publishes annotated detections
- `src/milton_final_project/milton_final_project/yolo_web_stream.py`: ROS 2 node that serves the annotated YOLO stream over HTTP

### Launch Files

- `src/milton_final_project/launch/yolo_launch.py`: launches the YOLO pipeline
- `src/milton_final_project/launch/face_yolo_launch.py`: launches the face display together with the YOLO pipeline

### Generated Build Output

- `build/`: colcon build artifacts generated during compilation
- `install/`: installed ROS 2 package output created by `colcon build`
- `log/`: colcon build logs and run logs

These generated folders are useful for running the project locally after a build,
but they are not the primary source files you edit during development.
