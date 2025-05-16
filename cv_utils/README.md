frtl_2024_cv_utils/              # Top-level ROS 2 package directory
|
├── package.xml
|
├── models/                  # Directory for model files
│   └── my_model.pt
|
├── CMakeLists.txt
├── src/  # C++ source files
├── include/  # C++ headers
|
├── launch/                      # Directory for launch files
│   └── my_launch_file.launch.py
├── yolo_classifier/          # Python module directory for yolo classifier
│   ├── __init__.py
│   ├── yolo_classifier_node.py  # Main node script
│   ├── utils.py                 # Utility functions


pip install torch torchvision opencv-python
sudo apt install ros-humble-vision-msgs
