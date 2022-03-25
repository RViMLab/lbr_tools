# LBR Tools
Tools for the KUKA LBR iiwa/med 7 R800 robots.

## Build
```shell
mkdir -p lbr_tools_ws/src && cd lbr_tools_ws && \
wget https://raw.githubusercontent.com/RViMLab/lbr_tools/main/lbr_tools/repos.yml -P src && \
vcs import src < src/repos.yml && \
colcon build
```

## Launch
```shell
source install/setup.bash && \
ros2 launch lbr_tools view_robot.launch.py description_package:=da_vinci_endoscope_description  # or any other description_package
```
