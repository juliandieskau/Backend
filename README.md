# ECTS
Web UI and TUI are submodules

## Backend
Setup dev environment
```bash
~$ ./dev.sh
source /workspace/devel/setup.bash
source /workspace/devel/local_setup.bash
```
In this context:
launch roscore:
```bash
roscore -v
```
Launch rosbridge:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
Finally use:
```bash
catkin_make && rosrun ects ects /workspace/ects_config.json
```
