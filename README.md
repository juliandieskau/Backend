# ECTS - Extensible 
Web UI and TUI are submodules

## Setup development environment
### Backend
Just launch the container
```bash
~$ ./dev.sh
```
In the container shell:
Build and run the backend with roscore and rosbridge:
```bash
catkin_make && roslaunch ects ects.launch config:=/workspace/ects_config.json
```

Alternatively as separate commands:
Launch roscore:
```bash
roscore -v
```
Launch rosbridge:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
Finally use this to build and run the backend:
```bash
catkin_make && rosrun ects ects /workspace/ects_config.json
```
Run Backend Unit Tests:
```
catkin_make run_tests
```
Run clang-tidy:
```
rm -rf build
catkin_make --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1
find -name '*pp' | grep -v nlohmann | grep './src/' | xargs clang-tidy -p build/ -checks='<YOUR CHECKS HERE>' --fix
```
On the host, use `./lint.sh` before committing.

## Testing
Publish a message
```
rostopic pub /ects/retransmit ects/ForceRetransmit '{header: auto, reload_all: false, topic: "/ects/sometopic"}'
```
Call a service
```
rosservice call /ects/ects_status
```
