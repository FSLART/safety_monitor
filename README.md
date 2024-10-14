# SafetyMonitor

Ros2 package responsible for monitoring the publishing rates of the nodes in the pipeline.

## Setup ["docker"]
If you have docker installed you can build the package:
```bash
docker build -t safety_monitor .
```
And run it like so:
```bash
docker run -it --net=host --ipc=host safety_monitor
```

## Setup ["native"]
If Ros2 humble is installed in your machine, just clone this package into the **/src** of your workspace. Then go to your workspace directory and run the following, so that all the dependencies are installed:
```bash
rosdep install --from-paths src --ignore-src -y -r
```
Build it:
```bash
colcon build --parallel-workers 6 --symlink-install
```
After that you need to source it:
```bash
source install/setup.bash
```
And finally you can launch it:
```bash
ros2 launch safety_monitor safetymonitor.launch.xml 
```
