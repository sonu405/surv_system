
# How to run?

### Running for the first time?
```
./setup.sh
```

### Long Time No See 

1. Source Env
```
source env.sh
```

2. Building project
```
colcon build --symlink-install
source install/setup.bash
```

# Useful Commands
1. Creating a new package
```
ros2 pkg create --build-type ament_python pkg_name --dependencies rclpy sensor_msgs cv_bridge
```


