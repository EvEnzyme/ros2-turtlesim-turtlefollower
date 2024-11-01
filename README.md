## How to run this code:

### 1. Navigate to the `/src` directory in your ros2 workspace
```
cd ~/ros2_ws/src
```
### 2. clone this repository
```
git clone https://github.com/EvEnzyme/ros2-turtlesim-turtlefollower.git
```
### 3. build the package
```
colcon build --packages-select challenge_cpp
```
### 4. source and run the executable
```
source install/setup.bash
ros2 run challenge_cpp turtle_follower_no_pid
```

