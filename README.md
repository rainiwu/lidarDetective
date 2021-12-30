# LIDAR Detective

> Autonomous vehicle behaviors using object detection with LIDAR

Take a look at a more detailed article here about this project: https://guitar.ucsd.edu/maeece148/index.php/2021FallTeam2

LIDAR Detective is a reinforcement learning system for LiDAR-based autonomous navigation built from scratch for the CUDA-enabled Jetson Nano SBC. By utilizing the CUDA cores found on the Jetson Nano, adaptive autonomous behavior can be enabled with a real-time 100Hz steering control loop. The result is a blazingly fast intelligent control system enabling obstacle avoidance, target following, and other behaviors using a single low-cost LiDAR sensor. 

## File Structure
* `hdr` - Header files
* `src` - Source files

## Prerequisites
1. pthread
2. udev
3. CUDA
4. i2c

Prerequisites can be installed on Ubuntu using the following command:
`apt -y install libudev-dev libpthread-stubs0-dev nvidia-cuda-toolkit libi2c-dev i2c-tools`

## Build Instructions
1. Create new build directory in project root
2. Navigate to newly created directory
3. Run `cmake ..`
4. Run `make`
5. Built files will be found in build directory
