# LIDAR Detective

> Autonomous vehicle behaviors using object detection with LIDAR

## File Structure
* `hdr` - Header files
* `src` - Source files

## Prerequisites
1. pthread
2. udev
Prerequisites can be installed on Ubuntu using the following command:
`apt -y install libudev-dev libpthread-stubs0-dev`

## Build Instructions
1. Create new build directory in project root
2. Navigate to newly created directory
3. Run `cmake ..`
4. Run `make`
5. Built files will be found in build directory
