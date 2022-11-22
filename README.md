# Main state machine
> Main state machine for TEL

## Main smach object
> For controlling tha main system of the robot

- Use navigation state to moveTo specified places
    - With localization
- Arm state for catching blocks
- Camera state for finding the position of blocks
- Calibration state for calibrate the localization by calling navigation node

## Navigation state object
> For communicate with navigation node in package "nav_mec"

- Get path from path generator
- Set a timeout for preventing from moving too long

## Camera state object

## Arm state object

## :warning: Faced problem
1. Can't find service file in devel/include (22/oct)
    - Although the header file is exist, it still can't catkin_make the file
    - Discover : failed to include the file in devel/include
    - Need specify the address of the devel/include, so we first put absolute path in include file in CMakeList.txt. It succeeded !
    - Later we found that ```${catkin_INCLUDE_DIRS}``` can directly found the devel in current workspace
    - Solution : Just add ${catkin_INCLUDE_DIRS}/include in CMakeList.txt
2. Undefined reference for yaml-cpp
    - Just link the library in CMakeList.txt
    - Add : yaml-cpp
3. Service call always failed at start
    - Services need set up properly before client call
    - Add a service on launch file to set up service properly
4. Client call hanged in while loop
    - Set timeout to prevent the service is already failed
    - Remember to use spin