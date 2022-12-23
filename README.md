# Main state machine
> Main state machine for TEL

## :golf: Usage
- Communication launch file
    > With STM and Arduino
```bash
roslaunch main_state_machine communication.launch
```
- Launch the service file
```bash
roslaunch main_state_machine service_on.launch
```
- Launch the main launch
```bash
roslaunch main_state_machine debug.launch
```

## :dart: Main smach object
> For controlling tha main system of the robot

- Use navigation state to moveTo specified places
    - With localization
- Arm state for catching blocks
- Camera state for finding the position of blocks
- Calibration state for calibrate the localization by calling navigation node

### Transform from camera map to navigation map
> For reduce the load of the arm
- First cut the region into 4 quadrant
```cpp
void ClassifyBlocks(std::map<char, geometry_msgs::Point> blocks);
// Cut into
std::vector<geometry_msgs::Point> CategoryBlocks[4];
```
- Vector calculation
- Add block position and arm length for car position vector
- Transform the vector to basic map
```cpp
tempMovePoint.x = ((x.x + 2) / 100) + 0;
tempMovePoint.y = (x.y / 100.) - 0.195;

MovePoint.x = 0.98 + tempMovePoint.y;
MovePoint.y = -tempMovePoint.x - 0.065;

Points.push({MovePoint, 'b'});
```


## :dart: Navigation state object
> For communicate with navigation node in package "nav_mec"

- Get path from path generator
- Set a timeout for preventing from moving too long

## :dart: Camera state object

## :dart: Arm state object

## :warning: Faced problem
1. Can't find service file in devel/include (22/oct)
    - Although the header file is exist, it still can't catkin_make the file
    - Discover : failed to include the file in devel/include
    - Need specify the address of the devel/include, so we first put absolute path in include file in CMakeList.txt. It succeeded !
    - Later we found that ```${CATKIN_DEVEL_PREFIX}``` can directly found the devel in current workspace
    - Solution : Just add ${CATKIN_DEVEL_PREFIX}/include in CMakeList.txt
2. Undefined reference for yaml-cpp
    - Just link the library in CMakeList.txt
    - Add : yaml-cpp
3. Service call always failed at start
    - Services need set up properly before client call
    - Add a service on launch file to set up service properly
4. Client call hanged in while loop
    - Set timeout to prevent the service is already failed
    - Remember to use spin
