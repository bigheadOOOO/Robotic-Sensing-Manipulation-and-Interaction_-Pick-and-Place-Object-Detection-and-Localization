# Coursework 1 Project README

## Description
This project contains the code for solving various tasks related to robotics, including picking and placing objects using a robotic arm.

## Authors
- **Huijuan Ma**: Contributed to the implementation of task 1, task 2 and task 3.
- **Shiqi Wang**: Contributed to the implementation of task 3.
- **Zimu Xiao**: Contributed to the implementation of task 2 and task 3.

## Build and Run Instructions
To build the package, follow these steps:
1. Use Catkin to build the package:
    ```bash
    catkin build
    ```

To run the package, execute the following commands:
1. Launch the solution:
    ```bash
    roslaunch cw1_team_4 run_solution.launch
    ```
2. Call the task service with a specific number (replace `<NUM>` with the desired task number):
    ```bash
    rosservice call /task <NUM>
    ```
## 
## Task Breakdown
### Task 1
- **Total Time**: 2 hours
  - **Huijuan Ma**: 100%
  
### Task 2
- **Total Time**: 6 hours
  - **Huijuan Ma**: 40%
  - **Zimu Xiao**: 60%

### Task 3
- **Total Time**: 50 hours
  - **Huijuan Ma**: 60%
  - **Shiqi Wang**: 30%
  - **Zimu Xiao**: 10%


## Precautions
1. To avoid gazebo error 'process has died [pid 214020, exit code -11', make sure there is no gazebo running. 
    Kill gazebo: 
    ```bash
    killall gzserver
    killall gzclient
    ```
2. If boxes get stuck in the manipulator when initialization, please re-launch it again. This is because the initialization does not finish when the hand and random boxes occasionally appear in the same place.  

3. Rendering artifacts or glitches may cause wrong operations. Sometime a box shows in two different color. THis is because rendering artifacts or glitches can occur in visualization software like Gazebo. These issues may be caused by factors such as graphics card compatibility, rendering settings, or software bugs. Try updating Gazebo to the latest version or adjusting rendering settings to see if it resolves the issue.

4. When processing task 2 and 3, please wait after calling the task service untill it get the pointcloud information. Then, the manipulater will execute picking and dropping tasks. 

5. If there is a collide, please exit the program, stop the program, and launch it again.  
