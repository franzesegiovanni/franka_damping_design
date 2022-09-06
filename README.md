# FrankaAdvancedControllers
Joint and Position Impedance Controllers for Franka Emika Panda developed at TU Delft, NL. The results were presented at Human Friendly Robotics, Delft 2022. 

If this controller was useful for your research, please cite: 

``` 

'''

### How to use the controller 

Install track IK:

``` sudo apt-get install ros-<ros-distro>-trac-ik-kinematics-plugin '''

-In case you already have some versions of libfranka installed, remove them to avoid conflicts with:
```
sudo apt remove "*libfranka*"
sudo apt autoremove
```
Type the following commands to generate and build libfranka
```
cd
sudo apt install build-essential cmake git libpoco-dev libeigen3-dev
git clone --recursive https://github.com/frankaemika/libfranka
cd libfranka
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

This last comand may take several minutes. 

Now create a workspace (here called catkin_ws) and install franka_ros in it
```
cd
mkdir -p catkin_ws/src
cd catkin_ws
source /opt/ros/<ros-distro>/setup.sh
catkin_init_workspace src
git clone --recursive https://github.com/frankaemika/franka_ros src/franka_ros
rosdep install --from-paths src --ignore-src --rosdistro <ros-distro> -y --skip-keys libfranka
source devel/setup.sh
```
- Finally, install the controllers inside the folder "franka_ros" and build the code:
```
cd src/franka_ros
git clone https://github.com/franzesegiovanni/franka_advanced_controllers.git
cd ../..
source /opt/ros/<ros-distro>/setup.bash
catkin_make -DMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=~/libfranka/build
```

To run the controller:
- Switch on your Panda robot (make sure the gripper is initialized correctly), unlock its joints (and activate the FCI if necessary).
- Open a terminal, in every terminal: ```source devel/setup.bash```
- ```roslaunch franka_advanced_controllers cartesian_variable_impedance_controller.launch robot_ip:=ROBOT_IP load_gripper:=True```
- In a different terminal (don't forget to source again): ``` rosrun franka_advanced_controllers franka_gripper_online ```

To make it working with the simulator, please add: 
1. in franka_gazebo/CMakeList.txt  
in catkin_package(...)
add
```franka_advanced_controllers```

2. in franka_gazebo/CMakeList.txt  
in find_package(..)
add 
```franka_advanced_controllers```

3. in franka_gazebo/package.xml
add
```
<depend>franka_advanced_controllers</depend>
```
4. in franka_gazebo/config/sim_controllers.yaml
add
```
cartesian_impedance_advanced_controller:
  type: franka_advanced_controllers/CartesianImpedanceAdvancedController 
  arm_id: $(arg arm_id)
  joint_names:
    - $(arg arm_id)_joint1
    - $(arg arm_id)_joint2
    - $(arg arm_id)_joint3
    - $(arg arm_id)_joint4
    - $(arg arm_id)_joint5
    - $(arg arm_id)_joint6
    - $(arg arm_id)_joint7
```