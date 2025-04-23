# Simultaneous System Identification and Model Predictive Control with No Dynamic Regret

This repo contains the code associated to our paper [Simultaneous System Identification and Model Predictive Control with No Dynamic Regret](https://arxiv.org/abs/2407.04143). In the paper, we propose the algorithm SSI-MPC, for simultaneous model learning of unknown dyanmics/disturbance in a self-supervised manner using only the data collected on-the-go (i.e., without offline training), and model predictive control. 

We demonstrate the algorithm in reference trajectory tracking of a quadrotor in sumulation under drag effects, and in real-world under wind and ground effects.

![image](https://github.com/UM-iRaL/SSI-MPC/blob/main/img/Hardware.png)

### Citing

If you use this code in an academic context, please cite the following publication:

[Simultaneous System Identification and Model Predictive Control with No Dynamic Regret](https://arxiv.org/abs/2407.04143)

```
@article{zhou2024simultaneous,
  title={Simultaneous System Identification and Model Predictive Control with No Dynamic Regret},
  author={Zhou, Hongyu and Tzoumas, Vasileios},
  journal={IEEE Transactions on Robotics (T-RO)},
  year={2025}
}
```

We utilize Koopman Operator in the same online learning and control framework in the following paper:
```
@article{zhou2024koopman,
  title={No-Regret Model Predictive Control with Online Learning of Koopman Operators},
  author={Zhou, Hongyu and Tzoumas, Vasileios},
  booktitle={2025 American Control Conference (ACC)},
  year={2025}
}
```


## Installation

### Environments

The branch is tested with Ubuntu 20.04, Python 3.8 and ROS Noetic.

**Recommended**: 
Create a Python virtual environment for this package:
```
sudo pip3 install virtualenv
cd <PATH_TO_VENV_DIRECTORY>
virtualenv mpc_venv --python=/usr/bin/python3.8
source mpc_venv/bin/activate
```

**Installation of `acados` and its Python interface**: 
- Build and install the Acados C library. Please follow their [installation guide](https://docs.acados.org/installation/index.html). 
- Install the Python interface. Steps also provided in the [Python interface wiki](https://docs.acados.org/interfaces/index.html#installation).

**Additional Requirements**:
The code that runs on [RotorS](https://github.com/ethz-asl/rotors_simulator) builds on [rpg_quadrotor_control](https://github.com/uzh-rpg/rpg_quadrotor_control). Create a catkin workspace following these [installation instructions](https://github.com/uzh-rpg/rpg_quadrotor_control/wiki/Installation-Guide). 


### Initial setup

0. Source Python virtual environment if created:
   ```
   source <PATH_TO_VENV_DIRECTORY>/mpc_venv/bin/activate
   ```

1. Clone this repository into your catkin workspace where you build rpg_quadrotor_control:
   ```
   cd <CATKIN_WS_DIR>/src
   git clone git@github.com:UM-iRaL/SSI-MPC.git
   ```
   
2. Install the rest of required Python libraries:
   ```
   cd SSI_MPC
   python setup.py install
   ```
 
3. Build the catkin workspace:
   ```
   cd <CATKIN_WS_DIR>
   catkin build
   source devel/setup.bash
   ```


## Running simulation

### Running RotorS simulator
Run an empty world simulation, and enable the command override function. 
Due to the increased computational demand of running the Gazebo simulator in parallel to the controller, the following launchfile runs the gazebo simulator at 50% speed:
```
roslaunch ros_mpc quadrotor_empty_world.launch enable_command_feedthrough:=True
```

Then, click `Connect` and `Arm Bridge` on the RPG Quadrotor GUI.


### Running MPC controller
In a new ternimal, source Python virtual environment if created:
```
source <PATH_TO_VENV_DIRECTORY>/mpc_venv/bin/activate
```

Then, make sure to add to your Python path the main directory of this package:
```
export PYTHONPATH=$PYTHONPATH:<CATKIN_WS_DIR>/src/SSI_MPC/ros_mpc
```

Also, source the environment:
```
cd <CATKIN_WS_DIR>
source devel/setup.bash
```

#### Run Nominal MPC with circular trajectory
To run nominal MPC:
```
roslaunch ros_mpc mpc_wrapper.launch plot:=true n_rf:=0
```

The resulting RMSE is around 0.22m.

#### Run SSI-MPC with circular trajectory
To run SSI-MPC:
```
roslaunch ros_mpc mpc_wrapper.launch plot:=true n_rf:=50
```

The resulting RMSE is around 0.05m.

#### Run GP-MPC with circular trajectory
To run GP-MPC, first run the following script to execute several random trajectories on the Gazebo simulator and compile a dataset of the measured errors.
```
roslaunch ros_mpc gp_mpc_wrapper.launch recording:=True dataset_name:=gazebo_dataset environment:=gazebo flight_mode:=random n_seeds:=10
```

Leave the script running until it outputs the following message: 
```
[INFO] [1612101145.957326, 230.510000]: No more references will be received
```

Train a new GP model:
```
python src/model_fitting/gp_fitting.py --n_points 20 --model_name gazebo_sim_gp --x 7 --y 7
python src/model_fitting/gp_fitting.py --n_points 20 --model_name gazebo_sim_gp --x 8 --y 8
python src/model_fitting/gp_fitting.py --n_points 20 --model_name gazebo_sim_gp --x 9 --y 9
```

The models will be saved under the directory `ros_mpc/results/model_fitting/<git_hash>/`.

Then, run GP-MPC:
```
roslaunch ros_mpc gp_mpc_wrapper.launch environment:=gazebo plot:=True model_version:=<git_hash> model_name:=gazebo_sim_gp model_type:=gp
```

The resulting RMSE is around 0.11m.


#### Final notes
- **Trajectory types**

   The user can also run a `wrapped circle`, `lemniscate`, and `wrapped lemniscate` trajectories by configuring the `mpc_wrapper.launch` or `gp_mpc_wrapper.launch`:
   ```
   <rosparam file="$(find ros_mpc)/config/traj_circle.yaml"/>
   <!-- <rosparam file="$(find ros_mpc)/config/traj_wraped_circle.yaml"/> -->
   <!-- <rosparam file="$(find ros_mpc)/config/traj_lemniscate.yaml"/> -->
   <!-- <rosparam file="$(find ros_mpc)/config/traj_wraped_lemniscate.yaml"/> -->
   ``` 

- **Thrust level control** 
  
   Even though the MPC model operates at thrust level control, currently the ROS node sends total thrust + body rate 
   commands. To switch to single thrust level control, edit the following line from the [MPC ROS interface file](ros_mpc/src/quad_mpc/create_ros_mpc.py):
  
   From (body rate control):
   ```
   next_control.control_mode = 2
   ```
   Instead switch to (thrust level control):
   ```
   next_control.control_mode = 4
   ```
