# Cartesian Motions for Robotic Arm

## How to set it up
I've used the Franka Emika Panda robot that comes along with [MoveIt2 setup](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html). Therefore, we'll launch the simulation first...

```bash
cd ~/ws_moveit2
source install/setup.bash
ros2 launch moveit2_tutorials demo.launch.py
```

## Launching the controller
The controller has 4 pre-defined goal positions (within workspace) to show motion of the end effector moving in a 2D cartesian plane. The planner used in the controller is the `PILZ industrial motion planner` with the `LIN motion command`. The planner uses the Cartesian limits to generate a trapezoidal velocity profile in Cartesian space. 

```bash
# in another terminal
cd ~/ws_moveit2/src
source install/setup.bash

git clone git@github.com:kalashjain23/arm_10x.git
cd ~/ws_moveit2
colcon build --packages-select arm_controller_10x

ros2 run arm_controller_10x cartesian_constant_velocity_controller
```

## View the plot
The csv file consisting of the joint-velocities should be saved on the completion of the motions. To plot them:
```bash
# if uv is installed, install dependencies with
uv pip install -r uv.lock

# else
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

python3 src/arm_controller_10x/plot.py
```
