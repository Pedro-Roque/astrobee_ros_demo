# Astrobee ROS Guest Science Demo
This package contains two simple examples to interface custom ROS nodes for autonomous control: one using Python - `nodes/python_ros_node_template.py` - and one using C++ - `nodes/cpp_ros_node_template.py`. Both nodes have associated launch files under the `launch/` directory.

## Installation
To install this package, make sure that you followed the [Astrobee Installation Guide](https://nasa.github.io/astrobee/html/md_INSTALL.html) for building the code natively. 

Assuming your catkin workspace is set at `~/catkin_ws`, then
```
cd ~/catkin_ws
git clone https://github.com/Pedro-Roque/astrobee_ros_demo.git src/astrobee_ros_demo
catkin build
source devel/setup.bash
```
**NOTE:** If the package `python-catkin-tools` is not installed, replace `catkin build` with `catkin_make`.

## Run an Example
Once the installation is done, a simple example can be run with either the `cpp_template_interface.launch` or `python_template_interface.launch`. To this end. Make sure that you have the most up-to-date version of the Astrobee Simulator. 

1. Start the simulator with
```
roslaunch astrobee_ros_demo astrobee_sim.launch
```
2. Launch the template interface, for instance
```
roslaunch astrobee_ros_demo python_template_interface.launch
```
3. Make sure that Honey is not in a faulty state by overriding its state with
```
rostopic pub /honey/mgt/sys_monitor/state ff_msgs/FaultState '{state: 0}'
```
4. At this point, the template node should show "Sleeping..." as a ROS Info message. This means that the node is waiting ot be started. To start the node, call the starting service with
```
rosservice call /honey/start "data: true"
```

At this point, the template node output should show "Elapsed time: 0" as a ROS Info message, since there is no control input being generated.

## How to interface my own control algorithm?
This package serves as a template for those looking to use the Astrobee for autonomous operations. Your controllers can be included diretly on the templated code, by modifying the variables [self.u_traj](https://github.com/Pedro-Roque/astrobee_ros_demo/blob/main/nodes/python_ros_node_template.py#L298) in Python or [control_input_](https://github.com/Pedro-Roque/astrobee_ros_demo/blob/main/nodes/cpp_ros_node_template.cpp#L238) in C++. This can be accomplished either by 
1. Subscribing to another node that runs your control method that modifies these variables directly, or 
2. Modifying the template to include your control method in the class itself

A list of packages interfacing with Astrobee similarly are:
1. [ReSwarm DMPC](https://github.com/Pedro-Roque/reswarm_dmpc): unit tests and formation control for a group of Astrobee's


## Contributing
Contributions to this repository to include more functionaly are welcome, as long as they follow these guideles:
1. Python code should follow PEP-8 guidelines (exceptions are made for `E501` as long as each line is not over 120 characters, and `W503`)
2. Templates should be atomic, and properly documented (for Python, Sphinx is recommend, while for C++ Doxygen with Google-style)
3. A basic Readme should be provided on how to use the added code, or modifications to an existing Readme should be suggested

## Acknowledgements 
A special thanks goes for Brian Coltin (@bcoltin) and Rubén Ruiz (@rgarciaruiz), as well as to all the Astrobee Ops team, for their support in-view of the MPP ReSWARM test sessions and Astrobee Flight Software.
