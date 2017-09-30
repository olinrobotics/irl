### Edwin\_Moveit

## Install

1. Clone the Repositories:

```bash
cd ~/catkin_ws/src
git clone http://github.com/olinrobotics/edwin_description.git
git clone http://github.com/olinrobotics/edwin_moveit_config.git
```

2. Install the Dependencies:

```bash
rosdep update
rosdep install edwin_description
rosdep install edwin_moveit_config
```

3. Get LibSerial:

This project depends on [LibSerial](https://github.com/crayzeewulf/libserial). Navigate to the link, and follow the very simple instructions for getting libserial installed on your computer.

```bash
cd ${HOME}
git clone http://github.com/crayzeewulf/libserial.git
sudo apt-get install dh-autoreconf
```

4. Build the project:

```bash
catkin_make --pkg edwin_description
catkin_make --pkg edwin_moveit_config
```

### Running

If you have built the project immediately before this, be sure to run these in the new terminal.

#### Simulation

```bash
roslaunch edwin_moveit_config demo.launch
```

#### With Hardware

```bash
roslaunch edwin_moveit_config hardware.launch
roslaunch edwin_moveit_config real.launch
```


### TO-DOS For Documentation:

- [ ] How to setup URDF Packages
- [ ] How to configure MoveIt! For a custom robot
	- [ ] Setting up the hardware interface
	- [ ] Setting up move\_group interface`
- [ ] How to develop custon analytical IK solutions with IKFast
