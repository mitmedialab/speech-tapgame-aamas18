# "Tap Game" Demo 
---------------------

Speech-based Human-robot Interaction game implemented in Spaulding et al.'s paper "A Social Robot System for Modeling Children's Word Pronunciation" (AAMAS 2018). Includes 


Basics
--------------

Project Setup
---------------

### System Dependencies

This project was built and tested on Ubuntu 16.04, with a full desktop installation of ROS Kinetic

We recommend setting up a new virtualenv for this project, with Python 3.5.2 as the Python version

**All Python code should be Python 3 compatible.**

### Set up / activate virtual env
If you haven't done this yet, set up a python3 virtual env with: 

```bashrc
mkdir -p ~/python-virtualenvs
virtualenv -p $(which python3) ~/python-virtualenvs/tap-game-py3 --system-site-packages
```

YOU ONLY NEED TO DO THIS ONCE!

All subsequent times, you just need to activate the virtualenv with 

```bashrc
source ~/python-virtualenvs/tap-game-py3/bin/activate
```

### Install the project's non-python development and runtime requirements::	
	
#### UBUNTU 14.04 System dependencies
	
```bashrc
$ sudo apt-get install portaudio19-dev
$ sudo apt-get install xdotool
$ sudo apt-get install wmctrl
$ sudo apt-get install ros-kinetic-rosbridge-server
$ sudo apt-get install python3-tk
```
	
	# frequently needed for ROS-related code to work)
```bashrc
$ pip install rospkg
$ pip install catkin_pkg
$ pip install pymongo
$ pip install twisted
```
	
	# External catkin repos necessary for message passing and other functions
	# Clone these to ~/catkin_ws/src, then run "catkin_make" from ~/catkin_ws
	- [usb_cam](https://github.com/bosch-ros-pkg/usb_cam.git)
	- https://github.com/mitmedialab/r1d1_msgs.git (to communicate w Tega)
	- https://github.com/mitmedialab/jibo_msgs.git (to communicate w Jibo)


### Install the project's python development and runtime requirements:

    $ make init

### Configure SpeechAce API Key

- Add your SpeechAce API key and user ID to GameUtils/GlobalSettings.py
	
**Project setup is now complete!**


### Running Tap Game
-------------


Preparing Devices
------------

Install ADB
`sudo apt-get install android-tools-adb`

- Tega Android SmartPhone
0. Follow instructions to install Tega app [here](https://github.com/mitmedialab/Interaction_FSM/tree/huawei2018)

- Tap Game App
0. Disconnect Tega phone and connect a tablet that will be the game surface
1. Install tap game APK inside apk/ 
```bashrc
$ cd ../
$ adb install apk/tap_game.apk
```

- USB Microphone
0. Connect MXL AC-404 USB microphone to computer. If using a VM, make sure it connects to the Linux environment. You can confirm connection by running `$ lsusb` and seeing if a Texas Instruments device is connected.


Run Demo
------------
0. Activate the tap game virtualenv
```bashrc
source ~/python-virtualenvs/tap-game-py3/bin/activate
```
1. Start `roscore` and `rosbridge` *# do not switch to a different terminal window until done loading*
```bashrc
$ ./scripts/startROS.sh 
```

2. Start demo
```bashrc
./scripts/start_tap_study.sh p00 sam experiment no-record
```

3. Connect Tega and Tablet to ROS.
  - Note your `ROSMASTER_URI` from the host computer (where you started `roscore`. E.g., http://192.168.1.100:11311)
  - Start ROSVirtualTega App on the phone. (**input `ROSMASTER_URI`**). 
  - Start PRG Tap Game Demo app on the tablet (**only input `ROS_IP`, e.g., 192.168.1.100**).
  - Press Connect, and Button


Troubleshooting:
------------------
- **Problem**: `"cannot find module 'clev"`
	- **Solution**: Navigate to the install site of the `weighted-levenshtein` package, edit __init__.py, and remove
- **Problem**: `"cannot find module 'em'"`
	- **Solution**: `pip install empy`
