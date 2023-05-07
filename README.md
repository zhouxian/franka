
# Franka
This repo contains instructions and codes for setting up and operating the Franka Emika robot in katefgroup, CMU.

## System Setup
We use `franka_interface` and `frankapy` packages from Oliver's lab to control Franka. We neet to setup a Control PC (talks to robot) and a FrankaPy PC (sends user command).
- Control PC
    - info
        - name: `franka-control`
        - username: `katefgroup`
        - password: `katefgroup`
    - wifi
        - used phone hotspot to download CMU-SECURE certificate and connect to CMU-SECURE.
    - install ubuntu 20.04 and ROS Noetic
    - follow https://iamlab-cmu.github.io/franka-interface/
        - note that our firmware is 4.x (as shown in the GUI)
- FrankaPy PC
    - install ubuntu 20.04 and ROS Noetic
    - follow https://iamlab-cmu.github.io/frankapy/
        - create conda env: `franka`, and add it to your `.bashrc`.
        - I put `frankapy` under this repo (`franka/`) to be able to customize it. Need to clone it there first before any catkin building.
        - the original script is a bit buggy when handling the flags.
- **Notes**
    - Franka's web GUI
        - username: `franka`
        - password: `katefgroup`
    - Network configuration
        - the official setup uses 2 LAN, which is too clumsy. We put them together so that
            - we don't need two ethernet ports on Control PC
            - we can directly access Franka's GUI in a local browser, and doesn't have to ssh -X and use browser from control pc.
        - Official Network Config (For reference only)
            - LAN 1
                - Franka `172.16.0.2`
                - Control PC `172.16.0.1`
            - LAN 2
                - Control PC `192.168.1.3`
                - FrankaPy PC `192.168.1.2`
            - External
                - FrankaPy PC `128.*`
        - Our Config
            - LAN (connect all three via a switch)
                - Franka `192.168.1.1`
                - Control PC `192.168.1.2`
                - FrankaPy PC `192.168.1.3`
            - External
                - FrankaPy PC `128.*`

## Operate the robot
**Important**: whenever the robot is in operation, always be prepared to press the stop (black) button.

### Steps
1. Turn on the Control PC (`franka-control`) and the Control Box.
2. Lift up the red button and black button
3. Wait for the flashing yellow light to turn steady
3. In FrankaPy PC (`katef005`)
    - go to https://192.168.1.1 (Franka's web GUI)
    - click to unlock the system
4. Under this repo:
    ```
    bash frankapy/bash_scripts/start_control_pc.sh -u katefgroup -i franka-control -d /home/katefgroup/franka-interface -a 192.168.1.1 
    ```
    This should open 4 terminal windows.
5. (Optional) Resetting the arm:
    ```
    python frankapy/scripts/reset_arm.py
    ```
There are some other useful scripts under `franka/frankapy/scripts/`.


### Steps for shutting down
1. Shut all the terminal windows
2. Lock the robot in franka web GUI
3. Press down the red button and the black button
4. (Optional) Shut down the Control PC and the Control Box.


### Guiding mode
Press down the black button and hold the two black button at the gripper. Now you are able to drag the end-effector.

### Trouble Shooting
- Light
    - flashing yellow: booting
    - steady yellow: locked
    - blue: unclocked
    - white: guiding mode
- If the light does not light up, turn off both buttons and the control box, unplug power, and then turn on everything again.

## Camera
```
roslaunch azure_kinect_ros_driver driver_azcam_top.launch fps:=30 color_resolution:=720P
```
### Useful tools

## Teleoperation
We use SpaceMouse to teleop the franka arm. (credit to https://github.com/columbia-ai-robotics/diffusion_policy)

### Set Up
1. Install dependencies. 
```
sudo apt install libspnav-dev spacenavd
sudo systemctl start spacenavd
pip install https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz
```
2. Reboot.
3. Connect spacemouse via USB.
4. Run `python teleop/spacemouse.py`. Play with the device and you should be able to see realtime signal captured by the spacemouse.

### Operate
Run `python teleop/run.py`. Have fun!
