
# Franka
This repo contains instructions and codes for setting up and using the Franka Emika robot in katefgroup, CMU.

## System Setup
We use `franka_interface` and `frankapy` packages from oliver's lab to control Franka. We neet to setup a Control PC (talks to robot) and a FrankaPy PC (sends user command).
- Control PC
    - info
        - name: `franka-control`
        - username: `katefgroup`
        - password: `katefgroup`
    - wifi
        - used phone hotspot to download CMU-SECURE certificate and connect to CMU-SECURE.
    - install ubuntu 20.04 and ROS Noetic
    - follow https://iamlab-cmu.github.io/franka-interface/
        - note that our firmware is 4.x (as seen in the GUI)
- FrankaPy PC
    - install ubuntu 20.04 and ROS Noetic
    - follow https://iamlab-cmu.github.io/frankapy/
        - use conda env: `franka`
        - I put `frankapy` under this repo (`franka/`) to be able to customize it. Need to clone it there first before any catkin building.
        - the original script is a bit buggy when handling the flags.
- **Notes**
    - franka webpage GUI
        - username: `franka`
        - password: `katefgroup`
    - Network configuration
        - the official setup uses 2 LAN, which is too clumsy. We put them together so that
            - we don't need two ethernet ports on Control PC
            - we can directly go to Franka's address in browser, and doesn't have to ssh -X and use browser from control pc.
        - Official Network Config (For reference only)
            - LAN 1
                Franka `172.16.0.2`
                Control PC `172.16.0.1`
            - LAN 2
                Control PC `192.168.1.3`
                FrankaPy PC `192.168.1.2`
            - External
                FrankaPy PC `128.*`
        - Our Config
            - LAN (connect all three via a switch)
                Franka `192.168.1.1`
                Control PC `192.168.1.2`
                FrankaPy PC `192.168.1.3`
            - External
                FrankaPy PC `128.*`

## Run the robot


```
bash frankapy/bash_scripts/start_control_pc.sh -u katefgroup -i franka-control -d /home/katefgroup/franka-interface -a 192.168.1.1 
```

## Teleoperation
We use SpaceMouse to teleop the franka arm.

Install dependencies. (credit to https://github.com/columbia-ai-robotics/diffusion_policy)
```
sudo apt install libspnav-dev spacenavd; sudo systemctl start spacenavd
pip install https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz
```
Connect spacemouse via USB.
Run `python realworld/spacemouse.py` to verify.