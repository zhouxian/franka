

```
bash frankapy/bash_scripts/start_control_pc.sh -u katefgroup -i franka-control -d /home/katefgroup/franka-interface -a 192.168.1.1 
```

# Teleoperation
We use SpaceMouse to teleop the franka arm.

Install dependencies. (credit to https://github.com/columbia-ai-robotics/diffusion_policy)
```
sudo apt install libspnav-dev spacenavd; sudo systemctl start spacenavd
pip install https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz
```
Connect spacemouse via USB.
Run `python realworld/spacemouse.py` to verify.