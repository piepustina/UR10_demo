# Universal Robot DIAG Demo
Demo for the OpenDIAG event at Sapienza University of Rome. 

This code has been tested **only** on **Ubuntu 22.04**. 

# Installation
The script requires **Python 3** and the [ur-rtde library](https://pypi.org/project/ur-rtde/), which can be installed through pip

```
pip3 install ur_rtde
```

# Setup
Connect the PC running the code with the UR robot through the ethernet cable. Make sure to assign to the PC a **static IP** address on the same subnet of the robot. The default robot subnet and address are, respectively, 
```
192.168.0.0/24
```
and
```
192.168.0.10
```

# Run the demo
Run the demo by launching **script.py**
```
python3 script.py
```
