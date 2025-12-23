# frankik: Fast Analytical Inverse Kinematics Python Bindings for Franka Robots

**Blazing fast, analytical Inverse Kinematics for Franka Emika Panda and FR3 robots. Lightweight Python bindings, no ROS required.**

`frankik` is a standalone Python library that implements the analytical geometric IK solver proposed by **He & Liu (2021)**. It is designed for researchers and developers who need high-performance kinematics without the overhead of ROS, MoveIt, or hardware drivers.

## Example
```python
import numpy as np
from frankik import FrankaKinematics, RobotType
kinematics = FrankaKinematics(robot_type=RobotType.FR3) # Robot.Type.PANDA also supported
q_home = np.array([0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4])
pose_home = kinematics.forward(q_home, tcp_offset=kinematics.FrankaHandTCPOffset)
q = kinematics.inverse(pose_home, tcp_offset=kinematics.FrankaHandTCPOffset, q0=q_home)
assert np.allclose(q, q_home)
print(q)
```

## Installation from source
```shell
sudo apt install libeigen3-dev
git clone https://github.com/juelg/frankik.git
cd frankik
pip install .
```
### Development installation
```shell
pip install -ve '.[dev]'
```
### Development Tools
```shell
# python code formatting
make pyformat
# python code linting
make pylint
# cpp code linting
make cpplint
# automatic stubfile generation (for changes in bindings)
make stubgen
```


## Installation from PyPI
Coming soon...


## Speed Benchmark
See the [benchmark folder](benchmark/).
The outcome is based on 1000 seeded random trials (except IKPy, it only has 100 trials).
```shell
===============================================================================================
Library                   | Init (s)   | FK-Small(s)  | IK-Small(s)  | FK-Large(s)  | IK-Large(s) 
-----------------------------------------------------------------------------------------------
FrankIK                   | 0.00000    | 0.0000026    | 0.0000064    | 0.0000027    | 0.0000063   
RoboticstoolboxPython     | 0.74061    | 0.0000144    | 0.0000090    | 0.0000134    | 0.0000107   
FastIK                    | 0.00000    | 0.0000096    | 0.0000117    | 0.0000091    | 0.0000117   
RoboticsLibrary           | 0.00152    | 0.0000048    | 0.0001728    | 0.0000048    | 0.0001943   
PinocchioCPP              | 0.00213    | 0.0000040    | 0.0001340    | 0.0000040    | 0.0001577   
ManipulaPy                | 0.02772    | 0.0002526    | 0.0024135    | 0.0002532    | 0.0025403   
GenesisWorld              | 5.14627    | 0.0017383    | 0.0043463    | 0.0017304    | 0.0018944   
IKPy                      | 0.07135    | 0.0000744    | 0.0673117    | 0.0000734    | 0.0623341   
===============================================================================================
```

