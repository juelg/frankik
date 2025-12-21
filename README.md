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
```shell
===============================================================================================
Library                   | Init (s)   | FK-Small(s)  | IK-Small(s)  | FK-Large(s)  | IK-Large(s) 
-----------------------------------------------------------------------------------------------
FrankIK                   | 0.00000    | 0.0000031    | 0.0000077    | 0.0000032    | 0.0000082   
RoboticsLibrary           | 0.44945    | 0.0000061    | 0.0001870    | 0.0000059    | 0.0002118   
PinocchioCPP              | 0.00218    | 0.0000049    | 0.0001376    | 0.0000048    | 0.0001629   
ManipulaPy                | 0.75502    | 0.0002576    | 0.0025213    | 0.0002612    | 0.0026645   
===============================================================================================
```

