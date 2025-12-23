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
FrankIK                   | 0.00001    | 0.0000026    | 0.0000067    | 0.0000027    | 0.0000066   
RoboticstoolboxPython     | 0.74421    | 0.0000143    | 0.0000092    | 0.0000132    | 0.0000109   
RoboticsLibrary           | 0.00109    | 0.0000053    | 0.0001740    | 0.0000051    | 0.0001963   
PinocchioCPP              | 0.00155    | 0.0000042    | 0.0001371    | 0.0000041    | 0.0001623   
ManipulaPy                | 0.02860    | 0.0002592    | 0.0025176    | 0.0002584    | 0.0026479   
GenesisWorld              | 4.87590    | 0.0017312    | 0.0043803    | 0.0017249    | 0.0019003   
IKPy                      | 0.07561    | 0.0000749    | 0.0671296    | 0.0000749    | 0.0623110   
===============================================================================================
```

