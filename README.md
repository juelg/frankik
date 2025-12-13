# frankik: Fast Analytical Inverse Kinematics Python Bindings for Franka Robots

**Blazing fast, analytical Inverse Kinematics for Franka Emika Panda and FR3 robots. Lightweight Python bindings, no ROS required.**

`frankik` is a standalone Python library that implements the analytical geometric IK solver proposed by **He & Liu (2021)**. It is designed for researchers and developers who need high-performance kinematics without the overhead of ROS, MoveIt, or hardware drivers.

## Installation from source
```shell
sudo apt install libeigen3-dev
git clone https://github.com/juelg/frankik.git
cd frankik
pip install requirements.txt
pip install -ve '.[dev]' --no-build-isolation
```

## Installation from PyPI
Coming soon...


## Feature List
- [x] basic bindings from he
- [x] support for fr3
- [ ] nice python interface
- [ ] tests
- [ ] performance benchmarks

