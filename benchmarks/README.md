# FrankIK Benchmark
To run the benchmarks, install the dependencies below, then open the jupyter notebook [benchmarks.ipynb](benchmarks.ipynb) which contains the benchmark code.


## Pinocchio IK Installation
```shell
git clone https://github.com/RobotControlStack/robot-control-stack.git
cd robot-control-stack
sudo apt install $(cat debian_deps.txt)
pip install -r requirements.txt
pip install -ve . --no-build-isolation
```

## Robotics Library IK Installation
After rcs installation from above:
```shell
sudo apt install $(cat extensions/rcs_robotics_library/debian_deps.txt)
pip install -ve extensions/rcs_robotics_library --no-build-isolation
```

## FastIK
Install fastik_pybind following their [readme instructions](https://github.com/yijiangh/ikfast_pybind).
- Use python <= 3.10 or update the pybind version
- Install with `--no-build-isolation`



## Others Installation
```shell
pip install -r requirements.txt
# menagerie is required for genesis
git clone https://github.com/google-deepmind/mujoco_menagerie.git
```


## Outcome
Based on 1000 seeded random trials (except IKPy, it only has 100 trials).
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