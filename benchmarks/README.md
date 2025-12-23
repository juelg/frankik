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
FrankIK                   | 0.00001    | 0.0000026    | 0.0000067    | 0.0000027    | 0.0000066   
RoboticsLibrary           | 0.00109    | 0.0000053    | 0.0001740    | 0.0000051    | 0.0001963   
PinocchioCPP              | 0.00155    | 0.0000042    | 0.0001371    | 0.0000041    | 0.0001623   
RoboticstoolboxPython     | 0.74421    | 0.0000143    | 0.0000092    | 0.0000132    | 0.0000109   
ManipulaPy                | 0.02860    | 0.0002592    | 0.0025176    | 0.0002584    | 0.0026479   
GenesisWorld              | 4.87590    | 0.0017312    | 0.0043803    | 0.0017249    | 0.0019003   
IKPy                      | 0.07561    | 0.0000749    | 0.0671296    | 0.0000749    | 0.0623110   
===============================================================================================
```