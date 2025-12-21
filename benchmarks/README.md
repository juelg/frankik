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
```


## Outcome
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