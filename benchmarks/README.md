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
FrankIK                   | 0.00000    | 0.0000026    | 0.0000065    | 0.0000027    | 0.0000065   
RoboticstoolboxPython     | 0.81719    | 0.0000145    | 0.0000089    | 0.0000131    | 0.0000107   
FastIK                    | 0.00000    | 0.0000097    | 0.0000114    | 0.0000092    | 0.0000114   
PinocchioCPP              | 0.00205    | 0.0000040    | 0.0001352    | 0.0000039    | 0.0001597   
RoboticsLibrary           | 0.00127    | 0.0000050    | 0.0001733    | 0.0000048    | 0.0001957   
ManipulaPy                | 0.02817    | 0.0002529    | 0.0024451    | 0.0002555    | 0.0025822   
GenesisWorld              | 5.12017    | 0.0017409    | 0.0043826    | 0.0017325    | 0.0019170   
IKPy                      | 0.07299    | 0.0000734    | 0.0671459    | 0.0000723    | 0.0620768   
===============================================================================================
```

![benchmark bar plot](benchmark_results.svg)