# OMPL (Open Motion Planning Library) plugin for CoppeliaSim

### Requirements

- ompl-1.5.0 with the following modification:

in file `src/ompl/base/spaces/src/SO3StateSpace.cpp` change:

```cpp
static const double MAX_QUATERNION_NORM_ERROR = 1e-9;
```

to:

```cpp
static const double MAX_QUATERNION_NORM_ERROR = 1e-6;
```

### Compiling

1. Install required packages for [libPlugin](https://github.com/CoppeliaRobotics/libPlugin): see libPlugin's [README](external/libPlugin/README.md)
2. Compile OMPL library
2. Checkout and compile
```text
$ git clone --recursive https://github.com/CoppeliaRobotics/simExtOMPL.git
$ cd simExtOMPL
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
```
You may need to set `OMPL_INCLUDE_PATH` and `OMPL_LIB_PATH` cmake variables to the correct values, e.g.:
```text
$ cmake -G "MinGW Makefiles" -DOMPL_INCLUDE_PATH=c:\local\ompl-1.5.0\src -DOMPL_LIB_PATH=c:\local\ompl-1.5.0\build\lib .
$ cmake --build .
```
