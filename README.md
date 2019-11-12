# OMPL (Open Motion Planning Library) plugin for CoppeliaSim

### Compiling

1. Install required packages for [libPlugin](https://github.com/CoppeliaRobotics/libPlugin): see libPlugin's [README](external/libPlugin/README.md)
2. Compile ompl-1.1.0
2. Checkout and compile
```text
$ git clone --recursive https://github.com/CoppeliaRobotics/simExtOMPL.git
$ cd simExtOMPL
$ cmake .
$ cmake --build .
```
You may need to set `OMPL_INCLUDE_PATH` and `OMPL_LIB_PATH` cmake variables to the correct values, e.g.:
```text
$ cmake -G "MinGW Makefiles" -DOMPL_INCLUDE_PATH=c:\local\ompl-1.1.0-Source\src -DOMPL_LIB_PATH=c:\local\ompl-1.1.0-Source\build\lib .
$ cmake --build .
```
