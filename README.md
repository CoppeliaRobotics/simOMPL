# OMPL (Open Motion Planning Library) plugin for V-REP

### Compiling

1. Install required packages for [v_repStubsGen](https://github.com/fferri/v_repStubsGen): see v_repStubsGen's [README](external/v_repStubsGen/README.md)
2. Compile ompl-1.1.0
2. Checkout and compile
```text
$ git clone --recursive https://github.com/fferri/v_repExtOMPL.git
$ cd v_repExtOMPL
$ cmake .
$ cmake --build .
```
(you may need to set `OMPL_INCLUDE_PATH` and `OMPL_LIB_PATH` cmake variables to the correct values)
