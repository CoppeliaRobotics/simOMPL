# OMPL (Open Motion Planning Library) plugin for V-REP

### Compiling

1. Install required packages for [v_repStubsGen](https://github.com/fferri/v_repStubsGen): see v_repStubsGen's [README](external/v_repStubsGen/README.md)
2. Compile ompl in externa/ompl:
```text
$ cd external/ompl
$ mkdir build
$ cd build
$ cmake ..
$ make
```
3. Checkout and compile
```text
$ git clone --recursive https://github.com/fferri/v_repExtOMPL.git
$ make
```
