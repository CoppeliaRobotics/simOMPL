Building the OMPL plugin:
=========================

What you need:
    - C++ compiler
    - Python interpreter (2.7 or greater)
    - lxml package for Python
    - An XSLT Processor, such as SAXON (needed for generate documentation)

Build steps:

    1) Generate stubs for Lua callbacks:

        > python generate_stubs.py -h callbacks.xml > stubs.h

        > python generate_stubs.py -c callbacks.xml > stubs.cpp

    2) Compile stubs.cpp and v_repExtOMPL.cpp into the dll/dylib/so library

    3) (optional) generate documentation with an XSLT processor:

        > saxon -s:callbacks.xml -a:on -o:reference.html

