# SISL Toolbox
The **SISL Toolbox** library is built on top of *SISL* library.

## Features
Its major features are:

1. Definition of a class **Curve** to wrap the SISL (SINTEF Spline Library) routines of the library. Moreover, this class adds an in meters curve parametrization, internally applying a conversion from meters parametrization to Sisl parametrization.

2. Definition of a **Path** class to build a complex path starting from the Curve objects. The path is parametrized in meters with abscissa in the interval [0, pathLength].

3. Definition of a **PathFactory** class implementing the logic to automatically build different paths: [Polygonal Chain, Polygon, Hippodrome, Spiral, Race Track, Serpentine]. Each Method returns a shared_ptr **Path**.
## Dependencies
Before building the repository you will have to install the following dependencies:
* Eigen 3: `https://gitlab.com/libeigen/eigen/-/releases/3.4.0`
* SISL: `git clone https://github.com/SINTEF-Geometry/SISL.git`

## Building and installing

To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install

The CMakeLists.txt provides also an additional BUILD_TESTS option, which by default is set to OFF. If you want to build also the tests just run:

    $ cmake .. -DBUILD_TESTS=ON

### Mantainer

* <antoniobongio@gmail.com>
