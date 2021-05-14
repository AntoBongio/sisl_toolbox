# SISL Toolbox
The **SISL Toolbox** library is built on top of *SISL* library.

## Documentation

## Features


## Dependencies
Before building the repository you will have to install the following dependencies:
* Eigen 3: `sudo apt install libeigen3-dev`
* SISL: `git clone https://github.com/SINTEF-Geometry/SISL.git`
* Geographic Library 'sudo apt-get install -y libgeographic-dev'


## Building and installing

The build tool used for this project is CMake. To build and install the project navigate to the root of the cloned repo and execute the following commands:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install

The CMakeLists.txt provides also an additional BUILD_TESTS option, which by default is set to OFF. If you want to build also the tests just run:

    $ cmake .. -DBUILD_TESTS=ON

### Mantainer

* <antoniobongio@gmail.com>
