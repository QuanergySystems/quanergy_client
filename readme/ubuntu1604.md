# Ubuntu 16.04 LTS Build of QuanergyClient

## Install Prerequisites
The following will install prerequisites including PCL 

```
sudo apt-get install cmake git build-essential libboost-all-dev libpcl-dev libproj-dev libvtk6-dev
```
## Fix known issue with PCL in Ubuntu 16.04
Installing the following package eliminates this compile time warning:
'<command-line>:0:15: warning: ISO C++11 requires whitespace after the macro name'
See https://github.com/PointCloudLibrary/pcl/issues/1406 for more details.
```
sudo apt-get install libusb-1.0-0.dev
```
## Build Instructions
Clone the SDK repository

```
mkdir ~/QuanergySystems
cd ~/QuanergySystems
git clone https://github.com/QuanergySystems/quanergy_client.git
```
Build quanergy_client code and visualizer application

```
cd quanergy_client
mkdir build
cd build
cmake ..
make
```

## Troubleshooting
If during the build you see the error message:
```
cannot find -lvtkproj4
```
then add
```
# Workaround required due to a bug in VTK6 for Ubuntu 16.04
# See https://bugs.launchpad.net/ubuntu/+source/vtk6/+bug/1573234
if (NOT "${PCL_LIBRARIES}" STREQUAL "")
 list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
endif()
```
to CMakeLists.txt before the line
```
file(GLOB_RECURSE project_HEADERS
   "*.h"
   "*.hpp"
)
```
within CMakeLists.txt and try to build again.

This error is most likely due to an outdated version of one of quanergy_client's dependencies being installed onto the system. More recent versions no longer need the workaround for a successful build.

## Testing build
To test, run the visualizer application and follow the usage instructions

```
./visualizer --help
```
## Documentation
For documentation, run the following and then open doc/index.html in any browser.

```
sudo apt-get install doxygen
cd ~/QuanergySystems/quanergy_client/build
cmake ..
make doc
```
