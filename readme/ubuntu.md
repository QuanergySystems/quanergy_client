# Ubuntu 18.04/20.04/22.04 LTS Builds of QuanergyClient
These instructions work for Ubuntu 18.04 Bionic, 20.04 Focal, and 22.04 Jammy.

## Install Prerequisites and Dependencies
The following will install prerequisites and dependencies including PCL. 

```
sudo apt install git cmake g++ libboost-all-dev libpcl-dev
```
## Build Instructions
Clone the SDK repository.

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

## Testing build
On Ubuntu 22.04, as of June 2022, there is a bug in VTK that prevents the visualizer application from running as expected. The help command will print as expected. The application `./dynamic_connection` can be used to verify point clouds are received but does not allow visualization. Hopefully, this issue will get fixed in the stable repos.
```
cd ~/QuanergySystems/quanergy_client/build/
./visualizer --help
./visualizer --host <IP Address of Sensor>
```

Cloud Viewer window should appear and the point cloud will be displayed when the sensor is up to speed. In some cases, the point cloud won't appear until the interface is interacted with.

## Documentation
For documentation, run the following and then open doc/html/index.html in any browser.

```
sudo apt install doxygen graphviz
cd ~/QuanergySystems/quanergy_client/build
cmake ..
make doc
