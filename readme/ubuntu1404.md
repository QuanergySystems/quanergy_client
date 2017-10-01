# Ubuntu 14.04 LTS Build of QuanergyClient

## Install Prerequisites
The following will install prerequisites including PCL which requires a ppa be added.

```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install cmake git build-essential libboost-all-dev libpcl-all 
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
