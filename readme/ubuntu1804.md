# Ubuntu 18.04 LTS Build of QuanergyClient

## Install Prerequisites
The following will install prerequisites including PCL 

```
sudo apt install git cmake g++ libboost-all-dev libpcl-dev
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
For documentation, run the following and then open doc/html/index.html in any browser.

```
sudo apt install doxygen graphviz
cd ~/QuanergySystems/quanergy_client/build
cmake ..
make doc
