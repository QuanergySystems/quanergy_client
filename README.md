# m8client Build Procedure
Mark 8 sensor driver sample source

# Linux Build Instructions (tested on Ubuntu 14.04 LTS)
###
### See the windows/build_for_MSVC.txt file for instructions for the Windows platform

Install the following libraries (and git)

```
sudo apt-get install libpcap-dev git build-essential libboost-all-dev 
```
Install PCL

```
sudo add-apt-repository ppa:v-launchpad-jochen-sprickerhof-de/pcl
sudo apt-get update
sudo apt-get install libpcl-all
```
Clone the SDK repository

```
mkdir ~/QuanergySystems
cd ~/QuanergySystems
git clone https://github.com/QuanergySystems/m8client.git
git checkout master
```
Build m8client code and test application

```
cd m8client
mkdir build
cd build
cmake ..
make
```
To test, run the test application and follow the usage instructions
