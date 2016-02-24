# quanergy_client Build Procedure
Sample source for connecting to Quanergy sensors

# Linux Build Instructions (tested on Ubuntu 14.04 LTS)
###
### See the windows/build_for_MSVC.txt file for instructions for the Windows platform

Install the following libraries (and git)

```
sudo apt-get install git build-essential libboost-all-dev 
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
git clone https://github.com/QuanergySystems/quanergy_client.git
```
Build quanergy_client code and test application

```
cd quanergy_client
mkdir build
cd build
cmake ..
make
```
To test, run the test application and follow the usage instructions (Sensor IP address is xx.xx.xx.xx)

```
./test -host xx.xx.xx.xx 
```
For documentation, run the following and then open doc/index.html in any browser.

```
make doc
```
