# Visual Studio 2013 Build of QuanergyClient

These instructions were tested for the following configuration:
    - Windows 7 64-bit
    - Visual Studio 2013 (tested with Community Edition)
    - CMake (tested with latest version 3.9.4 available at time of writing here: https://cmake.org/files/v3.9/cmake-3.9.4-win64-x64.msi)
```
## Install Prerequisites

1. Install Visual C++ Redistributable Packages for Visual Studio 2013 64-bit (vcredist_x64.exe) found here: https://www.microsoft.com/en-us/download/details.aspx?id=40784
```
2. Install PCL 1.8.0 All-in-one for MSVC2013 (x64) found here: http://unanancyowen.com/?p=1255&lang=en
    - Select "Add PCL to the system path for all users."
    - Install PCL 1.8.0 in "C:\Program Files\PCL 1.8.0" 
```
3. Update environment variables
    a. Open "System Properties" dialog
       Start > Control Panel > System and Security > System > Advanced system settings
    b. From "System Properties", open "Environment Variables" dialog
       Advanced tab > Environment Variables...
    c. On "Environment Variables" dialog
       Edit "Path" variable under "System variables"
       Append ";%OPENNI2_REDIST64%" to "Path"
```
3. Download copy of QuanergyClient source from GitHub.
    git clone https://github.com/QuanergySystems/quanergy_client.git
```
4. Open CMake 3.2.3 to start converting cmake files to Visual Studio 2013 solutions.
    - Set "Where is the source code:" to C:\location\of\quanergy_client
    - Set "Where to build the binaries:" to C:\location\of\quanergy_client\build
    - Press "Configure" 
      - Select "Visual Studio 12 2013 Win64" as the generator.
      - Use default native compilers.
      - If an error occurs where it cannot find the Boost install in PCL, use "Add Entry" button to add the following:
        - PATH variable `BOOST_INCLUDEDIR = C:/Program Files/PCL 1.8.0/3rdParty/Boost/include/boost-1_61`
        - PATH variable `BOOST_LIBRARYDIR = C:/Program Files/PCL 1.8.0/3rdParty/Boost/lib`
        - Boost_USE_STATIC_LIBS set to true (check the box)
        - Press "Configure" again
    - Press "Generate" to create the Visual Studio 2013 solution of QuanergyClient
```
5. Configuring and Building QuanergyClient in Visual Studio 2013
    - Open QuanergyClient.sln in C:\location\of\quanergy_client_build
    - Right-click on ALL_BUILD in the Solution Explorer and select "Build".  Do this for both Debug and Release builds.
```
6. Verify build
   Use Windows Explorer to navigate to "C:\location\of\quanergy_client\build\Release"
   Hold Left-Shift and right-click for pop-up menu and select "Open command window here"
   At command prompt enter:
   ".\visualizer.exe --host <IP Address of Sensor>"
   Cloud Viewer window appears.  When sensor is up to speed, use click and move mouse to manipulate th view of point cloud.
   NOTE: The point cloud may not appear until the mouse has moved the view.

