# Visual Studio 2013 Build of QuanergyClient

## Prerequisites
    - Visual Studio 2013 (Express Edition in my case)
    - CMake 3.2.3

## Build Instructions
1. Install Visual C++ Redistributable Packages for Visual Studio 2013 found here: https://www.microsoft.com/en-us/download/pcl18

2. Install PCL 1.8.0 All-in-one for MSVC2013 (Win32 or x64) found here: http://unanancyowen.com/?p=1255&lang=en
    - Install PCL 1.8.0 in either "C:\Program Files\PCL 1.8.0" or "C:\Program Files (x86)\PCL 1.8.0"
    - In "System Properties"->"Advanced"->"Environment Variables", set the following user variables
```
PCL_ROOT     C:\Program Files\PCL 1.8.0 (or C:\Program Files (x86)\PCL 1.8.0)
Path         ;%PCL_ROOT%\bin;%PCL_ROOT%\3rdParty\FLANN\bin;%PCL_ROOT%\3rdParty\VTK\bin
```

3. Get copy of QuanergyClient source from GitHub.

4. Open CMake 3.2.3 to start converting cmake files to Visual Studio 2013 solutions.
    - Set "Where is the source code:" to C:\location\of\quanergy_client
    - Set "Where to build the binaries:" to C:\location\of\quanergy_client_build
    - Press "Configure" and select "Visual Studio 12 2013" or "Visual Studio 12 2013 Win64" as the generator.  Use default native compilers.
        If an error occurs where it cannot find the VTK install in PCL, add the variable `CMAKE_PREFIX_PATH = C:/Program Files/PCL 1.8.0/3rdParty/VTK` (or `C:/Program Files (x86)/PCL 1.8.0/3rdParty/VTK`). If an error occurs where it cannot find the Boost install in PCL, add the variables:
```
            BOOST_INCLUDEDIR = C:/Program Files/PCL 1.8.0/3rdParty/Boost/include/boost-1_57 (or C:/Program Files (x86)/PCL 1.8.0/3rdParty/Boost/include/boost-1_57
            BOOST_LIBRARYDIR = C:/Program Files/PCL 1.8.0/3rdParty/Boost/lib (or C:/Program Files (x86)/PCL 1.8.0/3rdParty/Boost/lib)
            Boost_USE_STATIC_LIBS set to true (check the box)
```
    - Press "Generate" to create the Visual Studio 2013 solution of QuanergyClient

5. Configuring and Building QuanergyClient in Visual Studio 2013
    - Open QuanergyClient.sln in C:\location\of\quanergy_client_build
    - Right-click on ALL_BUILD in the Solution Explorer and select "Build".  Do this for both Debug and Release builds.
