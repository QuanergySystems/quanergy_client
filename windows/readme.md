# Visual Studio 2013 Build of QuanergyClient

Prerequisites:
<ul>
    <li>Visual Studio 2013</li>
    <li>CMake 3.2.3</li>
</ul>

Steps:
<ol>
<li>Install Visual C++ Redistributable Packages for Visual Studio 2013 found here: 
	https://www.microsoft.com/en-us/download/details.aspx?id=40784
</li>
<li>Install PCL 1.7.2 All-in-one for MSVC2013 (Win32 or x64) found here: http://unanancyowen.com/?p=1255&amp;lang=en
    <ul>
    <li>Install PCL 1.7.2 in either "C:\Program Files\PCL 1.7.2" or "C:\Program Files (x86)\PCL 1.7.2"
    </li>
    <li>In "System Properties"->"Advanced"->"Environment Variables", set the following user variables
          <ul>
          <li>
          PCL_ROOT     C:\Program Files\PCL 1.7.2 (or C:\Program Files (x86)\PCL 1.7.2)
          </li>
          </ul>
    </li>
    <li>and append to the variable "Path" the following:
          <ul>
          <li>
             ;%PCL_ROOT%\bin
             ;%PCL_ROOT%\3rdParty\FLANN\bin
             ;%PCL_ROOT%\3rdParty\VTK\bin
          </li>
          </ul>
    </li>
    </ul>
</li>
<li>Get copy of QuanergyClient source from GitHub.
</li>
<li>Open CMake 3.2.3 to start converting cmake files to Visual Studio 2013 solutions.
    <ul>
    <li>Set "Where is the source code:" to C:\location\of\quanergy_client
    </li>
    <li>Set "Where to build the binaries:" to C:\location\of\quanergy_client_build
    </li>
    <li>Press "Configure" and select "Visual Studio 12 2013" or "Visual Studio 12 2013 Win64" as the generator.  Use default native compilers.
        If an error occurs where it cannot find the VTK install in PCL, add the variable:
          <ul>
          <li>CMAKE_PREFIX_PATH = C:/Program Files/PCL 1.7.2/3rdParty/VTK (or C:/Program Files (x86)/PCL 1.7.2/3rdParty/VTK)</li>
          </ul>
    </li>
    <li>
        If an error occurs where it cannot find the Boost install in PCL,
        add the variables:
          <ul>
          <li>
            BOOST_INCLUDEDIR = C:/Program Files/PCL 1.7.2/3rdParty/Boost/include/boost-1_57 (or C:/Program Files (x86)/PCL 1.7.2/3rdParty/Boost/include/boost-1_57)
          </li>
          <li>
            BOOST_LIBRARYDIR = C:/Program Files/PCL 1.7.2/3rdParty/Boost/lib (or C:/Program Files (x86)/PCL 1.7.2/3rdParty/Boost/lib)
          </li>
          <li>
            Boost_USE_STATIC_LIBS set to true (check the box)
          </li>
          </ul>
    </li>
    <li>Press "Generate" to create the Visual Studio 2013 solution of QuanergyClient
    </li>
    </ul>
</li>
<li>Configuring and Building QuanergyClient in Visual Studio 2013
    <ul>
    <li>Open QuanergyClient.sln in C:\location\of\quanergy_client_build
    </li>
    <li>Right-click on ALL_BUILD in the Solution Explorer and select "Build".  Do this for both Debug and Release builds.
    </li>
    </ul>
</li>
</ol>
