# First-person Tele-operation

This is the code for first-person tele-operation of the humanoid robot iCub associated with our paper:

L. Fritsche, F. Unverzagt, J. Peters, Calandra R. , First-Person Tele-Operation of a Humanoid Robot, IEEE-RAS International Conference on Humanoid Robots (HUMANOIDS), 2015

The standard setting makes use of Oculus Rift, Microsoft Kinect and SensorGloves to control the humanoid robot iCub. However, the software architecture is modular and new devices can easily be integrated. Similarly, the same software for the current devices can be used on different robots (as the devices communicate via TCP/IP with the controller).

If you use this code please cite us


# Usage
Prerequisites:
install CMake 
install icub-main, icub-dev, YARP, gazebo-yarp-plugins (optional) - can be acquired from official iCub website
install glew, SDL2
install LibOVR (Oculus Rift SDK) and KinectSDK

Windows: 
- iCubKinectControl
- iCubOculusControl

-> You'll find a CMakeLists.txt in each of these projects that can be used to create Visual Studio projects using CMake.
**Important**: Make sure that app.ico and BodyBasics.rc are included in the build path of iCubKinectControl or the program won't start without any warning.

Linux:
- HapticGloveTelecontrol
- ICubTelecontrol

-> We used Eclipse for these projects and the project configuration files are contained within them.


Note: 
- if there are problems with the YARP network, make sure that the communication is not blocked by your firewall.
- press ESC to close iCubOculusControl and Arrow-UP to reset the initial viewpoint.
- iCubKinectControl, iCubOculusControl and HapticGloveTelecontrol are only interfaces to their device while ICubTelecontrol uses these interfaces to control the iCub.
- You can activate and deactivate the devices within the main-class of ICubTelecontrol.
- If you want to use the project with the Gazebo simulator, you have to make sure to set the boolean gazebo in all YarpAdapters to true.

**************************************************************************

Your iCubKinectControl includes should look like this:
```
YOUR_PATH\yarp\build\generated_include;
YOUR_PATH\yarp\src\libYARP_OS\include;
YOUR_PATH\yarp\src\libYARP_sig\include;
YOUR_PATH\yarp\src\libYARP_math\include;
YOUR_PATH\yarp\src\libYARP_dev\include;
YOUR_PATH\yarp\src\libYARP_name\include;
YOUR_PATH\yarp\src\libYARP_manager\include;
YOUR_PATH\yarp\src\libYARP_logger\include;
YOUR_PATH\yarp\src\yarpserver\include;
YOUR_PATH\icub-telecontrol\Implementation\iCubKinectControl\include;
C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\inc;
C:\Program Files\Microsoft SDKs\Kinect\v2.0_1409\bin\x86;
%(AdditionalIncludeDirectories)
```

Your iCubOculusControl includes should look like this:
```
YOUR_PATH\iCubOculusControl\include;
YOUR_PATH\glew-1.12.0\include;
YOUR_PATH\SDL2-2.0.3\include;
YOUR_PATH\yarp\build\generated_include;
YOUR_PATH\yarp\src\libYARP_OS\include;
YOUR_PATH\yarp\src\libYARP_sig\include;
YOUR_PATH\yarp\src\libYARP_math\include;
YOUR_PATH\yarp\src\libYARP_dev\include;
YOUR_PATH\yarp\src\libYARP_name\include;
YOUR_PATH\yarp\src\libYARP_manager\include;
YOUR_PATH\yarp\src\libYARP_logger\include;
YOUR_PATH\yarp\src\yarpserver\include;
YOUR_PATH\LibOVR\Include;
YOUR_PATH\LibOVR\Src;
%(AdditionalIncludeDirectories)
```

Your iCubOculusControl library references should look like this:
```
YOUR_PATH\SDL2-2.0.3\lib\x86;
YOUR_PATH\glew-1.12.0\lib\Release\Win32;
YOUR_PATH\LibOVR\Lib\Win32\VS2013;
YOUR_PATH\LibOVR\Lib\x64\VS2013;
%(AdditionalLibraryDirectories)
```
