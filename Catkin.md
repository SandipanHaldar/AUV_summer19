# Catkin
## Catkin Configuration overview
ROS and other packages may be configured and built using catkin. Every catkin package must include package.xml and CMakeLists.txt files in its top-level directory.
### Package.xml
Your package must contain an XML file named package.xml, as specified by REP-0140. These components are all required:
```
<package format="2">
  <name>your_package</name>
  <version>1.2.4</version>
  <description>
    This package adds extra features to rosawesome.
  </description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
```
Substitute your name, e-mail and the actual name of your package, and please write a better description.
The maintainer is who releases the package, not necessarily the original author. You should generally add one or more <author> tags, giving appropriate credit:
Also, please provide some URL tags to help users find documentation and report problems:
### Metapackages
These are special-purpose catkin packages for grouping other packages. Users who install a metapackage binary will also get all packages directly or indirectly included in that group. Metapackages must not install any code or other files, the package.xml gets installed automatically. They can depend on other metapackages, if desired, but regular catkin packages may not.
Metapackages can be used to resolve [stack](http://wiki.ros.org/Stacks) dependencies declared be legacy [rosbuild](http://wiki.ros.org/rosbuild)
packages not yet converted to catkin. Catkin packages should depend directly on the packages they use, not on any metapackages.
A good use for metapackages is to group the major components of your robot and then provide a comprehensive grouping for your whole system.
In addition to the XML elements mentioned above, a metapackage package.xml must contain this:
```
<export>
  <metapackage/>
  <architecture_independent/>
</export>
```
In addition to the required <buildtool_depend> for catkin, metapackages list the packages in the group using <exec_depend> tags:
###CMakeList.txt
Catkin CMakeLists.txt files mostly contain ordinary CMake commands, plus a few catkin-specific ones. They begin like this:
```
cmake_minimum_required(VERSION 2.8.3)
project(your_package)
```
Substitute the actual name of your package in the project() command.

Metapackage CMakeLists.txt files should contain only these two additional lines:
```
find_package(catkin REQUIRED)
catkin_metapackage()
```
Regular catkin packages generally provide additional information for dependencies, building targets, installing files and running tests. They are required to use these two commands, usually with additional arguments:
```
find_package(catkin REQUIRED COMPONENTS ...)
...
catkin_package(...)
```
Package format 2 (recommended) pages describe those tasks in detail. As you follow them, observe the usual command order:

    cmake_minimum_required()
    project()
    find_package()
    add_message_files(), add_service_files(), add_action_files(), all catkin-specific
    generate_messages(), catkin-specific
    catkin_package(), catkin-specific
    add_library(), add_executable(), target_link_libraries()
    install()
    catkin_add_gtest(), catkin_add_nosetests(), add_rostest(), add_rostest_gtest(), all catkin-specific


