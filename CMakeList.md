# CMakeLists.txt
```
find_package(catkin REQUIRED COMPONENTS cpp_common geometry_msgs)
find_package(Log4cxx QUIET)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(
  CATKIN_DEPENDS cpp_common geometry_msgs
  DEPENDS Log4cxx
)
add_library(example_lib src/example.cpp)
target_link_libraries(example_lib ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
add_dependencies(example_lib geometry_msgs_gencpp)
```
In general, CMakeLists.txt is responsible for preparing and executing the build process.

As you can see with log4cxx, the name of the system library and the name to be used to find the package are often not the same, that's a reason why we have to specify that dependency again, instead of reading it from the package.xml.

Explanation Details

The calls to
```
find_package(catkin REQUIRED COMPONENTS cpp_common)
target_link_libraries(example_lib ${catkin_LIBRARIES} ${LOG4CXX_LIBRARIES})
```
are equivalent to these calls:
```
find_package(catkin REQUIRED)
find_package(cpp_common REQUIRED)
target_link_libraries(example_lib ${catkin_LIBRARIES} ${cpp_common_LIBRARIES} ${LOG4CXX_LIBRARIES})
```
with the main benefit that with the COMPONENTS notation you can use "catkin_" prefixed variables, and those will also have cpp_common values. In this example we only create one library "example_lib", but if we created more, it could be better to list for each library individually which dependency it has, rather than using ${catkin_LIBRARIES}.

The dependencies in

generate_messages(DEPENDENCIES geometry_msgs)

belongs to the message_generation macros, not catkin macros, but in this context they are still worth mentioning, showing that message generation does not use other declared catkin dependencies for the purpose. When Groovy was released, all transitive dependencies of messages had to be listed here, a later patch allows to only state direct dependencies.

The declarations in:
```
catkin_package(
  CATKIN_DEPENDS cpp_common
  DEPENDS Log4cxx
)
```
makes sure that if a 3rd package want to use our "example_pkg", the build flags for cpp_common and log4cxx are automatically used as well. So you need these only if that is the case (for dependencies that are <run_depend> entries in your package.xml).

The declaration in
```
add_dependencies(example_lib geometry_msgs_gencpp)
```
makes sure that the c++ message headers from geometry_msgs are build before attemoting to build example_lib, else you would get compiler errors about missing headers, when working with a catkin workspace (Outside catkin workspaces, this line has no effect). 
