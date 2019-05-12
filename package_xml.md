# package.xml
The package.xml definitions were specified in REP127 (http://ros.org/reps/rep-0127.html) and some details are provided here

In general, package.xml is responsible for:

   *Ordering of the configure step (cmake) sequence for catkin-packages in catkin workspaces

   * Define packaging dependencies for bloom (what dependencies to export when creating debian pkgs)
    Define system (non-catkin-pkgs) build dependencies for rosdep

   * Document build or install or runtime dependency for roswiki / graph tool (rqt_graph) 

Explanation Details

Any <build_depend> and <buildtool_depend> entry will make sure the given dependency is configured first when it is present in the same workspace. If the entry is not as source in this workspace, the dependency is ignored for configure step ordering. In the example this works only for roscpp_common, log4cxx will be ignored, since it is not a catkin package.

<buildtool_depend> implies that this dependency should be used off the current architecture, not the target architecture, when cross-compiling. When not cross-compiling, it is equivalent to <build_depend>.

<test_depend> are dependencies that are required for running tests. Catkin packages use macros that define make targets prefixed with run-tests. They can be run by invoking catkin_make run_tests[_...] or just using make run_tests[_...]. <test_depend> should declare dependencies that are only used during this testing process.

<build_depend> entries can also be used with rosdep for system installs so that users (or scripts) can easily install system dependencies before attempting to build.

A <run_depend> entry has two purposes. One is to declare what executable in the package will need to run. But it will also define any dependency that a 3rd package using our new package needs to also have in the environment (If your package B depends on package A, and another package C depends on your package B). This can happen for libraries at runtime, or for headers at build-time of the 3rd package. So "run" in <run_depend> can be slightly misleading. When a debian package is created, it should make sure that all <run_depend> dependencies are also installed by apt-get.

Very often, a <run_depend> entry will also be a <build_depend> entry. However, in the future we can imagine cases like this:
```
  <build_depend>cpp_common-dev</build_depend>
  <run_depend>cpp_common</run_depend>
```
which is why the dependencies have been separated.
```
<package>
  ...
  <name>example_pkg</name>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>cpp_common</build_depend>
  <build_depend>log4cxx</build_depend>
  <test_depend>gtest</test_depend>
...
  <run_depend>cpp_common</run_depend>
  <run_depend>log4cxx</run_depend>
</package>
```
