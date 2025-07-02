# UMRT ROS PoE Camera
ROS project repository for the ROS PoE Camera.

TO BE UPDATED 

New projects should be **forked** from this repo (not using this as a template, as that prevents template changes from
trickling down). Each new project must:
1. Fill in missing fields in package.in.xml
2. Fill in project name in CMakeLists.txt and Doxyfile
3. Go into `umrt-build` package settings and give the new repo read permission
4. Go into `umrt-apt-image` package settings and give the new repo read permission
5. Go into `UMRoboticsTeam` organisation secrets and add the new repo to:
   - `APT_DEPLOY_KEY`
   - `APT_SIGNING_KEY`
6. Copy the rulesets (branch protection rules) from a mature repository like
   [umrt-arm-firmware-lib](https://github.com/UMRoboticsTeam/umrt-arm-firmware-lib/)
7. Remove this notice and fill in below README template
8. Write something in mainpage.dox
9. Replace example files with real code, add source files to CMake targets, document it with Doxygen, and proceed

TO DO:
Check H265
Update Doxyfile
Update CMakeLists.txt to match the umrt-template-ros
Removal of unecesary src files 
Separating mobule_publisher.cpp to different cpp and hpp files

---
# UMRT ROS PoE Camera

This library/executable/project implements PoE Camera functionality for the University of Manitoba Robotics Team's 
rover/robotic arm.

[See the documentation](https://umroboticsteam.github.io/umrt-ros-poe-cam/)