# CMake generated Testfile for 
# Source directory: /home/maranmen/Quadruped_8DOF/src/rviz_visual_tools
# Build directory: /home/maranmen/Quadruped_8DOF/build/rviz_visual_tools
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(rviz_visual_tools_rvt_test "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/maranmen/Quadruped_8DOF/build/rviz_visual_tools/test_results/rviz_visual_tools/rviz_visual_tools_rvt_test.gtest.xml" "--package-name" "rviz_visual_tools" "--output-file" "/home/maranmen/Quadruped_8DOF/build/rviz_visual_tools/ament_cmake_gtest/rviz_visual_tools_rvt_test.txt" "--command" "/home/maranmen/Quadruped_8DOF/build/rviz_visual_tools/rviz_visual_tools_rvt_test" "--gtest_output=xml:/home/maranmen/Quadruped_8DOF/build/rviz_visual_tools/test_results/rviz_visual_tools/rviz_visual_tools_rvt_test.gtest.xml")
set_tests_properties(rviz_visual_tools_rvt_test PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/maranmen/Quadruped_8DOF/build/rviz_visual_tools/rviz_visual_tools_rvt_test" TIMEOUT "180" WORKING_DIRECTORY "/home/maranmen/Quadruped_8DOF/build/rviz_visual_tools" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/maranmen/Quadruped_8DOF/src/rviz_visual_tools/CMakeLists.txt;247;ament_add_gtest;/home/maranmen/Quadruped_8DOF/src/rviz_visual_tools/CMakeLists.txt;0;")
subdirs("gtest")
