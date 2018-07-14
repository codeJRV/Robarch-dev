# CMake generated Testfile for 
# Source directory: /home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/industrial_core/industrial_robot_simulator
# Build directory: /home/jrv/Research/RoboticArcitecture/abb_experimental_ws/build/industrial_robot_simulator
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_industrial_robot_simulator_roslaunch-check_launch "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/build/industrial_robot_simulator/catkin_generated/env_cached.sh" "/usr/bin/python" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/build/industrial_robot_simulator/test_results/industrial_robot_simulator/roslaunch-check_launch.xml" "--return-code" "/usr/local/bin/cmake -E make_directory /home/jrv/Research/RoboticArcitecture/abb_experimental_ws/build/industrial_robot_simulator/test_results/industrial_robot_simulator" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/build/industrial_robot_simulator/test_results/industrial_robot_simulator/roslaunch-check_launch.xml' '/home/jrv/Research/RoboticArcitecture/abb_experimental_ws/src/industrial_core/industrial_robot_simulator/launch' ")
subdirs("gtest")
