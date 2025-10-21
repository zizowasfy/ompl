# Install script for directory: /home/zizo/omplapp/ompl/demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/opt/ros/noetic")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/KinematicChainPathPlot.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/OptimalPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/PlannerData.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/Point2DPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/RandomWalkPlanner.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/RigidBodyPlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/RigidBodyPlanningWithControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/RigidBodyPlanningWithODESolverAndControls.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/SpaceTimePlanning.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE PROGRAM FILES "/home/zizo/omplapp/share/ompl/demos/StateSampling.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE FILE FILES
    "/home/zizo/omplapp/ompl/demos/CForestCircleGridBenchmark.cpp"
    "/home/zizo/omplapp/ompl/demos/Diagonal.cpp"
    "/home/zizo/omplapp/ompl/demos/GeometricCarPlanning.cpp"
    "/home/zizo/omplapp/ompl/demos/HybridSystemPlanning.cpp"
    "/home/zizo/omplapp/ompl/demos/HypercubeBenchmark.cpp"
    "/home/zizo/omplapp/ompl/demos/KinematicChainBenchmark.cpp"
    "/home/zizo/omplapp/ompl/demos/LTLWithTriangulation.cpp"
    "/home/zizo/omplapp/ompl/demos/OptimalPlanning.cpp"
    "/home/zizo/omplapp/ompl/demos/PlannerData.cpp"
    "/home/zizo/omplapp/ompl/demos/PlannerProgressProperties.cpp"
    "/home/zizo/omplapp/ompl/demos/Point2DPlanning.cpp"
    "/home/zizo/omplapp/ompl/demos/RigidBodyPlanning.cpp"
    "/home/zizo/omplapp/ompl/demos/RigidBodyPlanningWithControls.cpp"
    "/home/zizo/omplapp/ompl/demos/RigidBodyPlanningWithIK.cpp"
    "/home/zizo/omplapp/ompl/demos/RigidBodyPlanningWithIntegrationAndControls.cpp"
    "/home/zizo/omplapp/ompl/demos/RigidBodyPlanningWithODESolverAndControls.cpp"
    "/home/zizo/omplapp/ompl/demos/SpaceTimePlanning.cpp"
    "/home/zizo/omplapp/ompl/demos/StateSampling.cpp"
    "/home/zizo/omplapp/ompl/demos/ThunderLightning.cpp"
    "/home/zizo/omplapp/ompl/demos/TriangulationDemo.cpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/zizo/omplapp/ompl/demos/Koules")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xomplx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ompl/demos" TYPE DIRECTORY FILES "/home/zizo/omplapp/ompl/demos/VFRRT")
endif()

