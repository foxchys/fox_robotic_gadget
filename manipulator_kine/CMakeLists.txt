#  This file is part of fox_robotic_gadget/manipulator_kine.
  
#  Copyright (C) 2021-2022, by ChyS(foxchys)

#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Affero General Public License as published
#  by the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.

#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Affero General Public License for more details.

#  You should have received a copy of the GNU Affero General Public License
#  along with this program.  If not, see <https://www.gnu.org/licenses/>.
  
#  ChyS(foxchys): https://github.com/foxchys

cmake_minimum_required(VERSION 3.9)
project(manipulator_kine)


set(CMAKE_CXX_STANDARD 11)
# SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_BUILD_TYPE "Debug")


message("** Project ${PROJECT_NAME} **")

set(VERSION_MAJOR   0   CACHE STRING "Project major version number.")
set(VERSION_MINOR   0   CACHE STRING "Project minor version number.")
set(VERSION_PATCH   1   CACHE STRING "Project patch version number.")
mark_as_advanced(VERSION_MAJOR VERSION_MINOR VERSION_PATCH)

# Setup the default build environment
SET(CMAKE_AR "gcc-ar")
SET(CMAKE_NM "gcc-nm")
SET(CMAKE_RANLIB "gcc-ranlib")
SET(C "gcc")
SET(CXX "g++")

option(BUILD_EXAMPLES  "Set to ON to build examples" ON)

# find_package( OpenMP REQUIRED)
# if(OPENMP_FOUND)
#     message("OPENMP FOUND")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#     set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
# endif()


# set(OpenCV_DIR /opt/xxx)
# find_package(OpenCV 4.5.1 REQUIRED)
# find_package(Ceres REQUIRED)
# target_link_libraries(helloworld Ceres::ceres)
# find_package(Boost REQUIRED)

find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()

include_directories( ${PROJECT_SOURCE_DIR}/include
                     #${OpenCV_INCLUDE_DIRS}
                     ${EIGEN3_INCLUDE_DIR}
)


# /////lib for robot model//////////////////////////////////////
add_library( ${PROJECT_NAME}_robot_model SHARED
             ${PROJECT_SOURCE_DIR}/src/robotbox.cpp
)
set_target_properties( ${PROJECT_NAME}_robot_model PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                       ${PROJECT_BINARY_DIR}/lib/robot_model
)
target_link_libraries( ${PROJECT_NAME}_robot_model
                      #  ${Boost_LIBS}
)
# //////////////////////////////////////////////////////






# TODO /examples/CMakeLists.txt
if(BUILD_EXAMPLES)
# ***********************************build examples***********************************

  # /////demo for marker//////////////////////////////////////
  add_executable( ${PROJECT_NAME}_examples_main
                  ${PROJECT_SOURCE_DIR}/examples/example_main.cpp
  )
  set_target_properties( ${PROJECT_NAME}_examples_main PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                        ${PROJECT_BINARY_DIR}/bin/examples/
  )
  add_dependencies( ${PROJECT_NAME}_examples_main
                    ${PROJECT_NAME}_robot_model
  )
  target_link_libraries(${PROJECT_NAME}_examples_main
                        ${PROJECT_NAME}_robot_model
  )
  # //////////////////////////////////////////////////////

endif()
# *******************************************************************************
