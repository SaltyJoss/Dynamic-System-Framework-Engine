# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-src")
  file(MAKE_DIRECTORY "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-src")
endif()
file(MAKE_DIRECTORY
  "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-build"
  "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix"
  "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix/tmp"
  "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
  "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix/src"
  "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
)

set(configSubDirs Debug)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/Joss/source/repos/RoboticArm_MathModelling/Sim_Engine/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
