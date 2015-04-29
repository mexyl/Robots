cmake_minimum_required(VERSION 2.8)

# add prefix to every string in a variable
FUNCTION(PREPEND var prefix)
   SET(listVar "")
   FOREACH(f ${ARGN})
      LIST(APPEND listVar "${prefix}/${f}")
   ENDFOREACH(f)
   SET(${var} "${listVar}" PARENT_SCOPE)
ENDFUNCTION(PREPEND)

project(ROBOTS)

# set compile options
if(UNIX)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif(UNIX)

# set binary and library folders
set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

# set include and lib folders
if(UNIX)
include_directories("/usr/include/Lapack")
include_directories("/usr/include/Aris")
link_directories(/usr/lib/Lapack)
link_directories(/usr/lib/Aris)
endif(UNIX)
if(WIN32)
include_directories("C:/include/Lapack")
include_directories("C:/include/Aris")
link_directories(C:/lib/Lapack)
link_directories(C:/lib/Aris)
endif(WIN32)

# set link files
if(WIN32)
set(SYSTEM_LINK_LIBS ws2_32)
set(LAPACK_LINK_LIBS liblapacke liblapack cblas libblas)
set(ARIS_LINK_LIBS Aris_Plan Aris_DynKer Aris_ExpCal Aris_Core)
endif(WIN32)
if(UNIX)
set(SYSTEM_LINK_LIBS pthread gfortran)
set(LAPACK_LINK_LIBS lapacke lapack cblas refblas)
set(ARIS_LINK_LIBS Aris_Plan Aris_DynKer Aris_ExpCal Aris_Core)
endif(UNIX)


# build programs
set(SOURCE_FILES main.cpp Hexapod_Robot_Plan.cpp Hexapod_Robot.cpp Hexapod_Robot_Plan.h Hexapod_Robot.h)
PREPEND(FULL_SRC Hexapod ${SOURCE_FILES})
add_executable(Hexapod ${FULL_SRC})
target_link_libraries(Hexapod ${ARIS_LINK_LIBS} ${LAPACK_LINK_LIBS} ${SYSTEM_LINK_LIBS})



