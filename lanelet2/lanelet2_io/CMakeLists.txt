set(MRT_PKG_VERSION 4.0.0)
# Modify only if you know what you are doing!
cmake_minimum_required(VERSION 3.5.1)
project(lanelet2_io)

###################
## find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(UseMrtAutoTarget)

include(GatherDeps)
# Remove libs which cannot be found automatically
#list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

# Manually resolve removed dependend packages
#find_package(...)

# Mark files or folders for display in IDEs
mrt_add_to_ide(README.md .gitlab-ci.yml)


########################
## add python modules ##
########################
# This adds a python module if located under src/{PROJECT_NAME)
mrt_python_module_setup()

file(GLOB PROJECT_PYTHON_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "python_api/*.cpp")
if (PROJECT_PYTHON_SOURCE_FILES_SRC)
    # Add a cpp-python api library. Make sure there are no name collisions with python modules in this project
    mrt_add_python_api( ${PROJECT_NAME}
        FILES ${PROJECT_PYTHON_SOURCE_FILES_SRC}
        )
endif()

############################
## read source code files ##
############################
file(GLOB_RECURSE PROJECT_HEADER_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "include/*.h" "include/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_INC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.h" "src/*.hpp")
file(GLOB PROJECT_SOURCE_FILES_SRC RELATIVE "${CMAKE_CURRENT_LIST_DIR}" "src/*.cpp")

if (PROJECT_SOURCE_FILES_SRC)
    set(LIBRARY_NAME ${PROJECT_NAME})
endif()

###################################
## catkin specific configuration ##
###################################
# The catkin_package macro generates cmake config files for your package
# Declare things to be passed to dependent projects
# INCLUDE_DIRS: uncomment this if you package contains header files
# LIBRARIES: libraries you create in this project that dependent projects also need
# CATKIN_DEPENDS: catkin_packages dependent projects also need
# DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include ${mrt_EXPORT_INCLUDE_DIRS}
    LIBRARIES ${LIBRARY_NAME} ${mrt_EXPORT_LIBRARIES}
    CATKIN_DEPENDS ${catkin_EXPORT_DEPENDS}
    )

###########
## Build ##
###########
# Add include and library directories
include_directories(
    include/${PROJECT_NAME}
    ${mrt_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )

link_directories(
    ${mrt_LIBRARY_DIRS}
    )

# Declare a cpp library
mrt_add_library(${PROJECT_NAME}
    INCLUDES ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC} 
    SOURCES ${PROJECT_SOURCE_FILES_SRC}
    )

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files)
mrt_install(PROGRAMS scripts FILES res data)

#############
## Testing ##
#############
# Add test targets for cpp and python tests
if (CATKIN_ENABLE_TESTING)
    mrt_add_tests(test)
    mrt_add_nosetests(test)
endif()
###################
## Find packages ##
###################
find_package(mrt_cmake_modules REQUIRED)
include(UseMrtStdCompilerFlags)
include(GatherDeps)

# You can add a custom.cmake in order to add special handling for this package. E.g. you can do:
# list(REMOVE_ITEM DEPENDEND_PACKAGES <package name 1> <package name 2> ...)
# To remove libs which cannot be found automatically. You can also "find_package" other, custom dependencies there.
# You can also set PROJECT_INSTALL_FILES to install files that are not installed by default.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
    include("${CMAKE_CURRENT_SOURCE_DIR}/custom.cmake")
endif()

find_package(AutoDeps REQUIRED COMPONENTS ${DEPENDEND_PACKAGES})

mrt_parse_package_xml()

########################
## Add python modules ##
########################
# This adds a python module if located under src/{PROJECT_NAME)
mrt_python_module_setup()

mrt_glob_files(PROJECT_PYTHON_SOURCE_FILES_SRC "python_api/*.cpp")
if (PROJECT_PYTHON_SOURCE_FILES_SRC)
    # Add a cpp-python api library. Make sure there are no name collisions with python modules in this project
    mrt_add_python_api( ${PROJECT_NAME}
        FILES ${PROJECT_PYTHON_SOURCE_FILES_SRC}
        )
endif()

############################
## Read source code files ##
############################
mrt_glob_files_recurse(PROJECT_HEADER_FILES_INC "include/*.h" "include/*.hpp" "include/*.cuh")
mrt_glob_files(PROJECT_SOURCE_FILES_INC "src/*.h" "src/*.hpp" "src/*.cuh")
mrt_glob_files(PROJECT_SOURCE_FILES_SRC "src/*.cpp" "src/*.cu")

###########
## Build ##
###########
# Declare a cpp library
mrt_add_library(${PROJECT_NAME}
    INCLUDES ${PROJECT_HEADER_FILES_INC} ${PROJECT_SOURCE_FILES_INC}
    SOURCES ${PROJECT_SOURCE_FILES_SRC}
    )

#############
## Install ##
#############
# Install all targets, headers by default and scripts and other files if specified (folders or files).
# This command also exports libraries and config files for dependent packages and this supersedes catkin_package.
mrt_install(PROGRAMS scripts FILES res data ${PROJECT_INSTALL_FILES})

#############
## Testing ##
#############
# Add test targets for cpp and python tests
if (CATKIN_ENABLE_TESTING)
    mrt_add_tests(test)
    mrt_add_nosetests(test)
endif()
