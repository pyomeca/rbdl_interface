cmake_minimum_required(VERSION 3.8)
if (${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.0")
    cmake_policy(SET CMP0042 NEW)
endif()

project(biorbd VERSION 1.6.1)
set(BIORBD_ROOT_FOLDER ${PROJECT_SOURCE_DIR})
set (BIORBD_NAME_NO_SUFFIX ${PROJECT_NAME})
set (CMAKE_CXX_STANDARD 11)

# Set some variables
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/modules")

# Because of Eigen, it is not possible to compile biorbd as a dynamic library
if (WIN32)
    SET(BUILD_SHARED_LIBS FALSE CACHE BOOL
        "Choose if the dynamic library should be build" FORCE)
else()
    SET(BUILD_SHARED_LIBS TRUE CACHE BOOL
        "Choose if the dynamic library should be build")
endif()
# Set a default build type to 'Release' if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build.")
    # Set the possible values of build type for cmake-gui
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY
        STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Choose the math's library backend to use
set(MATH_LIBRARY_BACKEND Eigen3 CACHE STRING
    "Choose the backend library for the linear algebra.")
# Set the possible values of build type for cmake-gui
set_property(CACHE MATH_LIBRARY_BACKEND PROPERTY STRINGS "Eigen3" "Casadi")

# Prepare linkings
if(${MATH_LIBRARY_BACKEND} STREQUAL "Eigen3")
    set(BIORBD_USE_EIGEN3_MATH true)
    set(BIORBD_USE_CASADI_MATH false)
    set(BIORBD_LIB_SUFFIX "eigen")
elseif(${MATH_LIBRARY_BACKEND} STREQUAL "Casadi")
    set(BIORBD_USE_EIGEN3_MATH false)
    set(BIORBD_USE_CASADI_MATH true)
    set(BIORBD_LIB_SUFFIX "casadi")
endif()
set (BIORBD_NAME ${PROJECT_NAME}_${BIORBD_LIB_SUFFIX})

if(${MATH_LIBRARY_BACKEND} STREQUAL "Eigen3")
    find_package(Eigen3 REQUIRED)
    set(MATH_BACKEND_INCLUDE_DIR "${EIGEN3_INCLUDE_DIR}")
    set(MATH_BACKEND_LIBRARIES "")
    find_package(RBDL REQUIRED)
elseif(${MATH_LIBRARY_BACKEND} STREQUAL "Casadi")
    find_package(Casadi REQUIRED)
    set(MATH_BACKEND_INCLUDE_DIR
        "${Casadi_INCLUDE_DIR}"
        "${Casadi_INCLUDE_DIR}/.."
    )
    set(MATH_BACKEND_LIBRARIES "${Casadi_LIBRARY}")
    find_package(RBDLCasadi REQUIRED)
    set(RBDL_INCLUDE_DIR ${RBDLCasadi_INCLUDE_DIR})
    set(RBDL_LIBRARY ${RBDLCasadi_LIBRARY})
endif()
find_package(IPOPT)
find_package(TinyXML)

# Manage options
option(MODULE_KALMAN "If Kalman filter should be compiled" ON)
if (BIORBD_USE_CASADI_MATH AND MODULE_KALMAN)
    message(WARNING "Casadi and Kalman cannot be used alongside, MODULE_KALMAN is automatically set to OFF")
    set(MODULE_KALMAN OFF CACHE BOOL "" FORCE)
endif()

if (CMAKE_BUILD_TYPE MATCHES Debug)
    option(SKIP_ASSERT
        "Some checks slow the code down at run-time, but provide more robust
        code. By default, it is ON when compiling in debug, but OFF in release."
        ON
    )
else()
    option(SKIP_ASSERT
        "Some checks slow the code down at run-time, but provide more robust
        code. By default, it is ON when compiling in debug, but OFF in release."
        OFF
    )
endif()

option(MODULE_STATIC_OPTIM "If Static optimization should be compiled" ON)
if (IPOPT_FOUND)
    if (BIORBD_USE_CASADI_MATH AND MODULE_STATIC_OPTIM)
        message(WARNING "Static optimization can't be compiled along with Casadi")
        set(MODULE_STATIC_OPTIM OFF CACHE BOOL "" FORCE)
    endif()
else()
    set(IPOPT_INCLUDE_DIR "")
    set(IPOPT_LIBRARY "")
    if (MODULE_STATIC_OPTIM)
        message(WARNING "Static optimization can't be compiled since Ipopt was not found")
        set(MODULE_STATIC_OPTIM OFF CACHE BOOL "" FORCE)
    endif()
endif()

option(MODULE_VTP_FILES_READER "If reader for geometry vtp files from opensim should be compiled" ON)
if (NOT TinyXML_FOUND)
    set(TinyXML_INCLUDE_DIR "")
    set(TinyXML_LIBRARY "")
    if (MODULE_VTP_FILES_READER)
        message(WARNING "VTP cannot be used since TinyXML was not found")
        set(MODULE_VTP_FILES_READER OFF CACHE BOOL "" FORCE)
    endif()
endif()


# Prepare add library
set(SRC_LIST
    "src/BiorbdModel.cpp"
    "src/ModelReader.cpp"
    "src/ModelWriter.cpp"
)
if (BUILD_SHARED_LIBS)
    add_library(${BIORBD_NAME} SHARED ${SRC_LIST})
else()
    add_library(${BIORBD_NAME} STATIC ${SRC_LIST})
endif()
set_target_properties(${BIORBD_NAME} PROPERTIES DEBUG_POSTFIX "_debug")

# Remember the BINARY_DIR (important if biorbd is used as submodule)
set(BIORBD_BINARY_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Create the configuration header file associated with the asked options
configure_file(${CMAKE_SOURCE_DIR}/include/biorbdConfig.h.in
    "${BIORBD_BINARY_DIR}/include/biorbdConfig.h"
)

# Add headers
# Include directories when building the library:
target_include_directories(${BIORBD_NAME} PRIVATE
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
    "${TinyXML_INCLUDE_DIR}"
)
# Include directories when other targets in this project use this library:
target_include_directories(${BIORBD_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${BIORBD_BINARY_DIR}/include>
)

# Installation directories
if(WIN32)
    set(${BIORBD_NAME}_LIB_FOLDER "Lib")
    set(${BIORBD_NAME}_BIN_FOLDER "bin")
    set(${BIORBD_NAME}_INCLUDE_FOLDER "include/${BIORBD_NAME}")
	set(${BIORBD_NAME}_CMAKECONFIG_DIR ${${BIORBD_NAME}_LIB_FOLDER}/${BIORBD_NAME}/cmake/)
else()
    set(${BIORBD_NAME}_LIB_FOLDER "lib/${BIORBD_NAME}")
    set(${BIORBD_NAME}_BIN_FOLDER "${CMAKE_INSTALL_PREFIX}/lib/${BIORBD_NAME}")
    set(${BIORBD_NAME}_INCLUDE_FOLDER "include/${BIORBD_NAME}")
	set(${BIORBD_NAME}_CMAKECONFIG_DIR ${${BIORBD_NAME}_LIB_FOLDER}/cmake/)
endif()
set_target_properties(${BIORBD_NAME} PROPERTIES
    INSTALL_RPATH "${${BIORBD_NAME}_BIN_FOLDER}"
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Add the different modules of biorbd
set(BIORBD_MODULE_NAMES)

# Utils
add_subdirectory("src/Utils")
list(APPEND BIORBD_MODULE_NAMES "${UTILS_MODULE_NAME}")

# Utils
add_subdirectory("src/RigidBody")
list(APPEND BIORBD_MODULE_NAMES "${RIGIDBODY_MODULE_NAME}")

# Actuators
option(MODULE_ACTUATORS
    "Choose if the module actuators is to be added to biorbd" ON)
if (MODULE_ACTUATORS)
    add_subdirectory("src/Actuators")
    list(APPEND BIORBD_MODULE_NAMES "${ACTUATOR_MODULE_NAME}")
endif()

# Muscles
option(MODULE_MUSCLES
    "Choose if the module actuators is to be added to biorbd" ON)
if (MODULE_MUSCLES)
    add_subdirectory("src/Muscles")
    list(APPEND BIORBD_MODULE_NAMES "${MUSCLE_MODULE_NAME}")
endif()

# Add linker
target_link_libraries(${BIORBD_NAME}
    "${BIORBD_MODULE_NAMES}"
    "${RBDL_LIBRARY}"
    "${MATH_BACKEND_LIBRARIES}"
    "${IPOPT_LIBRARY}"
    "${TinyXML_LIBRARY}"
)

# install target
install(
    TARGETS ${BIORBD_NAME} EXPORT biorbdTargets
    ARCHIVE DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
    RUNTIME DESTINATION "${${BIORBD_NAME}_BIN_FOLDER}"
    LIBRARY DESTINATION "${${BIORBD_NAME}_LIB_FOLDER}"
)

install(DIRECTORY include/
    DESTINATION "${${BIORBD_NAME}_INCLUDE_FOLDER}"
    PATTERN "*.h.in" EXCLUDE
)
# Include directories when a client uses the library from an installed biorbd:
target_include_directories(${BIORBD_NAME} INTERFACE
    $<INSTALL_INTERFACE:${${BIORBD_NAME}_INCLUDE_FOLDER}>
)
install(FILES "${BIORBD_BINARY_DIR}/include/biorbdConfig.h"
    DESTINATION "${${BIORBD_NAME}_INCLUDE_FOLDER}"
)

# Prepare share
if(WIN32 AND NOT CYGWIN)
  set(CMAKE_SHARE_DIR CMake/${BIORBD_NAME})
else()
  set(CMAKE_SHARE_DIR lib/cmake/${BIORBD_NAME})
endif()
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "share/${BIORBD_NAME}Config.cmake.in"
    "${BIORBD_BINARY_DIR}/${BIORBD_NAME}Config.cmake"
    INSTALL_DESTINATION "${CMAKE_SHARE_DIR}"
)
install(FILES 
    "${BIORBD_BINARY_DIR}/${BIORBD_NAME}Config.cmake"
    DESTINATION "${CMAKE_SHARE_DIR}"
)

# uninstall target
if(NOT TARGET uninstall)
    configure_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake_uninstall.cmake.in"
        "${BIORBD_BINARY_DIR}/cmake_uninstall.cmake"
        IMMEDIATE @ONLY
    )

    add_custom_target(uninstall
        COMMAND "${CMAKE_COMMAND}" -P
        "${BIORBD_BINARY_DIR}/cmake_uninstall.cmake"
    )
endif()

        
# Add binding subdirectory
add_subdirectory("binding")

# Add the example if asked
option(BUILD_EXAMPLE "Build a C++ example" ON)
if (BUILD_EXAMPLE)
    add_subdirectory("examples")
endif()

# Doc
option(BUILD_DOC "Build documentation" OFF)
if (BUILD_DOC)
    add_subdirectory("doc")
endif (BUILD_DOC)

# Testing
option(BUILD_TESTS "Build all tests." OFF) # Makes boolean 'test' available.
if (BUILD_TESTS)
    add_subdirectory("test")
endif()
