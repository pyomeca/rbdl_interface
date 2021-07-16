project(${BIORBD_NAME}_c)
cmake_minimum_required(VERSION 3.8)

set(C_BINDING_PATH ${CMAKE_SOURCE_DIR}/binding/c)

SET(BUILD_SHARED_LIBS_C TRUE CACHE BOOL
    "Choose if the static library should be build")

# Get the file to compile
set(C_FILE ${C_BINDING_PATH}/${BIORBD_NAME_NO_SUFFIX}_c.cpp)
if (BUILD_SHARED_LIBS_C)
    add_library(${PROJECT_NAME} SHARED ${C_FILE})
else()
    add_library(${PROJECT_NAME} STATIC ${C_FILE})
endif()
    
# Add headers
target_include_directories(${PROJECT_NAME} PRIVATE
    "${C_BINDING_PATH}"
    "${CMAKE_SOURCE_DIR}/include"
    "${BIORBD_BINARY_DIR}/include"
    "${RBDL_INCLUDE_DIR}"
    "${RBDL_INCLUDE_DIR}/.."
    "${MATH_BACKEND_INCLUDE_DIR}"
)

# Set runtime path for linux
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES
    INSTALL_RPATH ${${BIORBD_NAME}_BIN_FOLDER}
    INSTALL_RPATH_USE_LINK_PATH TRUE
)

# Linker and instalation
target_link_libraries(${PROJECT_NAME}
    ${BIORBD_NAME}
)

install(TARGETS ${PROJECT_NAME}
    DESTINATION "${${BIORBD_NAME}_BIN_FOLDER}"
)

