cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

project(serial)

# Option to control whether to build as shared library (ON) or static library (OFF)
option(BUILD_SHARED_LIBS "Build shared library" ON)

if(APPLE)
    find_library(IOKIT_LIBRARY IOKit)
    find_library(FOUNDATION_LIBRARY Foundation)
endif()

if(UNIX AND NOT APPLE)
    # If Linux, add rt and pthread
    set(rt_LIBRARIES rt)
    set(pthread_LIBRARIES pthread)
endif()

## Sources
set(serial_SRCS
    src/serial.cc
    include/serial/serial.h
    include/serial/libserial_exports.h
)
if(APPLE)
    # If OSX
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_osx.cc)
elseif(UNIX)
    # If unix
    list(APPEND serial_SRCS src/impl/unix.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
else()
    # If windows
    list(APPEND serial_SRCS src/impl/win.cc)
    list(APPEND serial_SRCS src/impl/list_ports/list_ports_win.cc)
endif()

## Add serial libraryPOSITION_INDEPENDENT_CODE
if (BUILD_SHARED_LIBS)
    if (WIN32)
        # export all symbols on windows
        set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    endif()

    add_library(${PROJECT_NAME} SHARED ${serial_SRCS})
else()
    add_library(${PROJECT_NAME} STATIC ${serial_SRCS})
endif ()

if(APPLE)
    target_link_libraries(${PROJECT_NAME} ${FOUNDATION_LIBRARY} ${IOKIT_LIBRARY})
elseif(UNIX)
    target_link_libraries(${PROJECT_NAME} rt pthread)
else()
    target_link_libraries(${PROJECT_NAME} setupapi)
endif()

## Uncomment for example
add_executable(serial_example examples/serial_example.cc)
add_dependencies(serial_example ${PROJECT_NAME})
target_link_libraries(serial_example ${PROJECT_NAME})

## Include headers
include_directories(include)
