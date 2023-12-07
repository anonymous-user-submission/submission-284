cmake_minimum_required(VERSION 3.10)
project(mqnic)

set(CMAKE_CXX_STANDARD 17)
SET(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Library dependency check
# DPDK
set(DPDK_INSTALL_PATH /path/to/dpdk)

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:${DPDK_INSTALL_PATH}/lib/x86_64-linux-gnu/pkgconfig")

find_package(PkgConfig REQUIRED)
pkg_search_module(DPDK REQUIRED IMPORTED_TARGET libdpdk)

set(ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CPP_OPTIONS ${PLATFORM_CXXFLAGS} -march=native -m64 -fomit-frame-pointer -fPIC -std=c++17
    -O3 -ggdb3
    -D_XOPEN_SOURCE=700
    -D_DEFAULT_SOURCE
    -Wall
    -Wextra
    -Wformat=2
    -Wno-sign-compare
    -Wno-attributes
    -Wno-unused-parameter
    -DENABLE_HEXDUMP
    -DQINGNIAO
)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/common
    ${DPDK_INSTALL_PATH}/include
    ${ROCKSDB_INSTALL_PATH}/include
)

file(GLOB MQNIC_SRC src/mqnic/*.cpp)
list(APPEND MQNIC_SRC src/common/helpers.cpp)
list(APPEND MQNIC_SRC src/common/app_thrd_info.cpp)
list(APPEND MQNIC_SRC src/proxy_proc/qingniao_proc.cpp)
list(APPEND MQNIC_SRC src/simple-kvs/api.cpp)
list(APPEND MQNIC_SRC src/simple-kvs/rmtp.cpp)
list(APPEND MQNIC_SRC src/simple-kvs/app.cpp)
add_executable(main src/main.cpp ${MQNIC_SRC})
target_compile_options(main PUBLIC ${CPP_OPTIONS})
target_link_libraries(main elf pthread PkgConfig::DPDK ${PLATFORM_LDFLAGS})
