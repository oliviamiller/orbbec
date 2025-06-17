cmake_minimum_required(VERSION 3.10)
project(orbbec)

set(CMAKE_CXX_STANDARD 17)

add_executable(orbbec src/main.cpp)

set(OrbbecSDK_DIR lib/orbbecsdk/lib)
include_directories(include)
find_package(OrbbecSDK REQUIRED
    PATHS lib/ 
    NO_DEFAULT_PATH)
find_package(OpenCV REQUIRED)
target_link_libraries(orbbec ob::OrbbecSDK ${OpenCV_LIBS})





