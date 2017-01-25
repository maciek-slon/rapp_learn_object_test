cmake_minimum_required(VERSION 2.8)
project(learn_object)

add_executable(main.cpp)

find_package(rapp REQUIRED)
message(STATUS "libraries: ${RAPP_STATIC_LIBRARIES}")
find_library(OPENCV_CORE_LIBRARY NAMES opencv_core HINTS ${LIB_PATH})
message(STATUS ${OPENCV_CORE_LIBRARY})
find_library(OPENCV_HIGHGUI_LIBRARY NAMES opencv_highgui HINTS ${LIB_PATH})
message(STATUS ${OPENCV_HIGHGUI_LIBRARY})

#COMMON
find_library(Boost_SYSTEM NAMES boost_system HINTS ${LIB_PATH})
find_library(Boost_THREAD NAMES boost_thread HINTS ${LIB_PATH})
find_library(Boost_CHRONO NAMES boost_chrono HINTS ${LIB_PATH})
set(Boost_LIBRARIES ${Boost_SYSTEM}
                    ${Boost_CHRONO}
                    ${Boost_THREAD})
message(STATUS ${Boost_LIBRARIES})

find_library(Threads NAMES pthread HINTS ${LIB_PATH})
message(STATUS ${Threads})

find_library(SSL NAMES ssl HINTS ${LIB_PATH})
find_library(CRYPTO NAMES crypto HINTS ${LIB_PATH})
set(OPENSSL_LIBRARIES ${SSL}
                      ${CRYPTO})
message(STATUS ${OPENSSL_LIBRARIES})
find_library(PTHREAD NAMES libpthread.so.0 )

set(NAO_LIBRARIES ${OPENCV_CORE_LIBRARY}
                  ${OPENCV_HIGHGUI_LIBRARY}
                  ${Boost_LIBRARIES}
                  ${PTHREAD})

target_link_libraries(face_detection ${RAPP_STATIC_LIBRARIES}
                                     ${Boost_LIBRARIES} 
                                     ${Threads}
                                     ${OPENSSL_LIBRARIES}
                                     ${NAO_LIBRARIES}
)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -static-libstdc++")
set(CMAKE_CXX_FLAGS "-std=gnu++1y -march=atom -mtune=atom -mfpmath=sse")
