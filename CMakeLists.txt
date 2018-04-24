cmake_minimum_required (VERSION 3.1)

set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_SYSTEM_VERSION 21)
set(CMAKE_ANDROID_ARCH_ABI arm64-v8a)
#set(ANDROID_ABI arm64-v8a)
set(CMAKE_ANDROID_NDK /home/{$USER}/Android/Sdk/ndk-bundle)
set(CMAKE_ANDROID_STL_TYPE c++_shared)
set(CMAKE_BUILD_TYPE Release)
#set(ANDROID_NATIVE_API_LEVEL 21)
#set(ANDROID_TOOLCHAIN clang)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

#set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -Wall -mfpu=neon")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -mfpu=neon")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -mfpu=neon")

if (NOT DEFINED OPENCV_DIR)
find_package(OpenCV REQUIRED)
else()
message("opencv set to ${OPENCV_DIR}")
set(OpenCV_INCLUDE_DIRS ${OPENCV_DIR}/${CMAKE_ANDROID_ARCH_ABI}/sdk/native/jni/include/)
set(OpenCV_LIB_DIRS ${OPENCV_DIR}/${CMAKE_ANDROID_ARCH_ABI}/sdk/native/libs)
set(OpenCV_LIBRARIES
    ${OpenCV_LIB_DIRS}/libopencv_core.so
    ${OpenCV_LIB_DIRS}/libopencv_calib3d.so
    ${OpenCV_LIB_DIRS}/libopencv_highgui.so
    ${OpenCV_LIB_DIRS}/libopencv_imgproc.so
    ${OpenCV_LIB_DIRS}/libopencv_imgcodecs.so
    ${OpenCV_LIB_DIRS}/libopencv_videoio.so
    ${OpenCV_LIB_DIRS}/libopencv_flann.so
    ${OpenCV_LIB_DIRS}/libopencv_features2d.so)
endif()

#list(APPEND LIBS ${OpenCV_LIBRARIES})

set(fastcv_dir ${CMAKE_SOURCE_DIR}/fastcv)
set(fastcv_inc ${fastcv_dir}/include)

add_library( fastcv STATIC IMPORTED )
set_target_properties( fastcv PROPERTIES IMPORTED_LOCATION ${fastcv_dir}/libs/${CMAKE_ANDROID_ARCH_ABI}/libfastcv.a )
list(APPEND LIBS fastcv)

set(computelibrary_dir ${CMAKE_SOURCE_DIR}/computelibrary)
set(computelibrary_inc ${computelibrary_dir}/include ${computelibrary_dir})

add_library( computelibrary STATIC IMPORTED )
set_target_properties( computelibrary PROPERTIES IMPORTED_LOCATION ${computelibrary_dir}/lib/android-${CMAKE_ANDROID_ARCH_ABI}-neon/libarm_compute-static.a )
list(APPEND LIBS computelibrary)

add_library( computelibrary_core STATIC IMPORTED )
set_target_properties( computelibrary_core PROPERTIES IMPORTED_LOCATION ${computelibrary_dir}/lib/android-${CMAKE_ANDROID_ARCH_ABI}-neon/libarm_compute_core-static.a )
list(APPEND LIBS computelibrary_core)

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${computelibrary_inc})
#include_directories(
#  "${PROJECT_SOURCE_DIR}/include"
#  "${EIGEN3_INCLUDE_DIR}"
#  "${GTEST_INCLUDE_DIRS}"
#  )

add_executable(stitching_performance stitching_performance.cpp)
target_link_libraries(stitching_performance ${LIBS} ${OpenCV_LIBRARIES})

add_executable(scale_performance scale_perf.cpp ${computelibrary_dir}/utils/Utils.cpp)
target_link_libraries(scale_performance ${LIBS})

add_executable(fastorb fastorb.cpp ${computelibrary_dir}/utils/Utils.cpp)
target_link_libraries(fastorb ${LIBS})
