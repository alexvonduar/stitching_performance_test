cmake_minimum_required (VERSION 3.1)

if (NOT DEFINED CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE Release)
endif()

if (NOT DEFINED OPENCV_DIR)
find_package(OpenCV REQUIRED)
else()
message("opencv set to ${OPENCV_DIR}")
#/home/alex/work/opencv/android_build/arm64/install/sdk/native/jni/include/
set(OpenCV_INCLUDE_DIRS ${OPENCV_DIR}/arm64/install/sdk/native/jni/include/)
set(OpenCV_LIB_DIRS ${OPENCV_DIR}/arm64/install/sdk/native/staticlibs/${CMAKE_ANDROID_ARCH_ABI}/)
set(OpenCV_3RD_LIB_DIRS ${OPENCV_DIR}/arm64/install/sdk/native/3rdparty/libs/${CMAKE_ANDROID_ARCH_ABI}/)
if (ANDROID)
set(OpenCV_LIBRARIES
    ${OpenCV_LIB_DIRS}/libopencv_calib3d.a
    ${OpenCV_LIB_DIRS}/libopencv_features2d.a
    ${OpenCV_LIB_DIRS}/libopencv_imgproc.a
    ${OpenCV_LIB_DIRS}/libopencv_imgcodecs.a
    ${OpenCV_3RD_LIB_DIRS}/liblibwebp.a
    ${OpenCV_3RD_LIB_DIRS}/libIlmImf.a
    ${OpenCV_3RD_LIB_DIRS}/liblibjasper.a
    ${OpenCV_3RD_LIB_DIRS}/liblibjpeg-turbo.a
    ${OpenCV_3RD_LIB_DIRS}/liblibpng.a
    ${OpenCV_3RD_LIB_DIRS}/liblibtiff.a
    ${OpenCV_LIB_DIRS}/libopencv_videoio.a
    ${OpenCV_LIB_DIRS}/libopencv_flann.a
    ${OpenCV_LIB_DIRS}/libopencv_core.a
    ${OpenCV_3RD_LIB_DIRS}/libtegra_hal.a
    ${OpenCV_3RD_LIB_DIRS}/libittnotify.a)
else()
set(OpenCV_LIBRARIES
    ${OpenCV_LIB_DIRS}/libopencv_core.a
    ${OpenCV_LIB_DIRS}/libopencv_calib3d.a
    ${OpenCV_LIB_DIRS}/libopencv_highgui.a
    ${OpenCV_LIB_DIRS}/libopencv_imgproc.a
    ${OpenCV_LIB_DIRS}/libopencv_imgcodecs.a
    ${OpenCV_LIB_DIRS}/libopencv_videoio.a
    ${OpenCV_LIB_DIRS}/libopencv_flann.a
    ${OpenCV_LIB_DIRS}/libopencv_features2d.a
    ${OpenCV_3RD_LIB_DIRS}/libtegra_hal.a
    ${OpenCV_3RD_LIB_DIRS}/libittnotify.a)
endif()
endif()

include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS} ${CMAKE_SOURCE_DIR} )

#if (CMAKE_SYSTEM_NAME EQUAL Android)
if (ANDROID)

#set(CMAKE_SYSTEM_NAME Android)
#set(CMAKE_SYSTEM_VERSION 21)
#set(CMAKE_ANDROID_ARCH_ABI arm64-v8a)
#set(ANDROID_ABI arm64-v8a)
#set(CMAKE_ANDROID_NDK /home/{$USER}/Android/Sdk/ndk-bundle)
#set(CMAKE_ANDROID_STL_TYPE c++_shared)
#set(ANDROID_NATIVE_API_LEVEL 21)
#set(ANDROID_TOOLCHAIN clang)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_STANDARD 99)

#set(CMAKE_ASM_FLAGS "${CMAKE_ASM_FLAGS} -Wall -mfpu=neon")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -fopenmp -static-openmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fopenmp -static-openmp")
#add_compile_option(-Wall -fopenmp -static-openmp)
set(CMAKE_LINK_FLAGS "${CMAKE_LINK_FLAGS} -static-openmp")
set(CMAKE_C_FLAGS_RELEASE ${CMAKE_C_FLAGS})
set(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_CXX_FLAGS})
set(CMAKE_C_FLAGS_DEBUG ${CMAKE_C_FLAGS})
set(CMAKE_CXX_FLAGS_DEBUG ${CMAKE_CXX_FLAGS})

#list(APPEND LIBS ${OpenCV_LIBRARIES})

set(fastcv_dir ${CMAKE_SOURCE_DIR}/fastcv)
set(fastcv_inc ${fastcv_dir}/include)

add_library( fastcv STATIC IMPORTED )
set_target_properties( fastcv PROPERTIES IMPORTED_LOCATION ${fastcv_dir}/libs/${CMAKE_ANDROID_ARCH_ABI}/libfastcv.a )
list(APPEND LIBS fastcv)

set(computelibrary_dir ${CMAKE_SOURCE_DIR}/computelibrary)
set(computelibrary_inc ${computelibrary_dir}/${CMAKE_ANDROID_ARCH_ABI}/include)

add_library( computelibrary STATIC IMPORTED )
set_target_properties( computelibrary PROPERTIES IMPORTED_LOCATION ${computelibrary_dir}/${CMAKE_ANDROID_ARCH_ABI}/lib/libarm_compute-static.a )
#list(APPEND LIBS computelibrary)

add_library( computelibrary_graph STATIC IMPORTED )
set_target_properties( computelibrary_graph PROPERTIES IMPORTED_LOCATION ${computelibrary_dir}/${CMAKE_ANDROID_ARCH_ABI}/lib/libarm_compute_graph-static.a )
#list(APPEND LIBS computelibrary)

add_library( computelibrary_core STATIC IMPORTED )
set_target_properties( computelibrary_core PROPERTIES IMPORTED_LOCATION ${computelibrary_dir}/${CMAKE_ANDROID_ARCH_ABI}/lib/libarm_compute_core-static.a )
#list(APPEND LIBS computelibrary_core)

include_directories(SYSTEM ${computelibrary_inc} ${CMAKE_SOURCE_DIR} ${fastcv_inc})
#include_directories(
#  "${PROJECT_SOURCE_DIR}/include"
#  "${EIGEN3_INCLUDE_DIR}"
#  "${GTEST_INCLUDE_DIRS}"
#  )

add_executable(stitching_performance stitching_performance.cpp)
target_link_libraries(stitching_performance ${LIBS} ${OpenCV_LIBRARIES} log z)

add_executable(scale_performance scale_perf.cpp ${computelibrary_inc}/utils/Utils.cpp)
target_link_libraries(scale_performance ${LIBS} computelibrary computelibrary_graph computelibrary_core log)

add_executable(fastorb fastorb.cpp ${computelibrary_inc}/utils/Utils.cpp)
target_link_libraries(fastorb ${LIBS} computelibrary computelibrary_graph computelibrary_core log)

else()

add_executable(video_test video_test.cpp ORBextractor.cc)
target_link_libraries(video_test ${LIBS} ${OpenCV_LIBRARIES} log)

add_executable(show_homography show_homography.cpp)
target_link_libraries(show_homography ${LIBS} ${OpenCV_LIBRARIES} log)

endif()
