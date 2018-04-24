LOCAL_PATH:= $(call my-dir)
include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libtegra_hal
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/3rdparty/libs/libtegra_hal.a
LOCAL_SHARED_LIBRARIES := libc++
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libopencv_core
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/staticlibs/libopencv_core.a
LOCAL_SHARED_LIBRARIES := libc++
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libopencv_imgcodecs
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/staticlibs/libopencv_imgcodecs.a
LOCAL_SHARED_LIBRARIES := libc++ libpng libjasper libtiff libvpx
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libopencv_imgproc
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/staticlibs/libopencv_imgproc.a
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libopencv_features2d
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/staticlibs/libopencv_features2d.a
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libopencv_calib3d
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/staticlibs/libopencv_calib3d.a
include $(BUILD_PREBUILT)

include $(CLEAR_VARS)
LOCAL_IS_HOST_MODULE =
LOCAL_MODULE = libopencv_highgui
LOCAL_MODULE_CLASS = STATIC_LIBRARIES
LOCAL_MODULE_PATH =
LOCAL_MODULE_RELATIVE_PATH =
LOCAL_MODULE_SUFFIX = .a
#LOCAL_MULTILIB = 64
LOCAL_SRC_FILES_64 = opencv/$(TARGET_ARCH)/sdk/native/staticlibs/libopencv_highgui.a
include $(BUILD_PREBUILT)

#include $(CLEAR_VARS)
#LOCAL_STATIC_DIR := $(LOCAL_PATH)/opencv/$(TARGET_ARCH)/sdk/native/staticlibs
#LOCAL_MODULE := opencv_core
#LOCAL_SRC_FILES := $(LOCAL_STATIC_DIR)/libopencv_core.a
#LOCAL_EXPORT_C_INCLUDES := $(LOCAL_PATH)/opencv/$(TARGET_ARCH)/sdk/native/jni/include
#include $(PREBUILT_STATIC_LIBRARY)

include $(CLEAR_VARS)

LOCAL_ARM_MODE := arm


LOCAL_SRC_FILES := \
    stitching_performance.cpp 

LOCAL_C_INCLUDES := \
    $(LOCAL_PATH)/include \
	$(LOCAL_PATH)/opencv/$(TARGET_ARCH)/sdk/native/jni/include

ifneq (,$(TARGET_BUILD_APPS))
# unbundled branch, built against NDK.
LOCAL_SDK_VERSION := 17
endif

LOCAL_CFLAGS += -O3 -fstrict-aliasing -fprefetch-loop-arrays -ftree-vectorize -frtti -fexceptions
LOCAL_CPP_FEATURES := rtti
LOCAL_SHARED_LIBRARIES := libpng libtiff libvpx
LOCAL_MODULE := stitching_performance

LOCAL_WHOLE_STATIC_LIBRARIES := \
    libopencv_core \
    libopencv_imgcodecs \
    libopencv_imgproc \
    libopencv_features2d \
    libopencv_calib3d \
    libopencv_highgui \
    libtegra_hal \
    


LOCAL_SHARED_LIBRARIES := libc++
#$(error "target arch abi: $(TARGET_ARCH) $(APP_ABI)")

include $(BUILD_EXECUTABLE)
