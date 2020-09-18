NDK_PATH=/opt/Android/Sdk/ndk/21.3.6528147/ #{full path to the NDK directory-- for example,
                                            #/opt/android/android-ndk-r16b}
TOOLCHAIN=clang                             #{"gcc" or "clang"-- "gcc" must be used with NDK r16b and earlier,
                                            #and "clang" must be used with NDK r17c and later}
ANDROID_VERSION=22                          # {the minimum version of Android to support-- for example,
                                            #"16", "19", etc.}

#cd {build_directory}
abi=arm64-v8a #armeabi-v7a
working_directory=${PWD}

export CXXFLAGS="-fopenmp"
export CFLAGS="-fopenmp"


cmake \
	-G"Unix Makefiles" \
	-DOPENCV_DIR=/home/alex/work/opencv/android_build \
	-DANDROID=ON \
	-DANDROID_ABI=${abi} \
	-DANDROID_STL=c++_shared \
	-DANDROID_PLATFORM=android-${ANDROID_VERSION} \
	-DANDROID_TOOLCHAIN=${TOOLCHAIN} \
	-DCMAKE_TOOLCHAIN_FILE=${NDK_PATH}/build/cmake/android.toolchain.cmake \
	-DCMAKE_ASM_FLAGS="--target=arm-linux-androideabi${ANDROID_VERSION}" \
	-DCMAKE_BUILD_TYPE=Release \
	-DANDROID_ARM_MODE=arm \
	-DCMAKE_SYSTEM_NAME=Android \
	-DCMAKE_ANDROID_NDK=/home/${USER}/Android/Sdk/ndk \
	-DCMAKE_ANDROID_ARCH_ABI=${abi} \
	-DCMAKE_ANDROID_STL_TYPE=c++_static \
	-DCMAKE_INSTALL_PREFIX=${install_directory} \
	-DANDROID_ARM_NEON=ON \
	-DENABLE_NEON=ON \
	..

