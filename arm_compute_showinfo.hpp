#pragma once

#include <iostream>

#include "arm_compute/core/Types.h"
//#include "arm_compute/core/PyramidInfo.h"
#include "arm_compute/runtime/Tensor.h"
#include "arm_compute/runtime/Pyramid.h"

static inline void show_tensor_info(const arm_compute::Tensor& t)
{
	arm_compute::ITensorInfo * info = t.info();
	std::cout << "c " << info->num_channels() << " d " << info->num_dimensions()
              << " e " << info->element_size() << " type " << (int)info->data_type()
			  << " format " << (int)info->format() << std::endl;
	std::cout << "total size " << info->total_size() << std::endl;
    std::cout << "stride " << info->strides_in_bytes().x() << " " << info->strides_in_bytes().y()
	          << " " << info->strides_in_bytes().z() << std::endl;
	std::cout << "first element offset " << info->offset_first_element_in_bytes() << std::endl;
	std::cout << "fixed point position " << info->fixed_point_position() << std::endl;
	std::cout << "has padding " << info->has_padding() << std::endl;
	if (info->has_padding()) {
		std::cout << info->padding().top << " "
		          << info->padding().bottom << " "
				  << info->padding().left << " "
				  << info->padding().right << std::endl;
	}
}

static inline void show_tensor_info(const arm_compute::Tensor * t)
{
	const arm_compute::ITensorInfo * info = t->info();
	std::cout << "c " << info->num_channels() << " d " << info->num_dimensions()
              << " e " << info->element_size() << " type " << (int)info->data_type()
			  << " format " << (int)info->format() << std::endl;
	std::cout << "total size " << info->total_size() << std::endl;
    std::cout << "stride " << info->strides_in_bytes().x() << " " << info->strides_in_bytes().y()
	          << " " << info->strides_in_bytes().z() << std::endl;
	std::cout << "first element offset " << info->offset_first_element_in_bytes() << std::endl;
	std::cout << "fixed point position " << info->fixed_point_position() << std::endl;
	std::cout << "has padding " << info->has_padding() << std::endl;
	if (info->has_padding()) {
		std::cout << info->padding().top << " "
		          << info->padding().bottom << " "
				  << info->padding().left << " "
				  << info->padding().right << std::endl;
	}
}

static inline void show_pyramid_info(const arm_compute::Pyramid& p)
{
	const arm_compute::PyramidInfo * info = p.info();
	std::cout << "pyramid level: " << info->num_levels() << std::endl;
	for (int i = 0; i < info->num_levels(); ++i) {
		const arm_compute::Tensor * t = p.get_pyramid_level(i);
		std::cout << "level: " << i << std::endl;
		show_tensor_info(t);
	}
}