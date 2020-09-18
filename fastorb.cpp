#include "arm_compute/runtime/NEON/NEFunctions.h"
#include "arm_compute/runtime/NEON/NEScheduler.h"

#include "arm_compute/core/Types.h"
#include "utils/ImageLoader.h"
//#include "utils/Utils.h"

//using namespace arm_compute;
//using namespace utils;


int main(int argc, char **argv)
{
    arm_compute::utils::PPMLoader ppm;
    arm_compute::Image       src{};
    arm_compute::Pyramid dst{};
    arm_compute::NEGaussianPyramidOrb  scale{};
    std::string output_filename{};
    std::cout << "threads " << arm_compute::NEScheduler::get().num_threads() << std::endl;
    std::cout << "suggest " << arm_compute::NEScheduler::get().num_threads_hint() << " threads" << std::endl;
    arm_compute::NEScheduler::get().set_num_threads(4);
    std::cout << "threads " << arm_compute::NEScheduler::get().num_threads() << std::endl;

    if(argc < 2)
    {
        // Print help
        std::cout << "Usage: ./build/neon_scale[input_image.ppm]\n\n";
        std::cout << "No input_image provided, creating a dummy 640x480 image\n";
        // Create an empty grayscale 640x480 image
        src.allocator()->init(arm_compute::TensorInfo(640, 480, arm_compute::Format::U8));
    }
    else
    {
        ppm.open(argv[1]);
        ppm.init_image(src, arm_compute::Format::U8);
    }

    constexpr int scale_factor = 2;

    //arm_compute::TensorInfo dst_tensor_info(src.info()->dimension(0) / scale_factor, src.info()->dimension(1) / scale_factor,
    //                               arm_compute::Format::U8);
    arm_compute::PyramidInfo dst_info(4, 1.2, src.info()->dimension(0), src.info()->dimension(1), arm_compute::Format::U8);

    // Configure the destination image
    //dst.allocator()->init(dst_tensor_info);
    dst.init(dst_info);

    // Configure Scale function object:
    scale.configure(&src, &dst, arm_compute::BorderMode::UNDEFINED, 16);

    // Allocate all the images
    src.allocator()->allocate();
    //dst.allocator()->allocate();
    dst.allocate();

    // Fill the input image with the content of the PPM image if a filename was provided:
    if(ppm.is_open())
    {
        ppm.fill_image(src);
        output_filename = std::string(argv[1]) + "_out.ppm";
    }

    std::clock_t begin = std::clock();
    // Run the scale operation:
    const int LOOP = 1000;
    for (int i = 0; i < LOOP; ++i) {
    scale.run();
    }
    std::clock_t end = std::clock();
    std::cout << "CPU  Time: " << (end - begin) / ((double)(CLOCKS_PER_SEC / 1000) * 100) << " ms" << std::endl;

    // Save the result to file:
    if(!output_filename.empty())
    {
        arm_compute::Tensor * t = dst.get_pyramid_level(0);
        arm_compute::utils::save_to_ppm((*t), output_filename); // save_to_ppm maps and unmaps the image to store as PPM
    }

}
