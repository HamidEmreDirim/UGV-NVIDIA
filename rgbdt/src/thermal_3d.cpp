#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    // RealSense setup
    rs2::pipeline pipe;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH);
    cfg.enable_stream(RS2_STREAM_COLOR);
    pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // Thermal camera setup
    int thermal_device = 0; // Default thermal camera device number
    if (argc > 1)
        thermal_device = std::stoi(argv[1]);
    cv::VideoCapture cap(thermal_device, cv::CAP_V4L);
    if (!cap.isOpened())
    {
        std::cerr << "Failed to open thermal camera." << std::endl;
        return -1;
    }
    cv::namedWindow("Thermal", cv::WINDOW_NORMAL);

    // Main loop
    while (true)
    {
        // RealSense
        rs2::frameset frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);
        rs2::video_frame color_frame = frames.get_color_frame();
        rs2::depth_frame depth_frame = frames.get_depth_frame();
        cv::Mat color(cv::Size(color_frame.get_width(), color_frame.get_height()), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);

        // Thermal camera
        cv::Mat thermal_frame;
        cap >> thermal_frame;
        if (thermal_frame.empty())
        {
            std::cerr << "Failed to capture thermal frame." << std::endl;
            break;
        }

        // Display aligned stream
        cv::imshow("Thermal", color);

        // Check for user input to exit
        char key = cv::waitKey(1);
        if (key == 'q')
            break;
    }

    // Release resources
    pipe.stop();
    cap.release();
    cv::destroyAllWindows();

    return 0;
}