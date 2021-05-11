#include "opencv4/opencv2/opencv.hpp"

int main()
{
    std::string vid_src = "./test_video.mp4";
    cv::VideoCapture cap(vid_src);
    if (!cap.isOpened())
    {
        return -1;
    }

    cv::Mat frame;

    while (1)
    {
        if(cap.read(frame))
        {
            cv::imshow("img", frame);
            cv::waitKey(1);
        }
        else
        {
            std::cout<<"Video Finished!!!\n";
            break;
        }
    }

    return 0;
}