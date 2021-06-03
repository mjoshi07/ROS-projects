#include "opencv4/opencv2/opencv.hpp"

int main()
{
    // path to the video file
    std::string vid_src = "./test_video.mp4";
    
    // create an Opencv Video Capture Object
    cv::VideoCapture cap(vid_src);
    
    // check if video capture object is open
    if (!cap.isOpened())
    {
        return -1;
    }

    // create an Opencv Mat object
    cv::Mat frame;

    while (1)
    {
        // input the video data into opencv mat object, frame
        if(cap.read(frame))
        {
            // display the frame
            cv::imshow("img", frame);
            
            // wait for 1 millisecond and then get a new image
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
