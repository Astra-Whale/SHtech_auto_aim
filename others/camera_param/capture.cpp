#include "hikcam_wrapper.h"

int main()
{
    WrapperHead *video = nullptr;
    std::string path = "../../picture/";
    video = new HikCamWrapper();
    bool state = video->init();
    std::cout << "camera init" << state;
    cv::namedWindow("hello");
    cv::Mat frame;
    int i = 0;
    while (state)
    {
        video->read(frame);
        if (!frame.empty())
        {
            if (32 == cv::waitKey(20))
            {
                std::string name = path + std::to_string(i) + ".jpg";
                cv::imwrite(name, frame);
                std::cout << name << std::endl;
                i++;
            }
            cv::imshow("hello", frame);

            if (97 == cv::waitKey(20))
            {
                break;
            }
        }
    }
}