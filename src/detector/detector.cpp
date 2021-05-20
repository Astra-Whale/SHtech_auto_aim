#include "detector.hpp"
#include "image.hpp"

ArmorDetector::ArmorDetector(const std::string &classesFile, const std::string &modelConfig, const std::string &modelWeights, bool enemy)
{
    this->enemy = enemy;
    if (enemy == ENEMY_BLUE)
    {
        enemy_label = "armor_blue";
    }
    else
    {
        enemy_label = "armor_red";
    }

    this->classesFile = classesFile;
    this->modelConfig = modelConfig;
    this->modelWeights = modelWeights;
    std::ifstream ifs;
    ifs.open(classesFile.c_str());
    std::string line;
    while (getline(ifs, line))
    {
        classes.push_back(line);
    }
    ifs.close();
    detector = new Detector(modelConfig, modelWeights, 0);
}

ArmorDetector::~ArmorDetector()
{
    free(detector);
}

bool ArmorDetector::detect(std::shared_ptr<image_t> detImg, const Image &frame, std::vector<bbox_t> &outs)
{

    std::vector<bbox_t> tmp = detector->detect_resized(*detImg, frame.frame.cols, frame.frame.rows, 0.5);
    outs.clear();
    for (unsigned int i = 0; i < tmp.size(); i++)
    {
        if (classes[tmp[i].obj_id] == enemy_label)
        {
            outs.push_back(tmp[i]);
        }
    }
    return outs.size();
}

//画出检测结果
void ArmorDetector::Drawer(cv::Mat &frame, const std::vector<bbox_t> &outs, const std::vector<std::string> &classes)
{
    //获取所有最佳检测框信息
    for (unsigned int i = 0; i < outs.size(); i++)
    {
        DrawBoxes(frame, classes, outs[i].obj_id, outs[i].prob, outs[i].x, outs[i].y,
                  outs[i].x + outs[i].w, outs[i].y + outs[i].h);
    }
}

//画出检测框和相关信息
void ArmorDetector::DrawBoxes(cv::Mat &frame, std::vector<std::string> classes, int classId, float conf, int left, int top, int right, int bottom)
{
    //画检测框
    cv::rectangle(frame, cv::Point(left, top), cv::Point(right, bottom), cv::Scalar(255, 178, 50), 3);
    //该检测框对应的类别和置信度
    std::string label = cv::format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }
    //将标签显示在检测框顶部
    int baseLine;
    cv::Size labelSize = getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = std::max(top, labelSize.height);
    cv::rectangle(frame, cv::Point(left, top - round(1.5 * labelSize.height)), cv::Point(left + round(1.5 * labelSize.width), top + baseLine), cv::Scalar(0, 0, 0), cv::FILLED);
    putText(frame, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(255, 255, 255), 1);
}
