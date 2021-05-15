#ifndef _ARMOR_POS_HPP_
#define _ARMOR_POS_HPP_

#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <cmath>
#include <algorithm>

#define ENEMY_BLUE 0
#define ENEMY_RED 1

#define BL 0
#define TL 1
#define TR 2
#define BR 3

/******************* 灯条类定义 ***********************/
class LightBlob
{
public:
    cv::RotatedRect rect; //灯条位置
    double area_ratio;
    double length;      //灯条长度
    uint8_t blob_color; //灯条颜色

    LightBlob(cv::RotatedRect &r, double ratio, uint8_t color) : rect(r), area_ratio(ratio), blob_color(color)
    {
        length = std::max(rect.size.height, rect.size.width);
    };
    LightBlob() = default;
};

struct Vector2f
{
    cv::Point2f v;
    cv::Point2f begin;
    cv::Point2f end;
    float norm;
    void flip(void)
    {
        v = -v;
        cv::swap(begin, end);
    }
    float dot(const Vector2f &v) const
    {
        return this->v.dot(v.v);
    }
    bool operator==(const Vector2f &a)
    {
        return a.v == this->v && a.begin == this->begin && a.end == this->end;
    }
    Vector2f()
    {
        v = cv::Point2f(0, 0);
        begin = cv::Point2f(0, 0);
        end = cv::Point2f(0, 0);
        norm = 0;
    }
    float GetNorm(const cv::Point2f &v) const
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }
    float GetNorm(void) const
    {
        return norm;
    }

    Vector2f(const LightBlob &b)
    {
        cv::Point2f *p_array = new cv::Point2f[4];
        b.rect.points(p_array);

        if (GetNorm(p_array[TL] - p_array[BL]) > GetNorm(p_array[BR] - p_array[BL]))
        {
            //get longer edge
            end = (p_array[TL] + p_array[TR]) / 2;
            begin = (p_array[BL] + p_array[BR]) / 2;
            v = begin - end;
            norm = GetNorm(v);
        }
        else
        {
            end = (p_array[BR] + p_array[TR]) / 2;
            begin = (p_array[BL] + p_array[TL]) / 2;
            v = begin - end;
            norm = GetNorm(v);
        }
    }
};

typedef std::vector<LightBlob> LightBlobs;

bool findLightBlobs(const cv::Mat &src, std::pair<Vector2f, Vector2f> &lb_v, const bool blob_color, const int light_threshold, const int dark_threshold);
Vector2f GetLightBlobVector(const LightBlob &b);
float GetNorm(const cv::Point2f &v);

#endif