#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>

void Init_color_list(std::vector<cv::Scalar> &color_list)
{
    color_list.push_back(cv::Scalar(0,255,0));
    color_list.push_back(cv::Scalar(255,255,0));
    color_list.push_back(cv::Scalar(34,34,178));
    color_list.push_back(cv::Scalar(240,34,160));
    color_list.push_back(cv::Scalar(0,165,255));
    color_list.push_back(cv::Scalar(0,0,255));
    color_list.push_back(cv::Scalar(255,0,0));
    color_list.push_back(cv::Scalar(79,79,47));
    color_list.push_back(cv::Scalar(203,192,255));
    color_list.push_back(cv::Scalar(255,0,255));

    return;
}