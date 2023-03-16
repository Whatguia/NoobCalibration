#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>

//初始化颜色列表
void initColorList(std::vector<cv::Scalar> &color_list)
{
    color_list.clear();
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