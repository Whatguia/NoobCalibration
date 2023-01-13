#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"
#include"projection.hpp"

//读取像素坐标系以及目标坐标系下的点对
void loadPoints(const std::string &filename,std::vector<cv::Point2f> &pixel_points,std::vector<cv::Point3f> &target_points)
{
    Json::Reader reader;
	Json::Value root;
    pixel_points.clear();
    target_points.clear();

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		if(root["pixel_points"].isNull()||root["pixel_points"].type()!=Json::arrayValue||root["target_points"].isNull()||root["target_points"].type()!=Json::arrayValue)
		{
            std::cout<<"Error points type:"<<filename<<std::endl;
			is.close();
			return;
        }
        if(root["pixel_points"].size()!=root["target_points"].size())
        {
            std::cout<<"Error size of pixel_points and target_points:"<<filename<<std::endl;
            is.close();
            return;
        }
        
        for(unsigned int i=0;i<root["pixel_points"].size();i++)
        {
            if(root["pixel_points"][i].isNull()||root["pixel_points"][i].type()!=Json::arrayValue||root["target_points"][i].isNull()||root["target_points"][i].type()!=Json::arrayValue)
            {
                std::cout<<"Error point type:"<<filename<<":"<<i<<std::endl;
                is.close();
                return;
            }
            if(root["pixel_points"][i].size()!=2||root["target_points"][i].size()!=3)
            {
                std::cout<<"Error point size:"<<filename<<":"<<i<<std::endl;
                is.close();
                return;
            }

            pixel_points.push_back(cv::Point2f(root["pixel_points"][i][0].asFloat(),root["pixel_points"][i][1].asFloat()));
            target_points.push_back(cv::Point3f(root["target_points"][i][0].asFloat(),root["target_points"][i][1].asFloat(),root["target_points"][i][2].asFloat()));
        }
    }

	is.close();
	return;
}

//根据目标坐标系中的点与投影矩阵，计算重投影在像素坐标系下的点的坐标，然后计算其与输入的像素坐标系中的点的距离，以获取重投影误差
std::vector<float> getProjectionError(std::vector<cv::Point2f> &pixel_points,std::vector<cv::Point3f> &target_points,cv::Mat projection_matrix)
{
    std::vector<float> projection_error;
    projection_error.clear();

    for(size_t i=0;i<target_points.size();i++)
    {
        cv::Point2f projection_point=ProjectPoint(target_points[i],projection_matrix);
        projection_error.push_back(sqrt(pow(pixel_points[i].x-projection_point.x,2)+pow(pixel_points[i].y-projection_point.y,2)));
    }

    return projection_error;
}