#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"
#include"projection.hpp"

//读取各项配置和像素坐标系以及目标坐标系下的点对
bool loadConfig(const std::string &filename,std::vector<cv::Point2d> &pixel_points,std::vector<cv::Point3d> &target_points,bool &useExtrinsicGuess,int &iterationsCount,float &reprojectionError,double &confidence,int &flags)
{
    Json::Reader reader;
    Json::Value root;
    pixel_points.clear();
    target_points.clear();

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return false;
	}

	if(reader.parse(is,root))
	{
        useExtrinsicGuess=(root["useExtrinsicGuess"].isNull()||root["useExtrinsicGuess"].type()!=Json::booleanValue)?false:root["useExtrinsicGuess"].asBool();
        iterationsCount=(root["iterationsCount"].isNull()||root["iterationsCount"].type()!=Json::intValue)?100:root["iterationsCount"].asInt();
        reprojectionError=(root["reprojectionError"].isNull()||root["reprojectionError"].type()!=Json::realValue)?8.0:root["reprojectionError"].asFloat();
        confidence=(root["confidence"].isNull()||root["confidence"].type()!=Json::realValue)?0.9899999999999999911:root["confidence"].asDouble();
        flags=(root["flags"].isNull()||root["flags"].type()!=Json::intValue)?1:root["flags"].asInt();

		if(root["points"].isNull()||root["points"].type()!=Json::objectValue)
		{
            std::cout<<"Error points type:"<<filename<<std::endl;
			is.close();
			return false;
        }

        Json::Value::Members points=root["points"].getMemberNames();
        for(const std::string point:points)
        {
            if(root["points"][point].isNull()||root["points"][point].type()!=Json::arrayValue)
            {
                std::cout<<"Error point type:"<<filename<<":"<<point<<std::endl;
                is.close();
                return false;
            }
            for(unsigned int i=0;i<root["points"][point].size();i++)
            {
                if(root["points"][point][i].isNull()||root["points"][point][i].type()!=Json::arrayValue)
                {
                    std::cout<<"Error point type:"<<filename<<":"<<point<<":"<<i<<std::endl;
                    is.close();
                    return false;
                }
                if(root["points"][point][i].size()!=5)
                {
                    std::cout<<"Error point size:"<<filename<<":"<<point<<":"<<i<<std::endl;
                    is.close();
                    return false;
                }

                pixel_points.push_back(cv::Point2d(root["points"][point][i][0].asDouble(),root["points"][point][i][1].asDouble()));
                target_points.push_back(cv::Point3d(root["points"][point][i][2].asDouble(),root["points"][point][i][3].asDouble(),root["points"][point][i][4].asDouble()));
            }
        }
    }

	is.close();
	return true;
}

//根据目标坐标系中的点与投影矩阵，计算重投影在像素坐标系下的点的坐标，然后计算其与输入的像素坐标系中的点的距离，以获取投影误差
std::vector<double> getProjectionError(std::vector<cv::Point2d> &pixel_points,std::vector<cv::Point3d> &target_points,cv::Mat projection_matrix)
{
    std::vector<double> projection_error;
    projection_error.clear();

    for(size_t i=0;i<target_points.size();i++)
    {
        cv::Point2d projection_point=ProjectPoint(target_points[i],projection_matrix);
        projection_error.push_back(sqrt(pow(pixel_points[i].x-projection_point.x,2)+pow(pixel_points[i].y-projection_point.y,2)));
    }

    return projection_error;
}