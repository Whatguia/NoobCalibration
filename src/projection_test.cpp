#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"
#include"color_list.hpp"

std::vector<cv::Scalar> color_list;

int main(int argc,char** argv)
{
    if(argc!=4)
	{
		std::cout<<"Usage: ./Coordinate_Test <image_path> <intrinsic_json_path> <extrinsic_json_path>\n"
				"example:\n"
				"\t./bin/Coordinate_Test ./data/test.png ./data/test.json ./data/test.json"
                <<std::endl;
		return 0;
	}
    std::string image_path=argv[1];
	std::string intrinsic_json_path=argv[2];
	std::string extrinsic_json_path=argv[3];
    cv::Mat intrinsic,distortion,extrinsic,image=cv::imread(image_path);
    Init_color_list(color_list);

    LoadIntrinsic(intrinsic_json_path,intrinsic,distortion);   //载入内参
    LoadExtrinsic(extrinsic_json_path,extrinsic);   //载入外参
    cv::Mat projection_matrix=get_projection_matrix(intrinsic,extrinsic);   //根据内参与外参计算投影矩阵，注意外参的目标

    cv::Mat bev=cv::Mat::zeros(1000,1000,CV_8UC3);  //bev
    cv::circle(bev,cv::Point(500,500),3,cv::Scalar(255,255,255),3,8);   //坐标原点
    cv::circle(bev,cv::Point(500-extrinsic.at<float>(1,3)*10,500-extrinsic.at<float>(0,3)*10),2,cv::Scalar(0,255,0),2,8);   //相机位置

    int i=0;
    while(true)
    {
        cv::Mat image_copy=image.clone();
        cv::Mat bev_copy=bev.clone();

        cv::Mat box(9,1,CV_32FC1);cv::Mat box_no_bev(9,1,CV_32FC1);
        float* row_data=box.ptr<float>(0);float* row_data_no_bev=box_no_bev.ptr<float>(0);
        // for(int i=0;i<7;i++)
        // {
        //     std::cin>>row_data[i];
        // }
        row_data[0]=i++;row_data_no_bev[0]=(float)i/2.0;    //x
        row_data[1]=2;row_data_no_bev[1]=-3;  //y
        row_data[2]=0;row_data_no_bev[2]=(float)i/100.0;  //z
        row_data[3]=2;row_data_no_bev[3]=2.5;  //w
        row_data[4]=4;row_data_no_bev[4]=5;  //l
        row_data[5]=2;row_data_no_bev[5]=2;  //h
        row_data[6]=0;row_data_no_bev[6]=0;  //roll
        row_data[7]=0;row_data_no_bev[7]=0;  //pitch
        row_data[8]=0.2;row_data_no_bev[8]=0;  //yaw

        draw_box_corners(image_copy,get_box(box),projection_matrix,color_list[i%10]);
        draw_box_bev(bev_copy,get_box(box),color_list[i%10]);
        draw_box_corners(image_copy,get_box(box_no_bev),projection_matrix,color_list[i%10+1]);
        cv::circle(image_copy,project_point(cv::Point3f(i,0,0),projection_matrix),1,color_list[(i+2)%10]);

        cv::imshow("result",image_copy);
        cv::imshow("bev",bev_copy);
        if(cv::waitKey()==27)
        {
            break;
        }
    }
    return 0;
}
