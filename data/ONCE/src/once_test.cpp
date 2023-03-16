#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"
#include"color_list.hpp"
#include"bat3D.hpp"

int main(int argc,char** argv)
{
    if(argc!=4&&argc!=5)
	{
		std::cout<<"Usage: ./test <image_path> <intrinsic_json_path> <extrinsic_json_path> Optional:<boxes_json_path>\n"
				"example:\n"
				"\t./bin/test ./data/test.jpg ./data/test.json ./data/test.json\n"
                "or:\n"
				"\t./bin/test ./data/test.jpg ./data/test.json ./data/test.json ./data/boxes.json"
                <<std::endl;
		return 0;
	}
    std::string image_path=argv[1]; //图像路径
	std::string intrinsic_json_path=argv[2];    //内参json文件路径
	std::string extrinsic_json_path=argv[3];    //外参json文件路径
    std::string boxes_json_path;    //可选参数，bat3D标注json文件路径

    cv::Mat image=cv::imread(image_path);

    cv::Mat intrinsic,distortion;   //相机内参、畸变系数
    cv::Size image_size;    //相机内参对应的图像大小
    loadIntrinsic(intrinsic_json_path,intrinsic,distortion,image_size);   //载入内参
    cv::Mat undistort_intrinsic=cv::getOptimalNewCameraMatrix(intrinsic,distortion,image_size,0.0,image_size);  //根据内参与畸变系数计算去畸变后的内参
    cv::Mat undistort_image;
    cv::undistort(image,undistort_image,intrinsic,distortion);  //图像去畸变

    cv::Mat extrinsic;
    loadExtrinsic(extrinsic_json_path,extrinsic);   //载入外参

    cv::Mat projection_matrix=getProjectionMatrix(intrinsic,extrinsic);   //根据内参与外参计算投影矩阵，注意外参的目标，默认从文件读取的是从相机到目标的外参，因此会使用外参矩阵的逆矩阵
    cv::Mat undistort_projection_matrix=getProjectionMatrix(intrinsic,extrinsic);   //去畸变后的投影矩阵
    
    std::cout<<"projection_matrix:\n"<<projection_matrix<<std::endl;
    std::cout<<"undistort_projection_matrix:\n"<<undistort_projection_matrix<<std::endl;

    std::vector<cv::Scalar> color_list;
    initColorList(color_list);  //初始化颜色列表，只是画框时好看一点，没啥用
    std::vector<cv::Mat> boxes;
    if(argc==5)
    {
        //当可选参数<boxes_json_path>启用时，对读入的bat3D标注进行绘制
        boxes_json_path=argv[4];    //bat3D标注json文件路径
        loadBat3D(boxes_json_path,boxes);   //载入bat3D标注

        for(size_t i=0;i<boxes.size();i++)
        {
            drawBoxCorners(undistort_image,getBox(boxes[i]),undistort_projection_matrix,color_list[i%10]);  //根据标注信息获取标注框，然后与投影矩阵进行运算获取像素坐标系下的框，并进行绘制
        }

        cv::imshow("test",undistort_image);
        cv::waitKey(0);
    }
    else
    {
        //当可选参数<boxes_json_path>未启用时，保存去畸变后的图像，然后随便整点东西看
        cv::imwrite(image_path+".undisrorted."+image_path.substr(image_path.length()-4,image_path.length()-1),undistort_image);
        
        int i=0;
        cv::Mat bev=cv::Mat::zeros(1000,1000,CV_8UC3);  //bev
        cv::circle(bev,cv::Point(500,500),3,cv::Scalar(255,255,255),3,8);   //坐标原点
        cv::circle(bev,cv::Point(500-extrinsic.at<float>(1,3)*10,500-extrinsic.at<float>(0,3)*10),2,cv::Scalar(0,255,0),2,8);   //相机位置
        while(true)
        {
            cv::Mat image_copy=image.clone();
            cv::Mat undistort_image_copy=undistort_image.clone();
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

            drawBoxCorners(image_copy,getBox(box),projection_matrix,color_list[i%10]);
            drawBoxCorners(undistort_image_copy,getBox(box),undistort_projection_matrix,color_list[i%10]);
            drawBoxBev(bev_copy,getBox(box),color_list[i%10]);
            drawBoxCorners(image_copy,getBox(box_no_bev),projection_matrix,color_list[i%10+1]);
            drawBoxCorners(undistort_image_copy,getBox(box_no_bev),undistort_projection_matrix,color_list[i%10+1]);
            cv::circle(image_copy,ProjectPoint(cv::Point3f(i,0,0),projection_matrix),1,color_list[(i+2)%10]);
            cv::circle(undistort_image_copy,ProjectPoint(cv::Point3f(i,0,0),undistort_projection_matrix),1,color_list[(i+2)%10]);

            cv::imshow("result",image_copy);
            cv::imshow("undistort_result",undistort_image_copy);
            cv::imshow("bev",bev_copy);
            if(cv::waitKey()==27)
            {
                break;
            }
        }
    }

    return 0;
}