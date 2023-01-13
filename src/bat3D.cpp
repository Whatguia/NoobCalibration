#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"
#include"color_list.hpp"

int main(int argc,char** argv)
{
    if(argc!=4&&argc!=5)
	{
		std::cout<<"Usage: ./projection_test <image_path> <intrinsic_json_path> <extrinsic_json_path> Optional:<boxes_json_path>\n"
				"example:\n"
				"\t./bin/projection_test ./data/test.png ./data/test.json ./data/test.json\n"
                "or:\n"
				"\t./bin/projection_test ./data/test.png ./data/test.json ./data/test.json ./data/boxes.json"
                <<std::endl;
		return 0;
	}
    std::string image_path=argv[1]; //图像路径
	std::string intrinsic_json_path=argv[2];    //内参json文件路径
	std::string extrinsic_json_path=argv[3];    //外参json文件路径
    std::string boxes_json_path;    //可选参数，bat3D标注json文件路径

    cv::Mat intrinsic,distortion,extrinsic,image=cv::imread(image_path);
    std::vector<cv::Mat> boxes;

    std::vector<cv::Scalar> color_list;
    initColorList(color_list);  //初始化颜色列表，只是画框时好看一点，没啥用
    loadIntrinsic(intrinsic_json_path,intrinsic,distortion);   //载入内参
    loadExtrinsic(extrinsic_json_path,extrinsic);   //载入外参
    cv::Mat projection_matrix=getProjectionMatrix(intrinsic,extrinsic);   //根据内参与外参计算投影矩阵，注意外参的目标，默认从文件读取的是从相机到目标的外参，因此会使用外参矩阵的逆矩阵

    cv::Mat undistort_intrinsic=cv::getOptimalNewCameraMatrix(intrinsic,distortion,cv::Size(1280,720),0.0,cv::Size(1280,720));  //根据内参与畸变系数计算去畸变后的内参
    cv::Mat undistort_projection_matrix=getProjectionMatrix(undistort_intrinsic,extrinsic);   //去畸变后的投影矩阵
    cv::Mat undistort_image;
    cv::undistort(image,undistort_image,intrinsic,distortion,undistort_intrinsic);  //图像去畸变

    if(argc==5)
    {
        //当可选参数<boxes_json_path>启用时，认为输入的图像是已经经过畸变矫正后的图像，因此在image上对读入的bat3D标注进行绘制
        boxes_json_path=argv[4];    //bat3D标注json文件路径
        loadBat3D(boxes_json_path,boxes);    //载入bat3D标注

        for(size_t i=0;i<boxes.size();i++)
        {
            drawBoxCorners(image,getBoxBat3D(boxes[i]),undistort_projection_matrix,color_list[i%10]);   //根据标注信息获取标注框，然后与投影矩阵进行运算获取像素坐标系下的框，并进行绘制
        }
    }
    else
    {
        //当可选参数<boxes_json_path>未启用时，认为输入的图像未经过畸变矫正图像，因此打印去畸变后的投影矩阵，并保存去畸变后的图像
        std::cout<<undistort_projection_matrix<<std::endl;
        cv::imwrite(image_path+".undisrorted."+image_path.substr(image_path.length()-4,image_path.length()-1),undistort_image);
        image=undistort_image;
    }

    cv::imshow("bat3D",image);
    cv::waitKey(0);

    return 0;
}
