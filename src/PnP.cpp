#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"

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

int main(int argc,char** argv)
{
    if(argc!=3&&argc!=4)
	{
		std::cout<<"Usage: ./PnP <points_json_path> <intrinsic_json_path> Optional:<extrinsic_json_path>\n"
				"example:\n"
				"\t./bin/PnP ./data/points.json ./data/test.json\n"
                "or:\n"
                "\t./bin/PnP ./data/points.json ./data/test.json ./data/test.json"
                <<std::endl;
		return 0;
	}
    std::string point_json_path=argv[1];
	std::string intrinsic_json_path=argv[2];
    std::string extrinsic_json_path;

    std::vector<cv::Point2f> pixel_points;  //像素坐标系下的点
    std::vector<cv::Point3f> target_points; //目标坐标系下的点

    cv::Mat intrinsic,distortion,extrinsic=cv::Mat::eye(4,4,CV_32FC1);  //相机内参、畸变系数、相机到目标的外参
    
    cv::Mat rvec=cv::Mat::zeros(3,1,CV_64FC1); //创建旋转矩阵
    cv::Mat tvec=cv::Mat::zeros(3,1,CV_64FC1); //创建平移矩阵
    cv::Mat rotate_Matrix=cv::Mat::eye(3,3,CV_64FC1);

    loadPoints(point_json_path,pixel_points,target_points); //载入像素坐标系以及目标坐标系中点的信息
    loadIntrinsic(intrinsic_json_path,intrinsic,distortion);   //载入内参
    if(argc==4)
    {
        extrinsic_json_path=argv[3];
        loadExtrinsic(extrinsic_json_path,extrinsic);   //载入外参
        extrinsic=extrinsic.inv();

        extrinsic(cv::Rect(0,0,3,3)).copyTo(rotate_Matrix);

        cv::Rodrigues(rotate_Matrix,rvec);  //将旋转矩阵变换成旋转向量
        tvec.at<double>(0,0)=(double)extrinsic.at<float>(0,3);
        tvec.at<double>(1,0)=(double)extrinsic.at<float>(1,3);
        tvec.at<double>(2,0)=(double)extrinsic.at<float>(2,3);
    }

    //求解pnp
    /*
    bool cv::solvePnPRansac(
        cv::InputArray objectPoints,    Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel, where N is the number of points. vector<Point3d> can be also passed here.
        cv::InputArray imagePoints,     Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel, where N is the number of points. vector<Point2d> can be also passed here.
        cv::InputArray cameraMatrix,    Input camera matrix
        cv::InputArray distCoeffs,      Input vector of distortion coefficients
        cv::OutputArray rvec,           Output rotation vector (see Rodrigues ) that, together with tvec, brings points from the model coordinate system to the camera coordinate system.
        cv::OutputArray tvec,           Output translation vector.
        bool useExtrinsicGuess = false, Parameter used for SOLVEPNP_ITERATIVE. If true (1), the function uses the provided rvec and tvec values as initial approximations of the rotation and translation vectors, respectively, and further optimizes them.
        int iterationsCount = 100,      Number of iterations.
        float reprojectionError = (8.0F),   Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
        double confidence = (0.9899999999999999911),    The probability that the algorithm produces a useful result.
        cv::OutputArray inliers = noArray(),    Output vector that contains indices of inliers in objectPoints and imagePoints .
        int flags = 0   Method for solving a PnP problem (see solvePnP ). The function estimates an object pose given a set of object points, their corresponding image projections, as well as the camera matrix and the distortion coefficients. This function finds such a pose that minimizes reprojection error, that is, the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints ) objectPoints. The use of RANSAC makes the function resistant to outliers.
    )
    */
    //切记，虽然Opencv的参数偏爱float类型，但是solvepnp中除了相机内参和畸变参数矩阵是用float类型外，其余的矩阵都是double类型，不然出出现计算结果不正确的情况。
    cv::solvePnPRansac(target_points,pixel_points,intrinsic,distortion,rvec,tvec,argc==4,100,5.0,0.9899999999999999911,cv::noArray(),1);
    
    cv::Rodrigues(rvec,rotate_Matrix);  //将旋转向量变换成旋转矩阵

    extrinsic.at<float>(0,0)=(float)rotate_Matrix.at<double>(0,0);
    extrinsic.at<float>(0,1)=(float)rotate_Matrix.at<double>(0,1);
    extrinsic.at<float>(0,2)=(float)rotate_Matrix.at<double>(0,2);
    extrinsic.at<float>(1,0)=(float)rotate_Matrix.at<double>(1,0);
    extrinsic.at<float>(1,1)=(float)rotate_Matrix.at<double>(1,1);
    extrinsic.at<float>(1,2)=(float)rotate_Matrix.at<double>(1,2);
    extrinsic.at<float>(2,0)=(float)rotate_Matrix.at<double>(2,0);
    extrinsic.at<float>(2,1)=(float)rotate_Matrix.at<double>(2,1);
    extrinsic.at<float>(2,2)=(float)rotate_Matrix.at<double>(2,2);
    extrinsic.at<float>(3,3)=1;
    extrinsic.at<float>(0,3)=(float)tvec.at<double>(0,0);
    extrinsic.at<float>(1,3)=(float)tvec.at<double>(1,0);
    extrinsic.at<float>(2,3)=(float)tvec.at<double>(2,0);

    cv::Mat projection_matrix=getProjectionMatrix(intrinsic,extrinsic,false);

    std::cout<<"\n内参:"<<std::endl;
    std::cout<<intrinsic<<std::endl;
    std::cout<<"\n畸变:"<<std::endl;
    std::cout<<distortion<<std::endl;
    std::cout<<"\n外参(目标到相机):"<<std::endl;
    std::cout<<extrinsic<<std::endl;
    std::cout<<"\n外参(相机到目标):"<<std::endl;
    std::cout<<extrinsic.inv()<<std::endl;
    std::cout<<"\n投影矩阵:"<<std::endl;
    std::cout<<projection_matrix<<std::endl;

    if(argc==4)
    {
        saveExtrinsic(extrinsic_json_path,extrinsic.inv());
    }
    // int x,y,z;
    // cv::Mat world_point=cv::Mat::zeros(4,1,CV_64F);
    // std::cout<<"输入世界坐标:"<<std::endl;
    // while (std::cin>>x>>y>>z)
    // {
    //     std::cout<<x<<" "<<y<<" "<<z<<" "<<std::endl;
    //     cv::Point2f pix=project_point(cv::Point3f(x,y,z),projection_matrix);
        
    //     std::cout<<"像素坐标:"<< pix <<std::endl<<pix.x<<" "<<pix.y<<std::endl;
    // }
    return 0;
}