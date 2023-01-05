#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"

int main(int argc,char** argv)
{
    if(argc!=2&&argc!=3)
	{
		std::cout<<"Usage: ./PnP <intrinsic_json_path> Optional:<extrinsic_json_path>\n"
				"example:\n"
				"\t./bin/PnP ./data/test.json\n"
                "or:\n"
                "\t./bin/PnP ./data/test.json ./data/test.json"
                <<std::endl;
		return 0;
	}
	std::string intrinsic_json_path=argv[1];
    cv::Mat intrinsic,distortion,extrinsic=cv::Mat::eye(4,4,CV_32FC1);
    
    cv::Mat rvec=cv::Mat::zeros(3,1,CV_64FC1); //创建旋转矩阵
    cv::Mat tvec=cv::Mat::zeros(3,1,CV_64FC1); //创建平移矩阵
    cv::Mat rotate_Matrix=cv::Mat::eye(3,3,CV_64FC1);

    LoadIntrinsic(intrinsic_json_path,intrinsic,distortion);   //载入内参
    if(argc==3)
    {
        std::string extrinsic_json_path=argv[2];
        LoadExtrinsic(extrinsic_json_path,extrinsic);   //载入外参
        extrinsic=extrinsic.inv();

        extrinsic(cv::Rect(0,0,3,3)).copyTo(rotate_Matrix);

        cv::Rodrigues(rotate_Matrix,rvec);  //将旋转矩阵变换成旋转向量
        tvec.at<double>(0,0)=(double)extrinsic.at<float>(0,3);
        tvec.at<double>(1,0)=(double)extrinsic.at<float>(1,3);
        tvec.at<double>(2,0)=(double)extrinsic.at<float>(2,3);
    }
std::cout<<extrinsic<<std::endl;
std::cout<<rvec<<std::endl;
std::cout<<tvec<<std::endl;
std::cout<<rotate_Matrix<<std::endl;

    //将控制点在世界坐标系的坐标压入容器
    std::vector<cv::Point3f> objP;
    objP.clear();  
    objP.push_back(cv::Point3f(55.411636,0.207077,0.922405));
    objP.push_back(cv::Point3f(54.457287,11.195105,4.145446));
    objP.push_back(cv::Point3f(54.768158,16.919571,1.955162));
    objP.push_back(cv::Point3f(56.251995,-7.482208,1.894812));
    objP.push_back(cv::Point3f(57.712543,-12.825306,1.983112));
    
    objP.push_back(cv::Point3f(10.536441,2.817712,-0.716556));
    objP.push_back(cv::Point3f(11.045214,-3.227581,-0.537572));
    objP.push_back(cv::Point3f(32.472347,-0.500810,0.928433));
    objP.push_back(cv::Point3f(13.188197,-0.191766,0.085958));
    objP.push_back(cv::Point3f(105.905647,-6.861143,7.829010));

    //将之前已经检测到的角点的坐标压入容器
    std::vector<cv::Point2f> points;
    points.clear();
    points.push_back(cv::Point2f(656,338));
    points.push_back(cv::Point2f(401,260));
    points.push_back(cv::Point2f(272,315));
    points.push_back(cv::Point2f(834,313));
    points.push_back(cv::Point2f(945,315));
    
    points.push_back(cv::Point2f(318,440));
    points.push_back(cv::Point2f(1032,420));
    points.push_back(cv::Point2f(681,323));
    points.push_back(cv::Point2f(680,347));
    points.push_back(cv::Point2f(745,268));

    

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
    cv::solvePnPRansac(objP,points,intrinsic,distortion,rvec,tvec,argc==3,100,5.0,0.9899999999999999911,cv::noArray(),1);
    
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

    
    cv::Mat projection_matrix=get_projection_matrix(intrinsic,extrinsic,false);

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

    int x,y,z;
    cv::Mat world_point=cv::Mat::zeros(4,1,CV_64F);
    std::cout<<"输入世界坐标:"<<std::endl;
    while (std::cin>>x>>y>>z)
    {
        std::cout<<x<<" "<<y<<" "<<z<<" "<<std::endl;
        cv::Point2f pix=project_point(cv::Point3f(x,y,z),projection_matrix);
        
        std::cout<<"像素坐标:"<< pix <<std::endl<<pix.x<<" "<<pix.y<<std::endl;
    }
    return 0;
}