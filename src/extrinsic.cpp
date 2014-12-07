#include <cmath>
#include <ras_utils/ras_utils.h>
#include <ras_utils/basic_node.h>
#include <ras_utils/pcl_utils.h>
#include <iostream>
#include <fstream>
//ROS
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/LU>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



// ** Camera intrinsics (from /camera/depth_registered/camera_info topic)
#define FX              574.0527954101562
#define FY              574.0527954101562
#define FX_INV          1.0/FX
#define FY_INV          1.0/FY
#define CX              319.5
#define CY              239.5
#define IMG_ROWS        480
#define IMG_COLS        640

#define ROI_X_MIN   0 //300
#define ROI_X_MAX   640//500
#define ROI_Y_MIN   0//300
#define ROI_Y_MAX   480

class Extrinsic_Calibration : rob::BasicNode
{
public:
    typedef message_filters::sync_policies::
        ApproximateTime<sensor_msgs::Image,
                        sensor_msgs::Image> RGBD_Sync_Policy;
    typedef message_filters::Synchronizer<RGBD_Sync_Policy> RGBD_Sync;

    Extrinsic_Calibration();
    bool finished(){return finished_;}
private:
    ros::Subscriber pcl_sub_;

    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    boost::shared_ptr<RGBD_Sync> rgbd_sync_;

    Eigen::Vector3f t_;
    Eigen::Matrix3f R_;
    Eigen::Vector3f pos_robot_frame_;

    bool computed_t_, computed_R_, finished_;

    cv::Mat ROI_;

    int n_frames_;
    tf::TransformListener tf_listener_;
    /**
     * @brief Callback to process registered point cloud
     * @param pcl_msg
     */
    void PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg);
    void RGBD_Callback(const sensor_msgs::Image::ConstPtr &rgb_msg,
                       const sensor_msgs::Image::ConstPtr &depth_msg);

    void extractPlane(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                            pcl::ModelCoefficients::Ptr coefficients);
    void computeTransformation(double theta, Eigen::Matrix4f &transform);

    void extractCorners(const cv::Mat &bgr, const cv::Mat &ROI, std::vector<cv::Point> &corners);
    void compute_t(const std::vector<cv::Point> &corners, const cv::Mat &depth_img, Eigen::Vector3f pos_3D, Eigen::Matrix3f rot_3D);

    void save_to_file(const std::string &path, const Eigen::Matrix4f &transform);
};


int main(int argc, char* argv[])
{
    // ** Init node
    ros::init(argc, argv, "extrinsic_calibration");

    Extrinsic_Calibration ec;

    while(!ec.finished())
    {
        ros::spinOnce();
    }

    return 0;
}

Extrinsic_Calibration::Extrinsic_Calibration()
    :computed_t_(false), computed_R_(false), n_frames_(0), finished_(false)
{
    // ** Subscriber
    // PCL
    pcl_sub_ = n.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >
            ("/camera/depth_registered/points", 2, &Extrinsic_Calibration::PCL_Callback,this);

    // RGBD
    rgb_sub_.subscribe(n, "/camera/rgb/image_rect_color", 2);
    depth_sub_.subscribe(n, "/camera/depth_registered/hw_registered/image_rect_raw",2);

    rgbd_sync_.reset(new RGBD_Sync(RGBD_Sync_Policy(2), rgb_sub_, depth_sub_));
    rgbd_sync_->registerCallback(boost::bind(&Extrinsic_Calibration::RGBD_Callback, this, _1, _2));

    // ** Create ROI
    ROI_ = cv::Mat::zeros(480, 640, CV_8UC1);
    for(unsigned int i = ROI_Y_MIN; i < ROI_Y_MAX; ++i)
    {
        for(unsigned int j = ROI_X_MIN; j < ROI_X_MAX; ++j)
        {
            ROI_.at<uint8_t>(i,j) = 255;
        }
    }

    // ** Define position in 3D world coordinates
    pos_robot_frame_ << 0.197, 0.0, 0.0;
}

void Extrinsic_Calibration::PCL_Callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &pcl_msg)
{
    // ** Extract dominant plane
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    extractPlane(pcl_msg, coefficients);

    // ** Extract normal vector (in camera coordinates)
    pcl::Normal n (coefficients->values[0], coefficients->values[1],coefficients->values[2]);
    std::cout << "Normal vector (camera coordinates): ["<<n.normal_x<<","<<n.normal_y<<","<<n.normal_z<<"]"<<std::endl;

    // ** Get rotation angle
    double theta = acos(fabs(n.normal_y));
    std::cout << "Theta : "<<theta * 180 / M_PI << " degrees"<< std::endl;

    // ** Get 4D transformation matrix
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    computeTransformation(theta, transform);

    if(computed_t_)
    {        
        // ** Save to file
        save_to_file(RAS_Names::CALIBRATION_PATH, transform);
        std::cout<< "Transform: \n"<<transform<<std::endl;
        finished_ = true;
    }
}

void Extrinsic_Calibration::RGBD_Callback(const sensor_msgs::Image::ConstPtr &rgb_msg,
                                          const sensor_msgs::Image::ConstPtr &depth_msg)
{
    ROS_INFO("%d", n_frames_);
    if(n_frames_ > 50 && !finished_)
    {
        // ** Convert to OpenCV format
        cv_bridge::CvImageConstPtr rgb_ptr   = cv_bridge::toCvShare(rgb_msg);
        cv_bridge::CvImageConstPtr depth_ptr   = cv_bridge::toCvShare(depth_msg);

        const cv::Mat& rgb_img     = rgb_ptr->image;
        const cv::Mat& depth_img   = depth_ptr->image;

        if(computed_R_)
        {
            // ** Extract 4 strongest corners
            std::vector<cv::Point> corners;
            extractCorners(rgb_img, ROI_, corners);

            // ** Get 3D position (in camera frame)
            compute_t(corners, depth_img, pos_robot_frame_, R_);
        }
    }
    else
        ++n_frames_;
}

void Extrinsic_Calibration::extractPlane(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud,
                                               pcl::ModelCoefficients::Ptr coefficients)
{
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }
}

void Extrinsic_Calibration::computeTransformation(double theta, Eigen::Matrix4f &transform)
{
    // 3D rotations
    Eigen::Matrix3f rx, ry, rz;
    double theta_x = 0;
    double theta_y = theta;
    double theta_z = 0;

    rx << 1,          0,           0,
          0, cos(theta_x), -sin(theta_x),
          0, sin(theta_x), cos(theta_x);

    ry << cos(theta_y),     0,      sin(theta_y),
          0,                1,      0,
          -sin(theta_y),    0,      cos(theta_y);

    rz << cos(theta_z),     -sin(theta_z),  0,
          sin(theta_z),      cos(theta_z),  0,
            0,              0,              1;

    R_ = rx*ry*rz;

    computed_R_ = true;

    if(computed_t_)
    {
        // Final transformation
        transform <<  R_   , t_,
                    0,0,0  , 1;
    }
}

void Extrinsic_Calibration::extractCorners(const cv::Mat &bgr, const cv::Mat &ROI, std::vector<cv::Point> &corners)
{
    // ** Convert to grayscale
    cv::Mat gray, bgr2;
    cv::cvtColor(bgr, gray, CV_BGR2GRAY);

    // ** Good Features to Track
    cv::goodFeaturesToTrack(gray, corners, 4, 0.01, 5, ROI);

    // ** Draw corners
    bgr.copyTo(bgr2);
    for(std::size_t i = 0; i < corners.size(); ++i)
    {
        cv::circle(bgr2, corners[i], 5,  cv::Scalar(0,0,255));
    }
    cv::imshow("Corners",bgr2);
    cv::waitKey();
}

void Extrinsic_Calibration::compute_t(const std::vector<cv::Point> &corners,
                                      const cv::Mat &depth_img,
                                      Eigen::Vector3f pos_3D,
                                      Eigen::Matrix3f rot_3D)
{
    // ** Get middle point
    double x, y;
    for(std::size_t i = 0; i < corners.size(); ++i)
    {
        const cv::Point &p = corners[i];
        x += p.x;
        y += p.y;
    }
    x /= corners.size();
    y /= corners.size();

    std::cout << "x: "<<x <<", y:" << y<<std::endl;
    // ** Compute 3D position (camera optical frame)
    double X, Y, Z;
    Z = depth_img.at<float>(y, x);
    X = Z * ((x - CX) * FX_INV);
    Y = Z * ((y - CY) * FY_INV);

    std::cout << "POINT OPTICAL COORDINATES: "<<X<<","<<Y<<","<<Z<<std::endl;
    // ** Transform into camera link coordinates
    pcl::PointXYZ p(X,Y,Z);
    tf::Transform t_rgb_optical_to_camera_link;
    PCL_Utils::readTransform(COORD_FRAME_CAMERA_LINK, COORD_FRAME_CAMERA_RGB_OPTICAL, this->tf_listener_, t_rgb_optical_to_camera_link);
    Eigen::Matrix4f eigen_tf;
    PCL_Utils::convertTransformToEigen4x4(t_rgb_optical_to_camera_link, eigen_tf);
    PCL_Utils::transformPoint(p, eigen_tf, p);

    X = p.x;
    Y = p.y;
    Z = p.z;

    std::cout << "POINT CAMERA LINK COORDINATES: "<<X<<","<<Y<<","<<Z<<std::endl;

    // ** Compute T
    t_(0,0) = pos_3D(0,0) - rot_3D(0,0)*X - rot_3D(0,1)*Y - rot_3D(0,2)*Z;
    t_(1,0) = pos_3D(1,0) - rot_3D(1,0)*X - rot_3D(1,1)*Y - rot_3D(1,2)*Z;
    t_(2,0) = pos_3D(2,0) - rot_3D(2,0)*X - rot_3D(2,1)*Y - rot_3D(2,2)*Z;

    computed_t_ = true;
}

void Extrinsic_Calibration::save_to_file(const std::string &path, const Eigen::Matrix4f &transform)
{
    std::ofstream file;
    file.open(path);
    for(unsigned int i = 0; i < 4; ++i)
    {
        for(unsigned int j = 0; j<4; ++j)
        {
            file << transform(i,j)<<std::endl;
        }
    }
    file.close();
}
