#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "pcl_ros/point_cloud.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/CompressedImage.h>

// image related
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

// pointcloud related
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <pthread.h>

#include "self_labeller.h" 


 
using namespace std;  
using namespace cv; 
tf::TransformListener* tfListener = NULL;

image_transport::Publisher pub_img_raw, pub_img_label, pub_img_depth;
ros::Publisher  pub_cloud, pub_path;

bool rosbag_finished_ = false;
/////////////////////////////////////////////////////////////
string input_frame_ = "base_link_oriented";
string world_frame_ = "world_corrected";
string base_frame_  = "base_link_oriented";
string camera_frame_= "kinect2_rgb_optical_frame";

vector<Eigen::Matrix4f> transforms_to_camera_;
vector<Eigen::Matrix4f> transforms_to_world_;
vector<Mat> raw_images_;
vector<sensor_msgs::PointCloud2> registered_clouds_;

pcl::PointCloud<pcl::PointXYZRGB> robot_path_;
sensor_msgs::PointCloud2 registered_cloud_;
Eigen::Matrix4f cloud_to_camera_;

int count_ = 0;

Cloud_Image_Mapper *ci_mapper;

// sensor_msgs::ImageConstPtr img_raw_;
// ros::Time cloud_in_time_; 

void publish(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZRGB> cloud, int type = 2)
{
    sensor_msgs::PointCloud2 pointlcoud2;
    pcl::toROSMsg(cloud, pointlcoud2);

    if(type == 2)
    {
        pub.publish(pointlcoud2);
    }
    else
    {
        sensor_msgs::PointCloud pointlcoud;
        sensor_msgs::convertPointCloud2ToPointCloud(pointlcoud2, pointlcoud);

        pointlcoud.header = pointlcoud2.header;
        pub.publish(pointlcoud);   
    }
}

sensor_msgs::PointCloud2 transform_cloud(sensor_msgs::PointCloud2 cloud_in, string frame_target)
{
    ////////////////////////////////// transform ////////////////////////////////////////
    sensor_msgs::PointCloud2 cloud_out;
    tf::StampedTransform to_target;
   
    try   
    { 
        // tf_listener_->waitForTransform(frame_target, cloud_in.header.frame_id, cloud_in.header.stamp, ros::Duration(1.0));
        tfListener->lookupTransform(frame_target, cloud_in.header.frame_id, cloud_in.header.stamp, to_target);
        // tfListener->lookupTransform(frame_target, cloud_in.header.frame_id, ros::Time(0), to_target);
    }
    catch (tf::TransformException& ex) 
    { 
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        // return cloud_in;
    }  

    Eigen::Matrix4f eigen_transform;
    pcl_ros::transformAsMatrix (to_target, eigen_transform);
    pcl_ros::transformPointCloud (eigen_transform, cloud_in, cloud_out);

    cloud_out.header.frame_id = frame_target;
    return cloud_out;
}

pcl::PointCloud<pcl::PointXYZRGB> cloud_filter(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  input_cloud       (new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr  cloud_passthrough (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (input_cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (1.0, 500);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_passthrough);
    // cout << "after z filter  " << cloud_passthrough->points.size() << endl;

    // pass.setInputCloud (cloud_passthrough);
    // pass.setFilterFieldName ("x");
    // pass.setFilterLimits (x_min, x_max);
    // pass.filter (*cloud_passthrough);
    // // cout << "after x filter  " << cloud_passthrough->points.size() << endl;

    // pass.setInputCloud (cloud_passthrough);
    // pass.setFilterFieldName ("y");
    // pass.setFilterLimits (y_min, y_max);
    // pass.filter (*cloud_passthrough);
    // // cout << "after y filter  " << cloud_passthrough->points.size() << endl;

    return *cloud_passthrough; 
} 
 

Mat label_path()
{
    // convert pointcloud    
    pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud, path_camera, cloud_camera, filtered_world;

    // sensor_msgs::PointCloud2 cloud_transformed = transform_cloud(*registered_cloud_, world_frame_);
    // pcl::fromROSMsg(cloud_transformed, pcl_cloud); 
 
    // int start = transforms_to_camera_.size() - 10;
    // if(start < 0) 
    //     start = 0; 

    // for(int i = start; i < transforms_to_camera_.size(); i++)
    for(int i = 0; i < transforms_to_camera_.size()-5; i++)
    {   
        cout << "pose: " << i << " " << transforms_to_camera_.size() << endl;

        // transform cloud to world
        // sensor_msgs::PointCloud2 cloud_transformed = transform_cloud(registered_clouds_[i], world_frame_);
        pcl::fromROSMsg(registered_clouds_[i], pcl_cloud); 
 
        Eigen::Matrix4f transform_to_camera = transforms_to_camera_[i];
        Eigen::Matrix4f transforms_to_world = transforms_to_world_[i];
        pcl::transformPointCloud (pcl_cloud, cloud_camera, transform_to_camera);
        pcl::transformPointCloud (robot_path_, path_camera, transform_to_camera);

        cloud_camera = cloud_filter(cloud_camera); 

        if(!ci_mapper->get_disparity(raw_images_[i], camera_frame_, cloud_camera))
            continue; 
  
        path_camera = ci_mapper->get_visiable_path(raw_images_[i], path_camera);

        pcl::PointCloud<pcl::PointXYZRGB> path_world;
        pcl::transformPointCloud (path_camera, path_world, transforms_to_world);
        path_world.header.frame_id = world_frame_;
        publish(pub_path, path_world); 
        publish(pub_cloud, pcl_cloud); 

        sensor_msgs::ImagePtr img_raw      = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_images_[i]).toImageMsg();
        sensor_msgs::ImagePtr img_label    = cv_bridge::CvImage(std_msgs::Header(), "mono8", ci_mapper->img_path_).toImageMsg();
        sensor_msgs::ImagePtr img_depth    = cv_bridge::CvImage(std_msgs::Header(), "mono16", ci_mapper->img_disparity_).toImageMsg();

        pub_img_raw.publish(img_raw);
        pub_img_label.publish(img_label);
        pub_img_depth.publish(img_depth);

        // sleep(1);
        // imshow("image_raw", raw_images_[i]);
        // waitKey(1000);

        // break;
    }


    // return labeled_img;
}


void imageCallback_raw(sensor_msgs::ImageConstPtr image_msg)
{
    // start processing
    // cout << "raw image recieved: " <<image_msg->header.frame_id << endl;
    camera_frame_ = image_msg->header.frame_id;

    // if(rosbag_finished_)
    // {
    //     label_path();
    //     robot_path_.header.frame_id = world_frame_;
    //     // publish(pub_cloud, robot_path_);
    // }
    // else
    {
        // get camera position in world frame and get transform from world to camera frame
        tf::StampedTransform base_to_world, world_to_camera, camera_to_world;
        try 
        {
            tfListener->waitForTransform(world_frame_, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(0.1));
            tfListener->lookupTransform(world_frame_, base_frame_,  image_msg->header.stamp, base_to_world);
            tfListener->lookupTransform(image_msg->header.frame_id, world_frame_, image_msg->header.stamp, world_to_camera);
            tfListener->lookupTransform(world_frame_, image_msg->header.frame_id, image_msg->header.stamp, camera_to_world);
        }
        catch (tf::TransformException& ex) 
        {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            return;
        }   
 
        cout << "raw image recieved: " << robot_path_.points.size() << endl;
        // save raw rgb image 
        try {
            Mat image_raw = cv_bridge::toCvShare(image_msg, "bgr8")->image;
            raw_images_.push_back(image_raw.clone());

            // imshow("image", image_raw);
            // imshow("image_raw", raw_images_[0]);
            // waitKey(100);
        }
        catch (cv_bridge::Exception& ex){
            cout << ex.what() << endl;
            return; 
        } 

        // save path point and transforms
        pcl::PointXYZRGB camera_position; 
        camera_position.x = base_to_world.getOrigin().x(); 
        camera_position.y = base_to_world.getOrigin().y();
        camera_position.z = base_to_world.getOrigin().z() + 0.1;

        cout << camera_position.x << " " << camera_position.y << " " << camera_position.z << endl;
        robot_path_.points.push_back(camera_position);

        Eigen::Matrix4f eigen_transform, eigen_transform_toworld; 
        pcl_ros::transformAsMatrix (world_to_camera, eigen_transform);
        pcl_ros::transformAsMatrix (camera_to_world, eigen_transform_toworld);

        transforms_to_camera_.push_back(eigen_transform);
        transforms_to_world_.push_back(eigen_transform_toworld);
        registered_clouds_.push_back(registered_cloud_);

        robot_path_.header.frame_id = world_frame_;
        // publish(pub_cloud, robot_path_);

        count_ ++;
    }
}

void callback_velodyne(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
{
    // registered_cloud_ = cloud_in;
    input_frame_ = cloud_in->header.frame_id;

    tf::StampedTransform to_camera;
    try 
    {
        registered_cloud_ = transform_cloud(*cloud_in, world_frame_);
    }
    catch (tf::TransformException& ex) 
    {
        ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
        return;
    }
}
 
int main(int argc, char** argv)
{
    ros::init(argc, argv, "self_labeller");
 
    ros::NodeHandle node;  
    tfListener      = new (tf::TransformListener);
    ci_mapper       = new Cloud_Image_Mapper(tfListener);

    // robot_path_ = pcl::PointCloud<pcl::PointXYZRGB>();

    ros::Subscriber sub_cloud   = node.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, callback_velodyne);
    pub_cloud                   = node.advertise<sensor_msgs::PointCloud2>("/cloud_filtered", 1);
    pub_path                   = node.advertise<sensor_msgs::PointCloud2>("/robot_path", 1);
    //image_transport::Publisher pub = it.advertise("camera/image", 1);
    
    image_transport::ImageTransport it(node);
    // image_transport::Subscriber sub_raw = it.subscribe("/image_raw", 1, imageCallback_raw);

    pub_img_raw         = it.advertise("/self_label/raw", 1);
    pub_img_label       = it.advertise("/self_label/label", 1);
    pub_img_depth       = it.advertise("/self_label/depth", 1);

    int skip_msg_count = 0;  
 
    ros::Rate loop_rate(1);
    while (ros::ok()) 
    {
        sensor_msgs::ImageConstPtr msg = ros::topic::waitForMessage<sensor_msgs::Image>("/image_raw", ros::Duration(5));
        // if(msg == NULL)
        if(skip_msg_count < 2)
        { 
            skip_msg_count ++;
            cout << "wait for topic msg " << skip_msg_count << endl;
        }
        // else if(msg == NULL || skip_msg_count > 20) // finish after cerain number of messages, for testing
        else if(msg == NULL && skip_msg_count > 2) // rosbag finish
        {
            cout << "rosbag finished  " << endl;
            label_path();
            robot_path_.header.frame_id = world_frame_;
            break;
        }
        else if(msg != NULL)
            imageCallback_raw(msg);
        
        skip_msg_count ++;
        ros::spinOnce();
        loop_rate.sleep();
    }
    cout << "out of loop" << endl;
    // ros::spinOnce();

    return 0;
}

