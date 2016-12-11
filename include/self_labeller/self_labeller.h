#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/passthrough.h>

using namespace std;
using namespace cv;

#define FLAT    0
#define ROUGH   100 
#define OBS     200

float f = 2262.52;
float base_line = 0.209313;
float scale = f*base_line/2.0;

class Cloud_Image_Mapper
{
  tf::TransformListener *tf_listener_;
  // image_geometry::PinholeCameraModel cam_model_;
  CvFont font_;

public:
  ros::Time cloud_in_time_;

  Mat img_ori, image_label;
  Mat img_seg_, img_path_, img_fused_, img_depth_, img_disparity_, img_all_, img_rgb_;

  pcl::PointCloud<pcl::PointXYZRGB> cloud_g;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_v;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_f;

  Cloud_Image_Mapper(tf::TransformListener *tf_listener)
  {
    tf_listener_ = tf_listener;
  }

  cv::Point2d project3D_to_image(cv::Point3d& xyz, string frame_id )
  {
    double fx, fy, cx, cy; 
    if(frame_id == "kinect2_rgb_optical_frame")
    {
      fx = 529.9732789120519;
      fy = 526.9663404399863;
      cx = 477.4416333879422;
      cy = 261.8692914553029;
    }
    else
    {
      fx = 775.3905399535238;
      fy = 775.3925549639409;
      cx = 651.1391917338947;
      cy = 394.3686338123942;
    }
    cv::Point2d uv_rect;
    uv_rect.x = (fx*xyz.x) / xyz.z + cx;
    uv_rect.y = (fy*xyz.y) / xyz.z + cy;
    return uv_rect;
  }

  void set_point_color(pcl::PointXYZRGB &point, int label)
  {
    if(label == FLAT)
    {
      point.r = 0;
      point.g = 0;
      point.b = 255;
    }
    else if(label == ROUGH)
    {
      point.r = 255;
      point.g = 255;
      point.b = 0;
    }
    else if(label == OBS)
    {
      point.r = 255;
      point.g = 0;
      point.b = 0;
    }      
  }

  Scalar get_pixel_color(int label)
  {
    Scalar pix_color;
    if(label == FLAT)
    {
      pix_color.val[2] = 0;
      pix_color.val[1] = 0;
      pix_color.val[0] = 255;
    }
    else if(label == ROUGH)
    {
      pix_color.val[2] = 255;
      pix_color.val[1] = 255;
      pix_color.val[0] = 0;
    }
    else if(label == OBS)
    {
      pix_color.val[2] = 255;
      pix_color.val[1] = 0;
      pix_color.val[0] = 0;
    }      

    return pix_color;
  }

  pcl::PointCloud<pcl::PointXYZRGB> get_visiable_path(Mat image_raw, pcl::PointCloud<pcl::PointXYZRGB> cloud_path)
  {
    // img_path_ = Mat(image_raw.rows, image_raw.cols, CV_8UC1, Scalar(2));
    // Mat img_display = image_raw.clone();
    int count_invisable = 0;
    for(int i = 0; i < cloud_path.points.size(); i++)
    {
      pcl::PointXYZRGB path_point = cloud_path.points[i];
      if(path_point.z > 5)
        continue;

      cv::Point3d pt_cv(path_point.x, path_point.y, path_point.z);

      cv::Point2d uv;
      uv = project3D_to_image(pt_cv, "kinect2_rgb_optical_frame");

      int boundary = 20;
      if(uv.x >= boundary && uv.x < img_depth_.cols - boundary && uv.y >= boundary && uv.y < img_depth_.rows - boundary)
      {
        // unsigned short depth_before = img_depth_.at<unsigned short>(uv.y, uv.x);
        float depth_before = img_depth_.at<float>(uv.y, uv.x);

        cv::Point3d pt_cv2(path_point.x + 0.30, path_point.y, path_point.z);
        cv::Point2d uv2;
        uv2 = project3D_to_image(pt_cv2, "kinect2_rgb_optical_frame");

        int radius = abs(uv2.x - uv.x);

        // unsigned short disparity = scale/path_point.z * 256 + 1;
        float dist_before = (256.0 * scale) / (depth_before - 1); 

        // cout << path_point.z << " " << depth_before << endl;

        if(path_point.z - depth_before > 0.1)
        {
          cloud_path.points[i].r = 255;
          // count_invisable ++;
          // cv::circle(img_path_, uv, radius, Scalar(0, 0, 255), -1);  

          // cv::circle(img_depth_, uv, 5, Scalar(0, 0, 255), -1);  
        }
        else
        {
          cloud_path.points[i].b = 255;
          cv::circle(img_path_, uv, radius, Scalar(1, 1, 1), -1);  
          cv::circle(img_rgb_, uv, radius, Scalar(255, 1, 1), -1);  

        }
      }
    }
    cout << "occluded path points: " << count_invisable << endl;
    
    // imshow("img_disparity_", img_disparity_);
    imshow("img_rgb_", img_rgb_);
    imshow("img_path_", img_path_);
    waitKey(50);
 
    return cloud_path;
  }

  int get_disparity(Mat image_raw, string frame_id, pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_filtered)
  {   
    // init output image
    if(pcl_cloud_filtered.points.size() == 0)
      return 0;
    
    img_rgb_ = image_raw.clone();

    // init path image and draw obstacle detected by geometric feature
    img_path_ = Mat(image_raw.rows, image_raw.cols, CV_8UC1, Scalar(2));

    string image_frame_id;
    if(frame_id == "kinect2_rgb_optical_frame")
      image_frame_id  = frame_id;
    else 
      image_frame_id  = "overhead_camera_link";

    img_depth_ = Mat(image_raw.rows, image_raw.cols, CV_32FC1, Scalar(0));
    img_disparity_ = Mat(image_raw.rows, image_raw.cols, CV_16UC1, Scalar(0));
    // img_all_   = Mat(image_raw.rows, image_raw.cols, CV_16UC4, Scalar(0));
    // Mat img_depth_display_   = Mat(image_raw.rows, image_raw.cols, CV_16UC1, Scalar(0));

    for(int i = 0; i < pcl_cloud_filtered.points.size(); i++)
    {
      pcl::PointXYZRGB point_transd = pcl_cloud_filtered.points[i];
      // if(point_transd.z < 0 || abs(point_transd.x) > 6)
      //   continue;
      
      cv::Point3d pt_cv(point_transd.x, point_transd.y, point_transd.z);
      
      cv::Point2d uv;
      uv = project3D_to_image(pt_cv, image_frame_id);

      static const int RADIUS = 7;
      
      if(uv.x >= 0 && uv.x < image_raw.cols && uv.y >= 0 && uv.y < image_raw.rows)
      {
        float depth = point_transd.z;
        unsigned short disparity = scale/point_transd.z * 256 + 1;

        float depth_before = img_depth_.at<float>(uv.y, uv.x);

        if(depth < depth_before || depth_before == 0)
        {
            // img_depth_.at<float>(uv.y, uv.x) = depth;
            cv::circle(img_depth_, uv, 5, Scalar(depth, depth, depth), -1);  
            cv::circle(img_disparity_, uv, 5, Scalar(disparity, disparity, disparity), -1);  

            if(point_transd.r != 0)
            {
              cv::circle(img_path_, uv, 5, Scalar(0), -1);  
              cv::circle(img_rgb_, uv, 5, Scalar(0, 255, 0), -1);  
            }

        }  
      }
    } 

    // Mat element = getStructuringElement(MORPH_RECT, Size(3,3));
    // morphologyEx(img_depth_, img_depth_, MORPH_DILATE, element);
    // cout << "got depth image" << img_depth_.rows << " " << img_depth_.cols << endl;

    // for(int row = 0; row < image_raw.rows; row ++)
    // {
    //   for(int col = 0; col < image_raw.cols; col ++)
    //   {
    //     Vec4s u4_value;
    //     Vec3b raw_color = image_raw.ptr<Vec3b>(row)[col];
    //     unsigned short depth = img_depth_.ptr<unsigned short>(row)[col];

    //     u4_value.val[0] = (unsigned short) (raw_color.val[0]);
    //     u4_value.val[1] = (unsigned short) (raw_color.val[1]);
    //     u4_value.val[2] = (unsigned short) (raw_color.val[2]);
    //     u4_value.val[3] = depth;

    //     img_all_.ptr<Vec4s>(row)[col] = u4_value;
    //     img_depth_display_.ptr<unsigned short>(row)[col] = depth;
    //   }
    // }
    // cout << "got all image " << img_all_.rows << " " << img_all_.cols << endl;
    imshow("image_disparity", img_disparity_);
    waitKey(100);
    return 1;  
  }
};

