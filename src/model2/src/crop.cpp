#include <ros/ros.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <cmath>
#include <math.h>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/integral_image_normal.h>

#include <pcl/console/parse.h>

// Global variables
ros::Publisher contact_area_pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

//Undeformed Point Cloud
PointCloud::Ptr undeformed_ptcld (new PointCloud);
bool undeformed_loaded = false;

bool SAVE_TO_PCD = false;
bool LOAD_FROM_PCD = false;
bool SET_PARAMS = false;

//threshold value for deformation detection
const float THRESHOLD = .001; 

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (255, 255, 255);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "cloud");
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, "deformed");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "deformed");

  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LUT, pcl::visualization::PCL_VISUALIZER_LUT_JET, "cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  viewer->removeCoordinateSystem();
  return (viewer);
}

float concavityOfPair(pcl::PointXYZRGB& a, pcl::PointXYZRGB& b, pcl::Normal& an, pcl::Normal bn){
  float dx = a.x - b.x;
  float dy = a.y - b.y;
  float dz = a.z - b.z;

  float dnx = an.normal_x - bn.normal_x;
  float dny = an.normal_y - bn.normal_y;
  float dnz = an.normal_z - bn.normal_z;

  float concavity = dx * dnx + dy * dny + dz * dnz;
}

float pDist(pcl::PointXYZRGB& a, pcl::PointXYZRGB& b) {
  return sqrt(pow(a.x - b.x, 2.0) + pow(a.y- b.y, 2.0) + pow(a.z - b.z, 2.0));
}

float pDistp(pcl::PointXYZ& a, pcl::PointXYZ& b) {
  return sqrt(pow(a.x - b.x, 2.0) + pow(a.y- b.y, 2.0) + pow(a.z - b.z, 2.0));
}

float surfaceElementArea(pcl::PointXYZRGB& o, pcl::PointXYZRGB& a1, pcl::PointXYZRGB& a2, pcl::PointXYZRGB& b1, pcl::PointXYZRGB& b2) {
  if (pcl::isFinite(a1) && pcl::isFinite(a2) && pcl::isFinite(b1) && pcl::isFinite(b2))  {
    float width = 0.5 * (pDist(o, a1) + pDist(o, a2));
    float height = 0.5 * (pDist(o, b1) + pDist(o, b2));
    return width * height;
  }  

  return 0.0;

}

float surfaceElementAreaP(pcl::PointXYZ& o, pcl::PointXYZ& a1, pcl::PointXYZ& a2, pcl::PointXYZ& b1, pcl::PointXYZ& b2) {
  float width = 0.5 * (pDistp(o, a1) + pDistp(o, a2));
  float height = 0.5 * (pDistp(o, b1) + pDistp(o, b2));
  return width * height;
}

void callback(const PointCloud::ConstPtr& msg)
{
  if(!undeformed_loaded)
  {
    undeformed_ptcld->width = msg->width;
    undeformed_ptcld->height = msg->height;
    undeformed_ptcld->resize(undeformed_ptcld->height*undeformed_ptcld->width);
    
    for (int c = 0; c < 224; c++) {
      for (int r = 0; r < 171; r++) {
        undeformed_ptcld->at(c, r).x = msg->at(c, r).x;
        undeformed_ptcld->at(c, r).y = msg->at(c, r).y;
        undeformed_ptcld->at(c, r).z = msg->at(c, r).z; 
      }
    }

    undeformed_loaded = true;
    printf ("Undeformed Cloud Loaded");
  }
  else
  {
  // printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
  // std::cout << msg->points.size() << std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_display (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr deformed_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_display_basic (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_membrane (new pcl::PointCloud<pcl::PointXYZRGB>);
  full_membrane->width = msg->width;
  full_membrane->height = msg->height;
  full_membrane->resize(full_membrane->height*full_membrane->width);

  PointCloud::Ptr contact_ptcld (new PointCloud);
  contact_ptcld->height = 1;


  uint8_t r_green = 113;
  uint8_t g_green = 164;
  uint8_t b_green = 252;
  uint32_t green = ((uint32_t) r_green << 16 | (uint32_t)g_green << 8 | (uint32_t)b_green);

  uint8_t r_white = 206;
  uint8_t g_white = 206;
  uint8_t b_white = 206;
  uint32_t white = ((uint32_t) r_white << 16 | (uint32_t)g_white << 8 | (uint32_t)b_white);

  uint8_t r_red =  252;
  uint8_t g_red = 196;
  uint8_t b_red = 113;
  uint32_t red = ((uint32_t) r_red << 16 | (uint32_t)g_red << 8 | (uint32_t)b_red);

  // ------------------------
  // -----Get center dot-----
  // ------------------------

  int cols[] = {112}; 
  int rows[] = {86}; 
  pcl::PointXYZRGB track_point;

  for (size_t c = 0; c < 1; c++) {
    int row = rows[c];
    int col = cols[c];
    track_point.x = msg->at(col, row).x;
    track_point.y = msg->at(col, row).y;
    track_point.z = msg->at(col, row).z;

    ros::spinOnce();

    uint8_t r = 255, g = 0;
    uint8_t b = 0;
    uint32_t rgb = ((uint32_t) r << 16 | (uint32_t)g << 8 | (uint32_t)b);     
    track_point.rgb = *reinterpret_cast<float*>(&rgb);
    deformed_cloud->points.push_back(track_point);    
  }


  float radius3d_deform_limit = 0.028; //0.028;

  // Create the point cloud for the full membrane
  for (int c = 0; c < 224; c++) {
    for (int r = 0; r < 171; r++) {
      if (pcl::isFinite(full_membrane->at(c, r))) {
        float radius_2d_squared = pow(msg->at(c, r).x - track_point.x, 2.0) + pow(msg->at(c, r).y - track_point.y, 2.0);
        //float radius3d = sqrt(pow(msg->at(c, r).x - track_point.x, 2.0) + pow(msg->at(c, r).y - track_point.y, 2.0) + pow(msg->at(c, r).z - 0.13, 2.0));

        // 2D radius to check whether on membrane or not
        if (radius_2d_squared < 0.0021) {
        // if (radius_2d_squared < 0.0012) {

          full_membrane->at(c, r).x = msg->at(c, r).x;
          full_membrane->at(c, r).y = msg->at(c, r).y;
          full_membrane->at(c, r).z = msg->at(c, r).z;  
          
          full_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&white);

          // 3D radius to check deformation
          // if (radius3d < radius3d_deform_limit) {
          //   full_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&green);
          // }
        }       
      }
    }
  }
  


  // ----------------------------------------------------------------
  // -----Calculate surface normals with a search radius of 0.05-----
  // ----------------------------------------------------------------
  //Calculate surface normals for organized point cloud
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
  ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
  ne.setMaxDepthChangeFactor(0.02f);
  ne.setNormalSmoothingSize(10.0f);
  ne.setInputCloud(full_membrane);
  ne.compute(*normals);

  // viewer->removePointCloud("normals", 0);
  // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (full_membrane, normals, 10, 0.02, "normals"); 

  // std::cout << "Normals: " <<  normals->width << " " << normals->height << " " << std::endl;
  int k = 15;
  // We iterate through whole array slicing off margin of 1 
  int index = 0;
  for (int c = k; c < 224 - k; c++) {
    for (int r = k; r < 171 - k; r++) {
      // Let's do column wise first (horizontal neighbours). Check if horizontal neighbours exist
      if (pcl::isFinite(full_membrane->at(c-k, r)) && pcl::isFinite(full_membrane->at(c+k, r))) {
        // Now check if vertical neighbours exist. Row-wise.
        if (pcl::isFinite(full_membrane->at(c, r-k)) && pcl::isFinite(full_membrane->at(c, r+k))) {
          float c_concavity = concavityOfPair(full_membrane->at(c-k, r), full_membrane->at(c+k, r), normals->at(c-k, r), normals->at(c+k, r));
          float r_concavity = concavityOfPair(full_membrane->at(c, r-k), full_membrane->at(c, r+k), normals->at(c, r-k), normals->at(c, r+k));

          // Check in here if deformed
          // float radius3d = sqrt(pow(msg->at(c, r).x - track_point.x, 2.0) + pow(msg->at(c, r).y - track_point.y, 2.0) + pow(msg->at(c, r).z - 0.09, 2.0));
          bool deformed_bool = false;
          if (std::abs(msg->at(c, r).z - undeformed_ptcld->at(c, r).z) > THRESHOLD) {
            deformed_bool = true;
          }

          if (c_concavity > 0 && r_concavity > 0 && deformed_bool) {
            // See what local surface area this point covers
            full_membrane->at(c, r).rgb = *reinterpret_cast<float*>(&red);
            pcl::PointXYZ point;
            point.x = full_membrane->at(c, r).x;
            point.y = full_membrane->at(c, r).y;
            point.z = full_membrane->at(c, r).z;
            contact_ptcld->points.push_back(point);
            index++;
          }
        }
      } 
    }
  }

  contact_ptcld->width = index;
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*contact_ptcld, output);
  contact_area_pub.publish(output);

  if (SAVE_TO_PCD) {
    pcl::io::savePCDFileASCII("deformed.pcd", *msg);
    SAVE_TO_PCD = false;
  }

  // Show on the viewer
  // sleep(1);
  viewer->updatePointCloud(full_membrane, "cloud");
  viewer->updatePointCloud(deformed_cloud, "deformed");
}
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;

  // ------------------------------------
  // -----Set parameters of the camera---
  // ------------------------------------
  if (SET_PARAMS) {
    ros::Publisher use_case_pub = nh.advertise<std_msgs::String>("/use_case", 1);
    ros::Publisher max_filter_pub = nh.advertise<std_msgs::Float32>("/max_filter", 1);
    ros::Publisher min_filter_pub = nh.advertise<std_msgs::Float32>("/min_filter", 1);
    ros::Publisher exposure_pub = nh.advertise<std_msgs::UInt32>("/expo_time", 1);

    sleep(5);

    std_msgs::String mode_msg;
    std::stringstream ss;
    ss << "MODE_5_45FPS_500";
    mode_msg.data = ss.str();
    use_case_pub.publish(mode_msg);

    std_msgs::Float32 max_filter_msg;
    max_filter_msg.data = 0.3;
    max_filter_pub.publish(max_filter_msg);

    std_msgs::Float32 min_filter_msg;
    min_filter_msg.data = 0.06;
    min_filter_pub.publish(min_filter_msg);


    std_msgs::UInt32 expo_time_msg;
    expo_time_msg.data = 50;
    exposure_pub.publish(expo_time_msg);
    sleep(3);

    ros::spinOnce();  
  }

  // ------------------------------------
  // -----Create example point cloud-----
  // ------------------------------------
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Loading point cloud\n\n";

  basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  basic_cloud_ptr->height = 1;
  point_cloud_ptr->width = (int) basic_cloud_ptr->points.size ();
  point_cloud_ptr->height = 1;

  viewer = simpleVis(point_cloud_ptr);
  std::cout << "Loaded point cloud\n";

  // ---------------------------
  // -----Create publishers-----
  // ---------------------------

  contact_area_pub = nh.advertise<sensor_msgs::PointCloud2>("/cropped", 1);
  sleep(2);

  // -------------------------------------------------
  // -----Load PCD cloud or subscribe to message -----
  // -------------------------------------------------
  if (LOAD_FROM_PCD) {
    // Load PCD cloud file
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("deformed.pcd", *load_cloud);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr load_cloud_const (load_cloud);
    callback(load_cloud_const);   
  }

  ros::Subscriber sub = nh.subscribe<PointCloud>("/buffer", 1, callback);
  


  while (!viewer->wasStopped ())
  {
    viewer->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    // sleep(1);
    ros::spinOnce();
  }


}