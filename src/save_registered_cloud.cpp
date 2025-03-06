#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <std_srvs/Trigger.h>

#include <ctime>
#include <vector>

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZ;
typedef pcl::PointXYZRGB PointTypeRGB;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;

int step = 2;

class Save_pointcloud
{
private:
  PointCloudXYZ::Ptr map; //   (new PointCloudXYZI());
  PointCloudXYZRGB::Ptr map_RGB;

  pcl::VoxelGrid<PointTypeRGB> dsrgb;

  std::vector< ros::Subscriber > sub_vector;
  ros::Subscriber sub_cloud_registered;
  ros::Subscriber sub_cloud_registered_1;
  ros::Subscriber sub_cloud_registered_rgb;
  ros::ServiceServer server_end_gtsam;
  ros::ServiceServer server_save_rgbmap;
  
  pcl::VoxelGrid<PointType> downSizeFilterMap;
  int cnts;
  double last_pc_time;
  char pcd_time[100];

public:
  Save_pointcloud(ros::NodeHandle &nh);

  void save_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc);
  void save_rgb_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc);
  bool service_sm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  bool save_rgb_map_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
  

  ~Save_pointcloud();
};

Save_pointcloud::Save_pointcloud(ros::NodeHandle &nh)
{
  map = boost::make_shared<PointCloudXYZ>();
  map_RGB = boost::make_shared<PointCloudXYZRGB>();

  cnts = 0;
  downSizeFilterMap.setLeafSize(0.2f, 0.2f, 0.2f);


  sub_cloud_registered_1 = nh.subscribe("/cloud_registered", 1000, &Save_pointcloud::save_map, this);
  sub_cloud_registered = nh.subscribe("/map_update_data", 1000, &Save_pointcloud::save_map, this);
  
  sub_vector.resize(41);  
  // for (size_t i = 0; i < 40; i++)
  // {
  //   std::cout <<  "/RGB_map_" + std::to_string(i) << std::endl;
  //   sub_vector[i] = nh.subscribe("/RGB_map_" + std::to_string(i), 1000, &Save_pointcloud::save_rgb_map, this);
  //   /* code */
  // }
  sub_vector[40] = nh.subscribe("/cloud_registered_rgb", 1000, &Save_pointcloud::save_rgb_map, this);
  sub_cloud_registered_rgb = nh.subscribe("/rgb_pts", 1000, &Save_pointcloud::save_rgb_map, this);
   
  server_end_gtsam = nh.advertiseService("save_map", &Save_pointcloud::service_sm, this);
  server_save_rgbmap = nh.advertiseService("save_rgb_map", &Save_pointcloud::save_rgb_map_srv, this);
  last_pc_time = 0;
  strcpy(pcd_time, "test_time");

}

void Save_pointcloud::save_rgb_map(const sensor_msgs::PointCloud2::ConstPtr &reg_rgb_points)
{
  // static int rgb_cnts = 1;
  // if (rgb_cnts++ % 20  != 0)
  // {
  //   return;
  // }

  PointCloudXYZRGB::Ptr one_rgb_frame(new PointCloudXYZRGB(reg_rgb_points->width, 1));
  pcl::fromROSMsg(*reg_rgb_points, *one_rgb_frame);

  // dsrgb.setLeafSize(0.05f, 0.05f, 0.05f);
  // dsrgb.setInputCloud(one_rgb_frame);
  // dsrgb.filter(*one_rgb_frame);
  (*map_RGB) += (*one_rgb_frame);
  ROS_WARN_ONCE("rgb points size %ld .", map_RGB->points.size());
}

bool Save_pointcloud::save_rgb_map_srv(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  std::cout << "save rgb map start ......" << std::endl;
  ROS_WARN("rgb points size %ld .", map_RGB->points.size());

  pcl::io::savePCDFile("/home/map/livo_rgbmap_original.pcd" , *map_RGB);
  dsrgb.setLeafSize(0.05f, 0.05f, 0.1f);
  dsrgb.setInputCloud(map_RGB);
  dsrgb.filter(*map_RGB);
  ROS_WARN("rgb points size %ld .", map_RGB->points.size());

  pcl::io::savePCDFile("/home/map/livo_rgbmap.pcd" , *map_RGB);
  std::cout << "save rgb map done ......" << std::endl;
  res.success = true;
  res.message = "rgb pcd file:  /home/map/livo_rgbmap.pcd ";
  return true;

}

Save_pointcloud::~Save_pointcloud()
{
  std_srvs::Trigger::Request req;
  std_srvs::Trigger::Response res;
  // service_sm(req, res);
  // if (!map->points.empty())
  // {
  //   std::cout << "pcd_name: " << pcd_time << std::endl;
  //   pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_original_auto.pcd", *map);
  //   std::cout << "original.pcd : " << map->width * map->height << std::endl; 
  // }
}

bool Save_pointcloud::service_sm(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  
  std::cout << "pcd_name: " << pcd_time << std::endl;
  pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_original.pcd", *map);

  if (!map->points.empty())
  {
    PointCloudXYZ temp_map = *map;
    // std::cout << "size of map <before filter >: " << map->width * map->height << std::endl;
    // downSizeFilterMap.setLeafSize(0.025f, 0.025f, 0.025f);
    // downSizeFilterMap.setInputCloud(map);
    // downSizeFilterMap.filter(temp_map);
    // std::cout << "size of map <after filter >:  " << temp_map.width * temp_map.height << std::endl;
    // // 保存点云到指定路径
    // std::cout << "START : save map to pcd file : "
    //           << "/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd" << std::endl;
    // // pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd", temp_map);
    // pcl::io::savePCDFile("/home/map/livo_map_25mm.pcd", temp_map);
    // std::cout << "FINISH : save map to pcd file : "
    //           << "/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd" << std::endl;

    downSizeFilterMap.setLeafSize(0.01f, 0.01f, 0.01f);
    downSizeFilterMap.setInputCloud(map);
    downSizeFilterMap.filter(temp_map);
    pcl::io::savePCDFile("/home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd", temp_map);
  }
  else
  {
    ROS_WARN("-------------- WARN: pointcloud is empty . -----------");
  }
  // map->points.clear();
  res.success = true;
  res.message = "pcd file:  /home/map/" + std::string(pcd_time) + "_Save_cloud_registered_10.pcd";
  return true;
}


void Save_pointcloud::save_map(const sensor_msgs::PointCloud2::ConstPtr &reg_pc)
{

  if (reg_pc->header.stamp.toSec() < last_pc_time || std::fabs(reg_pc->header.stamp.toSec() - last_pc_time) > 10000.0)
  {
    /* code */
    printf("-------------- clear map ");
    map->points.clear();
    cnts == 0;
    ROS_WARN("-------------- clear map: pointcloud is empty . -----------");
  }

  cnts++;
  // if (cnts == 1)
  // {
  //   time_t first_time = reg_pc->header.stamp.toSec();
  //   tm local_time;
  //   localtime_r(&first_time, &local_time);
  //   strftime(pcd_time, sizeof(pcd_time), "%Y-%m-%d-%H-%M-%S", &local_time);
  // }

  if (cnts % step == 0)
  {
    PointCloudXYZ::Ptr one_frame(new PointCloudXYZ());
    pcl::fromROSMsg(*reg_pc, *one_frame);
    // PointCloudXYZI::Ptr temp;
    // temp = boost::make_shared<PointCloudXYZI>();
    // pcl::copyPointCloud(*laserCloudFullResColor, *temp); //复制
    // (*temp_map) += (*temp); //
    // ROS_WARN("size of one_frame <before filter >: %d ", one_frame->width * one_frame->height);
    // downSizeFilterMap.setInputCloud(one_frame);
    // downSizeFilterMap.filter(*one_frame);
    // ROS_WARN("size of one_frame <after filter >: %d ", one_frame->width * one_frame->height);
    (*map) += (*one_frame);
    // ROS_INFO("size of map :  %d  ", map->width * map->height);
  }
  last_pc_time = reg_pc->header.stamp.toSec();
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_map");
  std::cout << "save_map node start ...... --" << std::endl;
  if (argc == 2)
  {
    step = std::stoi(argv[1]);
  }
  
  std::cout << "step: " << step << std::endl;
  ros::NodeHandle nh("~");

  Save_pointcloud sm(nh);
  ros::spin();

  return (0);
}
