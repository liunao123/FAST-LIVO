#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"

#include <map>

// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZI PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

int TARGET_LINE = 100;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;
  pcl::PointCloud<PointType> pcl_ls;

  std::map<int, int> Mymap_tag;
  std::map<int, int> Mymap_line;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      pt.intensity = livox_msg->points[i].reflectivity;
      // float s = livox_msg->points[i].offset_time / (float)time_end;
      
      Mymap_tag[livox_msg->points[i].tag]++;
      Mymap_line[livox_msg->points[i].line]++;

      // pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
      // pt.curvature = s*0.1;
          // ROS_INFO(" livox_msg->points[i].line : %d", livox_msg->points[i].line);
      if (((livox_msg->points[i].tag & 0x30) == 0x10 || (livox_msg->points[i].tag & 0x30) == 0x00))
      {
      // if ( livox_msg->points[i].line  == TARGET_LINE )
      // if ( livox_msg->points[i].tag  == TARGET_LINE )
      if ( pt.x > 1.0  )
          pcl_in.push_back(pt);
      // // if ( pt.x < 1.0 || pt.intensity < 100.0 || pt.y < -1.5 )
      // {
      //   continue;
      // }
      // else
      // {
      // }

      }

    }
  }

  // for (const auto  item: Mymap_tag )
  // {
  //    std::cout << "tag: " <<  item.first << " .  count : " << item.second  << std::endl  ;
  // }
  //    std::cout << "----------------------------------------------------"  << std::endl  ;
  
  // for (const auto  item: Mymap_line )
  // {
  //    std::cout << "tag: " <<  item.first << " .  count : " << item.second  << std::endl  ;
  // }
  // ROS_INFO("%d   %d %d ",pcl_in.size(), pcl_ls.size(),pcl_in.size() + pcl_ls.size() );


  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header  = livox_msg_in->header  ;
  pcl_ros_msg.header.frame_id = "base_footprint";
  // pcl_ros_msg.header.stamp = ros::Time::now() ;
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();

  // pcl::io::savePCDFileASCII("/opt/csg/slam/navs/line.pcd", pcl_in);

  // ROS_INFO("pcl_in %d time %f  ",pcl_in.size() , pcl_ros_msg.header.stamp.toSec() );

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pc2", 100);
  pub_pcl_out0 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pc2_lasi", 100);

  // ros::spin();
  ros::Rate r(100);
  while ( ros::ok() )
  {
    nh.getParam("TARGET_LINE", TARGET_LINE);
    // std::cout << "TARGET_LINE: " <<  TARGET_LINE  << std::endl  ;
    ros::spinOnce();
    r.sleep();
  }
  
  
}