#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "livox_ros_driver/CustomMsg.h"

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

ros::Publisher pub_pcl_out0, pub_pcl_out1;
uint64_t TO_MERGE_CNT = 1; 
constexpr bool b_dbg_line = false;
std::vector<livox_ros_driver::CustomMsgConstPtr> livox_data;
void LivoxMsgCbk1(const livox_ros_driver::CustomMsgConstPtr& livox_msg_in) {
  livox_data.push_back(livox_msg_in);
  if (livox_data.size() < TO_MERGE_CNT) return;

  pcl::PointCloud<PointType> pcl_in;
  pcl::PointCloud<PointType> pcl_ls;

  for (size_t j = 0; j < livox_data.size(); j++) {
    auto& livox_msg = livox_data[j];
    auto time_end = livox_msg->points.back().offset_time;
    for (unsigned int i = 0; i < livox_msg->point_num; ++i) {
      PointType pt;
      pt.x = livox_msg->points[i].x;
      pt.y = livox_msg->points[i].y;
      pt.z = livox_msg->points[i].z;
      float s = livox_msg->points[i].offset_time / (float)time_end;

      pt.intensity = livox_msg->points[i].line +livox_msg->points[i].reflectivity /10000.0 ; // The integer part is line number and the decimal part is timestamp
      pt.curvature = s*0.1;

      // auto fourth_group = livox_msg->points[ i ].tag & 0b11000000;
      // printf(" %d ", livox_msg->points[i].tag );
      // bool ls = false;
      // for (int i = 1; i < 16; i++)
      // {
      //   if ( livox_msg->points[i].tag  == i )
      //   {
      //     ROS_INFO(" %f %f %f ", pt.x, pt.y ,pt.z );
      //     pcl_ls.push_back(pt);
      //     ls = true;
      //     break;
      //   }
      // }
      
        // if ( ((livox_msg->points[i].tag & 0x30) == 0x10 || (livox_msg->points[i].tag & 0x30) == 0x00) )
        // {}
        // else
        //   pcl_ls.push_back(pt);

        // if ( livox_msg->points[i].tag  == 16 )
          pcl_in.push_back(pt);

      
    }
  }

  ROS_INFO("%d   %d %d ",pcl_in.size(), pcl_ls.size(),pcl_in.size()+ pcl_ls.size() );

  unsigned long timebase_ns = livox_data[0]->timebase;
  ros::Time timestamp;
  timestamp.fromNSec(timebase_ns);

  sensor_msgs::PointCloud2 pcl_ros_msg;
  pcl::toROSMsg(pcl_in, pcl_ros_msg);
  pcl_ros_msg.header.stamp = livox_data[0]->header.stamp ;
  pcl_ros_msg.header.frame_id = "livox";
  pub_pcl_out1.publish(pcl_ros_msg);
  livox_data.clear();

  pcl::toROSMsg(pcl_ls, pcl_ros_msg);
  pcl_ros_msg.header.stamp = livox_data[0]->header.stamp ;
  pcl_ros_msg.header.frame_id = "livox";
  pub_pcl_out0.publish(pcl_ros_msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "livox_repub");
  ros::NodeHandle nh;

  ROS_INFO("start livox_repub");

  ros::Subscriber sub_livox_msg1 = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LivoxMsgCbk1);
  pub_pcl_out1 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pc2", 100);
  pub_pcl_out0 = nh.advertise<sensor_msgs::PointCloud2>("/livox_pc2_lasi", 100);

  ros::spin();
}