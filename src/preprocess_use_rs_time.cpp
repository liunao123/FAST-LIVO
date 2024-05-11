#define PCL_NO_PRECOMPILE
#include "preprocess.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/point_cloud.h>
#include <pcl/pcl_macros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <math.h>

#define RETURN0     0x00
#define RETURN0AND1 0x10



Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)
{
  max_blind = 75.0;
  inf_bound = 4;
  N_SCANS   = 16;
  group_size = 8;
  disA = 0.01;
  disB = 0.1; // B?
  p2l_ratio = 225;
  limit_maxmid =6.25;
  limit_midmin =6.25;
  limit_maxmin = 3.24;
  jump_up_limit = 170.0;
  jump_down_limit = 8.0;
  cos160 = 160.0;
  edgea = 2;
  edgeb = 0.1;
  smallp_intersect = 172.5;
  smallp_ratio = 1.2;

  jump_up_limit = cos(jump_up_limit/180*M_PI);
  jump_down_limit = cos(jump_down_limit/180*M_PI);
  cos160 = cos(cos160/180*M_PI);
  smallp_intersect = cos(smallp_intersect/180*M_PI);
}

Preprocess::~Preprocess() {}

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;
  lidar_type = lid_type;
  blind = bld;
  point_filter_num = pfilt_num;
}

void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);
  
  *pcl_out = pl_surf;

  // static pcl::RadiusOutlierRemoval< PointType > outrem;
	// outrem.setInputCloud(pcl_out);
	// outrem.setRadiusSearch(0.5);
	// outrem.setMinNeighborsInRadius(5);
	// // apply filter
	// outrem.filter(*pcl_out);

}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)
  {
  case OUST64:
    ROS_WARN_ONCE("------------------------OUST64");
    oust64_handler(msg);
    break;

  case VELO16:
  {
    ROS_WARN_ONCE("------------------------VELO16");
    velodyne_handler(msg);
    break;
  }
  
  default:
    printf("Error LiDAR Type");
    break;
  }
  *pcl_out = pl_surf;
  
  // static pcl::RadiusOutlierRemoval< PointType > outrem;
	// outrem.setRadiusSearch(0.5);
	// outrem.setMinNeighborsInRadius(3);
	// outrem.setInputCloud(pcl_out);
	// // apply filter
	// outrem.filter(*pcl_out);

}

void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  double t1 = omp_get_wtime();
  uint plsize = msg->point_num;
  uint effect_ind = 0;

  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  pl_full.resize(plsize);

  for(int i=0; i<N_SCANS; i++)
  {
    pl_buff[i].clear();
    pl_buff[i].reserve(plsize);
  } 

  if (feature_enabled)
  {
    for(uint i=1; i<plsize; i++)
    {

      if (   (msg->points[i].line < N_SCANS) // && 
          // ( (msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00 )
         )
      {
        if(  ( ( msg->points[ i ].tag & 0x03 ) != 0x00 ) ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 ) ) 
        {
          continue;
        }
        // if( msg->points[i].z < -1.0 )
        // {
        //   continue;
        // }
        
        if ( msg->points[i].z * msg->points[i].z + msg->points[i].y * msg->points[i].y + msg->points[i].x * msg->points[i].x > max_blind * max_blind)
        {
          continue;
        }

        if (   
             (std::abs(  msg->points[i].x - msg->points[i - 1].x) > 1e-7) 
            || (std::abs(msg->points[i].y - msg->points[i - 1].y) > 1e-7) 
            || (std::abs(msg->points[i].z - msg->points[i - 1].z) > 1e-7) 
            && 
            (pl_full[i].z * pl_full[i].z + pl_full[i].y * pl_full[i].y + pl_full[i].x * pl_full[i].x) > blind * blind )
        {
          pl_full[i].x = msg->points[i].x;
          pl_full[i].y = msg->points[i].y;
          pl_full[i].z = msg->points[i].z;
          pl_full[i].intensity = msg->points[i].reflectivity;
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // use curvature as time of each laser points
          pl_buff[msg->points[i].line].push_back(pl_full[i]);
        }
      }

    }

    for(int j=0; j<N_SCANS; j++)
    {
      // printf("pl_buff[j].size(): %d \n", pl_buff[j].size());
      if(pl_buff[j].size() <= 5) continue;
      pcl::PointCloud<PointType> &pl = pl_buff[j];
      plsize = pl.size();
      // printf("plsize: %d \n", plsize);
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(plsize);
      plsize--;
      for(uint i=0; i<plsize; i++)
      {
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;
      give_feature(pl, types);
    }
  }
  else
  {
    for(uint i=1; i<plsize; i++)
    {
        if((abs(msg->points[i].x - msg->points[i-1].x) < 1e-8) 
            || (abs(msg->points[i].y - msg->points[i-1].y) < 1e-8)
            || (abs(msg->points[i].z - msg->points[i-1].z) < 1e-8)
            || (msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y + msg->points[i].z * msg->points[i].z < blind * blind)
            || (msg->points[i].line > N_SCANS)
            || ((msg->points[i].tag & 0x30) != RETURN0AND1))
        {
            continue;
        }
        
        if(  ( ( msg->points[ i ].tag & 0x03 ) != 0x00 ) ||  ( ( msg->points[ i ].tag & 0x0C ) != 0x00 )  ) 
        {
          continue;
        }
        // if ( msg->points[i].z < -1.0 )
        // {
        //   continue;
        // }

        if ( msg->points[i].z * msg->points[i].z + msg->points[i].y * msg->points[i].y + msg->points[i].x * msg->points[i].x > max_blind * max_blind)
        {
          continue;
        }

        effect_ind ++;

        if(effect_ind % point_filter_num == 0)
        {
            pl_full[i].x = msg->points[i].x;
            pl_full[i].y = msg->points[i].y;
            pl_full[i].z = msg->points[i].z;
            pl_full[i].intensity = msg->points[i].reflectivity;
            pl_full[i].curvature = msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points
            pl_surf.push_back(pl_full[i]);
        }
    }
  }
  // ROS_WARN(" pl_surf size: %ld ",pl_surf.size() );
  // printf("feature extraction time: %lf \n", omp_get_wtime()-t1);
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();
  pl_corn.clear();
  pl_full.clear();
  pcl::PointCloud<ouster_ros::Point> pl_orig;
  pcl::fromROSMsg(*msg, pl_orig);

  pl_orig.is_dense = false; // 万集的雷达必须加这一句
  std::vector<int> save_index;
  // ROS_ERROR("nan pl_orig->size()is %d",  pl_orig.size());
  pcl::removeNaNFromPointCloud(pl_orig, pl_orig, save_index);
  // ROS_ERROR("remove nan pl_orig->size()is %d",  pl_orig.size());

  uint plsize = pl_orig.size();
  pl_corn.reserve(plsize);
  pl_surf.reserve(plsize);
  if (feature_enabled)
  {
    for (int i = 0; i < N_SCANS; i++)
    {
      pl_buff[i].clear();
      pl_buff[i].reserve(plsize);
    }

    for (uint i = 0; i < plsize; i++)
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      if (range < blind) continue;
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;

      added_pt.curvature = pl_orig.points[i].t / 1e6;
      if(pl_orig.points[i].ring < N_SCANS)
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);
      }
    }

    for (int j = 0; j < N_SCANS; j++)
    {
      PointCloudXYZI &pl = pl_buff[j];
      uint linesize = pl.size();
      vector<orgtype> &types = typess[j];
      types.clear();
      types.resize(linesize);
      linesize--;
      for (uint i = 0; i < linesize; i++)
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
        vx = pl[i].x - pl[i + 1].x;
        vy = pl[i].y - pl[i + 1].y;
        vz = pl[i].z - pl[i + 1].z;
        types[i].dista = vx * vx + vy * vy + vz * vz;
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
      give_feature(pl, types);
    }
  }
  else
  {
    double time_stamp = msg->header.stamp.toSec();
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)
    {
      if (i % point_filter_num != 0) continue;

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;
      
      if (range < blind) continue;
      
      Eigen::Vector3d pt_vec;
      PointType added_pt;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;


      added_pt.curvature = pl_orig.points[i].t / 1e6;

      // cout<<"added_pt.curvature: "<<added_pt.curvature<<endl;
      pl_surf.points.push_back(added_pt);
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    // 用原始的 rslidar 不用 rs_to_velodyne 转
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<RsPointXYZIRT> pl_orig ;
    // pcl::PointCloud<RsPointXYZIRT>::Ptr cloud_undis(new pcl::PointCloud<RsPointXYZIRT>());
  
    pcl::fromROSMsg(*msg, pl_orig);
    // ROS_ERROR("pl_orig->size()is %d",  pl_orig.size());
    // ROS_WARN("first, last , msg->header.stamp.toSec() : %f . %f. %f ", pl_orig.points[0].timestamp , pl_orig.points.back().timestamp , msg->header.stamp.toSec());
    auto first_point_time = pl_orig.points[0].timestamp;
    // auto first_point_time = msg->header.stamp.toSec();

    // 根据索引把 nan 去掉
    // pcl::PointCloud<pcl::PointXYZ> pl_orig_xyz;
    // pcl::fromROSMsg(*msg, pl_orig_xyz);
    // // ROS_ERROR("pl_orig->size()is %d",  pl_orig.size());
  
    // // wanjee的激光雷达，需要手动去除 nan 点，后面的循环里有判断
    // // 速腾的雷达可可以去除，vanjee的不行
    // pl_orig_xyz->is_dense = false;
    // std::vector<int> save_index;
    // pcl::removeNaNFromPointCloud(pl_orig_xyz, pl_orig_xyz, save_index);
    // boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(save_index);
    // // ROS_ERROR("save_index %d",  save_index.size());
    
    // pcl::ExtractIndices<RsPointXYZIRT> extract;
    // extract.setInputCloud( pl_orig.makeShared() );
    // extract.setIndices(index_ptr);
    // extract.setNegative(false); // 保留 不是 索引的 数据 设置为  true
    // extract.filter(pl_orig);

    // 激光雷达，去除 nan 点
    pl_orig.is_dense = false; // 万集的雷达必须加这一句
    std::vector<int> save_index;
    // ROS_ERROR("pl_orig->size()is %d",  pl_orig.size());
    pcl::removeNaNFromPointCloud(pl_orig, pl_orig, save_index);
    // ROS_ERROR("pl_orig->size()is %d",  pl_orig.size());

    uint plsize = pl_orig.size();
    if ( plsize <= 0 )
    {
      return ;
    }

    // debug
    // pcl::PCDWriter pcd_writer;
    // cout << "saving...";
    // pcd_writer.writeBinary("/home/msg.pcd", pl_orig);
    bool is_first[16];
    bool is_jump[16]={false};       // if jump point
    double yaw_fp[20]={0};     // yaw of first scan point
    int layer;                 // layer number
    double omega_l=3.61;       // scan angular velocity
    float yaw_last[16]={0.0};  // yaw of last scan point
    float time_last[16]={0.0}; // last offset time
    float time_jump[16]={0.0}; // offset time before jump
    memset(is_first, true, sizeof(is_first));
    
    if(feature_enabled)
    {
      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        if (i % point_filter_num != 0)
        {
          continue;
        }

        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        layer=pl_orig.points[i].ring;
        // if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        // time off 
        // * 1000 是 后面 / 1000 了。。 为了与velodyne的代码兼容
        added_pt.curvature = ( pl_orig.points[i].timestamp - first_point_time  ) * 1000.0 ;
        // added_pt.curvature = ( pl_orig.points[i].timestamp - msg->header.stamp.toSec()  ) * 1000.0 ;
        // ROS_WARN("pl_orig.points[i].timestamp - msg->header.stamp.toSec() : %f . %f ", pl_orig.points[i].timestamp , msg->header.stamp.toSec());

        float range_temp_sqrt =added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z ;
        if(range_temp_sqrt < blind * blind || range_temp_sqrt > max_blind * max_blind) // max_blind 认为是 雷达的有效探测范围
        {
          continue;
        }
        
        if ( added_pt.y < -15.0  || added_pt.z  > 5.1  )
        {
          continue;
        }

        pl_buff[layer].points.push_back(added_pt);
      }

      for (int j = 0; j < N_SCANS; j++)
      {
        PointCloudXYZI &pl = pl_buff[j];
        uint linesize = pl.size();
        // fix bug when extract lio feature
        // ROS_WARN("linesize : %d", linesize);
        if (linesize == 0)
        {
          continue;
        }
        vector<orgtype> &types = typess[j];
        types.clear();
        types.resize(linesize);
        linesize--;
        for (uint i = 0; i < linesize; i++)
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
          vx = pl[i].x - pl[i + 1].x;
          vy = pl[i].y - pl[i + 1].y;
          vz = pl[i].z - pl[i + 1].z;
          types[i].dista = vx * vx + vy * vy + vz * vz;
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
        give_feature(pl, types);
      }

      // cout << "-----------saving feature pcd..." <<  __LINE__ << "         "<< pl_surf.points.size();

    }
    else
    {

      for (int i = 0; i < N_SCANS; i++)
      {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
      }
      
      for (int i = 0; i < plsize; i++)
      {
        if (i % point_filter_num != 0)
        {
          continue;
        }

        // wanjee的激光雷达，需要手动去除 nan 点
        // if(std::isnan(pl_orig.points[i].x) || std::isnan(pl_orig.points[i].y) || std::isnan(pl_orig.points[i].z) )
        // {
        //   continue;
        // }

        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        // layer=pl_orig.points[i].ring;
        // if (layer >= N_SCANS) continue;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;

        // time off 
        // * 1000 是 后面 / 1000 了。。 为了与velodyne的代码兼容
        added_pt.curvature = ( pl_orig.points[i].timestamp - first_point_time  ) * 1000.0 ;

        float range_temp_sqrt =added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z ;
        if(range_temp_sqrt < blind * blind || range_temp_sqrt > max_blind * max_blind) // max_blind 认为是 雷达的有效探测范围
        {
          continue;
        }

        pl_surf.points.push_back(added_pt);

      }

    }
        
    // pub_func(pl_surf, pub_full, msg->header.stamp);
    // pub_func(pl_surf, pub_surf, msg->header.stamp);
    // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  uint plsize = pl.size();
  uint plsize2;
  if(plsize == 0)
  {
    printf("something wrong\n");
    return;
  }
  uint head = 0;

  while(types[head].range < blind)
  {
    head++;
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());

  uint i_nex = 0, i2;
  uint last_i = 0; uint last_i_nex = 0;
  int last_state = 0;
  int plane_type;

  for(uint i=head; i<plsize2; i++)
  {
    if(types[i].range < blind)
    {
      continue;
    }

    i2 = i;

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
    
    if(plane_type == 1)
    {
      for(uint j=i; j<=i_nex; j++)
      { 
        if(j!=i && j!=i_nex)
        {
          types[j].ftype = Real_Plane;
        }
        else
        {
          types[j].ftype = Poss_Plane;
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)
      {
        double mod = last_direct.transpose() * curr_direct;
        if(mod>-0.707 && mod<0.707)
        {
          types[i].ftype = Edge_Plane;
        }
        else
        {
          types[i].ftype = Real_Plane;
        }
      }
      
      i = i_nex - 1;
      last_state = 1;
    }
    else // if(plane_type == 2)
    {
      i = i_nex;
      last_state = 0;
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;
    last_i_nex = i_nex;
    last_direct = curr_direct;
  }

  plsize2 = plsize > 3 ? plsize - 3 : 0;
  for(uint i=head+3; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i].ftype>=Real_Plane)
    {
      continue;
    }

    if(types[i-1].dista<1e-16 || types[i].dista<1e-16)
    {
      continue;
    }

    Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);
    Eigen::Vector3d vecs[2];

    for(int j=0; j<2; j++)
    {
      int m = -1;
      if(j == 1)
      {
        m = 1;
      }

      if(types[i+m].range < blind)
      {
        if(types[i].range > inf_bound)
        {
          types[i].edj[j] = Nr_inf;
        }
        else
        {
          types[i].edj[j] = Nr_blind;
        }
        continue;
      }

      vecs[j] = Eigen::Vector3d(pl[i+m].x, pl[i+m].y, pl[i+m].z);
      vecs[j] = vecs[j] - vec_a;
      
      types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
      if(types[i].angle[j] < jump_up_limit)
      {
        types[i].edj[j] = Nr_180;
      }
      else if(types[i].angle[j] > jump_down_limit)
      {
        types[i].edj[j] = Nr_zero;
      }
    }

    types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();
    if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_zero && types[i].dista>0.0225 && types[i].dista>4*types[i-1].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Prev))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_zero && types[i].edj[Next]== Nr_nor && types[i-1].dista>0.0225 && types[i-1].dista>4*types[i].dista)
    {
      if(types[i].intersect > cos160)
      {
        if(edge_jump_judge(pl, types, i, Next))
        {
          types[i].ftype = Edge_Jump;
        }
      }
    }
    else if(types[i].edj[Prev]==Nr_nor && types[i].edj[Next]==Nr_inf)
    {
      if(edge_jump_judge(pl, types, i, Prev))
      {
        types[i].ftype = Edge_Jump;
      }
    }
    else if(types[i].edj[Prev]==Nr_inf && types[i].edj[Next]==Nr_nor)
    {
      if(edge_jump_judge(pl, types, i, Next))
      {
        types[i].ftype = Edge_Jump;
      }
     
    }
    else if(types[i].edj[Prev]>Nr_nor && types[i].edj[Next]>Nr_nor)
    {
      if(types[i].ftype == Nor)
      {
        types[i].ftype = Wire;
      }
    }
  }

  plsize2 = plsize-1;
  double ratio;
  for(uint i=head+1; i<plsize2; i++)
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)
    {
      continue;
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)
    {
      continue;
    }

    if(types[i].ftype == Nor)
    {
      if(types[i-1].dista > types[i].dista)
      {
        ratio = types[i-1].dista / types[i].dista;
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)
      {
        if(types[i-1].ftype == Nor)
        {
          types[i-1].ftype = Real_Plane;
        }
        if(types[i+1].ftype == Nor)
        {
          types[i+1].ftype = Real_Plane;
        }
        types[i].ftype = Real_Plane;
      }
    }
  }

  int last_surface = -1;
  for(uint j=head; j<plsize; j++)
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)
    {
      if(last_surface == -1)
      {
        last_surface = j;
      }
    
      if(j == uint(last_surface+point_filter_num-1))
      {
        PointType ap;
        ap.x = pl[j].x;
        ap.y = pl[j].y;
        ap.z = pl[j].z;
        ap.curvature = pl[j].curvature;
        pl_surf.push_back(ap);

        last_surface = -1;
      }
    }
    else
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)
      {
        pl_corn.push_back(pl[j]);
      }
      if(last_surface != -1)
      {
        PointType ap;
        for(uint k=last_surface; k<j; k++)
        {
          ap.x += pl[k].x;
          ap.y += pl[k].y;
          ap.z += pl[k].z;
          ap.curvature += pl[k].curvature;
        }
        ap.x /= (j-last_surface);
        ap.y /= (j-last_surface);
        ap.z /= (j-last_surface);
        ap.curvature /= (j-last_surface);
        pl_surf.push_back(ap);
      }
      last_surface = -1;
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "lidar";
  output.header.stamp = ct;
  // pub_surf.publish(output);
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;
  group_dis = group_dis * group_dis;
  // i_nex = i_cur;

  double two_dis;
  vector<double> disarr;
  disarr.reserve(20);

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)
  {
    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    disarr.push_back(types[i_nex].dista);
  }
  
  for(;;)
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

    if(types[i_nex].range < blind)
    {
      curr_direct.setZero();
      return 2;
    }
    vx = pl[i_nex].x - pl[i_cur].x;
    vy = pl[i_nex].y - pl[i_cur].y;
    vz = pl[i_nex].z - pl[i_cur].z;
    two_dis = vx*vx + vy*vy + vz*vz;
    if(two_dis >= group_dis)
    {
      break;
    }
    disarr.push_back(types[i_nex].dista);
    i_nex++;
  }

  double leng_wid = 0;
  double v1[3], v2[3];
  for(uint j=i_cur+1; j<i_nex; j++)
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;
    v1[0] = pl[j].x - pl[i_cur].x;
    v1[1] = pl[j].y - pl[i_cur].y;
    v1[2] = pl[j].z - pl[i_cur].z;

    v2[0] = v1[1]*vz - vy*v1[2];
    v2[1] = v1[2]*vx - v1[0]*vz;
    v2[2] = v1[0]*vy - vx*v1[1];

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
    if(lw > leng_wid)
    {
      leng_wid = lw;
    }
  }


  if((two_dis*two_dis/leng_wid) < p2l_ratio)
  {
    curr_direct.setZero();
    return 0;
  }

  uint disarrsize = disarr.size();
  for(uint j=0; j<disarrsize-1; j++)
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])
      {
        leng_wid = disarr[j];
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)
  {
    curr_direct.setZero();
    return 0;
  }

  if(lidar_type==AVIA)
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  else
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];
    if(dismax_min >= limit_maxmin)
    {
      curr_direct.setZero();
      return 0;
    }
  }
  
  curr_direct << vx, vy, vz;
  curr_direct.normalize();
  return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)
  {
    if(types[i-1].range<blind || types[i-2].range<blind)
    {
      return false;
    }
  }
  else if(nor_dir == 1)
  {
    if(types[i+1].range<blind || types[i+2].range<blind)
    {
      return false;
    }
  }
  double d1 = types[i+nor_dir-1].dista;
  double d2 = types[i+3*nor_dir-2].dista;
  double d;

  if(d1<d2)
  {
    d = d1;
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);
  d2 = sqrt(d2);

  if(d1>edgea*d2 || (d1-d2)>edgeb)
  {
    return false;
  }
  
  return true;
}