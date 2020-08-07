/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: ES_MUSS_SEIN
 *********************************************************************/

#ifndef PCL_VOXEL_VIEW_H_
#define PCL_VOXEL_VIEW_H_

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>
#include<bits/stdc++.h>


namespace pcl_voxel_view{
/**
  * @class PclVoxelView
  * @brief Implements the function to view the voxels of pointcloud
  * @todo good view to the pointcloud
  */

class PclVoxelView{
public:
  /**
    * @brief Default constructor of the class
    */
  PclVoxelView();
  /**
    * @brief  Destructor of the class
    */
  ~PclVoxelView();
  /**
    * @brief Default initialization of the class
    */
  void initialize();
  /**
    * @brief publish the voxel view of pointcloud
    */
  void publishPclVoxel();

private:
  double voxel_size; // define the size of voxel size
  std::string pcl_path_one; // define the path of pointcloud to read
  std::string pcl_path_two; // define the path of pointcloud to read

  pcl::PointCloud<pcl::PointXYZI>::Ptr mappcl_one;
  pcl::PointCloud<pcl::PointXYZI>::Ptr mappcl_two;
  ros::NodeHandle nh;
  ros::Publisher voxel_pub_one;
  ros::Publisher voxel_pub_two;
  ros::Publisher voxel_pub_mixed;
  ros::NodeHandle private_nh;



};

};


#endif // TEB_LOCAL_PLANNER_ROS_H_
