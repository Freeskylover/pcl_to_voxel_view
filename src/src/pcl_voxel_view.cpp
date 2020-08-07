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

#include <pcl_voxel_view/pcl_voxel_view.h>
#include <set>
#include <sstream>

namespace pcl_voxel_view{
    PclVoxelView::PclVoxelView():private_nh("~"){
   
    }
    PclVoxelView::~PclVoxelView(){
        
    }

    void PclVoxelView::initialize(){
        
        private_nh.param("voxel_size", voxel_size, 1.0);
        private_nh.param("pcl_path_one", pcl_path_one, std::string("~/bag/view_one.pcd") );
        private_nh.param("pcl_path_two", pcl_path_two, std::string("~/bag/view_two.pcd") );

        voxel_pub_one = nh.advertise<visualization_msgs::MarkerArray>("/voxelview_one", 5);
        voxel_pub_two = nh.advertise<visualization_msgs::MarkerArray>("/voxelview_two", 5);
        voxel_pub_mixed = nh.advertise<visualization_msgs::MarkerArray>("/voxelview_mixed", 5);



        mappcl_one.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(pcl_path_one, *mappcl_one);
        mappcl_one->header.frame_id = "map";

        pcl::VoxelGrid<pcl::PointXYZI> voxelgrid_one;
        voxelgrid_one.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxelgrid_one.setInputCloud(mappcl_one);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_one(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid_one.filter(*filtered_one);

        mappcl_one = filtered_one;


        mappcl_two.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(pcl_path_two, *mappcl_two);
        mappcl_two->header.frame_id = "map";

        pcl::VoxelGrid<pcl::PointXYZI> voxelgrid_two;
        voxelgrid_two.setLeafSize(voxel_size, voxel_size, voxel_size);
        voxelgrid_two.setInputCloud(mappcl_two);

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_two(new pcl::PointCloud<pcl::PointXYZI>());
        voxelgrid_two.filter(*filtered_two);

        mappcl_two = filtered_two;
    }

    void PclVoxelView::publishPclVoxel(){

        visualization_msgs::MarkerArray marker_array_one;
        visualization_msgs::MarkerArray marker_array_two;
        visualization_msgs::MarkerArray marker_array_mixed;

        std::set<std::string> set_one;
        std::set<std::string> set_two;
        std::set<std::string> set_union;
        std::set<std::string> set_intersection;
        std::set<std::string> set_difference;
        std::set<std::string> set_difference_one;
        std::set<std::string> set_difference_two;




        
        visualization_msgs::Marker bbox_marker;
        bbox_marker.header.frame_id = "map";
        bbox_marker.header.stamp = ros::Time::now();
        bbox_marker.ns = "";
        bbox_marker.color.r = 0.0f;
        bbox_marker.color.g = 1.0f;
        bbox_marker.color.b = 0.0f;
        bbox_marker.color.a = 1.0;
        bbox_marker.lifetime = ros::Duration();
        bbox_marker.frame_locked = true;
        bbox_marker.scale.x = voxel_size;
        bbox_marker.scale.y = voxel_size;
        bbox_marker.scale.z = voxel_size;
        bbox_marker.type = visualization_msgs::Marker::CUBE;
        bbox_marker.action = visualization_msgs::Marker::ADD;

        int index = 0;

        for(int i = 0; i < mappcl_one->points.size(); i++){
            std::stringstream ss_one;
            std::stringstream ss_two;
            std::stringstream ss_three;

            std::string str_x;
            std::string str_y;
            std::string str_z;

            bbox_marker.id = index;
            if (mappcl_one->points[i].x > 0){
                bbox_marker.pose.position.x = int(((mappcl_one->points[i].x + voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_one << int(((mappcl_one->points[i].x + voxel_size / 2 )/ voxel_size));

            }else{
                bbox_marker.pose.position.x = int(((mappcl_one->points[i].x - voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_one << int(((mappcl_one->points[i].x - voxel_size / 2 )/ voxel_size));
            }
            if (mappcl_one->points[i].y > 0){
                bbox_marker.pose.position.y = int(((mappcl_one->points[i].y + voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_two << int(((mappcl_one->points[i].y + voxel_size / 2 )/ voxel_size));

            }else{
                bbox_marker.pose.position.y = int(((mappcl_one->points[i].y - voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_two << int(((mappcl_one->points[i].y - voxel_size / 2 )/ voxel_size));
            }            
            if (mappcl_one->points[i].z > 0){
                bbox_marker.pose.position.z = int(((mappcl_one->points[i].z + voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_three << int(((mappcl_one->points[i].z + voxel_size / 2 )/ voxel_size));

            }else{
                bbox_marker.pose.position.z = int(((mappcl_one->points[i].z - voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_three << int(((mappcl_one->points[i].z - voxel_size / 2 )/ voxel_size));
            }

            ss_one >> str_x;
            ss_two >> str_y;   
            ss_three >> str_z;

            if(set_one.insert(str_x + " " + str_y + " " + str_z).second){
                marker_array_one.markers.push_back(bbox_marker);
                index++;

            } 
        }
        ROS_WARN("size_one_set: %d", set_one.size());
        bbox_marker.color.r = 1.0f;
        bbox_marker.color.g = 0.0f;
        bbox_marker.color.b = 0.0f;
        bbox_marker.color.a = 1.0;
        index = 0;
        for(int i = 0; i < mappcl_two->points.size(); i++){
            bbox_marker.id = index;
            std::stringstream ss_one;
            std::stringstream ss_two;
            std::stringstream ss_three;

            std::string str_x;
            std::string str_y;
            std::string str_z;
            if (mappcl_two->points[i].x > 0){
                bbox_marker.pose.position.x = int(((mappcl_two->points[i].x + voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_one << int(((mappcl_two->points[i].x + voxel_size / 2 )/ voxel_size));

            }else{
                bbox_marker.pose.position.x = int(((mappcl_two->points[i].x - voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_one << int(((mappcl_two->points[i].x - voxel_size / 2 )/ voxel_size));

            }
            if (mappcl_two->points[i].y > 0){
                bbox_marker.pose.position.y = int(((mappcl_two->points[i].y + voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_two << int(((mappcl_two->points[i].y + voxel_size / 2 )/ voxel_size));

            }else{
                bbox_marker.pose.position.y = int(((mappcl_two->points[i].y - voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_two << int(((mappcl_two->points[i].y - voxel_size / 2 )/ voxel_size));

            }
            if (mappcl_two->points[i].z > 0){
                bbox_marker.pose.position.z = int(((mappcl_two->points[i].z + voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_three << int(((mappcl_two->points[i].z + voxel_size / 2 )/ voxel_size));

            }else{
                bbox_marker.pose.position.z = int(((mappcl_two->points[i].z - voxel_size / 2 )/ voxel_size)) * voxel_size;
                ss_three << int(((mappcl_two->points[i].z - voxel_size / 2 )/ voxel_size));

            }

            ss_one >> str_x;
            ss_two >> str_y;   
            ss_three >> str_z;

             if(set_two.insert(str_x + " " + str_y + " " + str_z).second) {
                 marker_array_two.markers.push_back(bbox_marker);
                 index++;
             } 
        }
        ROS_WARN("size_two_set: %d", set_two.size());

        std::set_intersection(set_one.begin(), set_one.end(), set_two.begin(), set_two.end(), inserter(set_intersection, set_intersection.begin()));
        std::set_union(set_one.begin(), set_one.end(), set_two.begin(), set_two.end(), inserter(set_union, set_union.begin()));
        std::set_difference(set_union.begin(), set_union.end(), set_intersection.begin(), set_intersection.end(), inserter(set_difference, set_difference.begin()));
        std::set_difference(set_one.begin(), set_one.end(), set_intersection.begin(), set_intersection.end(), inserter(set_difference_one, set_difference_one.begin()));
        std::set_difference(set_two.begin(), set_two.end(), set_intersection.begin(), set_intersection.end(), inserter(set_difference_two, set_difference_two.begin()));


        ROS_WARN("size_set_intersection: %d", set_intersection.size());
        ROS_WARN("size_set_union: %d", set_union.size());
        ROS_WARN("size_set_difference: %d", set_difference.size());
        ROS_WARN("size_set_difference_one: %d", set_difference_one.size());
        ROS_WARN("size_set_difference_two: %d", set_difference_two.size());

       
        ROS_WARN("the rate of intersection and union: %f", (float)set_intersection.size()/set_union.size() );




        index = 0;

        bbox_marker.color.r = 0.0f;
        bbox_marker.color.g = 1.0f;
        bbox_marker.color.b = 0.0f;
        bbox_marker.color.a = 1.0;
        for(auto iter = set_difference_one.begin(); iter != set_difference_one.end(); iter++){
            bbox_marker.id = index;
            std::stringstream tmp(*iter);
            int tmp_x, tmp_y, tmp_z;
            tmp >> tmp_x >> tmp_y >> tmp_z;
            bbox_marker.pose.position.x = tmp_x * voxel_size;
            bbox_marker.pose.position.y = tmp_y * voxel_size;
            bbox_marker.pose.position.z = tmp_z * voxel_size;
            marker_array_mixed.markers.push_back(bbox_marker);
            index++;
        }

        bbox_marker.color.r = 1.0f;
        bbox_marker.color.g = 1.0f;
        bbox_marker.color.b = 1.0f;
        bbox_marker.color.a = 1.0;
        for(auto iter = set_intersection.begin(); iter != set_intersection.end(); iter++){
            bbox_marker.id = index;
            std::stringstream tmp(*iter);
            int tmp_x, tmp_y, tmp_z;
            tmp >> tmp_x >> tmp_y >> tmp_z;
            bbox_marker.pose.position.x = tmp_x * voxel_size;
            bbox_marker.pose.position.y = tmp_y * voxel_size;
            bbox_marker.pose.position.z = tmp_z * voxel_size;
            marker_array_mixed.markers.push_back(bbox_marker);
            index++;
        }

        bbox_marker.color.r = 1.0f;
        bbox_marker.color.g = 0.0f;
        bbox_marker.color.b = 0.0f;
        bbox_marker.color.a = 1.0;
        for(auto iter = set_difference_two.begin(); iter != set_difference_two.end(); iter++){
            bbox_marker.id = index;
            std::stringstream tmp(*iter);
            int tmp_x, tmp_y, tmp_z;
            tmp >> tmp_x >> tmp_y >> tmp_z;
            bbox_marker.pose.position.x = tmp_x * voxel_size;
            bbox_marker.pose.position.y = tmp_y * voxel_size;
            bbox_marker.pose.position.z = tmp_z * voxel_size;
            marker_array_mixed.markers.push_back(bbox_marker);
            index++;
        }
        ROS_WARN("size: %d", marker_array_one.markers.size());
        ROS_WARN("size: %d", marker_array_two.markers.size());
        ROS_WARN("size: %d", marker_array_mixed.markers.size());





        ros::Rate loop_rate(5);
        while(ros::ok()){
            voxel_pub_one.publish(marker_array_one);
            voxel_pub_two.publish(marker_array_two);
            voxel_pub_mixed.publish(marker_array_mixed);

            //ros::spinOnce();
            ROS_WARN("pub one time");

            loop_rate.sleep();
        }

    }
}