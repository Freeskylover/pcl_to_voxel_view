#include <pcl_voxel_view/pcl_voxel_view.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "pcl_voxel_view");
    pcl_voxel_view::PclVoxelView view;
    view.initialize();
    view.publishPclVoxel();
    ros::spinOnce();
    return 0;
}