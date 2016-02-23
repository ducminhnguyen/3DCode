#include <iostream>
#include <cstdlib>

#include <boost/thread/thread.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/range_image_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/narf_descriptor.h>
#include <pcl/console/parse.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/io/obj_io.h>

namespace pcl
{
  template<>
    struct SIFTKeypointFieldSelector<PointXYZ>
    {
      inline float
      operator () (const PointXYZ &p) const
      {
    return p.z;
      }
    };
}

int main (int argc, char* argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::io::loadPCDFile(argv[1], *pc);
    pcl::io::loadOBJFile(argv[1], *pc);

    const float min_scale = atof(argv[2]); 
    const int nr_octaves = 3; 
    const int nr_scales_per_octave = 2; 
    const float min_contrast = 0; 

    pcl::SIFTKeypoint<pcl::PointXYZ, pcl::PointWithScale> sift;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointWithScale>::Ptr sifts (new pcl::PointCloud<pcl::PointWithScale>);
    sift.setInputCloud(pc);
    sift.setSearchSurface(pc); 
    sift.setSearchMethod (tree);
    sift.setScales(min_scale, nr_octaves, nr_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.getIndices();
    sift.compute (*sifts);

    cout << sifts->points.size() << endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints3D(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointXYZ tmp;
    for(pcl::PointCloud<pcl::PointWithScale>::iterator i = sifts->begin(); i!= sifts->end(); i++){
        tmp = pcl::PointXYZ((*i).x,(*i).y,(*i).z);
        keypoints3D->push_back(tmp);
    }

    cout << keypoints3D->points.size() << endl;
    
    //show point cloud
    cout << "Print img" << endl;
    pcl::io::savePCDFileASCII(pc + keypoint3D);
//    pcl::visualization::PCLVisualizer viewer ("3D Viewer");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> pccolor(pc, 0, 255, 0);
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scolor(keypoints3D, 255, 0, 0);
//    viewer.addPointCloud(pc,pccolor,"testimg.png");
//    viewer.addPointCloud(keypoints3D,scolor,"keypoints.png");
//    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints.png");
//    viewer.saveScreenshot("visual_img.png");
//    while (!viewer.wasStopped ())
//    {
//        viewer.spinOnce();
//        pcl_sleep (0.01);
//    }

}
