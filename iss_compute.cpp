// Generic pcl
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/range_image_visualizer.h>
 
// pcl keypoints
#include <pcl/keypoints/iss_3d.h>
#include <pcl/io/obj_io.h> 
// pcl features
#include <pcl/features/range_image_border_extractor.h>
 
using namespace std;
using namespace pcl;
 
// pcl definition
typedef PointCloud<PointXYZ> PointCloudXYZ;
 
double computeCloudResolution(const PointCloudXYZ::Ptr &cloud) {
    double resolution = 0.0;
    int numberOfPoints = 0;
    int nres;
    vector<int> indices(2);
    vector<float> squaredDistances(2);
    search::KdTree <PointXYZ> tree;
    tree.setInputCloud(cloud);
 
    for (size_t i = 0; i < cloud->size(); ++i) {
        if (!pcl_isfinite((*cloud)[i].x))
            continue;
 
        // Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
        if (nres == 2) {
            resolution += sqrt(squaredDistances[1]);
            ++numberOfPoints;
        }
    }
    if (numberOfPoints != 0)
        resolution /= numberOfPoints;
 
    return resolution;
}
 
int main(int argc, char **argv) {
    PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
    PointCloudXYZ::Ptr cloud_keypoints(new PointCloudXYZ);
 
//    if (pcl::io::loadPCDFile<PointXYZ>(argv[1], *cloud) != 0) {
    if (pcl::io::loadOBJFile(argv[1], *cloud) != 0) {
        return -1;
    }
 
    ISSKeypoint3D<PointXYZ, PointXYZ> detector;
    detector.setInputCloud(cloud);
    search::KdTree<PointXYZ>::Ptr kdtree(new search::KdTree<PointXYZ>);
    detector.setSearchMethod(kdtree);
 
    double resolution = computeCloudResolution(cloud);
    cout << "Resolution: " << resolution << endl;
 
    detector.setSalientRadius(6 * resolution);
    detector.setNonMaxRadius(4 * resolution);
    detector.setMinNeighbors(5);
    detector.setThreshold21(0.975);
    detector.setThreshold32(0.975);
    detector.compute(*cloud_keypoints);
    cout << cloud_keypoints->points.size() << " keypoints" << endl;
}
