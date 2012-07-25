#pragma once

#undef Success
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include <pcl/registration/icp.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/marching_cubes_greedy.h>

#include <pcl/keypoints/harris_keypoint3D.h>
#include <pcl/search/pcl_search.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/radius_outlier_removal.h>

#include "typedefs.h"

#include "ofConstants.h"
#include "ofMesh.h"

POINT_CLOUD_REGISTER_POINT_STRUCT (ofPoint,           // here we assume a XYZ + "test" (as fields)
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z))


namespace ofxPCL{
	void toOf(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor);
	void toOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);
	void toOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::Normal>::Ptr & normals, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);
	void toOf(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);
	void toOf(pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);

	void toPCL(const ofMesh & mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

	void smooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & mls_points, pcl::PointCloud<pcl::Normal>::Ptr & mls_normals, float radius=0.03);

	void getMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, ofMesh & mesh);
	void marchingCubes(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, ofMesh & mesh);
	void addIndices(ofMesh & mesh, pcl::PolygonMesh::Ptr & triangles);


	typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

	pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float normal_radius_=0.02);


	pcl::PointCloud<pcl::Normal>::Ptr calculateNormalsOrdered(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod normalEstimationMethod=pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::AVERAGE_3D_GRADIENT,float maxDepthChangeFactor=0.02, float normalSmoothingSize=10);

	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	LocalFeatures::Ptr
	computeLocalFeatures (const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, const pcl::PointCloud<pcl::Normal>::Ptr & normals,float feature_radius=.05);

	void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, pcl::PointCloud<pcl::PointXYZ> & cloudOut, float leafSize=0.005  );

	void statisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, pcl::PointCloud<pcl::PointXYZ> & cloudOut );

	void removeDistanteFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, pcl::PointCloud<pcl::PointXYZ> & cloudOut , float depth_limit = 1.0);

	pcl::PolygonMesh::Ptr gridProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float res = 0.005, float padding = 3);

	pcl::PointCloud<pcl::PointNormal>::Ptr movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);

	pcl::PolygonMesh::Ptr marchingCubes(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float leafSize);

	pcl::PolygonMesh::Ptr greedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);


	/* Use a PassThrough filter to remove points with depth values that are too large or too small */
	PointCloudPtr
	thresholdDepth (const PointCloudPtr & input, float min_depth, float max_depth);

	/* Use a VoxelGrid filter to reduce the number of points */
	PointCloudPtr
	downsample (const PointCloudPtr & input, float leaf_size);

	/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
	PointCloudPtr
	removeOutliers (const PointCloudPtr & input, float radius, int min_neighbors);

}
