#pragma once


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

#include "ofConstants.h"
#include "ofMesh.h"

POINT_CLOUD_REGISTER_POINT_STRUCT (ofPoint,           // here we assume a XYZ + "test" (as fields)
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z))


namespace ofxPCL{
	void toOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);
	void toOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::Normal>::Ptr & normals, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);
	void toOf(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, ofMesh & mesh, float xfactor=1, float yfactor=1, float zfactor=-1);

	void smooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & mls_points, pcl::PointCloud<pcl::Normal>::Ptr & mls_normals, float radius=0.03);

	pcl::PolygonMesh::Ptr getMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
	pcl::PolygonMesh::Ptr marchingCubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
	pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud);
	void addIndices(ofMesh & mesh, pcl::PolygonMesh::Ptr & triangles);
}
