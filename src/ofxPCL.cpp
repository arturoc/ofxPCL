#include "ofxPCL.h"
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/surface/surfel_smoothing.h>
#include <pcl/surface/marching_cubes_greedy.h>

namespace ofxPCL{
	void toOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor){
		mesh.setMode(OF_PRIMITIVE_POINTS);
		mesh.getVertices().resize(cloud->points.size());
		mesh.getColors().resize(cloud->points.size());
		int i=0, n=0;
		if(cloud->is_dense){
			for(int i=0;i<cloud->points.size();i++){
				mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
				mesh.getColors()[i] = ofColor(cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
			}
		}else{
			for(int y=0;y<(int)cloud->height;y++){
				for(int x=0;x<(int)cloud->width;x++){
					if(isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z)){
						mesh.getVertices()[i] = ofVec3f(float(x)/float(cloud->width)*xfactor,float(y)/float(cloud->height)*yfactor,0);
						mesh.getColors()[i] = ofColor(0,0,0);
						n++;
					}else{
						mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
						mesh.getColors()[i] = ofColor(cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
					}
					i++;
				}
			}
		}
	}

	void toOf(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::Normal>::Ptr & normals, ofMesh & mesh, float xfactor, float yfactor, float zfactor){
		mesh.setMode(OF_PRIMITIVE_POINTS);
		mesh.getVertices().resize(cloud->points.size());
		mesh.getColors().resize(cloud->points.size());
		if(cloud->is_dense){
			for(int i=0;i<cloud->points.size();i++){
				mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
				mesh.getColors()[i] = ofColor(cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
			}
			if(normals){
				mesh.getNormals().resize(normals->points.size());
				for(int i=0;i<normals->points.size();i++){
					mesh.getNormals()[i] = ofVec3f(normals->points[i].normal_x,normals->points[i].normal_y,normals->points[i].normal_z);
				}
			}
		}else{
			int i=0, n=0;
			for(int y=0;y<(int)cloud->height;y++){
				for(int x=0;x<(int)cloud->width;x++){
					if(isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z)){
						mesh.getVertices()[i] = ofVec3f(float(x)/float(cloud->width)*xfactor,float(y)/float(cloud->height)*yfactor,0);
						mesh.getColors()[i] = ofColor(0,0,0);
						n++;
					}else{
						mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
						mesh.getColors()[i] = ofColor(cloud->points[i].r,cloud->points[i].g,cloud->points[i].b);
					}
					i++;
				}
			}
			for(int i=0;i<normals->points.size();i++){
				mesh.getNormals()[i] = ofVec3f(normals->points[i].normal_x,normals->points[i].normal_y,normals->points[i].normal_z);
			}
		}
	}

	void toOf(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor){
		mesh.setMode(OF_PRIMITIVE_POINTS);
		mesh.getVertices().resize(cloud->points.size());
		//mesh.getColors().resize(cloud->points.size());
		for(int i=0;i<cloud->points.size();i++){
			mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
			//mesh.getColors()[i] = ofFloatColor(cloud->points[i].intensity,cloud->points[i].intensity,cloud->points[i].intensity);
		}
	}

	void addIndices(ofMesh & mesh, pcl::PolygonMesh::Ptr & triangles){
		mesh.getIndices().clear();
		mesh.setMode(OF_PRIMITIVE_TRIANGLES);
		for(int i=0;i<triangles->polygons.size();i++){
			for(int j=0;j<triangles->polygons[i].vertices.size();j++){
				mesh.addIndex(triangles->polygons[i].vertices[j]);
			}
		}
	}

	void smooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & mls_points, pcl::PointCloud<pcl::Normal>::Ptr & mls_normals, float radius){
		// Create a KD-Tree
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (cloud);

		// Init object (second point type is for the normals, even if unused)
		pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::Normal> mls;

		// Optionally, a pointer to a cloud can be provided, to be set by MLS
		mls.setOutputNormals (mls_normals);

		// Set parameters
		mls.setInputCloud (cloud);
		mls.setPolynomialFit (true);
		mls.setSearchMethod (tree);
		mls.setSearchRadius (radius);
		//mls.setPolynomialOrder(10);
		//mls.setSqrGaussParam(radius*radius);

		// Reconstruct
		mls.reconstruct (*mls_points);

		// Concatenate fields for saving
		//pcl::PointCloud<pcl::PointNormal> mls_cloud;
		//pcl::concatenateFields (mls_points, *mls_normals, mls_cloud);
	}

	void surfelSmooth(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & mls_points, pcl::PointCloud<pcl::Normal>::Ptr & mls_normals){
		//pcl::SurfelSmoothing smoother;
		//smoother.computeSmoothedCloud(mls_points,mls_normals)
	}

	pcl::PolygonMesh::Ptr getMesh(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
		if(!cloud->is_dense){
			pcl::OrganizedFastMesh<pcl::PointXYZRGB> gp;

			gp.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_RIGHT_CUT);
			gp.setInputCloud(cloud);
			gp.setTrianglePixelSize(3);
			gp.reconstruct(*triangles);

		}else{
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
			pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
			tree->setInputCloud (cloud);
			n.setInputCloud (cloud);
			n.setSearchMethod (tree);
			n.setKSearch (20);
			n.compute (*normals);

			// Concatenate the XYZ and normal fields*
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
			//* cloud_with_normals = cloud + normals

			// Create search tree*
			pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
			tree2->setInputCloud (cloud_with_normals);

			// Initialize objects
			pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
			// Set the maximum distance between connected points (maximum edge length)
			gp3.setSearchRadius (50);

			// Set typical values for the parameters
			gp3.setMu (2.5);
			gp3.setMaximumNearestNeighbors (100);
			gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			gp3.setMinimumAngle(M_PI/18); // 10 degrees
			gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			gp3.setNormalConsistency(false);

			// Get result
			gp3.setInputCloud (cloud_with_normals);
			gp3.setSearchMethod (tree2);
			gp3.reconstruct (*triangles);
		}
		return triangles;
	}

	pcl::PolygonMesh::Ptr marchingCubes(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
		tree->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::MarchingCubesGreedy<pcl::PointXYZRGBNormal> mc;


		// Get result
		mc.setInputCloud (cloud_with_normals);
		mc.setSearchMethod (tree2);
		mc.reconstruct (*triangles);
		return triangles;
	}

	pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud){
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
		pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>::Ptr organized (new pcl::search::OrganizedNeighbor<pcl::PointXYZRGB>);
		organized->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (organized);
		n.setKSearch (20);
		n.compute (*normals);

		return normals;
	}
}
