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
					cout << ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor) << endl;
				}
			}
		}
	}

	void toOf(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor){
		mesh.clear();
		mesh.setMode(OF_PRIMITIVE_POINTS);
		mesh.getVertices().resize(cloud->points.size());
		if(cloud->is_dense){
			for(int i=0;i<cloud->points.size();i++){
				mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
			}
		}else{
			int i=0, n=0;
			for(int y=0;y<(int)cloud->height;y++){
				for(int x=0;x<(int)cloud->width;x++){
					if(isnan(cloud->points[i].x) || isnan(cloud->points[i].y) || isnan(cloud->points[i].z)){
						mesh.getVertices()[i] = ofVec3f(float(x)/float(cloud->width)*xfactor,float(y)/float(cloud->height)*yfactor,0);
						n++;
					}else{
						mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
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

	void toOf(pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, ofMesh & mesh, float xfactor, float yfactor, float zfactor){
		mesh.setMode(OF_PRIMITIVE_POINTS);
		mesh.getVertices().resize(cloud->points.size());
		mesh.getNormals().resize(cloud->points.size());
		//mesh.getColors().resize(cloud->points.size());
		for(int i=0;i<cloud->points.size();i++){
			mesh.getVertices()[i] = ofVec3f(cloud->points[i].x*xfactor,cloud->points[i].y*yfactor,cloud->points[i].z*zfactor);
			mesh.getNormals()[i] = ofVec3f(cloud->points[i].normal[0],cloud->points[i].normal[1],cloud->points[i].normal[2]);

			//mesh.getColors()[i] = ofFloatColor(cloud->points[i].intensity,cloud->points[i].intensity,cloud->points[i].intensity);
		}
	}

	void toPCL(const ofMesh & mesh, pcl::PointCloud<pcl::PointXYZ>::Ptr pc){
		pc->clear();
		for(int i=0;i<mesh.getVertices().size();i++){
			const ofVec3f & v = mesh.getVertices()[i];
			pc->push_back(pcl::PointXYZ(v.x*.001,v.y*.001,v.z*.001));
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

	void getMesh(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, ofMesh & mesh){
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
		if(!cloud->is_dense){
			pcl::OrganizedFastMesh<pcl::PointXYZ> gp;

			gp.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_RIGHT_CUT);
			gp.setInputCloud(cloud);
			gp.setTrianglePixelSize(3);
			gp.reconstruct(*triangles);
			toOf(cloud,mesh,1000,1000,1000);
			addIndices(mesh,triangles);
		}else{
			pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormals(cloud);

			// Concatenate the XYZ and normal fields*
			pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
			pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
			//* cloud_with_normals = cloud + normals

			// Create search tree*
			pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
			tree2->setInputCloud (cloud_with_normals);

			// Initialize objects
			pcl::GreedyProjectionTriangulation<pcl::PointXYZINormal> gp3;
			// Set the maximum distance between connected points (maximum edge length)
			gp3.setSearchRadius (1);

			// Set typical values for the parameters
			gp3.setMu (1.5);
			gp3.setMaximumNearestNeighbors (100);
			gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
			gp3.setMinimumAngle(M_PI/18); // 10 degrees
			gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
			gp3.setNormalConsistency(false);

			// Get result
			gp3.setInputCloud (cloud_with_normals);
			gp3.setSearchMethod (tree2);
			gp3.reconstruct (*triangles);

			toOf(cloud_with_normals,mesh,1000,1000,1000);
			addIndices(mesh,triangles);
		}
	}


	void marchingCubes(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, ofMesh & mesh){
		pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormals(cloud);
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::MarchingCubesGreedy<pcl::PointXYZINormal> mc;


		// Get result
		mc.setInputCloud (cloud_with_normals);
		mc.setSearchMethod (tree2);
		mc.reconstruct (*triangles);

		toOf(cloud_with_normals,mesh,1000,1000,1000);
		addIndices(mesh,triangles);
	}

	pcl::PointCloud<pcl::Normal>::Ptr calculateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float normal_radius_){
		SearchMethod::Ptr search_method_xyz_;
		pcl::PointCloud<pcl::Normal>::Ptr normals_;
		normals_ = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);

		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
		norm_est.setInputCloud (cloud);
		norm_est.setSearchMethod (search_method_xyz_);
		norm_est.setRadiusSearch (normal_radius_);
		norm_est.compute (*normals_);

		return normals_;
	}


	pcl::PointCloud<pcl::Normal>::Ptr calculateNormalsOrdered(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal>::NormalEstimationMethod normalEstimationMethod,float maxDepthChangeFactor, float normalSmoothingSize){
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setNormalEstimationMethod (normalEstimationMethod);
		ne.setMaxDepthChangeFactor(maxDepthChangeFactor);
		ne.setNormalSmoothingSize(normalSmoothingSize);
		ne.setInputCloud(cloud);
		ne.compute(*normals);

		return normals;
	}

	typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
	LocalFeatures::Ptr
	computeLocalFeatures (const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, const pcl::PointCloud<pcl::Normal>::Ptr & normals,float feature_radius)
	{
	  LocalFeatures::Ptr features;
	  SearchMethod::Ptr search_method_xyz_;
	  features = LocalFeatures::Ptr (new LocalFeatures);


	  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
	  fpfh_est.setInputCloud (cloudIn);
	  fpfh_est.setInputNormals (normals);
	  fpfh_est.setSearchMethod (search_method_xyz_);
	  fpfh_est.setRadiusSearch (feature_radius);
	  fpfh_est.compute (*features);
	  return features;
	}

	void voxelGridFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, pcl::PointCloud<pcl::PointXYZ> & cloudOut, float leafSize ){
		pcl::VoxelGrid<pcl::PointXYZ> p;
		p.setInputCloud (cloudIn);
		//p.setFilterLimits (0.0, 0.5);
		//p.setFilterFieldName ("z");
		p.setLeafSize (leafSize, leafSize, leafSize);
		p.filter(cloudOut);
	}

	void statisticalOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, pcl::PointCloud<pcl::PointXYZ> & cloudOut ){
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> p;
		p.setInputCloud (cloudIn);
		p.setMeanK (50);
		p.setStddevMulThresh (1.0);
		p.filter(cloudOut);
	}

	void removeDistanteFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudIn, pcl::PointCloud<pcl::PointXYZ> & cloudOut , float depth_limit){
		  pcl::PassThrough<pcl::PointXYZ> pass;
		  pass.setInputCloud (cloudIn);
		  pass.setFilterFieldName ("z");
		  pass.setFilterLimits (0, depth_limit);
		  pass.filter (cloudOut);
	}

	pcl::PolygonMesh::Ptr gridProjection(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float res, float padding){
		pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormals(cloud,0.02);
		pcl::PolygonMesh::Ptr pm(new pcl::PolygonMesh);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		pcl::GridProjection<pcl::PointXYZINormal> gridProjection;
		pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
		tree2->setInputCloud (cloud_with_normals);
		// Set parameters
		gridProjection.setResolution(0.005);
		gridProjection.setPaddingSize(3);
		//gridProjection.setNearestNeighborNum(100);
		//gridProjection.setMaxBinarySearchLevel(10);

		gridProjection.setSearchMethod(tree2);
		gridProjection.setInputCloud(cloud_with_normals);
		gridProjection.reconstruct(*pm);
		return pm;
	}

	pcl::PointCloud<pcl::PointNormal>::Ptr movingLeastSquares(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud){
		  pcl::PointCloud<pcl::PointXYZ> mls_points;
		  pcl::PointCloud<pcl::Normal>::Ptr mls_normals (new pcl::PointCloud<pcl::Normal> ());
		  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::Normal> mls;
		  boost::shared_ptr<vector<int> > indices (new vector<int>);


		  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		  tree->setInputCloud (cloud);

		  // Set parameters
		  mls.setInputCloud (cloud);
		  //mls.setIndices (indices);
		  mls.setPolynomialFit (true);
		  mls.setSearchMethod (tree);
		  mls.setSearchRadius (0.03);

		  // Reconstruct
		  mls.setOutputNormals (mls_normals);
		  mls.reconstruct (mls_points);

		  pcl::PointCloud<pcl::PointNormal>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointNormal> ());
		  pcl::concatenateFields (mls_points, *mls_normals, *mls_cloud);

		  return mls_cloud;
	}

	pcl::PolygonMesh::Ptr marchingCubes(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, float leafSize){
		pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormals(cloud);
		pcl::PolygonMesh::Ptr pm(new pcl::PolygonMesh);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		pcl::MarchingCubesGreedy<pcl::PointXYZINormal> marchingCubes;
		pcl::search::KdTree<pcl::PointXYZINormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZINormal>);
		tree2->setInputCloud (cloud_with_normals);

		marchingCubes.setSearchMethod(tree2);
		marchingCubes.setLeafSize(leafSize);
		marchingCubes.setIsoLevel(0.5);   //ISO: must be between 0 and 1.0
		marchingCubes.setInputCloud(cloud_with_normals);
		marchingCubes.reconstruct(*pm);
		return pm;
	}

	pcl::PolygonMesh::Ptr greedyProjectionTriangulation(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud){
		pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);

		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals = movingLeastSquares(cloud);

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (0.3);

		// Set typical values for the parameters
		gp3.setMu (1.5);
		gp3.setMaximumNearestNeighbors (100);
		gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (*triangles);

		return triangles;
	}

	PointCloudPtr
	thresholdDepth (const PointCloudPtr & input, float min_depth, float max_depth)
	{
	  pcl::PassThrough<PointT> pass_through;
	  pass_through.setInputCloud (input);
	  pass_through.setFilterFieldName ("z");
	  pass_through.setFilterLimits (min_depth, max_depth);
	  PointCloudPtr thresholded (new PointCloud);
	  pass_through.filter (*thresholded);

	  return (thresholded);
	}

	/* Use a VoxelGrid filter to reduce the number of points */
	PointCloudPtr
	downsample (const PointCloudPtr & input, float leaf_size)
	{
	  pcl::VoxelGrid<PointT> voxel_grid;
	  voxel_grid.setInputCloud (input);
	  voxel_grid.setLeafSize (leaf_size, leaf_size, leaf_size);
	  PointCloudPtr downsampled (new PointCloud);
	  voxel_grid.filter (*downsampled);

	  return (downsampled);
	}

	/* Use a RadiusOutlierRemoval filter to remove all points with too few local neighbors */
	PointCloudPtr
	removeOutliers (const PointCloudPtr & input, float radius, int min_neighbors)
	{
	  pcl::RadiusOutlierRemoval<PointT> radius_outlier_removal;
	  radius_outlier_removal.setInputCloud (input);
	  radius_outlier_removal.setRadiusSearch (radius);
	  radius_outlier_removal.setMinNeighborsInRadius (min_neighbors);
	  PointCloudPtr inliers (new PointCloud);
	  radius_outlier_removal.filter (*inliers);

	  return (inliers);
	}
}
