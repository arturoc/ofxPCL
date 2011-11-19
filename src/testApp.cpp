#include "testApp.h"
#include "ofxPCL.h"
#include <pcl/surface/simplification_remove_unused_vertices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh_omp.h>

using namespace ofxPCL;

void testApp::createMeshes(){
	pcl::VoxelGrid<pcl::PointXYZRGB> voxelGridFilter;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);
	meshes.resize(clouds.size());
	of_keypoints.resize(clouds.size());
	of_clouds.resize(clouds.size());
	smooths.resize(clouds.size());
	for(int i=0;i<clouds.size();i++){
		//voxelGridFilter.setInputCloud(clouds[i]);
		//voxelGridFilter.filter(*smoothed);
		toOf(clouds[i],of_clouds[i],factorX,factorY,factorZ);
		keypoints.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>));
		detectKeyPoints(clouds[i],keypoints[i]);
		toOf(keypoints[i],of_keypoints[i],factorX,factorY,factorZ);

		/*if(doSmooth){
			smooth(clouds[i],smoothed,normals,smoothRadius);
		}else{
			smoothed = clouds[i];
		}*/
		pcl::PolygonMesh::Ptr triangles = getMesh(clouds[i]);

		toOf(clouds[i],meshes[i],factorX,factorY,factorZ);
		addIndices(meshes[i],triangles);
		//toOf(smoothed,normals,smoothedMesh,640,480,-640);

		ofLogNotice() << "created mesh" << i;

	}
	if(!clouds.empty())
		centroid = meshes[0].getCentroid();
}

void testApp::loadBYOS(){
	clouds.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>));
	clouds[0]->is_dense = true;

	ofFile meshFile("21575167_7_felipa_pardo.asc");
	ofVec3f v;
	pcl::PointXYZRGB vc;
	while(meshFile.good()){
		meshFile >> v;
		vc.x = v.x;
		vc.y = v.y;
		vc.z = v.z;
		vc.r = 255;
		vc.g = 255;
		vc.b = 255;
		clouds[0]->push_back(vc);
	}

	factorX = 1;
	factorY = 1;
	factorZ = 1;
	//smoothRadius = 30;
}

void testApp::initCamera(){
	pcl::Grabber* interface = new pcl::OpenNIGrabber();

	boost::function<void (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&)> f =	boost::bind (&testApp::cloud_cb_, this, _1);
	isNewFrame = false;
	interface->registerCallback (f);

	interface->start ();
}

void testApp::loadPCDCollection(string folder){
	ofDirectory dir;
	dir.allowExt("pcd");
	dir.listDir(folder);
	dir.sort();
	clouds.resize(dir.numFiles());
	for(int i=0;i<dir.numFiles();i++){
		clouds[i] = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (ofToDataPath(dir.getPath(i)), *clouds[i]) == -1){
			ofLogError() <<  "Couldn't read file test_pcd.pcd \n";
			continue;
		}
	}
}

void testApp::detectKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints){
    keypoint_detector->setInputCloud(input);
    keypoint_detector->compute(*keypoints);
}


void testApp::extractDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features){
	typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::copyPointCloud(*keypoints, *kpts);

	typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>::Ptr feature_from_normals = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> > (feature_extractor);

	if (feature_from_normals){
		typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
		normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
		normal_estimation.setRadiusSearch (0.01);
		normal_estimation.setInputCloud (input);
		normal_estimation.compute (*normals);

		feature_from_normals->setSearchSurface(input);
		feature_from_normals->setInputCloud(kpts);
		feature_from_normals->setInputNormals(normals);
		feature_from_normals->compute (*features);
	}else{
		feature_extractor->setSearchSurface(input);
		feature_extractor->setInputCloud(kpts);
		feature_extractor->compute (*features);
	}
}

void testApp::findCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target){
	pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints;
}


//--------------------------------------------------------------
void testApp::setup(){
	ofBackground(0,0,0);
	ofSetLogLevel(OF_LOG_NOTICE);

	factorX = 640;
	factorY = 480;
	factorZ = -640;
	//smoothRadius = .03;

	passthrough.setKeepOrganized(true);
	passthrough.setFilterFieldName("z");

	sift3D = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
	sift3D->setScales(0.01, 3, 2);
	sift3D->setMinimumContrast(0.0);

	keypoint_detector.reset(sift3D);

	feature_extractor = pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr (new pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33>);
    feature_extractor->setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    feature_extractor->setRadiusSearch (0.05);

	prevSmooth = false;
	doSmooth = false;

	normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);


	loadBYOS();
	//loadPCDCollection("minimouse");
	createMeshes();

	gui.setup("smooth");
	gui.add(smoothRadius.setup("smooth",30./factorX,0,1000/factorX));
	gui.add(smoothBtn.setup("smooth",false));
	//gui.add(minZ.setup("min z",0,0,10));
	//gui.add(maxZ.setup("max z",10,0,5));
	gui.add(rotY.setup("rot y",0,-180,180));
	gui.add(rotX.setup("rot x",0,-180,180));
	gui.add(rotZ.setup("rot z",0,-180,180));
	gui.add(pc_mesh.setup("pc/mesh",true));
	gui.add(wireframe.setup("wireframe",true));
	gui.add(showSmoothed.setup("showSmoothed",false));

	gui.add(currentMesh.setup("current",0,0,meshes.size()-1,true));
	gui.add(zoomZ.setup("zoom Z",0,0,ofGetHeight()*2));

}

void testApp::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &_cloud){
	mutex.lock();
	/**cloud = *_cloud;
	passthrough.setFilterLimits(minZ.getValue(),maxZ.getValue());
	passthrough.setInputCloud(cloud);
	passthrough.filter(*cloud);
	isNewFrame = true;*/
	mutex.unlock();
}

//--------------------------------------------------------------
void testApp::update(){
	if(smoothBtn.getValue()!=prevSmooth){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr smoothed(new pcl::PointCloud<pcl::PointXYZRGB>);

		smooth(clouds[currentMesh.getValue()],smoothed,normals,smoothRadius.getValue());
		//smoothed  = cloud;

		toOf(smoothed,normals,smooths[currentMesh.getValue()],factorX,factorY,factorZ);
		pcl::PolygonMesh::Ptr triangles = getMesh(smoothed);

		addIndices(smooths[currentMesh.getValue()],triangles);
		prevSmooth = smoothBtn.getValue();
	}

	if(isNewFrame){
		/*mutex.lock();

		pcl::PointCloud<pcl::Normal>::Ptr normals = calculateNormals(cloud);
		pcl::PolygonMesh::Ptr triangles = getMesh(cloud);

		toOf(cloud,normals,mesh,640,480,-640);
		addIndices(mesh,triangles);
		//toOf(smoothed,normals,smoothedMesh,640,480,-640);
		centroid = mesh.getCentroid();
		isNewFrame = false;
		mutex.unlock();*/
	}
}

//--------------------------------------------------------------
void testApp::draw(){

	glEnable(GL_DEPTH_TEST);

	//camera.begin();
	ofPushMatrix();
	ofTranslate(ofGetWidth()/2,ofGetHeight()/2,zoomZ.getValue());
	ofTranslate(centroid);
	ofRotateZ(rotZ.getValue());
	ofRotateY(rotY.getValue());
	ofRotateX(rotX.getValue());
	ofTranslate(-centroid);
	glPointSize(1);
	ofSetColor(ofColor::white);

	if(pc_mesh.getValue()){
		if(showSmoothed.getValue()){
			if(wireframe.getValue()){
				smooths[currentMesh.getValue()].drawWireframe();
			}else{
				smooths[currentMesh.getValue()].draw();
			}
		}else{
			if(wireframe.getValue()){
				meshes[currentMesh.getValue()].drawWireframe();
			}else{
				meshes[currentMesh.getValue()].draw();
			}
		}
	}else{
		of_clouds[currentMesh.getValue()].draw();
	}
	glPointSize(4);
	ofSetColor(ofColor::red);
	of_keypoints[currentMesh.getValue()].draw();

	ofPopMatrix();
	//camera.end();

	glDisable(GL_DEPTH_TEST);
	gui.draw();
	//ofDrawBitmapString(ofToString(ofGetFrameRate()),20,20);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
	switch(key){
	case 'p':{
		ofFile file("pointcloud.asc",ofFile::WriteOnly);
		for(int i=0;i<of_clouds[currentMesh.getValue()].getNumVertices();i++){
			file << of_clouds[currentMesh.getValue()].getVertex(i) << endl;
		}
		file.close();
	}
	break;

	case 'm':{
		ofFile file_mesh("mesh.ply",ofFile::WriteOnly);
		file_mesh << "ply" << endl;
		file_mesh << "format ascii 1.0" << endl;
		file_mesh << "comment made by arturo" << endl;

		file_mesh << "element vertex " << meshes[currentMesh.getValue()].getNumVertices() << endl;
		file_mesh << "property float x" << endl;
		file_mesh << "property float y" << endl;
		file_mesh << "property float z" << endl;
		file_mesh << "element face " << meshes[currentMesh.getValue()].getNumIndices()/3 << endl;
		file_mesh << "property list uchar int vertex_index" << endl;
		file_mesh << "end_header" << endl;
		for(int i=0;i<meshes[currentMesh.getValue()].getNumVertices();i++){
			const ofVec3f & v = meshes[currentMesh.getValue()].getVertex(i);
			file_mesh << v.x << " " << v.y << " " << v.z << endl;
		}
		int faces = 0;
		for(int i=0;i<meshes[currentMesh.getValue()].getNumIndices();i+=3){
			file_mesh << 3 << " ";
			file_mesh << meshes[currentMesh.getValue()].getIndex(i+0) << " ";
			file_mesh << meshes[currentMesh.getValue()].getIndex(i+1) << " ";
			file_mesh << meshes[currentMesh.getValue()].getIndex(i+2) << endl;
			faces++;
		}
		cout << "written " << faces << " should be " << meshes[currentMesh.getValue()].getNumIndices()/3 << endl;
		file_mesh << endl;
		file_mesh.close();
	}

	case 's':{
		ofFile file_mesh("mesh.ply",ofFile::WriteOnly);
		file_mesh << "ply" << endl;
		file_mesh << "format ascii 1.0" << endl;
		file_mesh << "comment made by arturo" << endl;

		file_mesh << "element vertex " << smooths[currentMesh.getValue()].getNumVertices() << endl;
		file_mesh << "property float x" << endl;
		file_mesh << "property float y" << endl;
		file_mesh << "property float z" << endl;
		file_mesh << "element face " << smooths[currentMesh.getValue()].getNumIndices()/3 << endl;
		file_mesh << "property list uchar int vertex_index" << endl;
		if(smooths[currentMesh.getValue()].getNumNormals()){
			file_mesh << "element normal " << smooths[currentMesh.getValue()].getNumNormals() << endl;
			file_mesh << "property float x" << endl;
			file_mesh << "property float y" << endl;
			file_mesh << "property float z" << endl;
		}
		file_mesh << "end_header" << endl;
		for(int i=0;i<smooths[currentMesh.getValue()].getNumVertices();i++){
			const ofVec3f & v = smooths[currentMesh.getValue()].getVertex(i);
			file_mesh << v.x << " " << v.y << " " << v.z << endl;
		}
		int faces = 0;
		for(int i=0;i<smooths[currentMesh.getValue()].getNumIndices();i+=3){
			file_mesh << 3 << " ";
			file_mesh << smooths[currentMesh.getValue()].getIndex(i+0) << " ";
			file_mesh << smooths[currentMesh.getValue()].getIndex(i+1) << " ";
			file_mesh << smooths[currentMesh.getValue()].getIndex(i+2) << endl;
			faces++;
		}
		for(int i=0;i<smooths[currentMesh.getValue()].getNumNormals();i++){
			const ofVec3f & v = smooths[currentMesh.getValue()].getNormal(i);
			file_mesh << v.x << " " << v.y << " " << v.z << endl;
		}
		cout << "written " << faces << " should be " << smooths[currentMesh.getValue()].getNumIndices()/3 << endl;
		file_mesh << endl;
		file_mesh.close();
	}
	break;
	}

}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){
	if(button==2){
		mutex.lock();
		//pcl::io::savePCDFile<pcl::PointXYZRGB>(ofToDataPath(ofGetTimestampString()+".pcd"),*cloud,true);
		mutex.unlock();
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}
