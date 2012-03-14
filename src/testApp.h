#pragma once

#include <pcl/io/openni_grabber.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl-1.3/pcl/keypoints/sift_keypoint.h>
#include "ofxPCL.h"



#include "ofMain.h"
#include "gui/ofSlider.h"
#include "gui/ofToggle.h"
#include "gui/ofPanel.h"
#include "ofxGrabCam.h"

class testApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();

		void keyPressed  (int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);

		void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud);

		void loadBYOS();
		void initCamera();
		void loadPCDCollection(string folder);
		void createMeshes();

		void detectKeyPoints(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints);
		void extractDescriptors(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);
		void findCorrespondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr src, pcl::PointCloud<pcl::PointXYZRGB>::Ptr target);

		ofVec3f centroid;
		ofMutex mutex;
		bool isNewFrame;
		pcl::PassThrough<pcl::PointXYZRGB> passthrough;


		pcl::PointCloud<pcl::Normal>::Ptr normals;

		ofPanel gui;
		ofSlider minZ, maxZ;
		ofSlider zoomZ;
		ofSlider rotY, rotX, rotZ;
		ofSlider smoothRadius;
		ofSlider currentMesh;
		ofToggle pc_mesh;
		ofToggle wireframe;
		ofToggle smoothBtn;
		ofToggle showSmoothed;

		bool prevSmooth;
		bool doSmooth;

		vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
		vector<ofVboMesh> of_clouds;
		vector<ofVboMesh> meshes;
		vector<ofVboMesh> smooths;

		float factorX, factorY, factorZ;//, smoothRadius;


		vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> keypoints;
		vector<ofMesh> of_keypoints;
		boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector;
		pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift3D;
		pcl::Feature<pcl::PointXYZRGB, pcl::FPFHSignature33>::Ptr feature_extractor;

		ofEasyCam camera;

        //Substract planes
		bool bSubstractPlane;
		double distanceThreshold, maxIters;

		void substractPlane();
};
