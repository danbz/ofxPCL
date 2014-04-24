// Example from http://pointclouds.org/documentation/tutorials/extract_indices.php

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	glEnable(GL_DEPTH_TEST);
	
	ofxPCL::PointXYZCloud cloud(new ofxPCL::PointXYZCloud::element_type);
	vector<ofxPCL::PointXYZCloud> clouds;
	
	cloud = ofxPCL::loadPointCloud<ofxPCL::PointXYZCloud>("table_scene_lms400.pcd");
	
	ofLogVerbose() << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";
	
	ofxPCL::downsample(cloud, ofVec3f(0.01f, 0.01f, 0.01f));
	
	ofLogVerbose() << "PointCloud after filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";
	
	//ofxPCL::savePointCloud("table_scene_lms400_inliers.pcd", cloud);
	
	clouds = ofxPCL::segmentation(cloud, pcl::SACMODEL_PLANE, 0.01, 10, 30);
	
	ofLogVerbose() << clouds.size() << " meshes extracted";
	
	for( int i = 0; i < clouds.size(); i++ ) {
		meshes.push_back(ofxPCL::toOF(clouds[i]));
		meshes.at(i).clearColors();
	}
	meshIndex = 0;
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	
	cam.begin();
	ofScale(100, 100, 100);
	
	for( int i = 0; i < meshes.size(); i++ ) {
		if( toggleRenderAll || i == meshIndex ) {
			ofColor color;
			color.setHsb(255 * i / meshes.size(), 255, 255);
			ofSetColor(color);
		} else {
			ofSetColor(ofColor::gray);
		}
		
		meshes.at(i).drawVertices();
	}
	
	cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == ' ') {
		toggleRenderAll = !toggleRenderAll;
	}
	if(key == OF_KEY_RIGHT) {
		meshIndex = (meshIndex + 1) % meshes.size();
	}
	if(key == OF_KEY_LEFT) {
		meshIndex = (meshIndex - 1 + meshes.size()) % meshes.size();
	}
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
