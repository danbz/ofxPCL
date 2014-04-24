// Example from http://pointclouds.org/documentation/tutorials/voxel_grid.php

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	ofSetLogLevel(OF_LOG_VERBOSE);
	
	glEnable(GL_DEPTH_TEST);
	
	dispRaw = false;
	
	ofxPCL::PointXYZCloud cloud(new ofxPCL::PointXYZCloud::element_type);
	
	cloud = ofxPCL::loadPointCloud<ofxPCL::PointXYZCloud>(string("table_scene_lms400.pcd"));
	
	meshRaw = ofxPCL::toOF(cloud);
	
	ofLogVerbose() << "PointCloud before filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";
	
	ofxPCL::downsample(cloud, ofVec3f(0.01f, 0.01f, 0.01f));
	
	ofLogVerbose() << "PointCloud after filtering: " << cloud->width * cloud->height << " data points (" << pcl::getFieldsList (*cloud) << ").";
	
	//ofxPCL::savePointCloud("table_scene_lms400_downsampled.pcd", cloud);
	
	mesh = ofxPCL::toOF(cloud);
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	
	cam.begin();
	ofScale(100, 100, 100);
	
	if( dispRaw ) {
		meshRaw.drawVertices();
	} else {
		mesh.drawVertices();
	}
	
	cam.end();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
	if(key == ' ') {
		dispRaw = !dispRaw;
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
