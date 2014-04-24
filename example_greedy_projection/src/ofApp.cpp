// Example from http://pointclouds.org/documentation/tutorials/greedy_projection.php

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	glEnable(GL_DEPTH_TEST);
	
	dispRaw = false;
	
	ofxPCL::PointXYZCloud cloud(new ofxPCL::PointXYZCloud::element_type);
	ofxPCL::PointNormalCloud cloud_with_normals(new ofxPCL::PointNormalCloud::element_type);
	
	cloud = ofxPCL::loadPointCloud<ofxPCL::PointXYZCloud>("bun0.pcd");
	
	ofxPCL::normalEstimation(cloud, cloud_with_normals);
	
	mesh = ofxPCL::triangulate(cloud_with_normals, 0.025);
}

//--------------------------------------------------------------
void ofApp::update(){

}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);
	
	cam.begin();
	
	ofScale(1000, 1000, 1000);
	
	if( dispRaw ) {
		mesh.drawVertices();
	} else {
		mesh.draw();
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
