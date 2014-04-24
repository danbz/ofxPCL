// Example from http://pointclouds.org/documentation/tutorials/resampling.php

#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
	glEnable(GL_DEPTH_TEST);
	
	dispRaw = false;
	
	ofxPCL::PointXYZCloud cloud(new ofxPCL::PointXYZCloud::element_type);
	ofxPCL::PointNormalCloud mls_points(new ofxPCL::PointNormalCloud::element_type);
	
	cloud = ofxPCL::loadPointCloud<ofxPCL::PointXYZCloud>("bun0.pcd");
	
	meshRaw = ofxPCL::toOF(cloud);
	
	ofxPCL::movingLeastSquares(cloud, mls_points, 0.03);
	
	mesh = ofxPCL::toOF(mls_points);
}

//--------------------------------------------------------------
void ofApp::update(){

}

void drawNormals(ofVboMesh &mesh)
{
	for (int i = 0; i < mesh.getNumVertices(); i++)
	{
		ofVec3f v = mesh.getVertex(i);
		ofVec3f n = mesh.getNormal(i);
		
		ofLine(v, v + n * 0.01);
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofEnableAlphaBlending();
	
	ofBackground(0);
	
	cam.begin();
	ofScale(1000, 1000, 1000);
	
	if( dispRaw ) {
		ofSetColor(255);
		meshRaw.drawVertices();
	} else {
		ofSetColor(255);
		mesh.drawVertices();
		
		ofSetColor(255, 0, 0, 127);
		drawNormals(mesh);
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
