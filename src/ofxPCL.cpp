#include "ofxPCL.h"
#include "pcl/impl/instantiate.hpp"

using namespace pcl;

namespace ofxPCL
{

template <typename T>
ofMesh triangulate(const T &cloud_with_normals, float search_radius)
{
	assert(cloud_with_normals);
	
	ofMesh mesh;
	
	if (cloud_with_normals->points.empty()) return mesh;
	
	KdTree<typename T::element_type::PointType> kdtree(cloud_with_normals);
	
	typename pcl::GreedyProjectionTriangulation<typename T::element_type::PointType> gp3;
	pcl::PolygonMesh triangles;
	
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(search_radius);
	
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(20);
	gp3.setMaximumSurfaceAngle(ofDegToRad(90));
	gp3.setMinimumAngle(ofDegToRad(10));
	gp3.setMaximumAngle(ofDegToRad(180));
	gp3.setNormalConsistency(false);
	gp3.setConsistentVertexOrdering(true);
	
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(kdtree.kdtree);
	gp3.reconstruct(triangles);
	
	convert(cloud_with_normals, mesh);
	
	for (int i = 0; i < triangles.polygons.size(); i++)
	{
		pcl::Vertices &v = triangles.polygons[i];
		
		if (v.vertices.size() == 3)
			mesh.addTriangle(v.vertices[0], v.vertices[1], v.vertices[2]);
	}
	
	return mesh;
}

#define PCL_INSTANTIATE_triangulate(T) template ofMesh ofxPCL::triangulate<pcl::PointCloud<T>::Ptr>(const pcl::PointCloud<T>::Ptr&, float);
PCL_INSTANTIATE(triangulate, PCL_NORMAL_POINT_TYPES);

ofMesh organizedFastMesh(const ofShortPixels& depthImage, const int skip, float scale)
{
	PointXYZCloud temp = New<PointXYZCloud>();
	
	convert(depthImage, temp, skip, scale);
	
	pcl::OrganizedFastMesh<PointXYZ> ofm;
	ofm.setTrianglePixelSize(1);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<PointXYZ>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud(temp);
	
	boost::shared_ptr<std::vector<pcl::Vertices> > verts(new std::vector<pcl::Vertices>);
	ofm.reconstruct(*verts);
	
	NormalCloud normals = New<NormalCloud>();
	integralImageNormalEstimation(temp, normals);
	
	ofMesh mesh;
	
	for (int i = 0; i < verts->size(); i++)
	{
		const pcl::Vertices &v = verts->at(i);
		
		if (v.vertices.size() != 3) continue;

		const Normal &n1 = normals->points[v.vertices[0]];
		const Normal &n2 = normals->points[v.vertices[1]];
		const Normal &n3 = normals->points[v.vertices[2]];

//		if (!isnormal(n1.normal_x)
//			|| !isnormal(n1.normal_y)
//			|| !isnormal(n1.normal_z)) continue;
		
		const PointXYZ &p1 = temp->points[v.vertices[0]];
		const PointXYZ &p2 = temp->points[v.vertices[1]];
		const PointXYZ &p3 = temp->points[v.vertices[2]];
		
		mesh.addNormal(ofVec3f(n1.normal_x, n1.normal_y, n1.normal_z));
		mesh.addNormal(ofVec3f(n2.normal_x, n2.normal_y, n2.normal_z));
		mesh.addNormal(ofVec3f(n3.normal_x, n3.normal_y, n3.normal_z));
		
		mesh.addVertex(ofVec3f(p1.x, p1.y, p1.z));
		mesh.addVertex(ofVec3f(p2.x, p2.y, p2.z));
		mesh.addVertex(ofVec3f(p3.x, p3.y, p3.z));
	}
	
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	
	return mesh;
}
	
ofMesh organizedFastMesh(const ofPixels& colorImage, const ofShortPixels& depthImage, const int skip, float scale)
{
	PointXYZRGBCloud temp = New<PointXYZRGBCloud>();
	
	convert(colorImage, depthImage, temp, skip, scale);
	
	pcl::OrganizedFastMesh<PointXYZRGB> ofm;
	ofm.setTrianglePixelSize(1);
	ofm.setTriangulationType(pcl::OrganizedFastMesh<PointXYZRGB>::TRIANGLE_RIGHT_CUT);
	ofm.setInputCloud(temp);
	
	boost::shared_ptr<std::vector<pcl::Vertices> > verts(new std::vector<pcl::Vertices>);
	ofm.reconstruct(*verts);
	
	NormalCloud normals = New<NormalCloud>();
	integralImageNormalEstimation(temp, normals);
		   
	ofMesh mesh;
	
	for (int i = 0; i < verts->size(); i++)
	{
		const pcl::Vertices &v = verts->at(i);
		
		if (v.vertices.size() != 3) continue;

		const Normal &n1 = normals->points[v.vertices[0]];
		const Normal &n2 = normals->points[v.vertices[1]];
		const Normal &n3 = normals->points[v.vertices[2]];
		
//		if (!isnormal(n1.normal_x)
//			|| !isnormal(n1.normal_y)
//			|| !isnormal(n1.normal_z)) continue;

		const PointXYZRGB &p1 = temp->points[v.vertices[0]];
		const PointXYZRGB &p2 = temp->points[v.vertices[1]];
		const PointXYZRGB &p3 = temp->points[v.vertices[2]];

		mesh.addColor(ofColor(p1.r, p1.g, p1.b));
		mesh.addColor(ofColor(p2.r, p2.g, p2.b));
		mesh.addColor(ofColor(p3.r, p3.g, p3.b));
		
		mesh.addNormal(ofVec3f(n1.normal_x, n1.normal_y, n1.normal_z));
		mesh.addNormal(ofVec3f(n2.normal_x, n2.normal_y, n2.normal_z));
		mesh.addNormal(ofVec3f(n3.normal_x, n3.normal_y, n3.normal_z));
		
		mesh.addVertex(ofVec3f(p1.x, p1.y, p1.z));
		mesh.addVertex(ofVec3f(p2.x, p2.y, p2.z));
		mesh.addVertex(ofVec3f(p3.x, p3.y, p3.z));
	}
	
	mesh.setMode(OF_PRIMITIVE_TRIANGLES);
	
	return mesh;
}
	
}