#include "ofxPCL.h"
#include "pcl/impl/instantiate.hpp"

using namespace pcl;

namespace ofxPCL
{

template <typename T>
vector<T> segmentation(T &cloud, pcl::SacModel model_type, float distance_threshold, int min_points_limit, int max_segment_count, bool makeCloudResidual)
{
	assert(cloud);
	
	vector<T> result;
	if (cloud->points.empty()) return result;
	
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
	
	pcl::SACSegmentation<typename T::element_type::PointType> seg;
	seg.setOptimizeCoefficients(false);
	
	seg.setModelType(model_type);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distance_threshold);
	seg.setMaxIterations(500);
	
	T temp(new typename T::element_type(*cloud));
	const size_t original_szie = temp->points.size();
	
	pcl::ExtractIndices<typename T::element_type::PointType> extract;
	
	int segment_count = 0;
	while (temp->size() > original_szie * 0.3)
	{
		if (segment_count > max_segment_count) break;
		segment_count++;
		
		seg.setInputCloud(temp);
		seg.segment(*inliers, *coefficients);
		
		if (inliers->indices.size() < min_points_limit)
			break;
		
		T filterd_point_cloud(new typename T::element_type);
		
		extract.setInputCloud(temp);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*filterd_point_cloud);
		
		if (filterd_point_cloud->points.size() > 0)
		{
			result.push_back(filterd_point_cloud);
		}
		
		extract.setNegative(true);
		extract.filter(*temp);
	}
	
	if( makeCloudResidual ) {
		cloud = temp;
	}
	
	return result;
}

#define PCL_INSTANTIATE_segmentation(T) \
	template vector<pcl::PointCloud<T>::Ptr> segmentation(pcl::PointCloud<T>::Ptr&, pcl::SacModel, float, int, int, bool);
PCL_INSTANTIATE(segmentation, PCL_XYZ_POINT_TYPES);

//
// cluster extraction
//
template <typename T>
vector<T> clusterExtraction(T &cloud, float distance_threshold, int min_points_limit, int max_segment_count)
{
	assert(cloud);
	
	vector<T> result;
	if (cloud->points.empty()) return result;
	
	// Creating the KdTree object for the search method of the extraction
	KdTree<typename T::element_type::PointType> tree(cloud);
	
	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<typename T::element_type::PointType> ec;
	ec.setClusterTolerance (distance_threshold);
	ec.setMinClusterSize (min_points_limit);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree.kdtree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);
	
	int segment_count = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		if( segment_count >= max_segment_count ) break;
		
		T cloud_cluster (new typename T::element_type);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
			cloud_cluster->points.push_back (cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		
		result.push_back(cloud_cluster);
		
		segment_count++;
	}
	return result;
}

#define PCL_INSTANTIATE_clusterExtraction(T) \
	template vector<pcl::PointCloud<T>::Ptr> clusterExtraction(pcl::PointCloud<T>::Ptr&, float, int, int);
PCL_INSTANTIATE(clusterExtraction, PCL_XYZ_POINT_TYPES);

template <typename T>
ofMesh triangulate(const T &cloud_with_normals, float search_radius, float mu, float maxNearNeighbors, float maxSurfaceAngle, float minAngle, float maxAngle, bool normalConsistency, bool consistentVertexOrdering)
{
	assert(cloud_with_normals);
	
	ofMesh mesh;
	
	if (cloud_with_normals->points.empty()) return mesh;
	
	KdTree<typename T::element_type::PointType> kdtree(cloud_with_normals);
	
	typename pcl::GreedyProjectionTriangulation<typename T::element_type::PointType> gp3;
	pcl::PolygonMesh triangles;
	
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(search_radius);
	
	gp3.setMu(mu);
	gp3.setMaximumNearestNeighbors(maxNearNeighbors);
	gp3.setMaximumSurfaceAngle(ofDegToRad(maxSurfaceAngle));
	gp3.setMinimumAngle(ofDegToRad(minAngle));
	gp3.setMaximumAngle(ofDegToRad(maxAngle));
	gp3.setNormalConsistency(normalConsistency);
	gp3.setConsistentVertexOrdering(consistentVertexOrdering);
	
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

#define PCL_INSTANTIATE_triangulate(T) \
	template ofMesh triangulate<pcl::PointCloud<T>::Ptr>(const pcl::PointCloud<T>::Ptr&, float, float, float, float, float, float, bool, bool);
PCL_INSTANTIATE(triangulate, (pcl::PointNormal) \
	(pcl::PointXYZRGBNormal) \
	(pcl::PointXYZINormal));

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

template <typename T>
void integralImageNormalEstimation(const T& cloud, NormalCloud& normals)
{
	assert(cloud->isOrganized());
	
	if (!normals)
		normals = New<NormalCloud>();
	
	typedef typename T::element_type::PointType PointType;
	
	pcl::IntegralImageNormalEstimation<PointType, Normal> ne;
	
	ne.setNormalEstimationMethod(pcl::IntegralImageNormalEstimation<PointType, pcl::Normal>::AVERAGE_3D_GRADIENT);
	
	ne.setMaxDepthChangeFactor(10.0f);
	ne.setNormalSmoothingSize(4.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
}

#define PCL_INSTANTIATE_integralImageNormalEstimation(T) \
	template void integralImageNormalEstimation<pcl::PointCloud<T>::Ptr>(const pcl::PointCloud<T>::Ptr&, NormalCloud&);
PCL_INSTANTIATE(integralImageNormalEstimation, PCL_XYZ_POINT_TYPES);
}
