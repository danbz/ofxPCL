#pragma once

#include "ofMain.h"

#ifdef nil
#undef nil
#endif

#include "Types.h"
#include "Utility.h"
#include "Tree.h"

// file io
#include <pcl/io/pcd_io.h>

// transform
#include <pcl/common/transforms.h>

// thresold
#include <pcl/filters/passthrough.h>

// outlier removal
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// segmentation
#include <pcl/sample_consensus/model_types.h>

// downsample
#include <pcl/filters/voxel_grid.h>

// segmentation
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/filters/extract_indices.h>

// cluster extraction
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>

// triangulate
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/Vertices.h>

// mls
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>

#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/integral_image_normal.h>

namespace ofxPCL
{

//
// file io
//
template <typename T>
inline T loadPointCloud(string path)
{
	T cloud(new typename T::element_type);
	path = ofToDataPath(path);

	if (pcl::io::loadPCDFile<typename T::element_type::PointType>(path.c_str(), *cloud) == -1)
		ofLogError("ofxPCL:loadPointCloud") << "file not found: " << path;

	return cloud;
}

template <typename T>
inline void savePointCloud(string path, T cloud)
{
	assert(cloud);
	
	if (cloud->points.empty()) return;

	path = ofToDataPath(path);
	pcl::io::savePCDFileBinary(path.c_str(), *cloud);
}

//
// transform
//
template <typename T>
void transform(T cloud, ofMatrix4x4 matrix)
{
	assert(cloud);
	
	if (cloud->points.empty()) return;

	Eigen::Matrix4f mat;
	memcpy(&mat, matrix.getPtr(), sizeof(float) * 16);
	pcl::transformPointCloud(*cloud, *cloud, mat);
}

//
// threshold
//
template <typename T>
inline void threshold(T cloud, const char *dimension = "X", float min = 0, float max = 100)
{
	assert(cloud);
	
	if (cloud->points.empty()) return;

	pcl::PassThrough<typename T::element_type::PointType> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(dimension);
	pass.setFilterLimits(min, max);
	pass.filter(*cloud);
}

//
// downsample
//
template <typename T>
inline void downsample(T cloud, ofVec3f resolution = ofVec3f(1, 1, 1))
{
	assert(cloud);
	
	if (cloud->points.empty()) return;

	pcl::VoxelGrid<typename T::element_type::PointType> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(resolution.x, resolution.y, resolution.z);
	sor.filter(*cloud);
}

//
// outlier removal
//
template <typename T>
inline void statisticalOutlierRemoval(T cloud, int nr_k = 50, double std_mul = 1.0)
{
	assert(cloud);
	
	if (cloud->points.empty()) return;

	pcl::StatisticalOutlierRemoval<typename T::element_type::PointType> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(nr_k);
	sor.setStddevMulThresh(std_mul);
	sor.filter(*cloud);
}

template <typename T>
inline void radiusOutlierRemoval(T cloud, double radius, int num_min_points)
{
	assert(cloud);
	
	if (cloud->points.empty()) return;

	pcl::RadiusOutlierRemoval<typename T::element_type::PointType> outrem;
	outrem.setInputCloud(cloud);
	outrem.setRadiusSearch(radius);
	outrem.setMinNeighborsInRadius(num_min_points);
	outrem.filter(*cloud);
}

//
// segmentation
//
template <typename T>
vector<T> segmentation(T& cloud, const pcl::SacModel model_type = pcl::SACMODEL_PLANE, const float distance_threshold = 1, const int min_points_limit = 10, const int max_segment_count = 30);

//
// cluster extraction
//
template <typename T>
vector<T> clusterExtraction(T& cloud, const pcl::SacModel model_type = pcl::SACMODEL_PLANE, const float distance_threshold = 1, const int min_points_limit = 10, const int max_segment_count = 30);

//
// normal estimation
//
template <typename T1, typename T2>
inline void normalEstimation(const T1 &cloud, T2 &output_cloud_with_normals)
{
	if (output_cloud_with_normals == NULL)
		output_cloud_with_normals = New<T2>();

	assert(cloud);
	
	if (cloud->points.empty()) return;

	pcl::NormalEstimation<typename T1::element_type::PointType, Normal> n;
	pcl::PointCloud<pcl::Normal> normals;

	KdTree<typename T1::element_type::PointType> kdtree(cloud);

	n.setInputCloud(cloud);
	n.setSearchMethod(kdtree.kdtree);
	n.setKSearch(20);
	n.compute(normals);
	
	pcl::concatenateFields(*cloud, normals, *output_cloud_with_normals);
}

//
// MLS
//
template <typename T1, typename T2>
void movingLeastSquares(const T1 &cloud, T2 &output_cloud_with_normals, float search_radius = 30)
{
	if (output_cloud_with_normals == NULL)
		output_cloud_with_normals = New<T2>();

	assert(cloud);
	
	if (cloud->points.empty()) return;

	KdTree<typename T1::element_type::PointType> kdtree;
	
	pcl::MovingLeastSquares<
		typename T1::element_type::PointType,
		typename T2::element_type::PointType
	> mls;

	mls.setComputeNormals(true);

	// Set parameters
	mls.setInputCloud(cloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(kdtree.kdtree);
	mls.setSearchRadius(search_radius);

	// Reconstruct
	mls.process(*output_cloud_with_normals);
}

//
// triangulate
//
template <typename T>
ofMesh triangulate(const T &cloud_with_normals, float search_radius = 30);

//
// GridProjection # doesn't work...?
//
template <typename T>
ofMesh gridProjection(const T &cloud_with_normals, float resolution = 1, int padding_size = 3)
{
	ofMesh mesh;

	if (cloud_with_normals->points.empty()) return mesh;

	typename pcl::KdTreeFLANN<typename T::element_type::PointType>::Ptr tree(new pcl::KdTreeFLANN<typename T::element_type::PointType>);
	tree->setInputCloud(cloud_with_normals);

	pcl::GridProjection<typename T::element_type::PointType> gp;
	pcl::PolygonMesh triangles;

	gp.setResolution(resolution);
	gp.setPaddingSize(padding_size);
	gp.setNearestNeighborNum(30);

	// Get result
	gp.setInputCloud(cloud_with_normals);
	gp.setSearchMethod(tree);
	gp.reconstruct(triangles);

	convert(cloud_with_normals, mesh);

	for (int i = 0; i < triangles.polygons.size(); i++)
	{
		pcl::Vertices &v = triangles.polygons[i];

		if (v.vertices.size() == 4)
		{
			mesh.addTriangle(v.vertices[0], v.vertices[1], v.vertices[2]);
			mesh.addTriangle(v.vertices[2], v.vertices[3], v.vertices[0]);
		}
	}

	return mesh;
}

ofMesh organizedFastMesh(const ofShortPixels& depthImage, const int skip = 4, float scale = 0.001);
ofMesh organizedFastMesh(const ofPixels& colorImage, const ofShortPixels& depthImage, const int skip = 4, float scale = 0.001);

template <typename T>
void integralImageNormalEstimation(const T& cloud, NormalCloud& normals);
}
