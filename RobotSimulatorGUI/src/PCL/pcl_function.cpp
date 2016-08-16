#include "stdafx.h"
#include "pcl_function.h"
using namespace std;
using namespace pcl;
using namespace Eigen;

PointCloud<PointXYZ>::Ptr passThrough(PointCloud<PointXYZ>::Ptr cloud, const string& direction, const vector<double>& limits){
	PassThrough<PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName(direction);
	pass.setFilterLimits(limits[0], limits[1]);
	PointCloud<PointXYZ>::Ptr cloudPass(new PointCloud<PointXYZ>);
	pass.filter(*cloudPass);
	return cloudPass;
}

PointCloud<PointXYZ>::Ptr downSampling(PointCloud<PointXYZ>::Ptr cloud, const vector<double>& leafSize){
	PCLPointCloud2::Ptr new_cloud(new PCLPointCloud2);
	PCLPointCloud2::Ptr cloud_down(new PCLPointCloud2);
	PointCloud<PointXYZ>::Ptr cloud_down_otherType(new PointCloud<PointXYZ>);
	
	toPCLPointCloud2(*cloud, *new_cloud);

	// create VoxelGrid filter
	VoxelGrid<PCLPointCloud2> sor;
	sor.setInputCloud(new_cloud);
	sor.setLeafSize(leafSize[0], leafSize[1], leafSize[2]);
	sor.filter(*cloud_down);

	// Convert to the templated PointCloud
	fromPCLPointCloud2(*cloud_down, *cloud_down_otherType);

	return cloud_down_otherType;
}

bool searchPlane(PointCloud<PointXYZ>::Ptr cloud, PointIndices::Ptr inliers, ModelCoefficients::Ptr coefficients, int maxIter, double distanceThreshold){
	// create the segmentation object
	SACSegmentation<PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(maxIter);
	seg.setDistanceThreshold(distanceThreshold);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);

	if (inliers->indices.size() == 0){
		//無法切出平面!!
		return false;
	}
	cout << *coefficients << endl;
	return true;
}

PointCloud<PointXYZ>::Ptr stayObjects(PointCloud<PointXYZ>::Ptr cloud, PointIndices::Ptr inliers, ModelCoefficients::Ptr coefficients){
	// 要去除桌面以下以及桌面的東西
	// first remove plane
	// create the filtering object
	PointCloud<PointXYZ>::Ptr cloud_remove_plane(new PointCloud<PointXYZ>);
	ExtractIndices<PointXYZ> extract;

	// Extract the inliers
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	// 刪除平面 只留平面選false
	extract.setNegative(true);
	extract.filter(*cloud_remove_plane);

	// 去除桌面以下的點
	PointCloud<PointXYZ>::Ptr cloud_objects(new PointCloud<PointXYZ>);
	Vector4f coef;
	coef << coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3];
	for (auto& it : *cloud_remove_plane){
		it.data[3] = 1;
		if (it.getVector4fMap().dot(coef) < 0){
			cloud_objects->push_back(PointXYZ(it.x, it.y, it.z));
		}
	}

	return cloud_objects;
}

void segmentation(PointCloud<PointXYZ>::Ptr cloud,vector<PointIndices>& cluster_indices, double tolerance, int minSize, int maxSize){
	// now segmentation the objects
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	EuclideanClusterExtraction<PointXYZ> ec;
	ec.setClusterTolerance(tolerance); // m
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	cout << "# of object in this cloud: " << cluster_indices.size() << endl;
	for (int i = 0, n = cluster_indices.size(); i < n; ++i){
		PointCloud<PointXYZ> object;
		for (const auto& it : cluster_indices[i].indices){
			object.push_back(cloud->points[it]);
		}
		// save ply object
		ostringstream os;
		os << "object" << i << ".ply";
		io::savePLYFileASCII(os.str(), object);
	}
}

void segmentation(PointCloud<PointXYZ>::Ptr cloud, vector<PointCloud<PointXYZ>::Ptr>& objects, double tolerance, int minSize, int maxSize){
	objects.clear();
	vector<PointIndices> cluster_indices;
	// Creating the KdTree object for the search method of the extraction
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	EuclideanClusterExtraction<PointXYZ> ec;
	ec.setClusterTolerance(tolerance); // m
	ec.setMinClusterSize(minSize);
	ec.setMaxClusterSize(maxSize);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);
	ec.extract(cluster_indices);

	cout << "# of object in this cloud: " << cluster_indices.size() << endl;
	for (int i = 0, n = cluster_indices.size(); i < n; ++i){
		objects.push_back(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>()));
		for (const auto& it : cluster_indices[i].indices){
			objects.back()->push_back(cloud->points[it]);
		}
	}
}

// searchRadius 0.01 (m) as usual
PointCloud<PointXYZ>::Ptr upSampling(PointCloud<PointXYZ>::Ptr cloud, double searchRadius){
	// filtering object
	MovingLeastSquares<PointXYZ, PointXYZ> filter;
	filter.setInputCloud(cloud);
	// object for searching
	search::KdTree<PointXYZ>::Ptr kdTree;
	filter.setSearchMethod(kdTree);
	// Use all neighbor in a radius
	filter.setSearchRadius(searchRadius);
	// Upsampling method. Other possibilites are DISTINCT_CLOUD, RANDOM_UNIFORM_DENSITY
	// and VOXEL_GRID_DILATION. NONE disables upsampling. Check the API for details.
	filter.setUpsamplingMethod(MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
	// Radius around each point, where the local plane will be sampled.
	filter.setUpsamplingRadius(searchRadius);
	// Sampling step size. Bigger values will yield less (if any) new points.
	filter.setUpsamplingStepSize(searchRadius);
	PointCloud<PointXYZ>::Ptr up(new PointCloud<PointXYZ>);
	filter.process(*up);

	return up;
}

PointCloud<Normal>::Ptr normalEstimation(PointCloud<PointXYZ>::Ptr cloud, double searchRadius){
	// Object for storing the normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Object for normal estimation.
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
	normalEstimation.setInputCloud(cloud);
	// For every point, use all neighbors in a radius of 3cm.
	normalEstimation.setRadiusSearch(searchRadius);
	// A kd-tree is a data structure that makes searches efficient. More about it later.
	// The normal estimation object will use it to find nearest neighbors.
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	normalEstimation.setSearchMethod(kdtree);
	// Calculate the normals.
	normalEstimation.compute(*normals);
	
	return normals;
}

PointCloud<PointNormal>::Ptr smoothingNormal(PointCloud<PointXYZ>::Ptr cloud, double searchRadius){
	// 要把質心拉到原點 我的需求
	Vector4f centroid;
	PointCloud<PointXYZ>::Ptr newCloud(new PointCloud<PointXYZ>());
	*newCloud = *cloud;
	compute3DCentroid(*cloud, centroid);
	for (auto& it : *newCloud){
		it.data[0] = it.data[0] - centroid[0];
		it.data[1] = it.data[1] - centroid[1];
		it.data[2] = it.data[2] - centroid[2];
	}
	// Create a KD-Tree
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	// Output has the PointNormal type in order to store the normals calculated by MLS
	pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
	// Init object (second point type is for the normals, even if unused)
	pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
	mls.setComputeNormals(true);
	// Set parameters
	mls.setInputCloud(newCloud);
	mls.setPolynomialFit(true);
	mls.setSearchMethod(tree);
	mls.setSearchRadius(searchRadius);
	// Reconstruct
	mls.process(*mls_points);
	return mls_points;
}

PolygonMesh::Ptr createMesh(PointCloud<PointNormal>::Ptr cloud_with_normals, double searchRadius){
	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud(cloud_with_normals);
	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius(searchRadius);
	// Set typical values for the parameters
	gp3.setMu(2.5);
	gp3.setMaximumNearestNeighbors(100);
	gp3.setMaximumSurfaceAngle(M_PI / 4); // 180 degrees
	gp3.setMinimumAngle(M_PI / 18); // 10 degrees
	gp3.setMaximumAngle(2 * M_PI / 3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud(cloud_with_normals);
	gp3.setSearchMethod(tree2);
	gp3.reconstruct(*triangles);
	return triangles;
}