/*
  ==============================================================================

	PCLHelpers.h
	Created: 5 Apr 2022 1:57:51pm
	Author:  bkupe

  ==============================================================================
*/

#pragma once

#pragma warning(push)
#pragma warning(disable : 4324 4201 4996 4189 4127 4018 4005)
#include <pcl/pcl_base.h>
#include <pcl/octree/octree_key.h> //include here to avoid warning when included from other pcl file 
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#pragma warning(pop)

typedef pcl::PointXYZ PPoint;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointIndices PIndices;

class Cluster
{
public:
	Cluster(int id, CloudPtr cloud) :
		id(id),
		cloud(cloud)
	{
		state = ENTERED;
		lastUpdateTime = Time::getMillisecondCounterHiRes() / 1000.0;
	}

	int id = 0;
	CloudPtr cloud;
	
	int frame = 0;
	float ghostAge = 0;
	float age = 0;

	double lastUpdateTime = 0;
	
	Vector3D<float> boundingBoxMin = { 0, 0, 0 };
	Vector3D<float> boundingBoxMax = { 0, 0, 0 };
	Vector3D<float> centroid = { 0, 0, 0 };
	Vector3D<float> velocity = { 0, 0, 0 };

	//Old data for processsing
	Vector3D<float> oldBoundingBoxMin = { 0, 0, 0 };
	Vector3D<float> oldBoundingBoxMax = { 0, 0, 0 };
	Vector3D<float> oldCentroid = { 0, 0, 0 };
	Vector3D<float> oldVelocity = { 0, 0, 0 };

	enum State { ENTERED, UPDATED, WILL_LEAVE, GHOST };
	State state;

	void update(std::shared_ptr<Cluster> newData)
	{
		cloud = newData->cloud;

		oldBoundingBoxMin = Vector3D<float>(boundingBoxMin);
		oldBoundingBoxMax = Vector3D<float>(boundingBoxMax);
		oldCentroid = Vector3D<float>(centroid);
		oldVelocity = Vector3D<float>(velocity);
		boundingBoxMin = newData->boundingBoxMin;
		boundingBoxMax = newData->boundingBoxMax;
		centroid = newData->centroid;
		velocity = newData->velocity;

		double curT = Time::getMillisecondCounterHiRes() / 1000.0;
		double delta = curT - lastUpdateTime;
		age += delta;
		
		lastUpdateTime = curT;

		state = UPDATED;
	}
};

typedef std::shared_ptr<Cluster> ClusterPtr;