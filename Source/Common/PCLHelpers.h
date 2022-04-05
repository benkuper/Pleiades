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
#pragma warning(pop)

typedef pcl::PointXYZ PPoint;
typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef pcl::PointIndices PIndices;


struct PCloud
{
	int id = 0;
	CloudPtr cloud;
};