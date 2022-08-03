/*
  ==============================================================================

	EuclideanSegmentationNode.cpp
	Created: 5 Apr 2022 10:45:21am
	Author:  bkupe

  ==============================================================================
*/

EuclideanClusterNode::EuclideanClusterNode(var params) :
	Node(getTypeString(), FILTER, params)
{
	in = addSlot("In", true, POINTCLOUD);
	inHighres = addSlot("In High Resolution", true, POINTCLOUD);
	out = addSlot("Out", false, CLUSTERS);

	tolerance = addFloatParameter("Tolerance", "The neighbour distance to tolerate when searching neighbours for clustering. In meters", .02f, .001f);
	minCount = addIntParameter("Min Count", "The minimum amount of points that a cluster can have", 100);
	maxCount = addIntParameter("Max count", "The maximum amount of points that a cluster can have", 25000);
	minSize = addPoint3DParameter("Min Size", "The minimum size for a cluster, in meters");
	maxSize = addPoint3DParameter("Max Size", "The maximum size for a cluster, in meters");
	var val;
	val.append(5);
	val.append(5);
	val.append(5);
	maxSize->setDefaultValue(val);

	computeBox = addBoolParameter("Compute Box", "Compte infos for each cluster", true);
}

EuclideanClusterNode::~EuclideanClusterNode()
{
}


void EuclideanClusterNode::processInternal()
{
	CloudPtr source = slotCloudMap[in];
	CloudPtr sourceHires = slotCloudMap[inHighres];

	if (source->empty()) return;
	
	CloudPtr cloud(new Cloud());
	pcl::copyPointCloud(*source, *cloud);

	CloudPtr hiResCloud = nullptr;

	if (sourceHires != nullptr && !sourceHires->empty())
	{
		hiResCloud.reset(new Cloud());
		pcl::copyPointCloud(*sourceHires, *hiResCloud);
	}

	NNLOG("Start extract, num input points : " << (int)cloud->size());

	pcl::search::KdTree<PPoint>::Ptr tree(new pcl::search::KdTree<PPoint>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance->floatValue());
	ec.setMinClusterSize(minCount->intValue());
	ec.setMaxClusterSize(maxCount->intValue());
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud);

	try
	{
		ec.extract(clusterIndices);
	}
	catch (...)
	{
		NLOGERROR(niceName, "Error trying to extract clusters");
		return;
	}

	NNLOG("Extracted, num clusters : " << (int)clusterIndices.size());

	if (out->isEmpty()) return;

	bool compute = computeBox->boolValue();

	Array<ClusterPtr> clusters;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
	{
		CloudPtr c(new Cloud());

		Vector3D<float> minP(INT32_MAX, INT32_MAX, INT32_MAX);
		Vector3D<float> maxP(INT32_MIN, INT32_MIN, INT32_MIN);
		Vector3D<float> average(0, 0, 0);

		for (const auto& idx : it->indices)
		{

			PPoint p = (*cloud)[idx];

			if (compute)
			{
				minP.x = jmin(p.x, minP.x);
				minP.y = jmin(p.y, minP.y);
				minP.z = jmin(p.z, minP.z);

				maxP.x = jmax(p.x, maxP.x);
				maxP.y = jmax(p.y, maxP.y);
				maxP.z = jmax(p.z, maxP.z);

				average += Vector3D(p.x, p.y, p.z);
			}

			c->push_back(p); //*
		}

		c->width = c->size();
		c->height = 1;
		c->is_dense = true;


		Vector3D<float> boundsMin = Vector3D<float>(minP.x, minP.y, minP.z);;
		Vector3D<float> boundsMax = Vector3D<float>(maxP.x, maxP.y, maxP.z);;

		Vector3D<float> clusterSize = boundsMax - boundsMin;

		if (clusterSize.x < minSize->x || clusterSize.y < minSize->y || clusterSize.z < minSize->z
			|| clusterSize.x > maxSize->x || clusterSize.y > maxSize->y || clusterSize.z > maxSize->z) continue;
		

	

		CloudPtr cc = nullptr;

		if (hiResCloud != nullptr)
		{
			cc.reset(new Cloud());

			pcl::CropBox<pcl::PointXYZ> filter;
			filter.setInputCloud(hiResCloud);
			filter.setMin(Eigen::Vector4f(boundsMin.x, boundsMin.y, boundsMin.z, 1));
			filter.setMax(Eigen::Vector4f(boundsMax.x, boundsMax.y, boundsMax.z, 1));
			filter.filter(*cc);
		}
		else
		{
			cc = c;
		}

		ClusterPtr pc(new Cluster(clusters.size(), cc));
		if (compute)
		{
			average /= it->indices.size();
			pc->boundingBoxMin = boundsMin;
			pc->boundingBoxMax = boundsMax;

			pc->centroid = Vector3D<float>(average.x, average.y, average.z);;
		}

		clusters.add(pc);
	}

	sendClusters(out, clusters);
}

void EuclideanClusterNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void EuclideanClusterNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
}
