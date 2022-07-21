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
	out = addSlot("Out", false, CLUSTERS);

	tolerance = addFloatParameter("Tolerance", "The neighbour distance to tolerate when searching neighbours for clustering. In meters", .02f, .001f);
	minSize = addIntParameter("Min Size", "The minimum amount of points that a cluster can have", 100);
	maxSize = addIntParameter("Max Size", "The maximum amount of points that a cluster can have", 25000);

	computeBox = addBoolParameter("Compute Box", "Compte infos for each cluster", false);
}

EuclideanClusterNode::~EuclideanClusterNode()
{
}


void EuclideanClusterNode::processInternal()
{
	CloudPtr source = slotCloudMap[in];
	if (source->empty()) return;

	NNLOG("Start extract, num input points : " << (int)source->size());

	pcl::search::KdTree<PPoint>::Ptr tree(new pcl::search::KdTree<PPoint>);
	tree->setInputCloud(source);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance->floatValue());
	ec.setMinClusterSize(minSize->intValue());
	ec.setMaxClusterSize(maxSize->intValue());
	ec.setSearchMethod(tree);
	ec.setInputCloud(source);
	ec.extract(clusterIndices);

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

			PPoint p = (*source)[idx];

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

		ClusterPtr pc(new Cluster(clusters.size(), c));

		if (compute)
		{
			average /= it->indices.size();
			pc->boundingBoxMin = Vector3D<float>(minP.x, minP.y, minP.z);
			pc->boundingBoxMax = Vector3D<float>(maxP.x, maxP.y, maxP.z);
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
