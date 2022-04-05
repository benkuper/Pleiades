/*
  ==============================================================================

	EuclideanSegmentationNode.cpp
	Created: 5 Apr 2022 10:45:21am
	Author:  bkupe

  ==============================================================================
*/

EuclideanClusterNode::EuclideanClusterNode(var params) :
	Node(getTypeString(), OUTPUT, params)
{
	in = addSlot("In", true, POINTCLOUD);
	out = addSlot("Out", false, CLUSTERS);

	tolerance = addFloatParameter("Tolerance", "The neighbour distance to tolerate when searching neighbours for clustering", .02f, .001f);
	minSize = addIntParameter("Min Size", "The minimum amount of points that a cluster can have", 100);
	maxSize = addIntParameter("Max Size", "The maximum amount of points that a cluster can have", 25000);
}

EuclideanClusterNode::~EuclideanClusterNode()
{
}


void EuclideanClusterNode::processInternal()
{
	PCloud source = slotCloudMap[in];
	if (source.cloud->empty()) return;

	NNLOG("Start extract, num input points : " << (int)source.cloud->size());

	pcl::search::KdTree<PPoint>::Ptr tree(new pcl::search::KdTree<PPoint>);
	tree->setInputCloud(source.cloud);

	std::vector<pcl::PointIndices> clusterIndices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(tolerance->floatValue()); // 2cm
	ec.setMinClusterSize(minSize->intValue());
	ec.setMaxClusterSize(maxSize->intValue());
	ec.setSearchMethod(tree);
	ec.setInputCloud(source.cloud);
	ec.extract(clusterIndices);

	NNLOG("Extracted, num clusters : " << clusterIndices.size());

	if (out->isEmpty()) return;

	Array<PCloud> clusters;
	for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin(); it != clusterIndices.end(); ++it)
	{
		CloudPtr c(new Cloud());
		for (const auto& idx : it->indices) c->push_back((*source.cloud)[idx]); //*

		c->width = c->size();
		c->height = 1;
		c->is_dense = true;
		clusters.add({ clusters.size(), c });
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
