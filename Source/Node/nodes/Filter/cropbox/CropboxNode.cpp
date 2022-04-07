/*
  ==============================================================================

	CropboxNode.cpp
	Created: 5 Apr 2022 10:45:43am
	Author:  bkupe

  ==============================================================================
*/

CropBoxNode::CropBoxNode(var params) :
	Node(getTypeString(), OUTPUT, params)
{
	addInOutSlot(&in, &out, POINTCLOUD);

	minPoint = addPoint3DParameter("Min", "Min Point");
	minPoint->setVector(-1, -1, -1);

	maxPoint = addPoint3DParameter("Max", "Max Point");
	maxPoint->setVector(1, 1, 1);

	keepOrganized = addBoolParameter("Keep Organized", "If checked, this will keep the 2D structure of the input cloud", false);
	cleanUp = addBoolParameter("Clean up", "If checked, this will remove any bad points", false);
}

CropBoxNode::~CropBoxNode()
{
}


void CropBoxNode::processInternal()
{

	CloudPtr source = slotCloudMap[in];
	if (source->empty()) return;

	Vector3D minV = minPoint->getVector();
	Vector3D maxV = maxPoint->getVector();
	CloudPtr cloud(new Cloud(source->width, source->height));

	CloudPtr sCloud = source;

	NNLOG("Cloud in size : " << source->size());
	if (cleanUp->boolValue())
	{

		sCloud = CloudPtr(new Cloud(source->width, source->height));
		pcl::Indices indices;
		pcl::removeNaNFromPointCloud(*source, *sCloud, indices);
		NNLOG("Clean cloud size : " << sCloud->size());
	}

	pcl::CropBox<pcl::PointXYZ> filter;
	filter.setInputCloud(sCloud);
	filter.setKeepOrganized(keepOrganized->boolValue());
	filter.setMin(Eigen::Vector4f(minV.x, minV.y, minV.z, 1));
	filter.setMax(Eigen::Vector4f(maxV.x, maxV.y, maxV.z, 1));
	filter.filter(*cloud);

	NNLOG("After crop : " << cloud->size());
	sendPointCloud(out, cloud);
}

void CropBoxNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void CropBoxNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
}
