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
	in = addSlot("In", true, POINTCLOUD);
	out = addSlot("Out", false, POINTCLOUD);

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

	PCloud source = slotCloudMap[in];
	if (source.cloud->empty()) return;

	Vector3D minV = minPoint->getVector();
	Vector3D maxV = maxPoint->getVector();
	CloudPtr cloud(new Cloud(source.cloud->width, source.cloud->height));

	CloudPtr sCloud = source.cloud;

	NNLOG("Cloud in size : " << source.cloud->size());
	if (cleanUp->boolValue())
	{

		sCloud = CloudPtr(new Cloud(source.cloud->width, source.cloud->height));
		pcl::Indices indices;
		pcl::removeNaNFromPointCloud(*source.cloud, *sCloud, indices);
		NNLOG("Clean cloud size : " << sCloud->size());
	}

	pcl::CropBox<pcl::PointXYZ> filter;
	filter.setInputCloud(sCloud);
	filter.setKeepOrganized(keepOrganized->boolValue());
	filter.setMin(Eigen::Vector4f(minV.x, minV.y, minV.z, 1));
	filter.setMax(Eigen::Vector4f(maxV.x, maxV.y, maxV.z, 1));
	filter.filter(*cloud);

	NNLOG("After crop : " << cloud->size());
	sendPointCloud(out, { source.id , cloud });
}

void CropBoxNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void CropBoxNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
}
