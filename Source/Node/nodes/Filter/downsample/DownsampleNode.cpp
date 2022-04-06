/*
  ==============================================================================

	DownsampleNode.cpp
	Created: 5 Apr 2022 10:46:00am
	Author:  bkupe

  ==============================================================================
*/

DownsampleNode::DownsampleNode(var params) :
	Node(getTypeString(), OUTPUT, params)
{
	in = addSlot("In", true, POINTCLOUD);
	out = addSlot("Out", false, POINTCLOUD);

	leafSize = addPoint3DParameter("Leaf size", "Size of voxels to use for downsampling.");
	leafSize->setVector(.01f, .01f, .01f);
}

DownsampleNode::~DownsampleNode()
{
}


void DownsampleNode::processInternal()
{
	CloudPtr source = slotCloudMap[in];
	if (source->empty()) return;

	NNLOG("Start downsample, num input points : " << (int)source->size());

	Vector3D ls = leafSize->getVector();

	CloudPtr cloud(new Cloud());

	pcl::VoxelGrid<PPoint> sor;
	sor.setInputCloud(source);
	sor.setLeafSize(ls.x, ls.y, ls.z);
	sor.filter(*cloud);

	NNLOG("After downsample, num clusters : " << cloud->size());

	sendPointCloud(out, cloud );
}