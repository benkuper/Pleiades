/*
  ==============================================================================

	VoxelGridNode.cpp
	Created: 5 Apr 2022 10:46:00am
	Author:  bkupe

  ==============================================================================
*/

VoxelGridNode::VoxelGridNode(var params) :
	Node(getTypeString(), OUTPUT, params)
{
	addInOutSlot(&in, &out, POINTCLOUD);

	leafSize = addPoint3DParameter("Leaf size", "Size of voxels to use for downsampling.");
	leafSize->setVector(.01f, .01f, .01f);
}

VoxelGridNode::~VoxelGridNode()
{
}


void VoxelGridNode::processInternal()
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

	NNLOG("After downsample, num clusters : " << (int)cloud->size());

	sendPointCloud(out, cloud );
}
