/*
  ==============================================================================

	TransformNode.cpp
	Created: 4 May 2022 5:56:35pm
	Author:  bkupe

  ==============================================================================
*/

TransformNode::TransformNode(var params) :
	Node(getTypeString(), FILTER, params)
{
	addInOutSlot(&in, &out, POINTCLOUD, "In", "Transformed");

	translate = addPoint3DParameter("Translate", "Translate the cloud");
	rotate = addPoint3DParameter("Rotate", "Rotate the cloud");
}

TransformNode::~TransformNode()
{
}


void TransformNode::processInternal()
{
	CloudPtr source = slotCloudMap[in];
	if (source == nullptr || source->empty()) return;

	if (!out->isEmpty())
	{
		CloudPtr transformedCloud(new Cloud(source->width, source->height));

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();

		Vector3D<float> trans = translate->getVector();

		Vector3D<float> rot = rotate->getVector();
		Eigen::Quaternionf rotQuat =pleiades::euler2Quaternion(rot.z, rot.x, rot.y);
		
		transform.translate(Eigen::Vector3f(trans.x, trans.y, trans.z));
		transform.rotate(rotQuat);

		pcl::transformPointCloud(*source, *transformedCloud, transform);
		sendPointCloud(out, transformedCloud);
	}
}