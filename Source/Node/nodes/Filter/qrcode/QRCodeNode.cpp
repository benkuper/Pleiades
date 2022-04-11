/*
  ==============================================================================

	QRCodeNode.cpp
	Created: 12 Apr 2022 12:46:58am
	Author:  bkupe

  ==============================================================================
*/

QRCodeNode::QRCodeNode(var params) :
	Node(getTypeString(), OUTPUT, params),
	findOnNextProcess(false)
{
	inColor = addSlot("Color", true, NodeConnectionType::RGB);
	addInOutSlot(&inDepth, &out, POINTCLOUD, "In", "Transformed");

	planeCenterSlot = addSlot("Plane Center", false, VECTOR);
	planeNormalSlot = addSlot("Plane Normal", false, VECTOR);

	continuous = addBoolParameter("Continuous Search", "If checked, search always for the plane. Otherwise, it will only search when triggering", false);
	findPlane = addTrigger("Find Plane", "Find the plane. Now.");
}

QRCodeNode::~QRCodeNode()
{
}


void QRCodeNode::processInternal()
{
	CloudPtr source = slotCloudMap[inDepth];
	ImagePtr img = slotImageMap[inColor];

	if (source->empty() || img->isNull()) return;

	if (continuous->boolValue() || findOnNextProcess)
	{
		reproj = Eigen::Quaternionf::Identity();

		NNLOG("Finding plane..");

		//Here find plane using OpenCV and QRCode, set planeCenter, planeNormal

		//NNLOG("Found plane with " << (int)inliers->indices.size() << " points in " << diff << "ms.\nPlane center " << planeCenter.x() << ", " << planeCenter.y() << ", " << planeCenter.z() << ".Plane Normal " << planeNormal.x() << ", " << planeNormal.y() << ", " << planeNormal.z());

		cv::Point3d a(0, 0, 0);

		reproj = Eigen::Quaternionf::FromTwoVectors(planeNormal, Eigen::Vector3f(0, -1, 0));


		findOnNextProcess = false;
	}

	if (!out->isEmpty())
	{
		CloudPtr transformedCloud(new Cloud(source->width, source->height));

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(reproj);
		transform.translate(-planeCenter);
		pcl::transformPointCloud(*source, *transformedCloud, transform);
		sendPointCloud(out, transformedCloud);
	}


	sendVector(planeCenterSlot, planeCenter);
	sendVector(planeNormalSlot, planeNormal);
}

var QRCodeNode::getJSONData()
{
	var data = Node::getJSONData();
	var planeData;
	planeData.append(planeCenter.x());
	planeData.append(planeCenter.y());
	planeData.append(planeCenter.z());

	planeData.append(planeNormal.x());
	planeData.append(planeNormal.y());
	planeData.append(planeNormal.z());

	planeData.append(reproj.w());
	planeData.append(reproj.x());
	planeData.append(reproj.y());
	planeData.append(reproj.z());

	data.getDynamicObject()->setProperty("planeData", planeData);
	return data;
}

void QRCodeNode::loadJSONDataItemInternal(var data)
{
	Node::loadJSONDataItemInternal(data);
	var planeData = data.getProperty("planeData", var());
	if (planeData.size() >= 10)
	{
		planeCenter = Eigen::Vector3f(planeData[0], planeData[1], planeData[2]);
		planeNormal = Eigen::Vector3f(planeData[3], planeData[4], planeData[5]);
		reproj = Eigen::Quaternionf(planeData[6], planeData[7], planeData[8], planeData[9]);
	}
}

void QRCodeNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void QRCodeNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
	if (t == findPlane) findOnNextProcess = true;
}
