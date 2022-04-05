/*
  ==============================================================================

	PlaneSegmentationNode.cpp
	Created: 5 Apr 2022 10:45:50am
	Author:  bkupe

  ==============================================================================
*/


PlaneSegmentationNode::PlaneSegmentationNode(var params) :
	Node(getTypeString(), OUTPUT, params),
	findOnNextProcess(false)
{
	in = addSlot("Cloud In", true, POINTCLOUD);
	out = addSlot("Transformed", false, POINTCLOUD);
	planeCloud = addSlot("Segmented", false, POINTCLOUD);
	planeCenterSlot = addSlot("Plane Center", false, VECTOR);
	planeNormalSlot = addSlot("Plane Normal", false, VECTOR);

	continuous = addBoolParameter("Continuous Search", "If checked, search always for the plane. Otherwise, it will only search when triggering", false);
	findPlane = addTrigger("Find Plane", "Find the plane. Now.");

	downSample = addIntParameter("Down Sample", "Down sample for the segmentation. The transformed cloud keep the source resolution", 1, 1, 16);
	distanceThreshold = addFloatParameter("Distance Threshold", "Distance Threshold", .01, 0);
	invertPlane = addBoolParameter("Invert Plane", "If checked, send the cloud without the plane points.If not, sends only the plane points", false);
	;
}

PlaneSegmentationNode::~PlaneSegmentationNode()
{
}


void PlaneSegmentationNode::processInternal()
{
	PCloud source = slotCloudMap[in];
	if (source.cloud->empty()) return;

	//jassert(source.cloud->isOrganized());

	int ds = downSample->intValue();



	CloudPtr cloud(new Cloud());
	if (ds == 1) pcl::copyPointCloud(*source.cloud, *cloud);
	else
	{
		cloud.reset(new Cloud(ceil(source.cloud->width * 1.0f / ds), ceil(source.cloud->height * 1.0f / ds)));

		for (int ty = 0; ty < (int)source.cloud->height; ty += ds)
		{
			for (int tx = 0; tx < (int)source.cloud->width; tx += ds)
			{
				cloud->at(floor(tx / ds), floor(ty / ds)) = source.cloud->at(tx, ty);
			}
		}
	}


	if (continuous->boolValue() || findOnNextProcess)
	{
		reproj = Eigen::Quaternionf::Identity();

		NNLOG("Finding plane..");
		long millis = Time::getMillisecondCounter();

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

		pcl::SACSegmentation<PPoint> seg;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.01);

		seg.setInputCloud(cloud);

		seg.segment(*inliers, *coefficients);

		if (inliers->indices.size() == 0)
		{
			LOGERROR("Could not estimate a planar model for the given dataset.\n");
			return;
		}

		planeNormal.x() = coefficients->values[0];
		planeNormal.y() = coefficients->values[1];
		planeNormal.z() = coefficients->values[2];

		planeCenter.setZero();
		for (auto& i : inliers->indices)
		{
			planeCenter += Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		}
		planeCenter /= inliers->indices.size();

		int diff = Time::getMillisecondCounter() - millis;
		NNLOG("Found plane with " << inliers->indices.size() << " points in " << diff << "ms.\nPlane center " << planeCenter.x() << ", " << planeCenter.y() << ", " << planeCenter.z() << ".Plane Normal " << planeNormal.x() << ", " << planeNormal.y() << ", " << planeNormal.z());

		reproj = Eigen::Quaternionf::FromTwoVectors(planeNormal, Eigen::Vector3f(0, -1, 0));


		if (!planeCloud->isEmpty())
		{
			pcl::ExtractIndices<PPoint> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			if (invertPlane->boolValue()) extract.setNegative(true);
			extract.filterDirectly(cloud);
			sendPointCloud(planeCloud, { source.id, cloud });
		}

		findOnNextProcess = false;
	}

	if (!out->isEmpty())
	{
		CloudPtr transformedCloud(new Cloud(source.cloud->width, source.cloud->height));

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.rotate(reproj);
		transform.translate(-planeCenter);
		pcl::transformPointCloud(*source.cloud, *transformedCloud, transform);
		sendPointCloud(out, { 0, transformedCloud });
	}


	sendVector(planeCenterSlot, planeCenter);
	sendVector(planeNormalSlot, planeNormal);
}

void PlaneSegmentationNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void PlaneSegmentationNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
	if (t == findPlane) findOnNextProcess = true;
}
