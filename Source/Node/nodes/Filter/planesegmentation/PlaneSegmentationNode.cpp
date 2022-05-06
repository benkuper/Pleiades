/*
  ==============================================================================

	PlaneSegmentationNode.cpp
	Created: 5 Apr 2022 10:45:50am
	Author:  bkupe

  ==============================================================================
*/


PlaneSegmentationNode::PlaneSegmentationNode(var params) :
	Node(getTypeString(), FILTER, params),
	findOnNextProcess(false)
{
	addInOutSlot(&in, &out, POINTCLOUD, "In", "Transformed");

	inCenter = addSlot("Plane Center In", true, VECTOR);

	planeCloud = addSlot("Segmented", false, POINTCLOUD);
	planeCenterSlot = addSlot("Plane Center", false, VECTOR);
	planeNormalSlot = addSlot("Plane Normal", false, VECTOR);
	outTransform = addSlot("Transform", false, TRANSFORM);

	continuous = addBoolParameter("Continuous Search", "If checked, search always for the plane. Otherwise, it will only search when triggering", false);
	findPlane = addTrigger("Find Plane", "Find the plane. Now.");

	downSample = addIntParameter("Down Sample", "Down sample for the segmentation. The transformed cloud keep the source resolution", 1, 1, 16);
	distanceThreshold = addFloatParameter("Distance Threshold", "Distance Threshold", .01, 0);
	transformPlane = addBoolParameter("Transform Plane", "If checked, applies the transformation that aligns the floor and centers it around 0", true);
	invertDetection = addBoolParameter("Invert Detection", "If checked, send the cloud without the plane points.If not, sends only the plane points", false);
	cleanUp = addBoolParameter("Clean Up", "If checked, this will clean bad points (i.e. points at 0,0,0) before transformation", true);
}

PlaneSegmentationNode::~PlaneSegmentationNode()
{
}


void PlaneSegmentationNode::processInternal()
{
	CloudPtr source = slotCloudMap[in];
	if (source == nullptr || source->empty()) return;

	//jassert(source->isOrganized());

	int ds = downSample->intValue();


	if (cleanUp->boolValue())
	{
		source->erase(std::remove_if(source->points.begin(), source->points.end(), [](PPoint p) { return (p.x == 0 && p.y == 0 && p.z == 0) || std::isinf(p.x) || std::isinf(p.y) || std::isinf(p.z); }), source->points.end());
	}

	CloudPtr cloud(new Cloud());
	if (ds == 1) pcl::copyPointCloud(*source, *cloud);
	else
	{
		if (source->isOrganized())
		{
			cloud.reset(new Cloud(ceil(source->width * 1.0f / ds), ceil(source->height * 1.0f / ds)));

			for (int ty = 0; ty < (int)source->height; ty += ds)
			{
				for (int tx = 0; tx < (int)source->width; tx += ds)
				{
					cloud->at(floor(tx / ds), floor(ty / ds)) = source->at(tx, ty);
				}
			}
		}
		else
		{
			for (int i = 0; i < source->size(); i += ds)
			{
				cloud->push_back(source->points[i]);
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


		if (inCenter->isEmpty())
		{
			planeCenter.setZero();
			for (auto& i : inliers->indices)
			{
				planeCenter += Eigen::Vector3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
			}
			planeCenter /= inliers->indices.size();
		}

		planeNormal.x() = coefficients->values[0];
		planeNormal.y() = coefficients->values[1];
		planeNormal.z() = coefficients->values[2];

		int diff = Time::getMillisecondCounter() - millis;

		NNLOG("Found plane with " << (int)inliers->indices.size() << " points in " << diff << "ms.\nPlane center " << planeCenter.x() << ", " << planeCenter.y() << ", " << planeCenter.z() << ".Plane Normal " << planeNormal.x() << ", " << planeNormal.y() << ", " << planeNormal.z());

		reproj = Eigen::Quaternionf::FromTwoVectors(planeNormal, Eigen::Vector3f(0, -1, 0));


		if (!planeCloud->isEmpty())
		{
			pcl::ExtractIndices<PPoint> extract;
			extract.setInputCloud(cloud);
			extract.setIndices(inliers);
			if (invertDetection->boolValue()) extract.setNegative(true);
			extract.filterDirectly(cloud);
			sendPointCloud(planeCloud, cloud);
		}

		findOnNextProcess = false;
	}

	if (!inCenter->isEmpty()) planeCenter = Eigen::Vector3f(slotVectorMap[inCenter]);


	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.rotate(reproj);
	transform.translate(-planeCenter);

	if (!out->isEmpty())
	{
		if (transformPlane->boolValue())
		{
			CloudPtr transformedCloud(new Cloud(source->width, source->height));
			pcl::transformPointCloud(*source, *transformedCloud, transform);
			sendPointCloud(out, transformedCloud);
		}
		else
		{
			sendPointCloud(out, source);
		}
	}


	sendVector(planeCenterSlot, planeCenter);
	sendVector(planeNormalSlot, planeNormal);
	sendTransform(outTransform, transform);
}

var PlaneSegmentationNode::getJSONData()
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

void PlaneSegmentationNode::loadJSONDataItemInternal(var data)
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

void PlaneSegmentationNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void PlaneSegmentationNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
	if (t == findPlane) findOnNextProcess = true;
}
