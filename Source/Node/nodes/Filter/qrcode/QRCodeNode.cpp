/*
  ==============================================================================

	QRCodeNode.cpp
	Created: 12 Apr 2022 12:46:58am
	Author:  bkupe

  ==============================================================================
*/

QRCodeNode::QRCodeNode(var params) :
	Node(getTypeString(), FILTER, params),
	findOnNextProcess(false)
{
	addInOutSlot(&inDepth, &out, POINTCLOUD, "In", "Out");
	inColor = addSlot("Color", true, NodeConnectionType::RGB);

	planeCenterSlot = addSlot("Plane Center", false, VECTOR);

	continuous = addBoolParameter("Continuous Search", "If checked, search always for the plane. Otherwise, it will only search when triggering", false);
	findPlane = addTrigger("Find Plane", "Find the plane. Now.");

	transformPlane = addBoolParameter("Transform Plane", "If checked, this will tranform the plane", false);
}

QRCodeNode::~QRCodeNode()
{
}


void QRCodeNode::processInternal()
{
	CloudPtr source = slotCloudMap[inDepth];
	Image img = slotImageMap[inColor];

	if (source == nullptr || source->empty() || !img.isValid()) return;

	if (continuous->boolValue() || findOnNextProcess)
	{
		NNLOG("Finding plane..");

		Image::BitmapData bmd(img, Image::BitmapData::readOnly);
		cv::Mat image(img.getHeight(), img.getWidth(), CV_8UC3, bmd.data);

		//to fill
		cv::QRCodeDetector detector;

		std::vector<std::string> decodedInfo;
		std::vector<cv::Point> points;
		detector.detectAndDecodeMulti(image, decodedInfo, points);


		NNLOG("Found " << decodedInfo.size() << " tags");

		qrImage = img.createCopy();
		Graphics g(qrImage);

		Eigen::Vector3f planeOffset;
		Eigen::Vector3f rot;

		for (int i = 0; i < decodedInfo.size(); i++)
		{
			Vector3D<float> coord;
			StringArray sSplit;
			sSplit.addTokens(decodedInfo[i], ",");
			coord.x = sSplit[0].getFloatValue();
			coord.y = sSplit[1].getFloatValue();
			coord.z = sSplit[2].getFloatValue();

			if (i == 0)
			{
				cv::Point p = points[0];
				int tx = p.x * source->width / image.cols;
				int ty = p.y * source->height / image.rows;

				PPoint pp = source->at(tx, ty);
				NNLOG(p.x << "," << p.y << " > " << pp.x << "," << pp.y << "," << pp.z);

				planeCenter.x() = pp.x;
				planeCenter.y() = pp.y;
				planeCenter.z() = pp.z;
			}

			int index = i * 4;
			for (int j = 0; j < 4; j++)
			{
				g.setColour(Colour::fromHSV((index + j) / 4.0, 1, 1, 1));
				g.drawEllipse(Rectangle<float>(0, 0, 10, 10).withCentre(Point<float>(points[index + j].x, points[index + j].y)), 2);
				g.drawLine(points[index + j].x, points[index + j].y, points[index + (j + 1)%4].x, points[index + (j + 1) % 4].y, 2);
			}
		}

		findOnNextProcess = false;
	}

	if (!out->isEmpty())
	{
		if (transformPlane->boolValue())
		{
			CloudPtr transformedCloud(new Cloud(source->width, source->height));

			Eigen::Affine3f transform = Eigen::Affine3f::Identity();
			transform.translate(-planeCenter);
			pcl::transformPointCloud(*source, *transformedCloud, transform);
		}
		else
		{
			sendPointCloud(out, source);
		}
	}


	sendVector(planeCenterSlot, planeCenter);
}

var QRCodeNode::getJSONData()
{
	var data = Node::getJSONData();
	var planeData;
	planeData.append(planeCenter.x());
	planeData.append(planeCenter.y());
	planeData.append(planeCenter.z());

	data.getDynamicObject()->setProperty("planeData", planeData);
	return data;
}

void QRCodeNode::loadJSONDataItemInternal(var data)
{
	Node::loadJSONDataItemInternal(data);
	var planeData = data.getProperty("planeData", var());
	if (planeData.size() >= 3)
	{
		planeCenter = Eigen::Vector3f(planeData[0], planeData[1], planeData[2]);
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

Image QRCodeNode::getPreviewImage()
{
	return qrImage;
}
