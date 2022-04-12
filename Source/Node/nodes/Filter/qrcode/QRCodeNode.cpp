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

	previewType = addEnumParameter("Preview Type", "Preview Image to show");
	previewType->addOption("Input", PREVIEW_INPUT)->addOption("Output", PREVIEW_OUTPUT)->addOption("Binary", BINARY)->addOption("Contour", CONTOUR)->addOption("Contour Binarized", CONTOUR_BINARIZED)->addOption("Lines", LINES)->addOption("Extracted Image", EXTRACTED_IMAGE);
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
		cv::Mat camera_matrix, dist_coeffs;

		CodeFinder codeFinder(image, false);
		cv::Mat outputImage = codeFinder.find();

		cv::QRCodeDetector detector;

		std::vector<std::string> decodedInfo;
		std::vector<cv::Point> points;
		detector.detectAndDecodeMulti(image, decodedInfo, points);

		if (points.size() > 0)
		{
			Vector3D<float> coord;
			for (auto& d : decodedInfo)
			{
				StringArray sSplit;
				sSplit.addTokens(d, ",");
				coord.x = sSplit[0].getFloatValue();
				coord.y = sSplit[1].getFloatValue();
				coord.z = sSplit[2].getFloatValue();
			}

			cv::Point p = points[0];
			int tx = p.x * source->width / image.cols;
			int ty = p.y * source->height / image.rows;

			PPoint pp = source->at(tx, ty);
			NNLOG(p.x << "," << p.y << " > " << pp.x << "," << pp.y << "," << pp.z);

			planeCenter.x() = pp.x;
			planeCenter.y() = pp.y;
			planeCenter.z() = pp.z;
		}




		/*
		std::vector<QRCode> codes = codeFinder.getAllCodes();
		QRCode bestCode;
		if (codes.size() > 0)
		{
			NNLOG("Found " << (int)codes.size() << "codes");
			for (auto& c : codes) if (c.verifyPercentage > bestCode.verifyPercentage) bestCode = c;
		}
		else
		{
			NNLOG("No QR Code found.");
		}


		DBG("Best code data : " << bestCode.)

			//Generate preview image
			cv::Mat previewImage;
		PreviewImageType t = previewType->getValueDataAsEnum<PreviewImageType>();

		switch (t)
		{
		case PREVIEW_INPUT: previewImage = image; break;
		case PREVIEW_OUTPUT: previewImage = outputImage; break;
		case BINARY: previewImage = codeFinder.drawBinaryImage(); break;
		case CONTOUR: previewImage = codeFinder.drawAllContours(); break;
		case CONTOUR_BINARIZED: previewImage = codeFinder.drawAllContoursBinarized(); break;
		case LINES: previewImage = codeFinder.drawAllLines(); break;
		case EXTRACTED_IMAGE: previewImage = codeFinder.drawExtractedCodes()[0];
		}

		Image::PixelFormat fmt = previewImage.type() == CV_8UC3 ? Image::PixelFormat::RGB : Image::PixelFormat::SingleChannel;
		qrImage = Image(fmt, previewImage.cols, previewImage.rows, true);

		Image::BitmapData outBMD(qrImage, Image::BitmapData::writeOnly);
		memcpy(outBMD.data, previewImage.data, outBMD.size);

		*/

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
