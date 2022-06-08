/*
  ==============================================================================

	AstraProNode.cpp
	Created: 4 May 2022 11:22:53am
	Author:  bkupe

  ==============================================================================
*/

bool AstraProNode::astraIsInit = false;

AstraProNode::AstraProNode(var params) :
	Node(getTypeString(), Node::SOURCE, params),
	Thread("Astra Pro"),
	pointsData(nullptr),
	pointsDataSize(0),
	newFrameAvailable(false)
{
	outDepth = addSlot("Out Cloud", false, POINTCLOUD);
	outColor = addSlot("Out Color", false, RGB);


	deviceIndex = addIntParameter("Device Index", "Index of the device", 0, 0);
	downSample = addIntParameter("Down Sample", "Simple downsampling from the initial 640x480 point cloud. Value of 2 will result in a 320x240 point cloud", 2, 1, 16);

	processDepth = addBoolParameter("Process Depth", "If checked, will process depth frames", true);
	processColor = addBoolParameter("Process Color", "If checked, will process color frames", false);
	alignDepthToColor = addBoolParameter("Align Depth to Color", "If checked, this  will change the depth frame to pixel match the color frame", true);
	processOnlyOnNewFrame = addBoolParameter("Process only on new frame", "If checked, this will skip processing when no new frame available", false);


}

AstraProNode::~AstraProNode()
{

}

void AstraProNode::clearItem()
{
	stopThread(1000);

	Node::clearItem();


	outDepth = nullptr;
	outColor = nullptr;

	if (pointsData != nullptr)
	{
		free(pointsData);
		pointsData = nullptr;
	}

}

bool AstraProNode::initInternal()
{
	startThread();
	return true;
}


void AstraProNode::processInternal()
{
	if (isClearing) return;

	if (!enabled->boolValue()) return;

	if (pointsData == nullptr) return;
	if (!newFrameAvailable && processOnlyOnNewFrame->boolValue()) return;

	int ds = downSample->intValue();
	int downW = ceil(depthWidth * 1.0f / ds);
	int downH = ceil(depthHeight * 1.0f / ds);
	CloudPtr cloud(new Cloud(downW, downH));

	{
		GenericScopedLock lock(frameLock);

		for (int ty = 0; ty < depthHeight; ty += ds)
		{
			for (int tx = 0; tx < depthWidth; tx += ds)
			{
				int index = tx + ty * depthWidth;
				astra::Vector3f p = pointsData[index];
				cloud->at(floor(tx * 1.0f / ds), floor(ty * 1.0f / ds)) = pcl::PointXYZ(p.x / 1000.0f, p.y / 1000.0f, p.z / 1000.0f);
			}
		}
	}

	sendPointCloud(outDepth, cloud);
	sendImage(outColor, colorImage);
	newFrameAvailable = false;
}

void AstraProNode::processInternalPassthroughInternal()
{
	processInternal(); //same
}

void AstraProNode::run()
{

	astra_status_t result = astra_initialize();
	if (result != 0)
	{
		LOGWARNING("Astra init error : " << result);
		return;
	}

	astraIsInit = true;

	String uri = "device/sensor" + deviceIndex->stringValue();
	astra::StreamSet streamSet(uri.toStdString().c_str());
	astra::StreamReader reader = streamSet.create_reader();

	//ReaderListener listener;

	auto depthStream = reader.stream<astra::DepthStream>();
	depthStream.start();

	auto pointStream = reader.stream<astra::PointStream>();
	pointStream.start();

	auto colorStream = reader.stream<astra::ColorStream>();
	colorStream.start();

	char serialnumber[256];
	depthStream.serial_number(serialnumber, 256);

	mapper = (astra::CoordinateMapper*)&reader.stream<astra::DepthStream>().coordinateMapper();

	LOG("depthStream -- hFov: "
		<< reader.stream<astra::DepthStream>().hFov()
		<< " vFov: "
		<< reader.stream<astra::DepthStream>().vFov()
		<< " serial number: "
		<< serialnumber);

	const uint32_t chipId = depthStream.chip_id();

	switch (chipId)
	{
	case ASTRA_CHIP_ID_MX400:
		LOG("Chip ID: MX400");
		break;
	case ASTRA_CHIP_ID_MX6000:
		LOG("Chip ID: MX6000");
		break;
	case ASTRA_CHIP_ID_UNKNOWN:
	default:
		LOG("Chip ID: Unknown");
		break;
	}

	const astra_usb_info_t usbinfo = depthStream.usb_info();

	LOG("usbInfo ---pid:" << (int)usbinfo.pid << " vid: " << (int)usbinfo.vid);

	while (!threadShouldExit() && enabled->boolValue())
	{
		wait(5);
		astra_update();

		if (isClearing || threadShouldExit()) return;

		if (reader.has_new_frame())
		{
			astra::Frame frame = reader.get_latest_frame();

			if (processDepth->boolValue())
			{
				astra::PointFrame pointFrame = frame.get<astra::PointFrame>();
				if (pointFrame.is_valid())
				{
					GenericScopedLock lock(frameLock);
					depthWidth = pointFrame.width();
					depthHeight = pointFrame.height();
					int dataSize = depthWidth * depthHeight * pointFrame.bytes_per_pixel();
					if (pointsDataSize != dataSize)
					{
						pointsDataSize = dataSize;
						pointsData = (astra::Vector3f*)malloc(pointsDataSize);
					}
					memcpy(pointsData, pointFrame.data(), pointsDataSize);
				}
			}

			if (processColor->boolValue())
			{
				const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();
				if (colorFrame.is_valid())
				{
					GenericScopedLock lock(imageLock);

					if (colorImage.isNull()) colorImage = Image(Image::PixelFormat::RGB, colorFrame.width(), colorFrame.height(), true);
					Image::BitmapData bmd(colorImage, Image::BitmapData::writeOnly);
					memcpy(bmd.data, colorFrame.data(), bmd.width * bmd.height * bmd.pixelStride);
				}
			}

			newFrameAvailable = true;
		}
	}

	{
		GenericScopedLock lock(frameLock);
		pointsDataSize = 0;
		free(pointsData);
		pointsData = nullptr;
	}

	//astra::terminate();
}

void AstraProNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
	if (p == enabled)
	{
		if (enabled->boolValue()) startThread();
		else stopThread(1000);
	}
	else if (p == deviceIndex)
	{
		stopThread(1000);
		startThread();
	}
}

Image AstraProNode::getPreviewImage()
{
	return colorImage;
}
