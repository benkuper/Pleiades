/*
  ==============================================================================

	AstraPlusNode.cpp
	Created: 5 Apr 2022 10:38:21am
	Author:  bkupe

  ==============================================================================
*/

std::unique_ptr<ob::Context> AstraPlusNode::ctx;

AstraPlusNode::AstraPlusNode(var params) :
	Node(getTypeString(), Node::SOURCE, params),
	Thread("AstraPlus"),
	depthWidth(0),
	depthHeight(0),
	ifx(0),
	ify(0),
	pointsData(nullptr),
	timeAtlastDeviceQuery(0),
	newFrameAvailable(false)
{
	outDepth = addSlot("Out Cloud", false, POINTCLOUD);
	outColor = addSlot("Out Color", false, RGB);
	outCamMatrix = addSlot("Out Camera Matrix", false, MATRIX);
	outDistCoeffs = addSlot("Out Distortion Coeffs", false, MATRIX);

	deviceIndex = addIntParameter("Device Index", "Index of the device", 0, 0);
	downSample = addIntParameter("Down Sample", "Simple downsampling from the initial 640x480 point cloud. Value of 2 will result in a 320x240 point cloud", 2, 1, 16);

	processDepth = addBoolParameter("Process Depth", "If checked, will process depth frames", true);
	processColor = addBoolParameter("Process Color", "If checked, will process color frames", false);
	alignDepthToColor = addBoolParameter("Align Depth to Color", "If checked, this  will change the depth frame to pixel match the color frame", true);
	processOnlyOnNewFrame = addBoolParameter("Process only on new frame", "If checked, this will skip processing when no new frame available", false);
}

AstraPlusNode::~AstraPlusNode()
{
	stopThread(1000);
}

void AstraPlusNode::clearItem()
{
	Node::clearItem();
	outDepth = nullptr;
	outColor = nullptr;
}

bool AstraPlusNode::initInternal()
{
	if (!enabled->boolValue()) return false;

	//do not init too much if in loop (when device is not available)
	long t = Time::getMillisecondCounter();
	if (t - timeAtlastDeviceQuery < 1000) return false;
	timeAtlastDeviceQuery = t;

	if (ctx == nullptr)
	{
		ctx.reset(new ob::Context());
		//ctx->setLoggerServerity(OBLogServerity::OB_LOG_SEVERITY_ERROR);
	}

	if (pipeline == nullptr || device == nullptr)
	{
		try
		{
			auto deviceList = ctx->queryDeviceList();
			
			if (device == nullptr)
			{
				device = deviceList->getDevice(deviceIndex->intValue());
				if (device == nullptr)
				{
					if (getWarningMessage().isEmpty())
					{
						NLOGWARNING(niceName, "Astra+ device not found at index " << deviceIndex->intValue());
						setWarningMessage("Astra+ not connected.");
					}
					return false;
				}
			}

			pipeline.reset(new ob::Pipeline(device));
			setupProfiles();
			setupPipeline();

			pipeline->start(config);

			clearWarning();

			NLOG(niceName, "Astra+ connected and initialized.");
		}
		catch (std::exception e)
		{
			if (getWarningMessage().isEmpty())
			{
				NLOGWARNING(niceName, "Could not initialize Astra+. Is it connected ? " << e.what());
				setWarningMessage("Astra+ not connected");
			}
		}
	}

	if (pipeline == nullptr) return false;

	depthWidth = depthProfile->width();
	depthHeight = depthProfile->height();


	colorImage = Image(Image::PixelFormat::RGB, depthProfile->width(), depthProfile->height(), true);

	startThread();

	return true;
}

void AstraPlusNode::setupProfiles()
{
	auto colorProfiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);

	const int targetWidth = 640;
	const int targetHeight = 480;

	for (int i = 0; i < (int)colorProfiles->count(); i++)
	{
		auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
		if (profile->format() == OB_FORMAT_MJPG && profile->width() == targetWidth)
		{
			NNLOG("Using color profile " << (int)profile->width() << "x" << (int)profile->height() << " @ " << (int)profile->fps() << " fps");
			colorProfile = profile;
			break;
		}
	}


	auto depthProfiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
	for (int i = 0; i < (int)depthProfiles->count(); i++)
	{
		auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
		if (profile->format() == OB_FORMAT_Y16 && profile->width() == targetWidth && profile->height() == targetHeight)
		{
			NNLOG("Using depth profile " << (int)profile->width() << "x" << (int)profile->height() << " @ " << (int)profile->fps() << " fps");
			depthProfile = profile;
			break;
		}
	}

	
}

void AstraPlusNode::setupPipeline()
{
	config = std::make_shared<ob::Config>();
	if (colorProfile != nullptr && processColor->boolValue()) config->enableStream(colorProfile);
	if (depthProfile != nullptr && processDepth->boolValue())
	{
		config->enableStream(depthProfile);

		if (pipeline->getDevice()->isPropertySupported(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, OB_PERMISSION_WRITE))
			pipeline->getDevice()->setBoolProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, alignDepthToColor->boolValue());


		pointCloudFilter.reset(new ob::PointCloudFilter());
		auto cameraParam = pipeline->getCameraParam();
		pointCloudFilter->setCameraParam(cameraParam);
	}
	else
	{
		pointCloudFilter.reset();
	}
}

void AstraPlusNode::processInternal()
{
	if (isClearing) return;

	if (pipeline == nullptr) init();
	jassert(pipeline != nullptr);

	if (ifx == 0 || ify == 0)
	{
		//OBSensorType sensorType = alignDepthToColor->boolValue() ? OBSensorType::OB_SENSOR_COLOR : OBSensorType::OB_SENSOR_DEPTH;
		//OBCameraIntrinsic intrinsic = pipeline->getDevice()->getCameraIntrinsic(sensorType);
		//ifx = intrinsic.fx;
		//ify = intrinsic.fy;
		//need to update to the new sdk code
		ifx = 1;
		ify = 1;

		camMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
		camMatrix.at<double>(0, 0) = ifx;
		camMatrix.at<double>(1, 1) = ify;
		sendMatrix(outCamMatrix, camMatrix);
		//, 0, 3, intrinsic.cx, 0, intrinsic.fy, intrinsic.cy, 0, 0, 1);
	}

	GenericScopedLock lock(frameLock);

	if (pointsData == nullptr) return;
	if (!newFrameAvailable && processOnlyOnNewFrame->boolValue()) return;


	int ds = downSample->intValue();
	int downW = ceil(depthWidth * 1.0f / ds);
	int downH = ceil(depthHeight * 1.0f / ds);
	CloudPtr cloud(new Cloud(downW, downH));

	//float fx = 2 * atan(depthWidth * 1.0f / (2.0f * ifx));
	//float fy = 2 * atan(depthHeight * 1.0f / (2.0f * ify));

	{
		for (int ty = 0; ty < depthHeight; ty += ds)
		{
			for (int tx = 0; tx < depthWidth; tx += ds)
			{
				//float relX = .5f - (tx * 1.0f / depthWidth);
				//float relY = .5f - (ty * 1.0f / depthHeight);

				int index = tx + ty * depthWidth;
				//int dp = depthData[index * 2 + 1] << 8 | depthData[index * 2];

				//float d = dp / 1000.0f;

				//float x = relX * d * fx;
				//float y = relY * d * fy;
				//float z = d;

				OBPoint p = pointsData[index];
				cloud->at(floor(tx * 1.0f / ds), floor(ty * 1.0f / ds)) = pcl::PointXYZ(-p.x / 1000.0f, p.y / 1000.0f, p.z / 1000.0f);
			}
		}
	}

	sendPointCloud(outDepth, cloud);
	sendImage(outColor, colorImage);
	newFrameAvailable = false;
}

void AstraPlusNode::processInternalPassthroughInternal()
{
	processInternal(); //same
}

void AstraPlusNode::run()
{
	wait(10);

	const char* uid = pipeline->getDevice()->getDeviceInfo()->uid();

	timeAtlastDeviceQuery = Time::getMillisecondCounter();

	while (!threadShouldExit())
	{
		wait(2);


		long t = Time::getMillisecondCounter();
		if (t - timeAtlastDeviceQuery > 1000)
		{

			{
				try
				{
					auto deviceList = ctx->queryDeviceList();
					const char* tuid = deviceList->uid(deviceIndex->intValue());
					if (strcmp(uid, tuid) != 0)
					{
						NLOGWARNING(niceName, "Device disconnected");
						isInit = false;
						pipeline.reset();
						break;
					}
				}
				catch (std::exception e)
				{
					//NLOGERROR(niceName, "Error querying device : " << e.what());
				}

				timeAtlastDeviceQuery = t;
			}
		}

		auto frameset = pipeline->waitForFrames(100);
		if (frameset == nullptr) continue;

		if (threadShouldExit()) break;


		if (processDepth->boolValue() && frameset->depthFrame() != nullptr)
		{
			GenericScopedLock lock(frameLock);
			pointCloudFilter->reset();
			pointCloudFilter->setCreatePointFormat(OB_FORMAT_POINT);
			if (auto frame = pointCloudFilter->process(frameset))
			{
				pointsData = (OBPoint*)frame->data();

				newFrameAvailable = true;
			}
		}

		if (processColor->boolValue())
		{
			if (std::shared_ptr<ob::ColorFrame> frame = frameset->colorFrame())
			{
				uint8_t* newColorData = (uint8_t*)frame->data();
				int colorDataSize = frame->dataSize();

				if (newColorData != nullptr && colorImage.isValid())
				{
					MemoryInputStream is(newColorData, colorDataSize, false);
					JPEGImageFormat format;

					GenericScopedLock lock(imageLock);
					colorImage = format.decodeImage(is);

					newFrameAvailable = true;
				}
			}
		}
	}

	{
		GenericScopedLock lock(frameLock);
		pointsData = nullptr;
	}

	NNLOG("Astraplus stop reading frames");
}

void AstraPlusNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);

	if (!isCurrentlyLoadingData)
	{
		if (p == enabled)
		{
			if (!enabled->boolValue())
			{
				stopThread(1000);
				if (pipeline != nullptr) pipeline->stop();
			}
			else if (pipeline != nullptr)
			{
				isInit = false;
				pipeline.reset();
				device.reset();
			}
		}
		else if (p == alignDepthToColor || p == processColor || p == processDepth || p == deviceIndex)
		{
			stopThread(100);
			isInit = false;
			pipeline.reset();
			device.reset();
			ifx = 0;
			ify = 0;
		}
	}
}

Image AstraPlusNode::getPreviewImage()
{
	return colorImage;
}
