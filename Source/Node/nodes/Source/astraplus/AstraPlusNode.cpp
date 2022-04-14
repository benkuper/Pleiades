/*
  ==============================================================================

	AstraPlusNode.cpp
	Created: 5 Apr 2022 10:38:21am
	Author:  bkupe

  ==============================================================================
*/

std::unique_ptr<ob::Context> AstraPlusNode::ctx;
std::unique_ptr<ob::Pipeline> AstraPlusNode::pipeline;
std::shared_ptr<ob::VideoStreamProfile> AstraPlusNode::colorProfile;
std::shared_ptr<ob::VideoStreamProfile>AstraPlusNode::depthProfile;

AstraPlusNode::AstraPlusNode(var params) :
	Node(getTypeString(), Node::SOURCE, params),
	Thread("AstraPlus"),
	ifx(0),
	ify(0),
	depthData(nullptr),
	newFrameAvailable(false)
{
	outDepth = addSlot("Out Cloud", false, POINTCLOUD);
	outColor = addSlot("Out Color", false, RGB);
	outCamMatrix = addSlot("Out Camera Matrix", false, MATRIX);
	outDistCoeffs = addSlot("Out Distortion Coeffs", false, MATRIX);



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

	if (depthData != nullptr) free(depthData);
}

bool AstraPlusNode::initInternal()
{

	if (ctx == nullptr)
	{
		ctx.reset(new ob::Context());
		ctx->setLoggerServerity(OBLogServerity::OB_LOG_SEVERITY_ERROR);
	}

	if (pipeline == nullptr)
	{
		NLOG(niceName, "Init");
		pipeline.reset(new ob::Pipeline());
		setupProfiles();
		setupPipeline();

		pipeline->start(config);
	}

	//Init depth data
	if (depthData != nullptr) free(depthData);
	int dataSize = depthProfile->width() * depthProfile->height() * 2;
	depthData = (uint8_t*)malloc(dataSize);
	memset(depthData, 0, dataSize);


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

		if (pipeline->getDevice()->isPropertySupported(OB_DEVICE_PROPERTY_DEPTH_ALIGN_HARDWARE_BOOL))
			pipeline->getDevice()->setBoolProperty(OB_DEVICE_PROPERTY_DEPTH_ALIGN_HARDWARE_BOOL, alignDepthToColor->boolValue());
		else if (pipeline->getDevice()->isPropertySupported(OB_DEVICE_PROPERTY_DEPTH_ALIGN_SOFTWARE_BOOL))
			pipeline->getDevice()->setBoolProperty(OB_DEVICE_PROPERTY_DEPTH_ALIGN_SOFTWARE_BOOL, alignDepthToColor->boolValue());
	}
}

void AstraPlusNode::processInternal()
{
	if (isClearing) return;

	if (pipeline == nullptr) init();
	jassert(pipeline != nullptr);

	if (ifx == 0 || ify == 0)
	{
		OBSensorType sensorType = alignDepthToColor->boolValue() ? OBSensorType::OB_SENSOR_COLOR : OBSensorType::OB_SENSOR_DEPTH;
		OBCameraIntrinsic intrinsic = pipeline->getDevice()->getCameraIntrinsic(sensorType);
		ifx = intrinsic.fx;
		ify = intrinsic.fy;

		camMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
		camMatrix.at<double>(0, 0) = intrinsic.fx;
		camMatrix.at<double>(1, 1) = intrinsic.fy;
		sendMatrix(outCamMatrix, camMatrix);
		//, 0, 3, intrinsic.cx, 0, intrinsic.fy, intrinsic.cy, 0, 0, 1);
	}


	if (depthData == nullptr) return;
	if (!newFrameAvailable && processOnlyOnNewFrame->boolValue()) return;

	int dw = depthProfile->width();
	int dh = depthProfile->height();

	int ds = downSample->intValue();
	int downW = ceil(dw * 1.0f / ds);
	int downH = ceil(dh * 1.0f / ds);
	CloudPtr cloud(new Cloud(downW, downH));

	float fx = 2 * atan(dw * 1.0f / (2.0f * ifx));
	float fy = 2 * atan(dh * 1.0f / (2.0f * ify));

	{
		GenericScopedLock lock(frameLock);
		for (int ty = 0; ty < dh; ty += ds)
		{
			for (int tx = 0; tx < dw; tx += ds)
			{
				float relX = .5f - (tx * 1.0f / dw);
				float relY = .5f - (ty * 1.0f / dh);

				int index = tx + ty * dw;
				int dp = depthData[index * 2 + 1] << 8 | depthData[index * 2];

				float d = dp / 1000.0f;

				float x = relX * d * fx;
				float y = relY * d * fy;
				float z = d;

				try
				{
					cloud->at(floor(tx / ds), floor(ty / ds)) = pcl::PointXYZ(x, y, z);
				}
				catch (...)
				{
					LOG("here");
				}
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

	while (!threadShouldExit())
	{
		auto frameset = pipeline->waitForFrames(100);
		if (frameset == nullptr)
		{
			wait(20);
			continue;
		}

		if (threadShouldExit()) break;

		uint8_t* newDepthData = nullptr;
		uint8_t* newColorData = nullptr;

		int depthDataSize = 0;
		int colorDataSize = 0;

		if (processDepth->boolValue())
		{
			if (std::shared_ptr<ob::DepthFrame> frame = frameset->depthFrame())
			{
				newDepthData = (uint8_t*)frame->data();
				depthDataSize = frame->dataSize();
			}
		}

		if (processColor->boolValue())
		{
			if (std::shared_ptr<ob::ColorFrame> frame = frameset->colorFrame())
			{
				newColorData = (uint8_t*)frame->data();
				colorDataSize = frame->dataSize();
			}
		}

		if (newDepthData != nullptr || newColorData != nullptr)
		{
			GenericScopedLock lock(frameLock);

			if (depthData != nullptr && newDepthData != nullptr) memcpy(depthData, newDepthData, depthDataSize);
			if (newColorData != nullptr && colorImage.isValid())
			{
				//Image::BitmapData bmd(colorImage, Image::BitmapData::writeOnly);
				//memcpy(bmd.data, newColorData, colorDataSize);

				MemoryInputStream is(newColorData, colorDataSize, false);

				JPEGImageFormat format;

				GenericScopedLock lock(imageLock);
				colorImage = format.decodeImage(is);
			}

			newFrameAvailable = true;
		}

		wait(20);
	}

	NNLOG("Astraplus stop reading frames");
}

void AstraPlusNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);

	if (p == enabled)
	{
		if (!enabled->boolValue())
		{
			stopThread(1000);
		}
		else if (pipeline != nullptr) startThread();
	}
	else if (p == alignDepthToColor || p == processColor || p == processDepth)
	{
		if (pipeline != nullptr)
		{
			pipeline->stop();
			setupPipeline();
			pipeline->start(config);
		}

		//force recalculate intrinsics
		ifx = 0;
		ify = 0;
	}
}

Image AstraPlusNode::getPreviewImage()
{
	return colorImage;
}
