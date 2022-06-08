/*
  ==============================================================================

	KinectAzureNode.cpp
	Created: 20 May 2022 1:13:11am
	Author:  bkupe

  ==============================================================================
*/

KinectAzureNode::KinectAzureNode(var params) :
	Node(getTypeString(), Node::SOURCE, params),
	Thread("KinectAzure"),
#if USE_AZURE
	pointCloudBuffer(nullptr),
#endif
	timeAtLastInit(0),
	newFrameAvailable(false)
{
	outDepth = addSlot("Out Cloud", false, POINTCLOUD);
	outColor = addSlot("Out Color", false, RGB);
	outCamMatrix = addSlot("Out Camera Matrix", false, MATRIX);
	outDistCoeffs = addSlot("Out Distortion Coeffs", false, MATRIX);

	deviceIndex = addIntParameter("Device Index", "Index of the device to open", 0, 0);
	depthMode = addEnumParameter("Depth Mode", "Mode to process depth");
	depthMode->addOption("NFOV Unbinned", K4A_DEPTH_MODE_NFOV_UNBINNED)->addOption("NFOV Binned 2x2", K4A_DEPTH_MODE_NFOV_2X2BINNED)
		->addOption("WFOV Unbinned", K4A_DEPTH_MODE_WFOV_UNBINNED)->addOption("WFOV Binned 2x2", K4A_DEPTH_MODE_WFOV_2X2BINNED);

	downSample = addIntParameter("Down Sample", "Simple downsampling from the initial 640x480 point cloud. Value of 2 will result in a 320x240 point cloud", 2, 1, 16);

	processDepth = addBoolParameter("Process Depth", "If checked, will process depth frames", true);
	processColor = addBoolParameter("Process Color", "If checked, will process color frames", false);
	processOnlyOnNewFrame = addBoolParameter("Process only on new frame", "If checked, this will skip processing when no new frame available", false);
}

KinectAzureNode::~KinectAzureNode()
{
	stopThread(1000);
}

void KinectAzureNode::clearItem()
{
	Node::clearItem();
	outDepth = nullptr;
	outColor = nullptr;

	stopThread(1000);

#if USE_AZURE


#endif


	//if (depthData != nullptr) free(depthData);
}

bool KinectAzureNode::initInternal()
{
#if USE_AZURE

	long t = Time::getMillisecondCounter();
	if (t - timeAtLastInit < 1000) return false;

	timeAtLastInit = t;
	NNLOG("Init Kinect Azure...");
	try
	{
		k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
		device_config.depth_mode = (k4a_depth_mode_t)(int)depthMode->getValueData();;

		if(device.is_valid()) device.close();
		device = k4a::device::open(deviceIndex->intValue());
		device.start_cameras(&device_config);
		
		transformation = k4a::transformation(device.get_calibration(device_config.depth_mode, device_config.color_resolution));

	}
	catch (const std::exception& e)
	{
		if (getWarningMessage().isEmpty())
		{
			String w = "Kinect Azure init failed with exception :" + String(e.what());
			NLOGERROR(niceName, w);
			setWarningMessage(w);
		}
		return false;
	}

	clearWarning();

	NLOG(niceName, "Kinect Azure is initialized");
	startThread();
	return true;

#else
	LOGERROR("Kinect has not been compiled in this platform");
	return false;
#endif   
}


void KinectAzureNode::processInternal()
{
#if USE_AZURE
	if (isClearing) return;

	//CameraIntrinsics* intrinsics = NULL;
	//HRESULT ihr = coordinateMapper->GetDepthCameraIntrinsics(intrinsics);
	//if (SUCCEEDED(ihr))
	//{
	//	cv::Mat camMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
	//	camMatrix.at<double>(0, 0) = intrinsics->FocalLengthX;
	//	camMatrix.at<double>(1, 1) = intrinsics->FocalLengthY;
	//	sendMatrix(outCamMatrix, camMatrix);
	//}


	if (pointCloudBuffer == nullptr) return;
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
				int index = (tx + ty * depthWidth) * 3;

				float px = -pointCloudBuffer[index] / 1000.0f;
				float py = -pointCloudBuffer[index + 1] / 1000.0f;
				float pz = pointCloudBuffer[index + 2] / 1000.0f;

				int ix = floor(tx * 1.0f / ds);
				int iy = floor(ty * 1.0f / ds);
				cloud->at(ix, iy) = pcl::PointXYZ(px, py, pz);
			}
		}
	}

	sendPointCloud(outDepth, cloud);
	if (colorImage.isValid()) sendImage(outColor, colorImage);
	newFrameAvailable = false;
#endif
}

void KinectAzureNode::processInternalPassthroughInternal()
{
	processInternal(); //same
}

void KinectAzureNode::run()
{
#if USE_AZURE

	wait(10);

	k4a::image depthImage;
	while (!threadShouldExit())
	{
		wait(20);

		if (!device.is_valid())
		{
			LOGERROR("Kinect Azure disconnected");
			break;
		}

		k4a::capture sensor_capture;
		if (device.get_capture(&sensor_capture, std::chrono::milliseconds(2000)))
		{
			depthImage = sensor_capture.get_depth_image();

			{
				GenericScopedLock lock(frameLock);
				pointCloudImage = transformation.depth_image_to_point_cloud(depthImage, K4A_CALIBRATION_TYPE_DEPTH);
				depthWidth = pointCloudImage.get_width_pixels();
				depthHeight = pointCloudImage.get_height_pixels();
				pointCloudBuffer = (int16_t*)pointCloudImage.get_buffer();
				newFrameAvailable = true;
			}
		}
		else
		{
			// It should never hit time out when K4A_WAIT_INFINITE is set.
			NLOGERROR(niceName, "Error! Get depth frame time out!");
			break;
		}
	}

	if(device.is_valid()) device.close();

	{
		GenericScopedLock lock(frameLock);
		pointCloudBuffer = nullptr;
	}

	NNLOG("KinectAzure stop reading frames");
#endif
}

void KinectAzureNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);

	if (p == enabled)
	{
		if (!enabled->boolValue()) stopThread(1000);
		else startThread();
	}
	else if (p == deviceIndex || p == depthMode)
	{
		stopThread(1000);
		isInit = false;
	}
}

Image KinectAzureNode::getPreviewImage()
{
	return colorImage;
}
