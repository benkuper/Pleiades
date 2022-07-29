/*
  ==============================================================================

	Kinect2Node.cpp
	Created: 3 May 2022 11:46:19am
	Author:  bkupe

  ==============================================================================
*/


Kinect2Node::Kinect2Node(var params) :
	Node(getTypeString(), Node::SOURCE, params),
	Thread("Kinect2"),
#if USE_KINECT
#if USE_FREENECT
#else
	kinect(nullptr),
	depthReader(nullptr),
	colorReader(nullptr),
	framePoints(nullptr),
#endif
#endif
	newFrameAvailable(false)
{
	outDepth = addSlot("Out Cloud", false, POINTCLOUD);
	outColor = addSlot("Out Color", false, RGB);
	outCamMatrix = addSlot("Out Camera Matrix", false, MATRIX);
	outDistCoeffs = addSlot("Out Distortion Coeffs", false, MATRIX);

	deviceIndex = addIntParameter("Device Index", "Choose the index of the device, only working on linux.",0,0,8);
	downSample = addIntParameter("Down Sample", "Simple downsampling from the initial 640x480 point cloud. Value of 2 will result in a 320x240 point cloud", 2, 1, 16);

	processDepth = addBoolParameter("Process Depth", "If checked, will process depth frames", true);
	processColor = addBoolParameter("Process Color", "If checked, will process color frames", false);
	processOnlyOnNewFrame = addBoolParameter("Process only on new frame", "If checked, this will skip processing when no new frame available", false);
}

Kinect2Node::~Kinect2Node()
{
	stopThread(1000);
}

void Kinect2Node::clearItem()
{
	Node::clearItem();
	outDepth = nullptr;
	outColor = nullptr;

	stopThread(1000);

#if USE_KINECT
#if USE_FREENECT

	
	delete pipeline;

#else
	SafeRelease(depthReader);
	SafeRelease(colorReader);
	SafeRelease(coordinateMapper);
	if (kinect) kinect->Close();
	SafeRelease(kinect);
	free(framePoints);
#endif
#endif


	//if (depthData != nullptr) free(depthData);
}

bool Kinect2Node::initInternal()
{
#if USE_KINECT
#if USE_FREENECT

	uint32 t = Time::getMillisecondCounter();
	if (t - timeAtLastInit < 1000) return false;
	timeAtLastInit = t;

	if(pipeline == nullptr)	pipeline = new libfreenect2::CpuPacketPipeline();

	if(pipeline == nullptr)
	{
		NLOGERROR(niceName, "Could not initialize libfreenect2 pipeline");
		return false;
	}

	int numDevices = freenect2.enumerateDevices();
	if(numDevices == 0)
  {
  	NLOGERROR(niceName, "No device connected");
  	return false;
  }

  NNLOG(numDevices << " devices detected");
  for(int i = 0; i < numDevices; i++)
  {
  	NNLOG("[" << i << "] " << freenect2.getDeviceSerialNumber(i));
  }

  serial = freenect2.getDeviceSerialNumber(deviceIndex->intValue());

  if(serial.isEmpty())
  {
	NLOGERROR(niceName, "Could not find device at index " << deviceIndex->intValue());
	return false;
  }

  dev = freenect2.openDevice(serial.toStdString());

  if(dev == nullptr)
  {
    NLOGERROR(niceName, "Could not open device " << serial);
    return false;
  }


#else
	HRESULT hr;

	hr = GetDefaultKinectSensor(&kinect);
	if (FAILED(hr))
	{
		LOGERROR("Kinect init failed");
		return false;
	}

	if (kinect)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IDepthFrameSource* depthSource = NULL;
		IColorFrameSource* colorSource = NULL;

		hr = kinect->Open();

		if (SUCCEEDED(hr)) hr = kinect->get_CoordinateMapper(&coordinateMapper);
		if (SUCCEEDED(hr)) hr = kinect->get_DepthFrameSource(&depthSource);
		if (SUCCEEDED(hr)) hr = depthSource->OpenReader(&depthReader);
		if (SUCCEEDED(hr)) hr = kinect->get_ColorFrameSource(&colorSource);
		if (SUCCEEDED(hr)) hr = colorSource->OpenReader(&colorReader);

		IFrameDescription* depthDesc = NULL;
		depthSource->get_FrameDescription(&depthDesc);
		depthDesc->get_Width(&depthWidth);
		depthDesc->get_Height(&depthHeight);

		framePoints = (CameraSpacePoint*)malloc(depthWidth * depthHeight * sizeof(CameraSpacePoint));

		IFrameDescription* colorDesc = NULL;
		colorSource->get_FrameDescription(&colorDesc);
		colorDesc->get_Width(&colorWidth);
		colorDesc->get_Height(&colorHeight);

		colorImage = Image(Image::PixelFormat::ARGB, colorWidth, colorHeight, true);

		SafeRelease(depthSource);
		SafeRelease(colorSource);
	}

	if (!kinect || FAILED(hr))
	{
		LOGERROR("No ready Kinect found");
		return false;
	}


#endif

	NNLOG("Kinect is initialized");
	startThread();

#else
	LOGERROR("Kinect has not been compiled in this platform");
	return false;
#endif


	return true;
}


void Kinect2Node::processInternal()
{
#if USE_KINECT
	if (isClearing) return;

#if !USE_FREENECT
	CameraIntrinsics* intrinsics = NULL;
	HRESULT ihr = coordinateMapper->GetDepthCameraIntrinsics(intrinsics);
	if (SUCCEEDED(ihr))
	{
		cv::Mat camMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
		camMatrix.at<double>(0, 0) = intrinsics->FocalLengthX;
		camMatrix.at<double>(1, 1) = intrinsics->FocalLengthY;
		sendMatrix(outCamMatrix, camMatrix);
	}

	if (framePoints == nullptr) return;
#endif

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

				#if USE_FREENECT
				Vector3D<float> p = points[index];
				#else
				CameraSpacePoint p = framePoints[index];
				#endif

				int ix = floor(tx * 1.0f / ds);
				int iy = floor(ty * 1.0f / ds);
				
				#if USE_FREENECT
				cloud->at(ix, iy) = pcl::PointXYZ(p.x, p.y, p.z);
				#else
				cloud->at(ix, iy) = pcl::PointXYZ(p.X, p.Y, p.Z);
				#endif
				
				//if (ty == depthHeight / 2) DBG(cloud->at(ix, iy).x);
			}
		}
	}

	sendPointCloud(outDepth, cloud);
	if (colorImage.isValid()) sendImage(outColor, colorImage);
	newFrameAvailable = false;
#endif
}

void Kinect2Node::processInternalPassthroughInternal()
{
	processInternal(); //same
}

void Kinect2Node::run()
{
#if USE_KINECT

#if USE_FREENECT
	int types = 0;
	libfreenect2::FrameMap frames;
  	libfreenect2::Registration registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(K2_DEPTH_WIDTH, K2_DEPTH_HEIGHT, 4);//, registered(512, 424, 4);
	
	libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
	if (processColor->boolValue()) types |= libfreenect2::Frame::Color;
  	if (processDepth->boolValue()) types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

	if(dev != nullptr)
	{
		dev->start();
		dev->setIrAndDepthFrameListener(&listener);
		dev->startStreams(processColor->boolValue(), processDepth->boolValue());
	}

#endif
	wait(10);

	while (!threadShouldExit())
	{
		wait(20);

#if USE_FREENECT
		if (!listener.waitForNewFrame(frames, 2*1000)) // 2 seconds
		{
			NLOGWARNING(niceName,"Frame timeout");
			continue;
		}

		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

		registration.undistortDepth(depth, &undistorted);
		
		NNLOG("Got a frame");
		int ds = downSample->intValue();

		{
			GenericScopedLock lock(frameLock);
			for(int tx=0;tx<K2_DEPTH_WIDTH;tx+=ds)
			{
				for(int ty=0;ty<K2_DEPTH_HEIGHT;ty++)
				{
					Vector3D<float> p;
					registration.getPointXYZ(&undistorted, ty,tx, p.x, p.y, p.z);
					points[ty*K2_DEPTH_WIDTH+tx] = p;
				}
			}

		}

		listener.release(frames);

		newFrameAvailable = true;
#else
		if (!depthReader)
		{
			return;
		}

		IDepthFrame* depthFrame = NULL;

		HRESULT hr = depthReader->AcquireLatestFrame(&depthFrame);

		if (SUCCEEDED(hr))
		{
			// Get data from frame
			unsigned int sz;
			unsigned short* depthFrameData;
			depthFrame->AccessUnderlyingBuffer(&sz, &depthFrameData);

			{
				GenericScopedLock lock(frameLock);
				coordinateMapper->MapDepthFrameToCameraSpace(
					depthWidth * depthHeight, depthFrameData,        // Depth frame data and size of depth frame
					depthWidth * depthHeight, framePoints); // Output CameraSpacePoint array and size

			}

			if (processColor->boolValue())
			{

				UINT nColorBufferSize = 0;

				IColorFrame* colorFrame;
				hr = colorReader->AcquireLatestFrame(&colorFrame);
				ColorImageFormat imageFormat = ColorImageFormat_None;
				if (SUCCEEDED(hr))
				{
					hr = colorFrame->get_RawColorImageFormat(&imageFormat);
				}

				if (SUCCEEDED(hr))
				{
					GenericScopedLock lock(imageLock);
					Image::BitmapData bmd(colorImage, Image::BitmapData::readWrite);

					if (imageFormat == ColorImageFormat_Bgra)
					{
						hr = colorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, &bmd.data);
					}
					else
					{
						nColorBufferSize = colorWidth * colorHeight * bmd.pixelStride;
						hr = colorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, bmd.data, ColorImageFormat_Bgra);
					}
				}

				SafeRelease(colorFrame);
			}

			SafeRelease(depthFrame);
			newFrameAvailable = true;
		}
#endif
	}


#if USE_FREENECT
	if(dev != nullptr)
	{
		dev->stop();
  		dev->close();
	}
#endif

	NNLOG("Kinect2 stop reading frames");
#endif
}

void Kinect2Node::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);

	if (p == enabled)
	{
		if (!enabled->boolValue()) stopThread(1000);
		else startThread();
	}
}

Image Kinect2Node::getPreviewImage()
{
	return colorImage;
}
