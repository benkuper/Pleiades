/*
  ==============================================================================

	Kinect2Node.cpp
	Created: 3 May 2022 11:46:19am
	Author:  bkupe

  ==============================================================================
*/


Kinect2Node::Kinect2Node(var params) :
	Node(getTypeString(), Node::SOURCE, params),
	Thread("AstraPlus"),
	kinect(nullptr),
	depthReader(nullptr),
	colorReader(nullptr),
	framePoints(nullptr),
	newFrameAvailable(false)
{
	outDepth = addSlot("Out Cloud", false, POINTCLOUD);
	outColor = addSlot("Out Color", false, RGB);
	outCamMatrix = addSlot("Out Camera Matrix", false, MATRIX);
	outDistCoeffs = addSlot("Out Distortion Coeffs", false, MATRIX);


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
	SafeRelease(depthReader);
	SafeRelease(colorReader);
	SafeRelease(coordinateMapper);
	if (kinect) kinect->Close();
	SafeRelease(kinect);

#endif

	free(framePoints);

	//if (depthData != nullptr) free(depthData);
}

bool Kinect2Node::initInternal()
{
#if USE_KINECT

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

	NNLOG("Kinect is initialized");

#else
	LOGERROR("Kinect has not been compiled in this version");
	return false;
#endif

	startThread();

	return true;
}


void Kinect2Node::processInternal()
{
	if (isClearing) return;

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
				CameraSpacePoint p = framePoints[index];

				int ix = floor(tx * 1.0f / ds);
				int iy = floor(ty * 1.0f / ds);
				cloud->at(ix, iy) = pcl::PointXYZ(p.X, p.Y, p.Z);
				//if (ty == depthHeight / 2) DBG(cloud->at(ix, iy).x);
			}
		}
	}

	sendPointCloud(outDepth, cloud);
	if (colorImage.isValid()) sendImage(outColor, colorImage);
	newFrameAvailable = false;
}

void Kinect2Node::processInternalPassthroughInternal()
{
	processInternal(); //same
}

void Kinect2Node::run()
{
	wait(10);

	while (!threadShouldExit())
	{
#if USE_KINECT
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

#endif
			wait(20);
		}
	}

	NNLOG("Kinect2 stop reading frames");
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
