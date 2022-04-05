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
	ifx(0),
	ify(0)
{
	out = addSlot("Out", false, POINTCLOUD);

	downSample = addIntParameter("Down Sample", "Simple downsampling from the initial 640x480 point cloud. Value of 2 will result in a 320x240 point cloud", 2, 1, 16);
}

AstraPlusNode::~AstraPlusNode()
{
}

void AstraPlusNode::clearItem()
{
	Node::clearItem();
	out = nullptr;
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
		//setupPointCloud();
	}

	return true;
}

void AstraPlusNode::setupProfiles()
{
	auto colorProfiles = pipeline->getStreamProfileList(OB_SENSOR_COLOR);

	for (int i = 0; i < (int)colorProfiles->count(); i++) {
		auto profile = colorProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
		if ((profile->format() == OB_FORMAT_YUYV || profile->format() == OB_FORMAT_I420) && profile->width() == 640) {
			NLOG(niceName, "Using color profile " << (int)profile->width() << "x" << (int)profile->height() << " @ " << (int)profile->fps() << " fps");
			colorProfile = profile;
			break;
		}
	}

	const int targetWidth = 640;
	const int targetHeight = 480;

	auto depthProfiles = pipeline->getStreamProfileList(OB_SENSOR_DEPTH);
	for (int i = 0; i < (int)depthProfiles->count(); i++) {
		auto profile = depthProfiles->getProfile(i)->as<ob::VideoStreamProfile>();
		if ((profile->format() == OB_FORMAT_YUYV || profile->format() == OB_FORMAT_Y16) && profile->width() == targetWidth && profile->height() == targetHeight) {
			NLOG(niceName, "Using depth profile " << (int)profile->width() << "x" << (int)profile->height() << " @ " << (int)profile->fps() << " fps");
			depthProfile = profile;
			break;
		}
	}
}

void AstraPlusNode::setupPipeline()
{
	config = std::make_shared<ob::Config>();
	//config->enableStream(colorProfile);
	config->enableStream(depthProfile);

	if (pipeline->getDevice()->isPropertySupported(OB_DEVICE_PROPERTY_DEPTH_ALIGN_HARDWARE_BOOL))
		pipeline->getDevice()->setBoolProperty(OB_DEVICE_PROPERTY_DEPTH_ALIGN_HARDWARE_BOOL, false);

}

//void AstraPlusNode::setupPointCloud()
//{
	//pointCloud = pipeline->createFilter<PointCloudFilter>();

	//try {
	//	CAMERA_PARA cameraParam = { 0 };
	//	uint32_t    len;
	//	pipeline->getDevice()->getStructuredData(OB_DATA_TYPE_CAMERA_PARA, &cameraParam, &len);
	//	pointCloud->setCameraPara(cameraParam);
	//	pointCloud->setCreatePointFormat(OB_FORMAT_POINT);
	//}
	//catch (...) {
	//	NLOG(niceName,"Set point cloud camera param failed!");
	//}

//}

void AstraPlusNode::processInternal()
{
	if (isClearing) return;

	if (pipeline == nullptr) init();
	jassert(pipeline != nullptr);


	auto frameset = pipeline->waitForFrames(100);
	if (frameset == nullptr || frameset->depthFrame() == nullptr) return;

	if (ifx == 0 || ify == 0)
	{
		OBCameraIntrinsic intrinsic = pipeline->getDevice()->getCameraIntrinsic(OBSensorType::OB_SENSOR_DEPTH);
		ifx = intrinsic.fx;
		ify = intrinsic.fy;
	}

	int dw = depthProfile->width();
	int dh = depthProfile->height();
	//int numPoints = dw * dh;

	std::shared_ptr<ob::DepthFrame> frame = frameset->depthFrame();
	uint8_t* depthData = (uint8_t*)frame->data();

	//DBG("Depth data size : " << (int)frame->dataSize() << " / num points " << numPoints);

	int ds = downSample->intValue();
	int downW = ceil(dw*1.0f / ds);
	int downH = ceil(dh*1.0f / ds);
	CloudPtr cloud(new Cloud(downW, downH));

	float fx = 2 * atan(dw * 1.0f / (2.0f * ifx));
	float fy = 2 * atan(dh * 1.0f / (2.0f * ify));

	for (int ty = 0; ty < dh; ty += ds)
	{
		for (int tx = 0; tx < dw; tx += ds)
		{
			float relX = .5f - (tx * 1.0f / dw);
			float relY = .5f - (ty * 1.0f / dh);


			int index = tx + ty * dw;
			int dp = depthData[index * 2 + 1] << 8 | depthData[index * 2];

			//float xzFactor = tan(fx / 2) * 2;
			//float yzFactor = tan(fy / 2) * 2;

			float d = dp / 1000.0f;


			float x = relX * d * fx;// x_over_z;// * p.z;
			float y = relY * d * fy;// y_over_z;// * p.z;
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

	sendPointCloud(out, { 0, cloud });
}
