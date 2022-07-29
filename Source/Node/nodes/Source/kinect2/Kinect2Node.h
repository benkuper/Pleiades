/*
  ==============================================================================

	Kinect2Node.h
	Created: 3 May 2022 11:46:19am
	Author:  bkupe

  ==============================================================================
*/

#pragma once

#ifndef USE_KINECT
#if JUCE_WINDOWS
#define USE_KINECT 1
#if JUCE_LINUX
#define USE_FREENECT 0
#endif //LINUX
#endif //WINLINUX
#endif //Ndef kinect



#ifndef USE_KINECT
#define USE_KINECT 0
#endif

#ifndef USE_FREENECT
#define USE_FREENECT 0
#endif

#if USE_KINECT
#if USE_FREENECT
	#include "libfreenect2/libfreenect2.hpp"
	#include "libfreenect2/frame_listener_impl.h"
	#include "libfreenect2/registration.h"
#else // use Microsoft SDK
	#define NOBITMAP
	#include "Kinect.h"
	#undef NOBITMAP
#endif //FREENECT
#endif //KINECT

class Kinect2Node :
	public Node,
	public Thread //process-independant frame grabbing
{
public:
	Kinect2Node(var params = var());
	~Kinect2Node();

	void clearItem() override;

#if USE_KINECT

#if USE_FREENECT

#define K2_DEPTH_WIDTH 512
#define K2_DEPTH_HEIGHT 424
  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = 0;
  libfreenect2::PacketPipeline *pipeline = 0;
  Vector3D<float> points[K2_DEPTH_WIDTH*K2_DEPTH_HEIGHT];
  
  uint32 timeAtLastInit = 0;
  String serial;
#else
	IKinectSensor* kinect;
	ICoordinateMapper* coordinateMapper;

	// Body reader
	IDepthFrameReader* depthReader;
	IColorFrameReader* colorReader;
	CameraSpacePoint* framePoints;    // Maps depth pixels to 3d coordinates
#endif //FREENECT
#endif

	NodeConnectionSlot* outDepth;
	NodeConnectionSlot* outColor;
	NodeConnectionSlot* outCamMatrix;
	NodeConnectionSlot* outDistCoeffs;

	IntParameter* deviceIndex;
	IntParameter* downSample;
	BoolParameter* processDepth;
	BoolParameter* processColor;
	BoolParameter* alignColorToDepth;
	BoolParameter* processOnlyOnNewFrame;

	int depthWidth;
	int depthHeight;
	int colorWidth;
	int colorHeight;

	SpinLock frameLock;
	Image colorImage;

	bool newFrameAvailable;

	bool initInternal() override;

	void processInternal() override;
	void processInternalPassthroughInternal() override;

	void run() override;

	void onContainerParameterChangedInternal(Parameter* p) override;

	Image getPreviewImage() override;

	template<class Interface>
	inline void SafeRelease(Interface*& pInterfaceToRelease)
	{
		if (pInterfaceToRelease != NULL)
		{
			pInterfaceToRelease->Release();
			pInterfaceToRelease = NULL;
		}
	}

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Kinect 2"; }
};