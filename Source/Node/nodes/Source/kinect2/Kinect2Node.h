/*
  ==============================================================================

	Kinect2Node.h
	Created: 3 May 2022 11:46:19am
	Author:  bkupe

  ==============================================================================
*/

#pragma once
#if !JUCE_WINDOWS
#define USE_KINECT 0
#endif

#ifndef USE_KINECT
#define USE_KINECT 1
#endif

#if USE_KINECT
#define NOBITMAP
#include "Kinect.h"
#undef NOBITMAP
#endif

class Kinect2Node :
	public Node,
	public Thread //process-independant frame grabbing
{
public:
	Kinect2Node(var params = var());
	~Kinect2Node();

	void clearItem() override;

#if USE_KINECT
	IKinectSensor* kinect;
	ICoordinateMapper* coordinateMapper;

	// Body reader
	IDepthFrameReader* depthReader;
	IColorFrameReader* colorReader;
#endif

	NodeConnectionSlot* outDepth;
	NodeConnectionSlot* outColor;
	NodeConnectionSlot* outCamMatrix;
	NodeConnectionSlot* outDistCoeffs;

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
	CameraSpacePoint* framePoints;    // Maps depth pixels to 3d coordinates
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