/*
  ==============================================================================

    KinectAzureNode.h
    Created: 20 May 2022 1:13:11am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

#ifndef USE_AZURE
#if !JUCE_WINDOWS
#define USE_AZURE 0
#endif
#endif

#ifndef USE_AZURE
#define USE_AZURE 1
#endif

#if USE_AZURE
#include <k4a/k4a.hpp>
#endif

class KinectAzureNode :
	public Node,
	public Thread //process-independant frame grabbing
{
public:
	KinectAzureNode(var params = var());
	~KinectAzureNode();

	void clearItem() override;

#if USE_AZURE
	k4a::device device;
	k4a::transformation transformation;
	k4a::image pointCloudImage;
	int16* pointCloudBuffer;
#endif

	NodeConnectionSlot* outDepth;
	NodeConnectionSlot* outColor;
	NodeConnectionSlot* outCamMatrix;
	NodeConnectionSlot* outDistCoeffs;

	IntParameter* deviceIndex;
	EnumParameter* depthMode;
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
	uint32 timeAtLastInit;

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
	static String getTypeStringStatic() { return "Kinect Azure"; }
};