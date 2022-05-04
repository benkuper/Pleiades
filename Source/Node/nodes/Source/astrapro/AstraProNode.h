/*
  ==============================================================================

	AstraProNode.h
	Created: 4 May 2022 11:22:53am
	Author:  bkupe

  ==============================================================================
*/

#pragma once
#pragma warning(push)
#pragma warning(disable : 4324 4201)
#include <astra/astra.hpp>

#pragma warning(pop)

class AstraProNode :
	public Node,
	public Thread //process-independant frame grabbing
{
public:
	AstraProNode(var params = var());
	~AstraProNode();

	void clearItem() override;


	std::unique_ptr<astra::StreamSet> streamSet;
	astra::StreamReader reader;
	astra::CoordinateMapper * mapper;

	NodeConnectionSlot* outDepth;
	NodeConnectionSlot* outColor;

	IntParameter* downSample;
	BoolParameter* processDepth;
	BoolParameter* processColor;
	BoolParameter* alignDepthToColor;
	BoolParameter* processOnlyOnNewFrame;


	SpinLock frameLock;
	int depthWidth;
	int depthHeight;
	astra::Vector3f* pointsData;

	Image colorImage;

	bool newFrameAvailable;

	bool initInternal() override;

	void processInternal() override;
	void processInternalPassthroughInternal() override;

	void run() override;

	void onContainerParameterChangedInternal(Parameter* p) override;

	Image getPreviewImage() override;

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Astra Pro"; }
};