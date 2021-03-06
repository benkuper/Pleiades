/*
  ==============================================================================

	RecorderNode.h
	Created: 3 May 2022 6:11:40pm
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class CloudRecordData
{
	CloudPtr cloud;
	double time;
};

class ClusterRecordData
{
	Array<ClusterPtr> clusters;
	double time;
};

class RecorderNode :
	public Node
{
public:
	RecorderNode(var params = var());
	~RecorderNode();

	NodeConnectionSlot* inCloud;
	NodeConnectionSlot* outCloud;

	NodeConnectionSlot* inClusters;
	NodeConnectionSlot* outClusters;

	FileParameter* directory;
	EnumParameter* fileList;
	StringParameter* fileName;
	
	Trigger* record;
	BoolParameter* overwrite;

	Trigger* play;
	Trigger* pause;
	Trigger* stop;

	BoolParameter* autoPlay;

	enum RecordState { IDLE, RECORDING, PLAYING, PAUSED };
	EnumParameter* recordState;

	Point2DParameter* loopRange;
	FloatParameter* progression;

	File cloudFile;
	std::unique_ptr<FileInputStream> cloudIS;
	std::unique_ptr<FileOutputStream> cloudOS;

	File clustersFile;
	std::unique_ptr<FileInputStream> clustersIS;
	std::unique_ptr<FileOutputStream> clustersOS;

	CloudPtr cloud;

	SpinLock cloudLock;

	struct FrameInfo
	{
		float time = 0;
		int filePosition = 0;
		int numPoints = 0;
	};

	Array<FrameInfo> frameInfos;

	bool changingProgressionFromPlay;
	bool forcePauseReadNextFrame;
	double timeAtRecord;
	double lastRecordedFrameTime;
	double lastTimeAtPlay;

	double curPlayTime;

	double totalTime;
	int numFramesWritten;
	
	float targetFrameTime;
	//PPoint* targetFrameData;
	int targetFrameDataSize;

	SpinLock stateLock;

	void processInternal() override;
	bool readNextFrame();

	void setState(RecordState s);

	void updateFileList();
	void setupFiles();

	bool isStartingNode() override;

	FrameInfo getFrameInfoForTime(float t, bool getNextFrame = false);

	void onContainerTriggerTriggered(Trigger* t) override;
	void onContainerParameterChangedInternal(Parameter* p) override;

	void afterLoadJSONDataInternal() override;

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Recorder"; }
};