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
	StringParameter* fileName;

	
	Trigger* record;
	Trigger* play;
	Trigger* pause;
	Trigger* stop;
	enum RecordState { IDLE, RECORDING, PLAYING, PAUSED };
	EnumParameter* recordState;

	FloatParameter* progression;

	File cloudFile;
	std::unique_ptr<FileInputStream> cloudIS;
	std::unique_ptr<FileOutputStream> cloudOS;

	File clustersFile;
	std::unique_ptr<FileInputStream> clustersIS;
	std::unique_ptr<FileOutputStream> clustersOS;

	CloudPtr cloud;

	bool changingProgressionFromPlay;
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

	void setupFiles();

	bool isStartingNode() override;


	void onContainerTriggerTriggered(Trigger* t) override;
	void onContainerParameterChangedInternal(Parameter* p) override;

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Recorder"; }
};