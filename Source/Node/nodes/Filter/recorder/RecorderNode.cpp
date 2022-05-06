/*
  ==============================================================================

	RecorderNode.cpp
	Created: 3 May 2022 6:11:40pm
	Author:  bkupe

  ==============================================================================
*/

RecorderNode::RecorderNode(var params) :
	Node(getTypeString(), FILTER, params),
	changingProgressionFromPlay(false),
	lastRecordedFrameTime(0),
	curPlayTime(0),
	totalTime(0),
	cloudIS(nullptr),
	cloudOS(nullptr),
	clustersIS(nullptr),
	clustersOS(nullptr)

{
	addInOutSlot(&inCloud, &outCloud, POINTCLOUD, "In Cloud", "Out Cloud");
	addInOutSlot(&inClusters, &outClusters, CLUSTERS, "In Clusters", " Out Clusters");

	directory = addFileParameter("Directory", "Folder to record/load data to/from");

	fileName = addStringParameter("File prefix", "Name of the file, .cloud and .clusters will be appended", "record");

	recordState = addEnumParameter("Record State", "The state of recording / playing");
	recordState->addOption("Idle", IDLE)->addOption("Recording", RECORDING)->addOption("Playing", PLAYING)->addOption("Paused", PAUSED);
	recordState->isSavable = false;
	
	record = addTrigger("Record", "Start recording the file");
	play = addTrigger("Play", "Play the file");
	stop = addTrigger("Stop", "Stop the recording or playing depending on the current state");
	pause = addTrigger("Pause", "Pause the recording or playing depending on the current state");

	progression = addFloatParameter("Progression", "Progression of the playback", 0, 0, 1);
	progression->isSavable = false;

	directory->setDefaultValue(File::getSpecialLocation(File::userDocumentsDirectory).getChildFile(String(ProjectInfo::projectName) + "/records").getFullPathName());
}

RecorderNode::~RecorderNode()
{
	//if(targetFrameData != nullptr) free(targetFrameData);
}


void RecorderNode::processInternal()
{
	CloudPtr cloudSource = slotCloudMap[inCloud];
	Array<ClusterPtr> clusterSources = slotClustersMap[inClusters];

	GenericScopedLock lock(stateLock);


	double curTime = Time::getMillisecondCounter() / 1000.;

	RecordState s = recordState->getValueDataAsEnum<RecordState>();
	switch (s)
	{
	case IDLE:
		sendPointCloud(outCloud, cloudSource);
		break;

	case RECORDING:

		if (cloudOS != nullptr)
		{
			double relTime = curTime - timeAtRecord;
			LOG("Write new frame at " << relTime << ", pos : " << cloudOS->getPosition());
			cloudOS->writeFloat(relTime);
			cloudOS->writeInt(cloudSource->size());
			cloudOS->write(cloudSource->points.data(), cloudSource->size() * sizeof(PPoint));
			//DBG(" > frame time : " << relTime << ", num points : " << cloudSource->size());
			lastRecordedFrameTime = relTime;
			numFramesWritten++;
		}

		sendPointCloud(outCloud, cloudSource);
		break;

	case PLAYING:
		if (cloudIS != nullptr)
		{
			curPlayTime += curTime - lastTimeAtPlay;
			
			
			if (readNextFrame())
			{

				changingProgressionFromPlay = true;
				progression->setValue(curPlayTime / totalTime);
				changingProgressionFromPlay = false;
				sendPointCloud(outCloud, cloud);
			}

			lastTimeAtPlay = curTime;
		}
		break;

	case PAUSED:
		sendPointCloud(outCloud, cloud);
		break;
	}
}

bool RecorderNode::readNextFrame()
{
	if (isClearing) return false;
	if (cloudIS == nullptr) return false;

	if (cloudIS->isExhausted() || curPlayTime > totalTime)
	{
		curPlayTime = 0;// Time::getMillisecondCounter() / 1000.;
		targetFrameTime = -1;
		cloudIS->setPosition(8); //after num frames and total time
	}

	if (targetFrameTime > curPlayTime) return false;

	//DBG("Read next frame at position " << cloudIS->getPosition());
	targetFrameTime = cloudIS->readFloat();
	targetFrameDataSize = cloudIS->readInt();

	//DBG(" > frame curPlayTime : " << targetFrameTime << ", num points " << targetFrameDataSize);

	if (targetFrameTime < curPlayTime)
	{
		cloudIS->setPosition(cloudIS->getPosition() + targetFrameDataSize * sizeof(PPoint));
		return readNextFrame();
	}

	if (cloud == nullptr) cloud.reset(new Cloud());
	cloud->resize(targetFrameDataSize);
	//memcpy(cloud->points.data(), targetFrameData, targetFrameDataSize * sizeof(PPoint));
	//targetFrameData = (PPoint*)malloc(targetFrameDataSize * sizeof(PPoint));
	cloudIS->read(cloud->points.data(), targetFrameDataSize * sizeof(PPoint));

	return true;
}

void RecorderNode::setState(RecordState s)
{
	GenericScopedLock lock(stateLock);
	RecordState prevState = recordState->getValueDataAsEnum<RecordState>();
	if (s == prevState) return;

	switch (s)
	{
	case IDLE:
		if (prevState == RECORDING)
		{
			cloudOS->setPosition(0);
			cloudOS->writeInt(numFramesWritten);
			cloudOS->writeFloat(lastRecordedFrameTime);
			if (cloudOS != nullptr) cloudOS->flush();
			cloudOS.reset();

			LOG("Recorded " << lastRecordedFrameTime << "s in " << numFramesWritten << " frames in file " << cloudFile.getFullPathName());
		}
		else if (prevState == PLAYING)
		{
			cloudIS.reset();
		}
		break;

	case RECORDING:
		if (prevState == PLAYING || prevState == PAUSED) cloudIS.reset();

		if (!directory->getFile().exists()) directory->getFile().createDirectory();

		if (cloudFile.exists()) cloudFile.deleteFile();

		cloudOS.reset(new FileOutputStream(cloudFile));
		if (cloudOS->failedToOpen())
		{
			LOGERROR("Failed to open file " << cloudFile.getFullPathName() << " to record");
			return;
		}
		cloudOS->writeInt(0); //will hold frames Written
		cloudOS->writeFloat(0); //will hold totalTime
		numFramesWritten = 0;
		timeAtRecord = Time::getMillisecondCounter() / 1000.;
		break;

	case PLAYING:
		if (prevState == RECORDING)
		{
			setState(IDLE);
			setState(PLAYING);
		}

		if (prevState == IDLE)
		{
			if (cloudOS != nullptr) cloudOS->flush();
			cloudOS.reset();

			cloudIS.reset(new FileInputStream(cloudFile));
			if (cloudIS->failedToOpen())
			{
				LOGERROR("Failed to open file " << cloudFile.getFullPathName() << " to record");
				return;
			}
			numFramesWritten = cloudIS->readInt();
			totalTime = cloudIS->readFloat();
			targetFrameTime = -1;
			LOG("Loading file : " << totalTime << "s, " << numFramesWritten << " frames from " << cloudFile.getFullPathName());
		}


		lastTimeAtPlay = Time::getMillisecondCounter() / 1000.;
		break;

	case PAUSED:
		break;
	}

	recordState->setValueWithData(s);
}

void RecorderNode::setupFiles()
{
	setState(IDLE);
	cloudFile = directory->getFile().getChildFile(fileName->stringValue() + ".cloud");
	clustersFile = directory->getFile().getChildFile(fileName->stringValue() + ".clusters");
}

bool RecorderNode::isStartingNode()
{
	RecordState s = recordState->getValueDataAsEnum<RecordState>();
	if (s == PLAYING || s == PAUSED) return true;

	return false;
}

void RecorderNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
	if (t == record) setState(RECORDING);
	else if (t == play) setState(PLAYING);
	else if (t == pause) setState(PAUSED);
	else if (t == stop) setState(IDLE);
}

void RecorderNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
	if (p == directory || p == fileName)
	{
		setupFiles();
	}
	else if (p == progression)
	{
		if (!isCurrentlyLoadingData)
		{
			RecordState s = recordState->getValueDataAsEnum<RecordState>();
			if ((s == PLAYING || s == PAUSED) && !changingProgressionFromPlay)
			{
				if (cloudIS != nullptr)
				{
					curPlayTime = progression->floatValue() * totalTime;
					cloudIS->setPosition(8); //reset read position
					targetFrameTime = curPlayTime;

					if (s == PAUSED)
					{
						readNextFrame();
					}
				}
			}
		}
	}
}
