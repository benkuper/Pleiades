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
	cloudIS(nullptr),
	cloudOS(nullptr),
	clustersIS(nullptr),
	clustersOS(nullptr),
	targetFrameData(nullptr)

{
	addInOutSlot(&inCloud, &outCloud, POINTCLOUD, "In Cloud", "Out Cloud");
	addInOutSlot(&inClusters, &outClusters, CLUSTERS, "In Clusters", " Out Clusters");

	directory = addFileParameter("Directory", "Folder to record/load data to/from");

	fileName = addStringParameter("File prefix", "Name of the file, .cloud and .clusters will be appended", "record");

	recordState = addEnumParameter("Record State", "The state of recording / playing");
	recordState->addOption("Idle", IDLE)->addOption("Recording", RECORDING)->addOption("Playing", PLAYING)->addOption("Paused", PAUSED);
	record = addTrigger("Record", "Start recording the file");
	play = addTrigger("Play", "Play the file");
	stop = addTrigger("Stop", "Stop the recording or playing depending on the current state");
	pause = addTrigger("Pause", "Pause the recording or playing depending on the current state");

	directory->setDefaultValue(File::getSpecialLocation(File::userDocumentsDirectory).getChildFile(String(ProjectInfo::projectName) + "/records").getFullPathName());
}

RecorderNode::~RecorderNode()
{
	free(targetFrameData);
}


void RecorderNode::processInternal()
{
	CloudPtr cloudSource = slotCloudMap[inCloud];
	Array<ClusterPtr> clusterSources = slotClustersMap[inClusters];

	GenericScopedLock lock(stateLock);

	double relTime = Time::getMillisecondCounter() / 1000. - timeAtRecordPlay;

	RecordState s = recordState->getValueDataAsEnum<RecordState>();
	switch (s)
	{
	case IDLE:
		sendPointCloud(outCloud, cloudSource);
		break;

	case RECORDING:

		if (cloudOS != nullptr)
		{
			//DBG("Write new frame at " << cloudOS->getPosition());
			cloudOS->writeFloat(relTime);
			cloudOS->writeInt(cloudSource->size());
			cloudOS->write(cloudSource->points.data(), cloudSource->size() * sizeof(PPoint));
			//DBG(" > frame time : " << relTime << ", num points : " << cloudSource->size());
			numFramesWritten++;
		}

		sendPointCloud(outCloud, cloudSource);
		break;

	case PLAYING:
		if (cloudIS != nullptr)
		{
			if (readNextFrame(relTime))
			{
				CloudPtr cloud(new Cloud());
				cloud->resize(targetFrameDataSize);
				memcpy(cloud->points.data(), targetFrameData, targetFrameDataSize * sizeof(PPoint));
				sendPointCloud(outCloud, cloud);
			}
			break;

	case PAUSED:
		break;
		}
	}
}

bool RecorderNode::readNextFrame(float time)
{
	if (isClearing) return false;
	if (cloudIS == nullptr) return false;

	if (cloudIS->isExhausted())
	{
		timeAtRecordPlay = Time::getMillisecondCounter() / 1000.;
		time = 0;
		targetFrameTime = -1;
		cloudIS->setPosition(4); //after num frames
	}

	if (targetFrameTime > time) return false;

	//DBG("Read next frame at position " << cloudIS->getPosition());
	targetFrameTime = cloudIS->readFloat();
	targetFrameDataSize = cloudIS->readInt();

	//DBG(" > frame time : " << targetFrameTime << ", num points " << targetFrameDataSize);

	if (targetFrameTime < time)
	{
		cloudIS->setPosition(cloudIS->getPosition() + targetFrameDataSize * sizeof(PPoint));
		return readNextFrame(time);
	}

	//DBG("Cool here");
	targetFrameData = (PPoint*)malloc(targetFrameDataSize * sizeof(PPoint));
	cloudIS->read(targetFrameData, targetFrameDataSize * sizeof(PPoint));

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
			DBG("Cloud OS size " << cloudFile.getSize());
			cloudOS->setPosition(0);
			cloudOS->writeInt(numFramesWritten);
			if (cloudOS != nullptr) cloudOS->flush();
			cloudOS.reset();
			DBG("Cloud OS size after write Int " << cloudFile.getSize());

			LOG("Recorded " << numFramesWritten << " frames in file " << cloudFile.getFullPathName());
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
		numFramesWritten = 0;
		timeAtRecordPlay = Time::getMillisecondCounter() / 1000.;
		break;

	case PLAYING:
		if (cloudOS != nullptr) cloudOS->flush();
		cloudOS.reset();

		cloudIS.reset(new FileInputStream(cloudFile));
		if (cloudIS->failedToOpen())
		{
			LOGERROR("Failed to open file " << cloudFile.getFullPathName() << " to record");
			return;
		}

		numFramesWritten = cloudIS->readInt();
		targetFrameTime = -1;

		LOG("Loading " << numFramesWritten << " frames from file " << cloudFile.getFullPathName());
		timeAtRecordPlay = Time::getMillisecondCounter() / 1000.;
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
}
