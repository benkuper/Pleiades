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
	forcePauseReadNextFrame(false),
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
	directory->directoryMode = true;

	fileList = addEnumParameter("Cloud Files", "List of available cloud files");
	fileName = addStringParameter("File prefix", "Name of the file, .cloud and .clusters will be appended", "record");

	recordState = addEnumParameter("Record State", "The state of recording / playing");
	recordState->addOption("Idle", IDLE)->addOption("Recording", RECORDING)->addOption("Playing", PLAYING)->addOption("Paused", PAUSED);
	recordState->isSavable = false;
	recordState->setControllableFeedbackOnly(true);

	record = addTrigger("Record", "Start recording the file");
	overwrite = addBoolParameter("Overwrite", "If checked, this will overwrite the record file is one is there. Otherwise recording will do nothing", false);
	play = addTrigger("Play", "Play the file");
	stop = addTrigger("Stop", "Stop the recording or playing depending on the current state");
	pause = addTrigger("Pause", "Pause the recording or playing depending on the current state");
	autoPlay = addBoolParameter("Play at load", "If checked, this will play on load", false);

	loopRange = addPoint2DParameter("Loop Range", "Loop Range");
	loopRange->setBounds(0, 0, 1, 1);
	var val = var();
	val.append(0);
	val.append(1);
	loopRange->setDefaultValue(val);

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

				float loopStart = loopRange->x * totalTime;
				float loopEnd = loopRange->y * totalTime;
				if (loopStart != loopEnd) progression->setValue((curPlayTime - loopStart) / (loopEnd - loopStart));

				changingProgressionFromPlay = false;
			}

			if (cloud != nullptr)
			{
				//GenericScopedLock lock(cloudLock);
				sendPointCloud(outCloud, cloud);
			}

			lastTimeAtPlay = curTime;
		}
		break;

	case PAUSED:
	{
		if (forcePauseReadNextFrame) readNextFrame();
		forcePauseReadNextFrame = false;
		//GenericScopedLock lock(cloudLock);
		sendPointCloud(outCloud, cloud);
	}
	break;
	}
}

bool RecorderNode::readNextFrame()
{
	if (isClearing) return false;
	if (cloudIS == nullptr) return false;

	float loopStart = loopRange->x * totalTime;
	float loopEnd = loopRange->y * totalTime;
	if (loopStart >= loopEnd) return false;


	if (cloudIS->isExhausted() || curPlayTime > loopEnd)
	{
		curPlayTime = loopStart;// Time::getMillisecondCounter() / 1000.;
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


	{		
		//GenericScopedLock lock(cloudLock);
		if (cloud == nullptr) cloud.reset(new Cloud());
		cloud->resize(targetFrameDataSize);
		if (targetFrameDataSize > 0) cloudIS->read(cloud->points.data(), targetFrameDataSize * sizeof(PPoint));
	}
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
			if (numFramesWritten > 0)
			{
				cloudOS->setPosition(0);
				cloudOS->writeInt(numFramesWritten);
				cloudOS->writeFloat(lastRecordedFrameTime);
				if (cloudOS != nullptr) cloudOS->flush();

				LOG("Recorded " << lastRecordedFrameTime << "s in " << numFramesWritten << " frames in file " << cloudFile.getFullPathName());

				String fileName = cloudOS->getFile().getFileName();
				updateFileList();
				fileList->setValue(fileName);

				cloudOS.reset();

			}
		}
		else if (prevState == PLAYING)
		{
			cloudIS.reset();
		}
		break;

	case RECORDING:
		if (prevState == PLAYING || prevState == PAUSED) cloudIS.reset();
		numFramesWritten = 0;

		if (!directory->getFile().exists()) directory->getFile().createDirectory();
		if (cloudFile.exists())
		{
			if (!overwrite->boolValue())  cloudFile = directory->getFile().getNonexistentChildFile(cloudFile.getFileNameWithoutExtension(), ".cloud");
			//else
			//{
			//	NLOGWARNING(niceName, "File already exists and overwrite disabled, not recording");
			//	//setState(IDLE);
			//	return;
		}

		cloudOS.reset(new FileOutputStream(cloudFile));
		if (cloudOS->failedToOpen())
		{
			LOGERROR("Failed to open file " << cloudFile.getFullPathName() << " to record");

			return;
		}
		cloudOS->writeInt(0); //will hold frames Written
		cloudOS->writeFloat(0); //will hold totalTime
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

			frameInfos.clear();
			int frameIndex = 0;
			while (frameIndex < numFramesWritten)
			{
				int posInFile = cloudIS->getPosition();
				FrameInfo f = { cloudIS->readFloat(), posInFile, cloudIS->readInt() };
				frameInfos.add(f);
				cloudIS->setPosition(cloudIS->getPosition() + f.numPoints * sizeof(PPoint));
				frameIndex++;
			}

			cloudIS->setPosition(8);


			LOG("Loading file : " << totalTime << "s, " << numFramesWritten << " frames from " << cloudFile.getFullPathName());
		}


		lastTimeAtPlay = Time::getMillisecondCounter() / 1000.;
		break;

	case PAUSED:
		break;
	}

	recordState->setValueWithData(s);
}

void RecorderNode::updateFileList()
{
	setState(IDLE);
	File dirFile = directory->getFile();
	if (dirFile.exists()) dirFile.createDirectory();

	fileList->clearOptions();
	Array<File> files = dirFile.findChildFiles(File::TypesOfFileToFind::findFiles, false, "*.cloud");
	for (auto& f : files) fileList->addOption(f.getFileNameWithoutExtension(), f.getFileNameWithoutExtension());
}

void RecorderNode::setupFiles()
{
	if (fileName->stringValue().isEmpty()) return;

	File dirFile = directory->getFile();
	String cFileName = fileName->stringValue();
	String clFileName = fileName->stringValue();
	cloudFile = dirFile.getChildFile(cFileName + ".cloud");
	clustersFile = dirFile.getChildFile(cFileName + ".clusters");

}

bool RecorderNode::isStartingNode()
{
	RecordState s = recordState->getValueDataAsEnum<RecordState>();
	if (s == PLAYING || s == PAUSED) return true;

	return false;
}

RecorderNode::FrameInfo RecorderNode::getFrameInfoForTime(float t, bool getNextFrame)
{
	if (frameInfos.size() == 0) return FrameInfo();

	for (int i = 0; i < frameInfos.size(); i++)
	{
		FrameInfo f = frameInfos[i];
		if (f.time == t) return f;
		if (f.time > t)
		{
			if (getNextFrame) return f;
			return frameInfos[i - 1];
		}
	}

	return frameInfos[0];
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
	if (p == directory) updateFileList();
	else if (p == fileList) fileName->setValue(fileList->getValueKey());
	else if (p == fileName) setupFiles();
	else if (p == progression)
	{
		if (!isCurrentlyLoadingData)
		{
			RecordState s = recordState->getValueDataAsEnum<RecordState>();
			if ((s == PLAYING || s == PAUSED) && !changingProgressionFromPlay)
			{
				if (cloudIS != nullptr)
				{
					float loopStart = loopRange->x * totalTime;
					float loopEnd = loopRange->y * totalTime;
					float targetPlayTime = loopStart + progression->floatValue() * (loopEnd - loopStart);

					FrameInfo closestFrame = getFrameInfoForTime(targetPlayTime);

					if (closestFrame.filePosition != 0)
					{
						cloudIS->setPosition(closestFrame.filePosition); //reset read position
						targetFrameTime = closestFrame.time;
						curPlayTime = targetFrameTime;

						if (s == PAUSED)
						{
							forcePauseReadNextFrame = true;
						}
					}
				}
			}
		}
	}
}

void RecorderNode::afterLoadJSONDataInternal()
{
	Node::afterLoadJSONDataInternal();
	if (autoPlay->boolValue()) play->trigger();
}
