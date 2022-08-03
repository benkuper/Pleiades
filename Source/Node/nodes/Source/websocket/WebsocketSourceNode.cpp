/*
  ==============================================================================

	WebsocketSourceNode.cpp
	Created: 17 May 2022 5:49:27pm
	Author:  bkupe

  ==============================================================================
*/

WebsocketSourceNode::WebsocketSourceNode(var params) :
	Node(getTypeString(), Node::SOURCE, params)
{
	for (int i = 0; i < 3; i++) outClouds.add(addSlot("Out Cloud " + String(i + 1), false, POINTCLOUD));
	outClusters = addSlot("Out Clusters", false, CLUSTERS);

	serverHost = addStringParameter("Server host", "IP of the websocket server", "127.0.0.1");
	serverPort = addIntParameter("Server port", "Port of the websocket server", 6060);
	autoClearTime = addFloatParameter("Auto-clear time", "Time to keep a cloud or cluster that has not been updated for a while", 1);

	processOnlyWhenAllConnectedNodesHaveProcessed = true;

	if (!Engine::mainEngine->isLoadingFile) setupClient();
}

WebsocketSourceNode::~WebsocketSourceNode()
{
}

void WebsocketSourceNode::clearItem()
{
	Node::clearItem();
	clearWarning();
	if (client != nullptr) client->stop();
	client.reset();
}


void WebsocketSourceNode::setupClient()
{
	if (client != nullptr) client->stop();
	client.reset();
	if (isCurrentlyLoadingData) return;

	if (!enabled->intValue()) return;

	client.reset(new SimpleWebSocketClient());
	client->addWebSocketListener(this);

	String serverPath = serverHost->stringValue() + ":" + serverPort->stringValue();
	client->start(serverPath);

}

void WebsocketSourceNode::processInternal()
{
	int i = 0;
	HashMap<int, CloudPtr, DefaultHashFunctions, CriticalSection>::Iterator cit(clouds);

	float curTime = Time::getMillisecondCounter() / 1000.0f;

	Array<int> cloudsToRemove;
	while (cit.next())
	{
		if (i >= outClouds.size()) break;
		sendPointCloud(outClouds[i], cit.getValue());
		if (curTime - idTimeMap[cit.getKey()] > 1) cloudsToRemove.add(cit.getKey());
	}

	HashMap<int, ClusterPtr, DefaultHashFunctions, CriticalSection>::Iterator clit(clusters);

	Array<ClusterPtr> cList;
	Array<int> clustersToRemove;
	while (clit.next())
	{
		cList.add(clit.getValue());
		if (clit.getValue()->state == Cluster::WILL_LEAVE || curTime - clit.getValue()->lastUpdateTime > 1)
		{
			clit.getValue()->state = Cluster::WILL_LEAVE;
			clustersToRemove.add(clit.getKey());
		}
	}
	sendClusters(outClusters, cList);


	//cleanup
	for (auto& cid : cloudsToRemove)
	{
		clouds.remove(cid);
		idTimeMap.remove(cid);
	}

	for (auto& clid : clustersToRemove) clusters.remove(clid);
}

void WebsocketSourceNode::connectionOpened()
{
	NLOG(niceName, "Connected to websocket server at " << client->serverPath);
	stopTimer();
}

void WebsocketSourceNode::messageReceived(const String& message)
{
}

void WebsocketSourceNode::dataReceived(const MemoryBlock& data)
{
	MemoryInputStream is(data, false);

	int type = is.readByte();

	if (type == CLEAR)
	{
		clouds.clear();
		clusters.clear();
		idTimeMap.clear();
	}

	int id = is.readInt();

	float curTime = Time::getMillisecondCounter() / 1000.0f;

	switch (type)
	{
	case CLOUD:
	{
		if (!clouds.contains(id)) clouds.set(id, CloudPtr(new Cloud()));
		CloudPtr cloud = clouds[id];

		idTimeMap.set(id, curTime);

		int numPoints = is.getNumBytesRemaining() / 12;

		cloud->resize(numPoints);
		for (int i = 0; i < numPoints; i++)
		{
			float tx = is.readFloat();
			float ty = is.readFloat();
			float tz = is.readFloat();
			cloud->at(i) = PPoint(tx,ty,tz);
		}
		NNLOG("Received cloud " << id << ", num points : " << (int)cloud->size());
	}
	break;

	case CLUSTER:
	{
		if (!clusters.contains(id)) clusters.set(id, ClusterPtr(new Cluster(id, CloudPtr(new Cloud()))));
		ClusterPtr cluster = clusters[id];

		cluster->state = (Cluster::State)is.readInt();

		cluster->centroid.x = is.readFloat();
		cluster->centroid.y = is.readFloat();
		cluster->centroid.z = is.readFloat();
		cluster->velocity.x = is.readFloat();
		cluster->velocity.y = is.readFloat();
		cluster->velocity.z = is.readFloat();
		cluster->boundingBoxMin.x = is.readFloat();
		cluster->boundingBoxMin.y = is.readFloat();
		cluster->boundingBoxMin.z = is.readFloat();
		cluster->boundingBoxMax.x = is.readFloat();
		cluster->boundingBoxMax.y = is.readFloat();
		cluster->boundingBoxMax.z = is.readFloat();

		cluster->lastUpdateTime = curTime;

		NNLOG("Received cluster " << cluster->id << ", state : " << (int)cluster->state << ", num points : " << (int)cluster->cloud->size());

		int numPoints = is.getNumBytesRemaining() / 12;
		cluster->cloud->resize(numPoints);
		for (int i = 0; i < numPoints; i++)
		{
			float x = is.readFloat();
			float y = is.readFloat();
			float z = is.readFloat();
			cluster->cloud->at(i) = PPoint(x, y, z);
		}

	}
	break;
	}
}

void WebsocketSourceNode::connectionClosed(int status, const String& reason)
{
	NLOGWARNING(niceName, "Connection closed (" << status << ") : " << reason);
	startTimerHz(1);
}

void WebsocketSourceNode::connectionError(const String& message)
{
	if (getWarningMessage().isEmpty())
	{
		NLOGERROR(niceName, "Could not connect to " << client->serverPath);
		setWarningMessage("Could not connect to " + client->serverPath);
	}

	startTimerHz(1);
}

void WebsocketSourceNode::timerCallback()
{
	if (client == nullptr || !client->isConnected) setupClient();
}

void WebsocketSourceNode::afterLoadJSONDataInternal()
{
	setupClient();
}

void WebsocketSourceNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);

	if (!isCurrentlyLoadingData)
	{
		if (p == enabled || p == serverHost || p == serverPort)
		{
			clearWarning();
			setupClient();
		}
	}
}