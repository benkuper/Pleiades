/*
  ==============================================================================

	WebsocketOutputNode.cpp
	Created: 5 Apr 2022 10:42:37am
	Author:  bkupe

  ==============================================================================
*/

WebsocketOutputNode::WebsocketOutputNode(var params) :
	Node(getTypeString(), OUTPUT, params)
{
	for (int i = 0; i < 4; i++) inClouds.add(addSlot("Cloud In " + String(i), true, POINTCLOUD));
	for (int i = 0; i < 2; i++) inClusters.add(addSlot("ClusterIn " + String(i), true, CLUSTERS));

	port = addIntParameter("Local Port", "Port to bind the server to", 6060, 1024, 65535);
	downSample = addIntParameter("Downsample", "Simple down sample before sending to the clients, not 2d downsampling, but once every x.", 1, 1, 16);
	doStreamClouds = addBoolParameter("Stream Clouds", "Stream Clouds", true);
	doStreamClusters = addBoolParameter("Stream Clusters", "Stream Clusters", true);
	streamClusterPoints = addBoolParameter("Stream Cluster Points", "Stream cloud inside clusters", true);

	processOnlyOnce = false;

	initServer();
}

WebsocketOutputNode::~WebsocketOutputNode()
{
	server.stop();
}


void WebsocketOutputNode::initServer()
{
	if (server.isConnected)
	{
		NNLOG("Stopping Server");
		server.stop();
	}

	if (!enabled->boolValue()) return;

	server.rootPath = File::getSpecialLocation(File::userDocumentsDirectory).getChildFile(ProjectInfo::projectName + String("/server"));
	server.start(port->intValue());

	if (server.isConnected)
	{
		NNLOG("Server is running on port " << port->intValue());
	}
	else
	{
		NLOGERROR(niceName, "Error starting server on port " << port->intValue());
	}
}

void WebsocketOutputNode::processInternal()
{
	if (doStreamClouds->boolValue())
	{
		int id = 0;
		for (auto& s : inClouds)
		{
			id++; //always increment to have consistent ids
			if (s->isEmpty()) continue;
			CloudPtr c = slotCloudMap[s];
			if (c == nullptr) continue;
			streamCloud(c, id);
		}
	}


	if (doStreamClusters->boolValue())
	{
		for (auto& s : inClusters)
		{
			if (s->isEmpty()) continue;
			Array<ClusterPtr> c = slotClustersMap[s];
			if (c.isEmpty()) continue;
			streamClusters(c);
		}
	}

	//clear after each send, avoid sending multiple time the same in one frame
	slotCloudMap.clear();
	slotClustersMap.clear();
}

void WebsocketOutputNode::streamCloud(CloudPtr cloud, int id)
{
	if (server.getNumActiveConnections() == 0) return;

	NNLOG("Send cloud " << id << " with " << cloud->size() << " points");

	MemoryOutputStream os;
	os.writeByte(CloudType); //cloud type
	os.writeInt(1000 + id); //write 1000+ id to specify that it doesn't have metadata

	int ds = downSample->intValue();
	for (int i = 0; i < cloud->size(); i += ds)
	{
		auto p = cloud->points[i];
		os.writeFloat(p.x);
		os.writeFloat(p.y);
		os.writeFloat(p.z);
	}
	server.send((char*)os.getData(), os.getDataSize());
}

void WebsocketOutputNode::streamClusters(Array<ClusterPtr> clusters)
{
	if (server.getNumActiveConnections() == 0) return;
	for (int i = 0; i < clusters.size(); i++) streamCluster(clusters[i]);
}

void WebsocketOutputNode::streamCluster(ClusterPtr cluster)
{
	if (server.getNumActiveConnections() == 0) return;
	if (cluster->cloud == nullptr) return;

	bool includeContent = streamClusterPoints->boolValue();
	MemoryOutputStream os;

	NNLOG("Send cluster with id " << cluster->id);

	os.writeByte(ClusterType); //cluster type
	os.writeInt(cluster->id);
	os.writeInt((int)cluster->state); //cluster type

	os.writeFloat(cluster->centroid.x);
	os.writeFloat(cluster->centroid.y);
	os.writeFloat(cluster->centroid.z);
	os.writeFloat(cluster->velocity.x);
	os.writeFloat(cluster->velocity.y);
	os.writeFloat(cluster->velocity.z);
	os.writeFloat(cluster->boundingBoxMin.x);
	os.writeFloat(cluster->boundingBoxMin.y);
	os.writeFloat(cluster->boundingBoxMin.z);
	os.writeFloat(cluster->boundingBoxMax.x);
	os.writeFloat(cluster->boundingBoxMax.y);
	os.writeFloat(cluster->boundingBoxMax.z);

	if (includeContent)
	{
		int ds = downSample->intValue();
		for (int i = 0; i < cluster->cloud->size(); i += ds)
		{
			auto p = cluster->cloud->points[i];
			os.writeFloat(p.x);
			os.writeFloat(p.y);
			os.writeFloat(p.z);
		}
	}
	server.send((char*)os.getData(), os.getDataSize());
}

void WebsocketOutputNode::onContainerParameterChangedInternal(Parameter* p)
{
	if (p == port) initServer();
	else if (p == enabled) initServer();
}
