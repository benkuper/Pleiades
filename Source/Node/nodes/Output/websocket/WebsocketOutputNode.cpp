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
	for (int i = 0; i < 4; i++)
	{
		inCloudSlots.add(addSlot("Cloud In "+String(i), true, POINTCLOUD));
	}

	port = addIntParameter("Local Port", "Port to bind the server to", 6060, 1024, 65535);
	downSample = addIntParameter("Downsample", "Simple down sample before sending to the clients, not 2d downsampling, but once every x.", 1, 1, 16);
	forceIds = addBoolParameter("Force IDs", "If checked, this will force ordered ids instead of taking the initial ones", false);

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
	int id = 0;
	for (auto& s : inCloudSlots)
	{
		if (s->isEmpty()) continue;
		PCloud c = slotCloudMap[s];
		if (forceIds->boolValue()) c.id = id++;
		streamCloud(c);
	}
}

void WebsocketOutputNode::streamCloud(PCloud cloud)
{
	if (server.getNumActiveConnections() == 0) return;
	if (cloud.cloud == nullptr) return;

	MemoryOutputStream os;
	os.writeInt(cloud.id);
	int ds = downSample->intValue();
	for (int i = 0; i < cloud.cloud->size(); i += ds)
	{
		auto p = cloud.cloud->points[i];
		os.writeFloat(p.x);
		os.writeFloat(p.y);
		os.writeFloat(p.z);
	}
	server.send((char*)os.getData(), os.getDataSize());
}

void WebsocketOutputNode::streamClusters(Array<PCloud> clusters)
{
	if (server.getNumActiveConnections() == 0) return;
	for (int i = 0; i < clusters.size(); i++) streamCloud(clusters[i]);
}

void WebsocketOutputNode::onContainerParameterChangedInternal(Parameter* p)
{
	if (p == port) initServer();
	else if (p == enabled) initServer();
}
