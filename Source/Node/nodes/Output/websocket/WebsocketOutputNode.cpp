#include "WebsocketOutputNode.h"
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
	inCloudSlot = addSlot("Cloud In", true, POINTCLOUD);
	inClustersSlot = addSlot("Clusters In", true, CLUSTER);

	port = addIntParameter("Local Port", "Port to bind the server to", 6060, 1024, 65535);
	downSample = addIntParameter("Downsample", "Simple down sample before sending to the clients, not 2d downsampling, but once every x.", 1, 1, 16);

	initServer();
}

WebsocketOutputNode::~WebsocketOutputNode()
{
	server.stop();
}

void WebsocketOutputNode::clearItem()
{
}

void WebsocketOutputNode::initServer()
{
	server.stop();

	server.rootPath = File::getSpecialLocation(File::userDocumentsDirectory).getChildFile(ProjectInfo::projectName + String("/server"));
	server.start(port->intValue());

	if (server.isConnected)
	{
		NLOG(niceName, "Server is running on port " << port->intValue());
	}
}

void WebsocketOutputNode::processInternal()
{
	if (slotCloudMap.contains(inCloudSlot))
	{
		streamCloud(slotCloudMap[inCloudSlot]);
	}
}


void WebsocketOutputNode::streamCloud(PCloud cloud)
{
	if (server.getNumActiveConnections() == 0) return;

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
	for (int i = 0; i < clusters.size(); i++)
	{
		streamCloud(clusters[i]);
	}
}

void WebsocketOutputNode::onContainerParameterChangedInternal(Parameter* p)
{
	if (p == port) initServer();
}
