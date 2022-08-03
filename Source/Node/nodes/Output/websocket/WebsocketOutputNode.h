/*
  ==============================================================================

    WebsocketOutputNode.h
    Created: 5 Apr 2022 10:42:37am
    Author:  bkupe

  ==============================================================================
*/

#pragma once


class WebsocketOutputNode :
    public Node,
    public SimpleWebSocketServer::Listener
{
public:
    WebsocketOutputNode(var params = var());
    ~WebsocketOutputNode();

    enum DataType {
        CloudType = 0,
        ClusterType = 1,
        DebugBoxType = 2,
        DebugPointType = 3,
        DebugLineType = 4,
        DebugPlaneType = 5
    };

    enum ControlType {
        Transform = 0,
        BoundingBox = 1
    };

    Array<NodeConnectionSlot*> inClouds;
    Array<NodeConnectionSlot*> inClusters;

    IntParameter* downSample;
    IntParameter* port;
	
	BoolParameter* doStreamClouds;
	BoolParameter* doStreamClusters;
    BoolParameter* streamClusterPoints;
    BoolParameter* sendControls;

    //BoolParameter* invertX;
    //BoolParameter* invertY;
    //BoolParameter* invertZ;

    std::unique_ptr<SimpleWebSocketServer> server;


    void initServer();

    void processInternal() override;

    void streamCloud(CloudPtr cloud, int id);
    void streamClusters(Array<ClusterPtr> clusters);
    void streamCluster(ClusterPtr cluster);

    void sendServerControls(var data = var());

    void connectionOpened(const String& id) override;

    void onContainerParameterChangedInternal(Parameter* p) override;

    void afterLoadJSONDataInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Websocket Out"; }
};
