/*
  ==============================================================================

    WebsocketOutputNode.h
    Created: 5 Apr 2022 10:42:37am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class WebsocketOutputNode :
    public Node
{
public:
    WebsocketOutputNode(var params = var());
    ~WebsocketOutputNode();

    void clearItem() override;

    NodeConnectionSlot* inCloudSlot;
    NodeConnectionSlot* inClustersSlot;

    IntParameter* downSample;
    IntParameter* port;

    SimpleWebSocketServer server;

    void initServer();

    void processInternal() override;

    void streamCloud(PCloud cloud);
    void streamClusters(Array<PCloud> clusters);

    void onContainerParameterChangedInternal(Parameter* p) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Websocket Out"; }
};