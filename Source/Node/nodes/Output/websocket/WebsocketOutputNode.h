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

    Array<NodeConnectionSlot*> inCloudSlots;

    IntParameter* downSample;
    IntParameter* port;
    BoolParameter* forceIds;

    SimpleWebSocketServer server;

    void initServer();

    void processInternal() override;

    void streamCloud(PCloud cloud);
    void streamClusters(Array<PCloud> clusters);

    void onContainerParameterChangedInternal(Parameter* p) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Websocket Out"; }
};