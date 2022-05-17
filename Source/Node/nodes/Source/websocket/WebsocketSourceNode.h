/*
  ==============================================================================

	WebsocketSourceNode.h
	Created: 17 May 2022 5:49:27pm
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class WebsocketSourceNode :
	public Node,
	public SimpleWebSocketClientBase::Listener,
	public Timer
{
public:
	WebsocketSourceNode(var params = var());
	~WebsocketSourceNode();

	void clearItem() override;

	Array<NodeConnectionSlot*> outClouds;
	NodeConnectionSlot* outClusters;

	StringParameter* serverHost;
	IntParameter* serverPort;
	FloatParameter* autoClearTime;

	HashMap<int, CloudPtr, DefaultHashFunctions, CriticalSection> clouds;
	HashMap<int, ClusterPtr, DefaultHashFunctions, CriticalSection> clusters;
	HashMap<int, float, DefaultHashFunctions, CriticalSection> idTimeMap;

	std::unique_ptr<SimpleWebSocketClient> client;

	enum ActionType { CLEAR = -1, CLOUD = 0, CLUSTER = 1 };
	enum DataType {
		CloudType = 0,
		ClusterType = 1,
		DebugBoxType = 2,
		DebugPointType = 3,
		DebugLineType = 4,
		DebugPlaneType = 5
	};

	void setupClient();
	void processInternal() override;

	virtual void connectionOpened() override;
	virtual void messageReceived(const String& message) override;
	virtual void dataReceived(const MemoryBlock& data) override;
	virtual void connectionClosed(int status, const String& reason) override;
	virtual void connectionError(const String& message) override;

	void timerCallback() override;

	void afterLoadJSONDataInternal() override;

	void onContainerParameterChangedInternal(Parameter* p) override;

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Websocket Source"; }
};