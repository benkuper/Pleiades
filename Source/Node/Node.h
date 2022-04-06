/*
  ==============================================================================

	Node.h
	Created: 15 Nov 2020 8:40:03am
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class NodeConnection;
class NodeAudioConnection;
class NodeMIDIConnection;
class BaseNodeViewUI;

class NodeAudioProcessor;

#define NNLOG(t) if(logEnabled->boolValue()) NLOG(niceName, t);


class Node :
	public BaseItem
{
public:
	enum NodeType { SOURCE, FILTER, OUTPUT };

	Node(StringRef name = "Node", NodeType type = SOURCE,
		var params = var());

	virtual ~Node();

	BoolParameter* logEnabled;

	bool isInit = false;

	NodeType type;

	OwnedArray<NodeConnectionSlot, CriticalSection> inSlots;
	OwnedArray<NodeConnectionSlot, CriticalSection> outSlots;

	//Buffer data
	HashMap<NodeConnectionSlot*, CloudPtr> slotCloudMap;
	HashMap<NodeConnectionSlot*, Array<ClusterPtr>> slotClustersMap;
	HashMap<NodeConnectionSlot*, Eigen::Matrix4f> slotMatrixMap;
	HashMap<NodeConnectionSlot*, Eigen::Vector3f> slotVectorMap;
	HashMap<NodeConnectionSlot*, PIndices> slotIndicesMap;

	//process
	SpinLock processLock;

	//Stats
	int processTimeMS;

	virtual void clearItem() override;

	virtual void process();
	void init() { isInit = initInternal(); }

	virtual bool initInternal() { return true; }
	virtual void processInternal() {}
	virtual void processInternalPassthrough() {}

	//Slots
	NodeConnectionSlot* addSlot(StringRef name, bool isInput, NodeConnectionType t);

	//IO
	virtual void receivePointCloud(NodeConnectionSlot* slot, CloudPtr cloud);
	virtual void receiveClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters);
	virtual void receiveMatrix(NodeConnectionSlot* slot, Eigen::Matrix4f matrix);
	virtual void receiveVector(NodeConnectionSlot* slot, Eigen::Vector3f vector);
	virtual void receiveIndices(NodeConnectionSlot* slot, PIndices indices);

	void sendPointCloud(NodeConnectionSlot* slot, CloudPtr cloud);
	void sendClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters);
	void sendMatrix(NodeConnectionSlot* slot, Eigen::Matrix4f matrix);
	void sendVector(NodeConnectionSlot* slot, Eigen::Vector3f vector);
	void sendIndices(NodeConnectionSlot* slot, PIndices indices);

	//Helpers
	NodeConnectionSlot* getSlotWithName(StringRef name, bool isInput);

	void addNextToProcess();
	void removeNextToProcess();

	DECLARE_ASYNC_EVENT(Node, Node, node, ENUM_LIST(INPUTS_CHANGED, OUTPUTS_CHANGED, VIEW_FILTER_UPDATED))
	virtual BaseNodeViewUI* createViewUI();
	WeakReference<Node>::Master masterReference;
};