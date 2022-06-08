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
	HashMap<NodeConnectionSlot*, cv::Mat> slotMatrixMap;
	HashMap<NodeConnectionSlot*, cv::Affine3f> slotTransformMap;
	HashMap<NodeConnectionSlot*, Eigen::Vector3f> slotVectorMap;
	HashMap<NodeConnectionSlot*, PIndices> slotIndicesMap;
	HashMap<NodeConnectionSlot*, Image> slotImageMap;

	HashMap<NodeConnectionSlot*, NodeConnectionSlot*> passthroughMap;

	//process
	SpinLock processLock;
	bool processOnlyOnce;
	bool hasProcessed; //if it has already processed in this frame
	bool processOnlyWhenAllConnectedNodesHaveProcessed;

	//Stats
	double lastProcessTime;
	double deltaTime;
	int processTimeMS;

	//ui image safety
	SpinLock imageLock;

	//ui
	virtual String getUIInfos();
	virtual Image getPreviewImage();

	virtual void clearItem() override;

	virtual void process();
	void init() { isInit = initInternal(); }

	virtual bool initInternal() { return true; }
	virtual void processInternal() {}
	virtual void processInternalPassthrough();
	virtual void processInternalPassthroughInternal() {}

	virtual void resetForNextLoop();
	virtual bool isStartingNode();
	
	virtual bool haveAllConnectedInputsProcessed();

	//Slots
	NodeConnectionSlot* addSlot(StringRef name, bool isInput, NodeConnectionType t);

	//IO
	virtual void receivePointCloud(NodeConnectionSlot* slot, CloudPtr cloud);
	virtual void receiveClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters);
	virtual void receiveMatrix(NodeConnectionSlot* slot, cv::Mat matrix);
	virtual void receiveTransform(NodeConnectionSlot* slot, cv::Affine3f transform);
	virtual void receiveVector(NodeConnectionSlot* slot, Eigen::Vector3f vector);
	virtual void receiveIndices(NodeConnectionSlot* slot, PIndices indices);
	virtual void receiveImage(NodeConnectionSlot* slot, Image indices);


	void clearSlotMaps();

	void sendPointCloud(NodeConnectionSlot* slot, CloudPtr cloud);
	void sendClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters);
	void sendMatrix(NodeConnectionSlot* slot, cv::Mat matrix);
	void sendTransform(NodeConnectionSlot* slot, cv::Affine3f transform);
	void sendVector(NodeConnectionSlot* slot, Eigen::Vector3f vector);
	void sendIndices(NodeConnectionSlot* slot, PIndices indices);
	void sendImage(NodeConnectionSlot* slot, Image indices);

	//Helpers
	void addInOutSlot(NodeConnectionSlot** in, NodeConnectionSlot** out, NodeConnectionType type, StringRef inName = "In", StringRef outName = "out", bool passthrough = true);

	NodeConnectionSlot* getSlotWithName(StringRef name, bool isInput);

	void checkAddNextToProcessForSlot(NodeConnectionSlot* s);
	void addNextToProcess();
	void removeNextToProcess();

	DECLARE_ASYNC_EVENT(Node, Node, node, ENUM_LIST(INPUTS_CHANGED, OUTPUTS_CHANGED, VIEW_FILTER_UPDATED))
	virtual BaseNodeViewUI* createViewUI();
	WeakReference<Node>::Master masterReference;
};