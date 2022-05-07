#include "Node.h"
/*
  =============================================================================

	 Node.cp
	 Created:15 Novr 200 8:40:03am
	 Author:  bkup

  =============================================================================
*/

Node::Node(StringRef name, NodeType type, var params) :
	BaseItem(name, true),
	type(type),
	isInit(false),
	processOnlyOnce(true),
	hasProcessed(false),
	lastProcessTime(0),
	deltaTime(0),
	processTimeMS(0),
	nodeNotifier(5)
{
	logEnabled = addBoolParameter("Log", "If enabled, this will show log messages for this node", false);
}

Node::~Node()
{
	slotCloudMap.clear();
	masterReference.clear();
}

String Node::getUIInfos()
{
	return String();
}

Image Node::getPreviewImage()
{
	return Image();
}

void Node::clearItem()
{
	GenericScopedLock lock(processLock);
	BaseItem::clearItem();
	inSlots.clear();
	outSlots.clear();
	masterReference.clear();
}

void Node::process()
{
	if (hasProcessed && processOnlyOnce)
	{
		removeNextToProcess();
		return;
	}

	GenericScopedLock lock(processLock);
	long ms = Time::getMillisecondCounter();

	deltaTime = (ms / 1000.0) - lastProcessTime;

	try
	{
		if (!isInit) init();
		if (isInit)
		{
			if (enabled->boolValue()) processInternal();
			else processInternalPassthrough();
		}
	}
	catch (std::exception e)
	{
		NLOGERROR(niceName, "Exception during process :\n" << e.what());
	}

	long t = Time::getMillisecondCounter();
	processTimeMS = t - ms;
	lastProcessTime = (t / 1000.0);

	removeNextToProcess();
	hasProcessed = true;
}

void Node::processInternalPassthrough()
{
	HashMap<NodeConnectionSlot*, NodeConnectionSlot*>::Iterator it(passthroughMap);
	while (it.next())
	{
		NodeConnectionSlot* in = it.getKey();
		NodeConnectionSlot* out = it.getValue();
		jassert(in->type == out->type);
		switch (in->type)
		{
		case NodeConnectionType::POINTCLOUD: sendPointCloud(out, slotCloudMap[in]); break;
		case NodeConnectionType::CLUSTERS: sendClusters(out, slotClustersMap[in]); break;
		case NodeConnectionType::MATRIX: sendMatrix(out, slotMatrixMap[in]); break;
		case NodeConnectionType::TRANSFORM: sendTransform(out, slotTransformMap[in]); break;
		case NodeConnectionType::VECTOR: sendVector(out, slotVectorMap[in]); break;
		case NodeConnectionType::INDICES: sendIndices(out, slotIndicesMap[in]); break;

		}
	}
	processInternalPassthroughInternal();
}

void Node::resetForNextLoop()
{
	clearSlotMaps();
	hasProcessed = false;
}

bool Node::isStartingNode()
{
	return type == SOURCE;
}

NodeConnectionSlot* Node::addSlot(StringRef name, bool isInput, NodeConnectionType t)
{
	jassert(getSlotWithName(name, isInput) == nullptr);
	NodeConnectionSlot* s = new NodeConnectionSlot(this, isInput, name, t);
	if (isInput) inSlots.add(s);
	else outSlots.add(s);
	return s;
}


void Node::receivePointCloud(NodeConnectionSlot* slot, CloudPtr cloud)
{
	slotCloudMap.set(slot, cloud);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}

void Node::receiveClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters)
{
	slotClustersMap.set(slot, clusters);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}

void Node::receiveMatrix(NodeConnectionSlot* slot, cv::Mat matrix)
{
	slotMatrixMap.set(slot, matrix);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}

void Node::receiveTransform(NodeConnectionSlot* slot, cv::Affine3f transform)
{
	slotTransformMap.set(slot, transform);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}

void Node::receiveVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
	slotVectorMap.set(slot, vector);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}

void Node::receiveIndices(NodeConnectionSlot* slot, PIndices indices)
{
	slotIndicesMap.set(slot, indices);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}

void Node::receiveImage(NodeConnectionSlot* slot, Image image)
{
	slotImageMap.set(slot, image);
	if (slot->processOnReceive && (!hasProcessed || !processOnlyOnce)) addNextToProcess();
}


void Node::clearSlotMaps()
{
	slotCloudMap.clear();
	slotClustersMap.clear();
	slotMatrixMap.clear();
	slotVectorMap.clear();
	slotIndicesMap.clear();
}

void Node::sendPointCloud(NodeConnectionSlot* slot, CloudPtr cloud)
{
	if (slot == nullptr) return;
	if (slot->isEmpty()) return;

	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receivePointCloud(c->dest, cloud);
	}
}

void Node::sendClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveClusters(c->dest, clusters);
	}
}

void Node::sendMatrix(NodeConnectionSlot* slot, cv::Mat matrix)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveMatrix(c->dest, matrix);
	}
}


void Node::sendTransform(NodeConnectionSlot* slot, cv::Affine3f transform)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveTransform(c->dest, transform);
	}
}

void Node::sendVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveVector(c->dest, vector);
	}
}

void Node::sendIndices(NodeConnectionSlot* slot, PIndices indices)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveIndices(c->dest, indices);
	}
}

void Node::sendImage(NodeConnectionSlot* slot, Image image)
{
	if (slot == nullptr) return;
	if (slot->isEmpty()) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveImage(c->dest, image);
	}
}

void Node::addInOutSlot(NodeConnectionSlot** in, NodeConnectionSlot** out, NodeConnectionType type, StringRef inName, StringRef outName, bool passthrough)
{
	*in = addSlot(inName, true, type);
	*out = addSlot(outName, false, type);
	if (passthrough) passthroughMap.set(*in, *out);
}

NodeConnectionSlot* Node::getSlotWithName(StringRef name, bool isInput)
{
	OwnedArray<NodeConnectionSlot, CriticalSection>* arr = isInput ? &inSlots : &outSlots;
	for (auto& i : *arr) if (i->name == name) return i;
	return nullptr;
}

void Node::addNextToProcess()
{
	if (Engine::mainEngine->isClearing) return;
	RootNodeManager::getInstance()->nextToProcess.addIfNotAlreadyThere(this);
}

void Node::removeNextToProcess()
{
	if (Engine::mainEngine->isClearing) return;
	RootNodeManager::getInstance()->nextToProcess.removeAllInstancesOf(this);
}

BaseNodeViewUI* Node::createViewUI()
{
	return new BaseNodeViewUI(this);
}