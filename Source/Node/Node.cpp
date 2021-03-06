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
	processOnlyWhenAllConnectedNodesHaveProcessed(false),
	lastProcessTime(0),
	deltaTime(0),
	processTimeMS(0),
	nodeNotifier(5)
{
	showWarningInUI = true;
	logEnabled = addBoolParameter("Log", "If enabled, this will show log messages for this node", false);
	showServerControls = addBoolParameter("Show Server Controls", "Show controls in web server", true);
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

	if (processOnlyWhenAllConnectedNodesHaveProcessed && !haveAllConnectedInputsProcessed())
	{
		removeNextToProcess();
		return;
	}


	GenericScopedLock lock(processLock);
	uint32 ms = Time::getMillisecondCounter();

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

	uint32 t = Time::getMillisecondCounter();
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

bool Node::haveAllConnectedInputsProcessed()
{
	for (auto& s : inSlots) if (!s->hasConnectedNodeProcessed(true)) return false;
	return true;
}

void Node::onContainerParameterChangedInternal(Parameter* p)
{
	if (p == enabled) notifyServerControlsUpdated();
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
	checkAddNextToProcessForSlot(slot);
}

void Node::receiveClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters)
{
	slotClustersMap.set(slot, clusters);
	checkAddNextToProcessForSlot(slot);
}

void Node::receiveMatrix(NodeConnectionSlot* slot, cv::Mat matrix)
{
	slotMatrixMap.set(slot, matrix);
	checkAddNextToProcessForSlot(slot);
}

void Node::receiveTransform(NodeConnectionSlot* slot, cv::Affine3f transform)
{
	slotTransformMap.set(slot, transform);
	checkAddNextToProcessForSlot(slot);
}

void Node::receiveVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
	slotVectorMap.set(slot, vector);
	checkAddNextToProcessForSlot(slot);
}

void Node::receiveIndices(NodeConnectionSlot* slot, PIndices indices)
{
	slotIndicesMap.set(slot, indices);
	checkAddNextToProcessForSlot(slot);
}

void Node::receiveImage(NodeConnectionSlot* slot, Image image)
{
	slotImageMap.set(slot, image);
	checkAddNextToProcessForSlot(slot);
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
		if(!checkConnectionCanSend(c)) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receivePointCloud(c->dest, cloud);
	}
}

void Node::sendClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if(!checkConnectionCanSend(c)) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveClusters(c->dest, clusters);
	}
}

void Node::sendMatrix(NodeConnectionSlot* slot, cv::Mat matrix)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if(!checkConnectionCanSend(c)) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveMatrix(c->dest, matrix);
	}
}


void Node::sendTransform(NodeConnectionSlot* slot, cv::Affine3f transform)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if(!checkConnectionCanSend(c)) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveTransform(c->dest, transform);
	}
}

void Node::sendVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if(!checkConnectionCanSend(c)) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveVector(c->dest, vector);
	}
}

void Node::sendIndices(NodeConnectionSlot* slot, PIndices indices)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if(!checkConnectionCanSend(c)) continue;
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
		if (!checkConnectionCanSend(c)) continue;
		//if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveImage(c->dest, image);
	}
}

bool Node::checkConnectionCanSend(NodeConnection* c)
{
	return c->enabled->boolValue() && c->dest != nullptr && c->dest->node != nullptr;
}

var Node::getServerControls()
{
	var data = new DynamicObject();
	data.getDynamicObject()->setProperty("enabled", enabled->boolValue());
	data.getDynamicObject()->setProperty("controls", new DynamicObject());
	return data;
}

void Node::notifyServerControlsUpdated()
{
	nodeListeners.call(&NodeListener::serverControlsUpdated, this);
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

void Node::checkAddNextToProcessForSlot(NodeConnectionSlot* slot)
{
	if (!slot->processOnReceive) return;
	if (hasProcessed && processOnlyOnce) return;

	addNextToProcess();
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