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

	GenericScopedLock lock(processLock);
	long ms = Time::getMillisecondCounter();

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

	processTimeMS = Time::getMillisecondCounter() - ms;
	removeNextToProcess();
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
	if (slot->processOnReceive) addNextToProcess();
}

void Node::receiveClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters)
{
	slotClustersMap.set(slot, clusters);
	if (slot->processOnReceive) addNextToProcess();
}

void Node::receiveMatrix(NodeConnectionSlot* slot, Eigen::Matrix4f matrix)
{
	slotMatrixMap.set(slot, matrix);
	if (slot->processOnReceive) addNextToProcess();
}

void Node::receiveVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
	slotVectorMap.set(slot, vector);
	if (slot->processOnReceive) addNextToProcess();
}

void Node::receiveIndices(NodeConnectionSlot* slot, PIndices indices)
{
	slotIndicesMap.set(slot, indices);
	if (slot->processOnReceive) addNextToProcess();
}

void Node::sendPointCloud(NodeConnectionSlot* slot, CloudPtr cloud)
{
	if (slot == nullptr) return;

	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receivePointCloud(c->dest, cloud);
	}
}

void Node::sendClusters(NodeConnectionSlot* slot, Array<ClusterPtr> clusters)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveClusters(c->dest, clusters);
	}
}

void Node::sendMatrix(NodeConnectionSlot* slot, Eigen::Matrix4f matrix)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveMatrix(c->dest, matrix);
	}
}

void Node::sendVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveVector(c->dest, vector);
	}
}

void Node::sendIndices(NodeConnectionSlot* slot, PIndices indices)
{
	if (slot == nullptr) return;
	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receiveIndices(c->dest, indices);
	}
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