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

	if (!isInit) init();

	if (isInit)
	{
		if (enabled->boolValue()) processInternal();
		else processInternalPassthrough();
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


void Node::receivePointCloud(NodeConnectionSlot* slot, PCloud cloud)
{
	slotCloudMap.set(slot, cloud);
	if (slot->processOnReceive) addNextToProcess();
}

void Node::receiveClusters(NodeConnectionSlot* slot, Array<PCloud> clusters)
{
}

void Node::receiveMatrix(NodeConnectionSlot* slot, Eigen::Matrix4f matrix)
{
}

void Node::receiveVector(NodeConnectionSlot* slot, Eigen::Vector3f vector)
{
}

void Node::receiveIndices(NodeConnectionSlot* slot, PIndices indices)
{
}

void Node::sendPointCloud(NodeConnectionSlot* slot, PCloud cloud)
{
	if (slot == nullptr) return;

	for (auto& c : slot->connections)
	{
		if (c->dest == nullptr || c->dest->node == nullptr) continue;
		if (!c->dest->node->enabled->boolValue()) continue;
		c->dest->node->receivePointCloud(c->dest, cloud);
	}
}

void Node::sendClusters(NodeConnectionSlot* slot, Array<PCloud> clusters)
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