/*
  ==============================================================================

	NodeConnection.cpp
	Created: 16 Nov 2020 10:00:05am
	Author:  bkupe

  ==============================================================================
*/

NodeConnection::NodeConnection(NodeManager* nodeManager, NodeConnectionSlot* source, NodeConnectionSlot* dest) :
	BaseItem("Connection", false, false),
	nodeManager(nodeManager),
	source(nullptr),
	dest(nullptr),
	connectionType(UNKNOWN),
	connectionNotifier(5)
{
	setSource(source);
	setDest(dest);
}

NodeConnection::~NodeConnection()
{
	clearItem();
	setSource(nullptr);
	setDest(nullptr);
}


void NodeConnection::setSource(NodeConnectionSlot* node)
{
	if (node == source) return;

	if (node != nullptr && dest != nullptr) jassert(node->type == dest->type);

	if (source != nullptr)
	{
		//source->removeNodeListener(this);
		source->node->removeInspectableListener(this);
		source->removeConnection(this);
	}


	source = node;

	if (source != nullptr)
	{
		//source->addNodeListener(this);
		source->node->addInspectableListener(this);
		source->addConnection(this);
		if (connectionType == UNKNOWN) connectionType = source->type;
		else jassert(connectionType == source->type);
	}

	handleNodesUpdated();

	connectionNotifier.addMessage(new ConnectionEvent(ConnectionEvent::SOURCE_CHANGED, this));
}

void NodeConnection::setDest(NodeConnectionSlot* node)
{
	if (node == dest) return;

	if (node != nullptr && source != nullptr) jassert(node->type == source->type);

	if (dest != nullptr)
	{
		//dest->removeNodeListener(this);
		dest->node->removeInspectableListener(this);
		dest->removeConnection(this);
	}

	dest = node;

	if (dest != nullptr)
	{
		//dest->addNodeListener(this);
		dest->node->addInspectableListener(this);
		dest->addConnection(this);
		if (connectionType == UNKNOWN) connectionType = dest->type;
		else jassert(connectionType == dest->type);
	}

	handleNodesUpdated();

	connectionNotifier.addMessage(new ConnectionEvent(ConnectionEvent::DEST_CHANGED, this));
}

void NodeConnection::handleNodesUpdated()
{
	if (isCurrentlyLoadingData) return;

	if (source != nullptr && dest != nullptr)
	{
		setNiceName(source->node->niceName + "." + source->name + " > " + dest->node->niceName + "." + dest->name);
	}
}


void NodeConnection::inspectableDestroyed(Inspectable* i)
{
	if (source != nullptr && i == source->node) setSource(nullptr);
	else if (dest != nullptr && i == dest->node) setSource(nullptr);
}

var NodeConnection::getJSONData()
{
	var data = BaseItem::getJSONData();
	if (source != nullptr)
	{
		data.getDynamicObject()->setProperty("source", source->node->shortName);
		data.getDynamicObject()->setProperty("sourceSlot", source->name);
	}
	if (dest != nullptr)
	{
		data.getDynamicObject()->setProperty("dest", dest->node->shortName);
		data.getDynamicObject()->setProperty("destSlot", dest->name);
	}

	data.getDynamicObject()->setProperty("connectionType", connectionType);
	return data;
}

void NodeConnection::loadJSONDataItemInternal(var data)
{
	if (data.hasProperty("source"))
	{
		if (Node* n = nodeManager->getItemWithName(data.getProperty("source", "")))
		{
			setSource(n->getSlotWithName(data.getProperty("sourceSlot", "").toString(), false));
		}
	}

	if (data.hasProperty("dest"))
	{
		if (Node* n = nodeManager->getItemWithName(data.getProperty("dest", "")))
		{
			setDest(n->getSlotWithName(data.getProperty("destSlot", "").toString(), true));
		}
	}
	loadJSONDataConnectionInternal(data);
}

Colour NodeConnection::getColorForType(NodeConnectionType t)
{
	switch (t)
	{
	case POINTCLOUD: return BLUE_COLOR;
	case CLUSTERS: return GREEN_COLOR;
	case MATRIX: return Colours::orange;
	case INDICES: return YELLOW_COLOR;
	case VECTOR: return Colours::purple;
	case RGB: return RED_COLOR;
	default: break;
	}

	return Colours::black;
}
