/*
  ==============================================================================

	BaseNodeViewUI.cpp
	Created: 15 Nov 2020 9:26:57am
	Author:  bkupe

  ==============================================================================
*/

juce_ImplementSingleton(ViewStatsTimer);

BaseNodeViewUI::BaseNodeViewUI(Node* node) :
	BaseItemUI(node, Direction::ALL, true)
{
	dragAndDropEnabled = true;
	autoHideWhenDragging = false;
	drawEmptyDragIcon = true;
	showRemoveBT = false;

	statsLabel.setFont(10);
	statsLabel.setColour(statsLabel.textColourId, NORMAL_COLOR.brighter());
	statsLabel.setOpaque(false);
	statsLabel.setJustificationType(Justification::centredRight);
	addAndMakeVisible(statsLabel);

	updateConnectors();
	item->addAsyncNodeListener(this);

	ViewStatsTimer::getInstance()->addListener(this);

}

BaseNodeViewUI::~BaseNodeViewUI()
{
	if (!inspectable.wasObjectDeleted()) item->removeAsyncNodeListener(this);
	if (ViewStatsTimer* vs = ViewStatsTimer::getInstanceWithoutCreating()) vs->removeListener(this);
}

NodeConnector* BaseNodeViewUI::getConnectorForSlot(NodeConnectionSlot* s)
{
	OwnedArray<NodeConnector>* arr = s->isInput ? &inConnectors : &outConnectors;
	for (auto& c : *arr) if (c->slot == s) return c;
	return nullptr;
}

Array<NodeConnector*> BaseNodeViewUI::getConnectorsForType(NodeConnectionType t, bool isInput)
{
	Array<NodeConnector*> result;

	OwnedArray<NodeConnector>* arr = isInput ? &inConnectors : &outConnectors;
	for (auto& c : *arr) if (c->slot->type == t) result.add(c);

	return result;
}

void BaseNodeViewUI::updateConnectors()
{
	inConnectors.clear();

	for (auto& i : item->inSlots)
	{
		NodeConnector* c = new NodeConnector(i, this);
		inConnectors.add(c);
		addAndMakeVisible(c);
	}

	outConnectors.clear();

	for (auto& i : item->outSlots)
	{
		NodeConnector* c = new NodeConnector(i, this);
		outConnectors.add(c);
		addAndMakeVisible(c);
	}
}

void BaseNodeViewUI::paint(Graphics& g)
{
	BaseItemUI::paint(g);

}

void BaseNodeViewUI::paintOverChildren(Graphics& g)
{
	BaseItemUI::paintOverChildren(g);
}

void BaseNodeViewUI::resized()
{
	BaseItemUI::resized();

	int w = 10;
	Rectangle<int> inR = getLocalBounds().removeFromLeft(w).reduced(0, 10);
	Rectangle<int> outR = getLocalBounds().removeFromRight(w).reduced(0, 10);

	//add some space on top of connectors
	inR.removeFromTop(20);
	outR.removeFromTop(20);

	for (auto& i : inConnectors)
	{
		i->setBounds(inR.removeFromTop(w));
		inR.removeFromTop(4);
	}

	for (auto& i : outConnectors)
	{
		i->setBounds(outR.removeFromTop(w));
		outR.removeFromTop(4);
	}
}

void BaseNodeViewUI::resizedInternalHeader(Rectangle<int>& r)
{
	BaseItemUI::resizedInternalHeader(r);
	statsLabel.setBounds(r.removeFromRight(80));
}

void BaseNodeViewUI::resizedInternalContent(Rectangle<int>& r)
{
	BaseItemUI::resizedInternalContent(r);
	resizedInternalContentNode(r);
}

Rectangle<int> BaseNodeViewUI::getMainBounds()
{
	return getLocalBounds().reduced(10, 0);
}

void BaseNodeViewUI::nodeInputsChanged()
{
}

void BaseNodeViewUI::nodeOutputsChanged()
{
}

void BaseNodeViewUI::viewFilterUpdated()
{
	resized();
}

void BaseNodeViewUI::refreshStats()
{
	if (inspectable.wasObjectDeleted()) return;
	statsLabel.setText(String(item->processTimeMS) + "ms", dontSendNotification);
}

void BaseNodeViewUI::newMessage(const Node::NodeEvent& e)
{
	if (e.type == e.INPUTS_CHANGED) nodeInputsChanged();
	else if (e.type == e.OUTPUTS_CHANGED) nodeOutputsChanged();
	else if (e.type == e.VIEW_FILTER_UPDATED) viewFilterUpdated();
}
