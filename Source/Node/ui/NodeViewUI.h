/*
  ==============================================================================

	NodeViewUI.h
	Created: 15 Nov 2020 9:26:57am
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class ViewStatsTimer : public Timer
{
public:
	juce_DeclareSingleton(ViewStatsTimer, false);

	ViewStatsTimer() { startTimerHz(10); }

	class Listener
	{
	public:
		virtual ~Listener() {}
		virtual void refreshStats() {}
	};

	ListenerList<Listener> listeners;
	void addListener(Listener* newListener) { listeners.add(newListener); }
	void removeListener(Listener* listener) { listeners.remove(listener); }


	void timerCallback() { listeners.call(&Listener::refreshStats); }
};

class BaseNodeViewUI :
	public BaseItemUI<Node>,
	public Node::AsyncListener,
	public ViewStatsTimer::Listener
{
public:
	BaseNodeViewUI(Node* node);
	virtual ~BaseNodeViewUI();

	OwnedArray<NodeConnector> inConnectors;
	OwnedArray<NodeConnector> outConnectors;

	Label statsLabel;

	NodeConnector* getConnectorForSlot(NodeConnectionSlot* s);
	Array<NodeConnector*> getConnectorsForType(NodeConnectionType t, bool isInput);

	void updateConnectors();

	void paint(Graphics& g) override;
	void paintOverChildren(Graphics& g) override;
	virtual void resized() override;
	virtual void resizedInternalHeader(Rectangle<int>& r) override;
	virtual void resizedInternalContent(Rectangle<int>& r) override;
	virtual void resizedInternalContentNode(Rectangle<int>& r) {}


	Rectangle<int> getMainBounds() override;

	virtual void nodeInputsChanged();
	virtual void nodeOutputsChanged();

	virtual void viewFilterUpdated();

	virtual void refreshStats() override;

	void newMessage(const Node::NodeEvent& e) override;
};

template<class T>
class NodeViewUI :
	public BaseNodeViewUI
{
public:
	NodeViewUI(T* node) : BaseNodeViewUI(node), node(node) {}
	~NodeViewUI() {}
	T* node;
};
