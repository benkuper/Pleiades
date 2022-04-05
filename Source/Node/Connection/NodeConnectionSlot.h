/*
  ==============================================================================

	NodeConnectionSlot.h
	Created: 5 Apr 2022 11:37:07am
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class Node;
class NodeConnection;

enum NodeConnectionType {UNKNOWN, POINTCLOUD, VECTOR3, MATRIX, CLUSTER, INDICES };

class NodeConnectionSlot
{
public:
	NodeConnectionSlot(Node* node, bool isInput, String name, NodeConnectionType type);
	~NodeConnectionSlot();

	bool isInput;
	bool processOnReceive;

	String name;
	NodeConnectionType type;

	WeakReference<Node> node;
	Array<NodeConnection*> connections;

	void addConnection(NodeConnection* c);
	void removeConnection(NodeConnection * c);

	bool isConnectedTo(NodeConnectionSlot* s);
};
