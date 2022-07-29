/*
  ==============================================================================

    NodeManager.h
    Created: 15 Nov 2020 8:39:59am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class NodeConnectionManager;

class NodeManager :
    public BaseManager<Node>
{
public:
    NodeManager();
    ~NodeManager();
    
    std::unique_ptr<NodeConnectionManager> connectionManager;

    void clear() override;

    virtual Array<UndoableAction*> getRemoveItemUndoableAction(Node* n) override;
    virtual Array<UndoableAction*> getRemoveItemsUndoableAction(Array<Node*> n) override;

    var getJSONData() override;
    void loadJSONDataManagerInternal(var data) override;
};


class RootNodeManager :
    public NodeManager,
    public EngineListener,
    public Thread
{
public:
    juce_DeclareSingleton(RootNodeManager, true);
    RootNodeManager();
    ~RootNodeManager();

    Array<Node*, CriticalSection> nextToProcess;

    IntParameter* fps;
    int processTimeMS;
    int averageFPS;

    SpinLock itemLoopLock;

    void clear() override;

    void run() override;

    void addItemInternal(Node* item, var data) override;
    void removeItemInternal(Node* item) override;

    void startLoadFile() override;

    void afterLoadJSONDataInternal() override;
};