#include "NodeManager.h"
/*
  =============================================================================

	 NodeManager.cp
	 Created:15 Novr 200 8:39:59am
	 Author:  bkup

  =============================================================================
*/

juce_ImplementSingleton(RootNodeManager); 

NodeManager::NodeManager() :
	BaseManager("Nodes")
{
	managerFactory = NodeFactory::getInstance();
	
	connectionManager.reset(new NodeConnectionManager(this));
	connectionManager->hideInRemoteControl = true;
	connectionManager->defaultHideInRemoteControl = true;
	addChildControllableContainer(connectionManager.get());
}


NodeManager::~NodeManager()
{
}


void NodeManager::clear()
{
	connectionManager->clear();
	BaseManager::clear();
}

Array<UndoableAction*> NodeManager::getRemoveItemUndoableAction(Node* item)
{
	Array<UndoableAction*> result;
	Array<Node*> itemsToRemove;
	itemsToRemove.add(item);
	result.addArray(connectionManager->getRemoveAllLinkedConnectionsActions(itemsToRemove));
	result.addArray(BaseManager::getRemoveItemUndoableAction(item));
	return result;
}

Array<UndoableAction*> NodeManager::getRemoveItemsUndoableAction(Array<Node*> itemsToRemove)
{
	Array<UndoableAction*> result;
	result.addArray(connectionManager->getRemoveAllLinkedConnectionsActions(itemsToRemove));
	result.addArray(BaseManager::getRemoveItemsUndoableAction(itemsToRemove));
	return result;
}

var NodeManager::getJSONData()
{
	var data = BaseManager::getJSONData();
	data.getDynamicObject()->setProperty(connectionManager->shortName, connectionManager->getJSONData());
	return data;
}

void NodeManager::loadJSONDataManagerInternal(var data)
{
	BaseManager::loadJSONDataManagerInternal(data);
	connectionManager->loadJSONData(data.getProperty(connectionManager->shortName, var()));
}


//ROOT

RootNodeManager::RootNodeManager() :
	NodeManager(),
	Thread("Nodes")
{
	Engine::mainEngine->addEngineListener(this);
}

RootNodeManager::~RootNodeManager()
{
	if(Engine::mainEngine != nullptr) Engine::mainEngine->removeEngineListener(this);
	stopThread(1000);
}

void RootNodeManager::run()
{
	while (!threadShouldExit())
	{
		try
		{
			{
				GenericScopedLock lock(items.getLock());
				for (auto& i : items) if (i->type == Node::SOURCE) i->process();
				while (!nextToProcess.isEmpty())
				{
					DBG("Next to process, " << nextToProcess.size());
					Array<Node*> processList;
					processList.addArray(nextToProcess);
					for (auto& n : processList)
					{
						DBG("Process : " << n->niceName);
						n->process();
					}
				}
			}
		}
		catch (std::exception e)
		{
			LOGERROR("Error during process : " << e.what());
		}

		wait(10); //to make dynamically changing with process time
	}
}

void RootNodeManager::startLoadFile()
{
	stopThread(1000);
}

void RootNodeManager::endLoadFile()
{
	startThread();
}
