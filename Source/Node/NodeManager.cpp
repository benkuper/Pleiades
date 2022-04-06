
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
	Thread("Nodes"),
	processTimeMS(1),
	averageFPS(0)
{
	Engine::mainEngine->addEngineListener(this);
}

RootNodeManager::~RootNodeManager()
{
	if (Engine::mainEngine != nullptr) Engine::mainEngine->removeEngineListener(this);
	stopThread(1000);
}


void RootNodeManager::addItemInternal(Node* item, var data)
{
	GenericScopedLock lock(itemLoopLock);
	NodeManager::addItemInternal(item, data);
}

void RootNodeManager::removeItemInternal(Node* item)
{
	GenericScopedLock lock(itemLoopLock);
	NodeManager::removeItemInternal(item);
}


void RootNodeManager::clear()
{
	stopThread(1000);
	NodeManager::clear();
}

void RootNodeManager::run()
{
	long lastFrameTime = Time::getMillisecondCounter();

	nextToProcess.clear();

	while (!threadShouldExit())
	{
		long millis = Time::getMillisecondCounter();

		try
		{
			{
				GenericScopedLock lock(itemLoopLock);
				for (auto& i : items) if (i->type == Node::SOURCE) i->process();
				while (!threadShouldExit() && !nextToProcess.isEmpty())
				{
					Array<Node*> processList;
					processList.addArray(nextToProcess);
					for (auto& n : processList) n->process();
				}
			}
		}
		catch (std::exception e)
		{
			LOGERROR("Error during process : " << e.what());
			nextToProcess.clear();
		}

		long t = Time::getMillisecondCounter();
		processTimeMS = t - millis;

		int frameDiff = t - lastFrameTime;
		averageFPS = 1000 / jmax(frameDiff, 1);
		lastFrameTime = t;

		int timeToWait = 10 - processTimeMS;
		if (timeToWait > 0) wait(timeToWait); //to make dynamically changing with process time
	}

	nextToProcess.clear();

}

void RootNodeManager::startLoadFile()
{
	startThread();
}


void RootNodeManager::afterLoadJSONDataInternal()
{
	NodeManager::afterLoadJSONDataInternal();
	startThread();
}
