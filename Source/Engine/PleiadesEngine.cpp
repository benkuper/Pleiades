/*
  ==============================================================================

	PleiadesEngine.cpp
	Created: 5 Apr 2022 10:35:04am
	Author:  bkupe

  ==============================================================================
*/

#include "PleiadesEngine.h"
#include "Node/NodeIncludes.h"

PleiadesEngine::PleiadesEngine() :
	Engine(ProjectInfo::projectName, ".star")
{
	Engine::mainEngine = this;
	addChildControllableContainer(RootNodeManager::getInstance());
}

PleiadesEngine::~PleiadesEngine()
{
	isClearing = true;
	RootNodeManager::deleteInstance();
	NodeFactory::deleteInstance();
}

void PleiadesEngine::clearInternal()
{
	RootNodeManager::getInstance()->clear();
}

var PleiadesEngine::getJSONData()
{
	var data = Engine::getJSONData();
	data.getDynamicObject()->setProperty(RootNodeManager::getInstance()->shortName, RootNodeManager::getInstance()->getJSONData());
	return data;
}

void PleiadesEngine::loadJSONDataInternalEngine(var data, ProgressTask* loadingTask)
{
	RootNodeManager::getInstance()->loadJSONData(data.getProperty(RootNodeManager::getInstance()->shortName, var()));
}

String PleiadesEngine::getMinimumRequiredFileVersion()
{
	return "1.0.0b1";
}

