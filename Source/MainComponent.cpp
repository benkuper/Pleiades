#include "MainComponent.h"
#include "Node/NodeIncludes.h"
#include "Viz/Viz.h"

String getAppVersion();

//==============================================================================
MainContentComponent::MainContentComponent() :
	OrganicMainContentComponent()
{
	//ParameterUI::showAlwaysNotifyOption = false;
	//ParameterUI::showControlModeOption = false;
	//ControllableUI::showDashboardOption = false;
	//ControllableUI::showDetectiveOption = false;
	//ControllableUI::showScriptControlAddressOption = false;
	//IntStepperUI::showHexModeOption = false;

	//getCommandManager().registerAllCommandsForTarget(this);

	ViewStatsTimer::getInstance(); //init timer
}

MainContentComponent::~MainContentComponent()
{
	ViewStatsTimer::deleteInstance();
}

void MainContentComponent::init()
{
	ShapeShifterFactory::getInstance()->defs.add(new ShapeShifterDefinition("Node List", &NodeManagerPanel::create));
	ShapeShifterFactory::getInstance()->defs.add(new ShapeShifterDefinition("Node View", &NodeManagerViewPanel::create));
	ShapeShifterFactory::getInstance()->defs.add(new ShapeShifterDefinition("Visualizer", &VizPanel::create));

	ShapeShifterManager::getInstance()->setDefaultFileData(BinaryData::default_playout);
	ShapeShifterManager::getInstance()->setLayoutInformations("playout", ProjectInfo::projectName + String("/layouts"));

	OrganicMainContentComponent::init();
}
