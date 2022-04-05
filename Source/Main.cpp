/*
  ==============================================================================

    This file contains the basic startup code for a JUCE application.

  ==============================================================================
*/

#include <JuceHeader.h>
#include "Main.h"

String getAppVersion();

PleiadesApplication::PleiadesApplication() :
	OrganicApplication("Pleiades", true, ImageCache::getFromMemory(BinaryData::icon_png, BinaryData::icon_pngSize))
{
}

PleiadesApplication::~PleiadesApplication()
{
}

void PleiadesApplication::initialiseInternal(const String&)
{

	// Intentional crash for testing
	//AppUpdater* myPointer;
	//myPointer->checkForUpdates();


	//AppUpdater::getInstance()->setURLs("", "", "Pleiades");
	//HelpBox::getInstance()->helpURL = URL("");
	CrashDumpUploader::getInstance()->init("", ImageCache::getFromMemory(BinaryData::crash_png, BinaryData::crash_pngSize));


	engine.reset(new PleiadesEngine());

	if (useWindow) mainComponent.reset(new MainContentComponent());

	//GlobalSettings::getInstance()->addChildControllableContainer(FusionSettings::getInstance(), false, 4);

}

bool PleiadesApplication::moreThanOneInstanceAllowed()
{
	return false;
}