/*
  ==============================================================================

    Main.h
    Created: 5 Apr 2022 10:34:29am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"
#pragma warning(disable:4244 4100 4305)

#include "MainComponent.h"
#include "Engine/PleiadesEngine.h"

//==============================================================================
class PleiadesApplication : public OrganicApplication
{
public:
	//==============================================================================
	PleiadesApplication();
	~PleiadesApplication();

	//==============================================================================
	void initialiseInternal(const String& /*commandLine*/) override;

	bool moreThanOneInstanceAllowed() override;
};



//==============================================================================
// This macro generates the main() routine that launches the app.
START_JUCE_APPLICATION(PleiadesApplication)
