/*
  ==============================================================================

    PleiadesEngine.h
    Created: 5 Apr 2022 10:35:04am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"

class PleiadesEngine :
    public Engine
{
public:
    PleiadesEngine();
    ~PleiadesEngine();

    void clearInternal() override;

    var getJSONData() override;
    void loadJSONDataInternalEngine(var data, ProgressTask* loadingTask) override;

    String getMinimumRequiredFileVersion() override;
};