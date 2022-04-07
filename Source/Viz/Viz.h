/*
  ==============================================================================

    Viz.h
    Created: 5 Apr 2022 10:41:47am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"

class VizPanel :
    public ShapeShifterContentComponent
{
public:
    VizPanel(StringRef name);
    ~VizPanel() {}
#if JUCE_WINDOWS
    WindowsWebView2WebBrowserComponent web;
#else
    WebBrowserComponent web;
#endif
    
    void resized() override;

    static VizPanel* create(const String& name) { return new VizPanel(name); }
};