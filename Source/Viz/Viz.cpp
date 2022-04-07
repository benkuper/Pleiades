/*
  ==============================================================================

    Viz.cpp
    Created: 5 Apr 2022 10:41:47am
    Author:  bkupe

  ==============================================================================
*/

#include "Viz.h"

VizPanel::VizPanel(StringRef name) :
    ShapeShifterContentComponent(name)
#if JUCE_WINDOWS
    , web(false, WebView2Preferences())
#endif
{
    web.goToURL("http://127.0.0.1:6060");
    addAndMakeVisible(web);
}

void VizPanel::resized()
{
    web.setBounds(getLocalBounds());
}
