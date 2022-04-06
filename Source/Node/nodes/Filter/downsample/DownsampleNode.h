/*
  ==============================================================================

    DownsampleNode.h
    Created: 5 Apr 2022 10:46:00am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class DownsampleNode :
    public Node
{
public:
    DownsampleNode(var params = var());
    ~DownsampleNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;

    Point3DParameter* leafSize;

    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Downsample"; }
};