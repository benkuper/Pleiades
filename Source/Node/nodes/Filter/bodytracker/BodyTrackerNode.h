/*
  ==============================================================================

    BodyTrackerNode.h
    Created: 13 Jun 2022 8:24:52pm
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class BodyTrackerNode :
    public Node
{
public:
    BodyTrackerNode(var params = var());
    ~BodyTrackerNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;


    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Body Tracker"; }
};