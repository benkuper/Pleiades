/*
  ==============================================================================

    MergeNode.h
    Created: 3 May 2022 6:10:52pm
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class MergeNode :
    public Node
{
public:
    MergeNode(var params = var());
    ~MergeNode();

    Array<NodeConnectionSlot*> ins;
    NodeConnectionSlot* out;

    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Merge"; }
};