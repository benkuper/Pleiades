/*
  ==============================================================================

    CropboxNode.h
    Created: 5 Apr 2022 10:45:43am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class CropBoxNode :
    public Node
{
public:
    CropBoxNode(var params = var());
    ~CropBoxNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;

    Point3DParameter* minPoint;
    Point3DParameter* maxPoint;

    BoolParameter * keepOrganized;
    BoolParameter* cleanUp;

    void processInternal() override;

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Crop Box"; }
};