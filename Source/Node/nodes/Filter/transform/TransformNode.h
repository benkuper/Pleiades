/*
  ==============================================================================

    TransformNode.h
    Created: 4 May 2022 5:56:35pm
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class TransformNode :
    public Node
{
public:
    TransformNode(var params = var());
    ~TransformNode();

    NodeConnectionSlot* inTransform;
    
    NodeConnectionSlot* in;
    NodeConnectionSlot* out;
    
    Point3DParameter* translate;
    Point3DParameter* rotate;
    //Point3DParameter* scale;

    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Transform"; }
};