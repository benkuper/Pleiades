/*
  ==============================================================================

    EuclideanSegmentationNode.h
    Created: 5 Apr 2022 10:45:21am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class EuclideanClusterNode :
    public Node
{
public:
    EuclideanClusterNode(var params = var());
    ~EuclideanClusterNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* inHighres;
    NodeConnectionSlot* out;

    FloatParameter * tolerance;
    IntParameter* minCount;
    IntParameter * maxCount;
    Point3DParameter* minSize;
    Point3DParameter* maxSize;
    BoolParameter* computeBox;

    void processInternal() override;

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Euclidean Cluster"; }
};