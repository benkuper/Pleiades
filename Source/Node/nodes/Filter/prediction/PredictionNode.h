/*
  ==============================================================================

    PredictionNode.h
    Created: 5 Apr 2022 10:44:58am
    Author:  bkupe

  ==============================================================================
*/

#pragma once


class PredictionNode :
    public Node
{
public:
    PredictionNode(var params = var());
    ~PredictionNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;

    Array<ClusterPtr> trackedClusters;

    FloatParameter* strength;
    FloatParameter* velocityThreshold;

    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Prediction"; }
};