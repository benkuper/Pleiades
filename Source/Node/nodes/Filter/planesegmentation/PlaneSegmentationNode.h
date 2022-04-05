/*
  ==============================================================================

    PlaneSegmentationNode.h
    Created: 5 Apr 2022 10:45:50am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class PlaneSegmentationNode :
    public Node
{
public:
    PlaneSegmentationNode(var params = var());
    ~PlaneSegmentationNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;
    NodeConnectionSlot* planeCloud;
    NodeConnectionSlot* planeCenterSlot;
    NodeConnectionSlot* planeNormalSlot;

    BoolParameter* continuous;
    Trigger* findPlane;

    IntParameter* downSample;
    FloatParameter* distanceThreshold;
    BoolParameter* invertPlane;

    Eigen::Vector3f planeCenter;
    Eigen::Vector3f planeNormal;
    Eigen::Quaternionf reproj;

    bool findOnNextProcess;

    void processInternal() override;

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Plane Segmentation"; }
};