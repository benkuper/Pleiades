/*
  ==============================================================================

    QRCodeNode.h
    Created: 12 Apr 2022 12:46:58am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class QRCodeNode :
    public Node
{
public:
    QRCodeNode(var params = var());
    ~QRCodeNode();

    NodeConnectionSlot* inColor;
    NodeConnectionSlot* inDepth;
    NodeConnectionSlot* out;
    NodeConnectionSlot* planeCenterSlot;
    NodeConnectionSlot* planeNormalSlot;

    BoolParameter* autoFind;
    BoolParameter* continuous;
    Trigger* findPlane;

    Eigen::Vector3f planeCenter;
    Eigen::Vector3f planeNormal;
    Eigen::Quaternionf reproj;

    bool findOnNextProcess;

    void processInternal() override;

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;

    var getJSONData() override;
    void loadJSONDataItemInternal(var data) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "QR Plane Detection"; }
};