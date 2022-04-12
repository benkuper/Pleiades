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

    enum PreviewImageType { BINARY, CONTOUR, CONTOUR_BINARIZED, LINES, PREVIEW_OUTPUT, PREVIEW_INPUT, EXTRACTED_IMAGE };
    EnumParameter* previewType;
    Image qrImage;

    BoolParameter* continuous;
    Trigger* findPlane;
    BoolParameter* transformPlane;

    Eigen::Vector3f planeCenter;
    bool findOnNextProcess;

    void processInternal() override;

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;

    Image getPreviewImage();

    var getJSONData() override;
    void loadJSONDataItemInternal(var data) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "QR Plane Detection"; }
};