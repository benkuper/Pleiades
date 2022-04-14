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
    NodeConnectionSlot* inMatrix;
    NodeConnectionSlot* out;
    NodeConnectionSlot* planeReferenceSlot;
    NodeConnectionSlot* planeOffsetSlot;
    NodeConnectionSlot* planeNormalSlot;
    NodeConnectionSlot* planeRotationSlot;

    Image qrImage;

    BoolParameter* calibrateCamera;
    BoolParameter* continuous;
    Trigger* findPlane;
    BoolParameter* transformPlane;
    BoolParameter* cleanUp;
    FloatParameter* rotOffset;

    Eigen::Vector3f planeReference;
    Eigen::Vector3f planeOffset;
    Eigen::Vector3f planeNormal;
    Eigen::Vector3f planeRotation;
    
    float angle;
    Eigen::Quaternionf rot;
    Eigen::AngleAxisf rotAA;
    Eigen::Quaternionf reproj;
    bool findOnNextProcess;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    Eigen::Matrix4f reprojMat;

    void processInternal() override;

    void calibCam(Image &img);
    void detectQR(CloudPtr source, Image &img);
    void transformAndSend(CloudPtr source);

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;

    Image getPreviewImage();

    var getJSONData() override;
    void loadJSONDataItemInternal(var data) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "QR Plane Detection"; }
};