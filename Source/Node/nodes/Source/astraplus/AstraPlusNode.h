/*
  ==============================================================================

    AstraPlusNode.h
    Created: 5 Apr 2022 10:38:21am
    Author:  bkupe

  ==============================================================================
*/

#pragma once


class AstraPlusNode :
    public Node,
    public Thread //process-independant frame grabbing
{
public:
    AstraPlusNode(var params = var());
    ~AstraPlusNode();

    void clearItem() override;

    static std::unique_ptr<ob::Context> ctx;
    std::unique_ptr<ob::Pipeline> pipeline;

    std::shared_ptr<ob::VideoStreamProfile> colorProfile;
    std::shared_ptr<ob::VideoStreamProfile> depthProfile;
    std::shared_ptr<ob::PointCloudFilter> pointCloudFilter;
    std::shared_ptr<ob::Config> config;

    NodeConnectionSlot* outDepth;
    NodeConnectionSlot* outColor;
    NodeConnectionSlot* outCamMatrix;
    NodeConnectionSlot* outDistCoeffs;

    IntParameter* deviceIndex;
    IntParameter* downSample;
    BoolParameter* processDepth;
    BoolParameter* processColor;
    BoolParameter* alignDepthToColor;
    BoolParameter* processOnlyOnNewFrame;

    float ifx;
    float ify;
    cv::Mat camMatrix;

    int depthWidth;
    int depthHeight;

    SpinLock frameLock;

    OBPoint* pointsData;
    
    Image colorImage;

    long timeAtlastDeviceQuery;
    bool newFrameAvailable;

    bool initInternal() override;

    void setupProfiles();
    void setupPipeline();

    void processInternal() override;
    void processInternalPassthroughInternal() override;

    void run() override;

    void onContainerParameterChangedInternal(Parameter* p) override;

    Image getPreviewImage() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Astra+"; }
};