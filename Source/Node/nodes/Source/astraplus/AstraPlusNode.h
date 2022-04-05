/*
  ==============================================================================

    AstraPlusNode.h
    Created: 5 Apr 2022 10:38:21am
    Author:  bkupe

  ==============================================================================
*/

#pragma once


class AstraPlusNode :
    public Node
{
public:
    AstraPlusNode(var params = var());
    ~AstraPlusNode();

    void clearItem() override;

    static std::unique_ptr<ob::Context> ctx;
    static std::unique_ptr<ob::Pipeline> pipeline;

    static std::shared_ptr<ob::VideoStreamProfile> colorProfile;
    static std::shared_ptr<ob::VideoStreamProfile> depthProfile;
    std::shared_ptr<ob::Config> config;
    //std::shared_ptr<PointCloudFilter> pointCloud;

    NodeConnectionSlot* out;

    IntParameter* downSample;


    float ifx;
    float ify;

    bool initInternal() override;

    void setupProfiles();
    void setupPipeline();
   // void setupPointCloud();

    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Astra+"; }
};