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
    static std::unique_ptr<ob::Pipeline> pipeline;

    static std::shared_ptr<ob::VideoStreamProfile> colorProfile;
    static std::shared_ptr<ob::VideoStreamProfile> depthProfile;
    std::shared_ptr<ob::Config> config;

    NodeConnectionSlot* out;
    IntParameter* downSample;

    float ifx;
    float ify;
    
    SpinLock frameLock;
    uint8_t* depthData;

    bool initInternal() override;

    void setupProfiles();
    void setupPipeline();
   // void setupPointCloud();

    void processInternal() override;

    void run() override;

    void onContainerParameterChangedInternal(Parameter* p) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Astra+"; }
};