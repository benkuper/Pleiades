/*
  ==============================================================================

    CropboxNode.h
    Created: 5 Apr 2022 10:45:43am
    Author:  bkupe

  ==============================================================================
*/

#pragma once



class CropBoxNode :
    public Node
{
public:
    CropBoxNode(var params = var());
    ~CropBoxNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;

    enum CropMode { ADD, SUBTRACT, INTERSECT };

    class CBox : 
        public BaseItem
    {
    public:
        CBox();
        ~CBox();

        EnumParameter* cropMode;
        Point3DParameter* minPoint;
        Point3DParameter* maxPoint;

        String getTypeString() const override { return "Box"; }
    };

    BaseManager<CBox> boxes;

    BoolParameter* keepOrganized;
    BoolParameter* cleanUp;

    void processInternal() override;

    void onContainerParameterChangedInternal(Parameter* p) override;
    void onContainerTriggerTriggered(Trigger* t) override;
    void onControllableFeedbackUpdateInternal(ControllableContainer* cc, Controllable* c) override;

    virtual var getServerControls() override;
    
    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Crop Box"; }
};