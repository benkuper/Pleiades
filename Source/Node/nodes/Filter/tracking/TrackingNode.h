/*
  ==============================================================================

    TrackingNode.h
    Created: 5 Apr 2022 10:45:07am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class TrackingNode :
    public Node
{
public:
    TrackingNode(var params = var());
    ~TrackingNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;

    Array<ClusterPtr> trackedClusters;

    //Global tracking
    int curTrackingID;

    Trigger* clearClusters;


    //Initial search
    FloatParameter* searchDist;

    //Age weighting in Hungarian search, not using for now, needs testing
    //FloatParameter* minDistanceMultiplier;// = 0.2; // the smaller mini is the more old objects are advantaged
    //float mu = 0;
    //FloatParameter* slowDecay;// = 2; // the higher sigma is the slower the coef decreases at the begining (function of the age)
    //FloatParameter* costAmplitude = 25;


    //Ghosting
    BoolParameter* enableGhosting;
    FloatParameter* ghostSearchDist;
    FloatParameter* minAgeForGhost; // Minimum required age to become a ghost, default 1
    FloatParameter* maxGhostAge; // Maximum time in sec a ghost can remain, default 0.5f

    HungarianAlgorithm hungarian;

    void processInternal() override;

    void onContainerTriggerTriggered(Trigger* t) override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Tracker"; }
};