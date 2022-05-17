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