/*
  ==============================================================================

    VoxelGridNode.h
    Created: 5 Apr 2022 10:46:00am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

class VoxelGridNode :
    public Node
{
public:
    VoxelGridNode(var params = var());
    ~VoxelGridNode();

    NodeConnectionSlot* in;
    NodeConnectionSlot* out;

    Point3DParameter* leafSize;

    void processInternal() override;

    String getTypeString() const override { return getTypeStringStatic(); }
    static String getTypeStringStatic() { return "Voxel Grid"; }
};