/*
  ==============================================================================

    BodyTrackerNode.cpp
    Created: 13 Jun 2022 8:24:52pm
    Author:  bkupe

  ==============================================================================
*/

#include "BodyTrackerNode.h"


BodyTrackerNode::BodyTrackerNode(var params) :
	Node(getTypeString(), FILTER, params)
{
	addInOutSlot(&in, &out, CLUSTERS);
}

BodyTrackerNode::~BodyTrackerNode()
{
}


void BodyTrackerNode::processInternal()
{

}