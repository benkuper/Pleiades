/*
  ==============================================================================

    NodeIncludes.cpp
    Created: 5 Apr 2022 10:36:31am
    Author:  bkupe

  ==============================================================================
*/


#include "NodeIncludes.h"

#include "Connection/NodeConnection.cpp"
#include "Connection/NodeConnectionSlot.cpp"
#include "Node.cpp"

#include "Connection/NodeConnectionManager.cpp"
#include "NodeFactory.cpp"
#include "NodeManager.cpp"

#include "nodes/Filter/cropbox/CropboxNode.cpp"
#include "nodes/Filter/downsample/DownsampleNode.cpp"
#include "nodes/Filter/euclideansegmentation/EuclideanSegmentationNode.cpp"
#include "nodes/Filter/passthrough/PassthroughNode.cpp"
#include "nodes/Filter/planesegmentation/PlaneSegmentationNode.cpp"
#include "nodes/Filter/prediction/PredictionNode.cpp"
#include "nodes/Filter/tracking/TrackingNode.cpp"
#include "nodes/Filter/transform/TransformNode.cpp"
#include "nodes/Output/augmenta/AugmentaOutputNode.cpp"
#include "nodes/Output/websocket/WebsocketOutputNode.cpp"

#include "nodes/Source/astraplus/AstraPlusNode.cpp"



#include "Connection/ui/NodeConnector.cpp"
#include "Connection/ui/NodeConnectionViewUI.cpp"
#include "Connection/ui/NodeConnectionManagerViewUI.cpp"

#include "ui/NodeUI.cpp"
#include "ui/NodeManagerUI.cpp"

#include "ui/NodeViewUI.cpp"
#include "ui/NodeManagerViewUI.cpp"