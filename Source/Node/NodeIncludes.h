/*
  ==============================================================================

    NodeIncludes.h
    Created: 5 Apr 2022 10:36:31am
    Author:  bkupe

  ==============================================================================
*/

#pragma once

#include "JuceHeader.h"

//external libraries
// 
//pcl
#include "Common/PCLHelpers.h"

//orbbec
#pragma warning(push)
#pragma warning(disable : 4324 4201 4996 4189 4127 4018 4005)
#include "libobsensor/ObSensor.hpp"
#pragma warning(pop)

//opencv
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"


typedef std::shared_ptr<Image> ImagePtr;

// classes

#include "Connection/NodeConnectionSlot.h"
#include "Connection/NodeConnection.h"
#include "Node.h"

#include "Connection/NodeConnectionManager.h"
#include "NodeFactory.h"
#include "NodeManager.h"

#include "nodes/Filter/cropbox/CropboxNode.h"
#include "nodes/Filter/voxelgrid/VoxelGridNode.h"
#include "nodes/Filter/euclideancluster/EuclideanClusterNode.h"
#include "nodes/Filter/planesegmentation/PlaneSegmentationNode.h"
#include "nodes/Filter/prediction/PredictionNode.h"
#include "nodes/Output/augmenta/AugmentaOutputNode.h"
#include "nodes/Output/websocket/WebsocketOutputNode.h"

#include "nodes/Filter/tracking/Hungarian.h"
#include "nodes/Filter/tracking/TrackingNode.h"

#include "nodes/Filter/oneeuro/OneEuroFilter.h"
#include "nodes/Filter/oneeuro/OneEuroFilterNode.h"

#include "nodes/Filter/qrcode/QRCodeNode.h"

#include "nodes/Source/astraplus/AstraPlusNode.h"



#include "Connection/ui/NodeConnector.h"
#include "Connection/ui/NodeConnectionViewUI.h"
#include "Connection/ui/NodeConnectionManagerViewUI.h"

#include "ui/NodeUI.h"
#include "ui/NodeManagerUI.h"

#include "ui/NodeViewUI.h"
#include "ui/NodeManagerViewUI.h"