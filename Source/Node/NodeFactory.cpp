/*
  ==============================================================================

    NodeFactory.cpp
    Created: 15 Nov 2020 8:56:42am
    Author:  bkupe

  ==============================================================================
*/

juce_ImplementSingleton(NodeFactory)

NodeFactory::NodeFactory()
{
    //defs.add(Definition::createDef<ContainerNode>("", ContainerNode::getTypeStringStatic()));
    defs.add(Definition::createDef<AstraPlusNode>("Source", AstraPlusNode::getTypeStringStatic()));
    defs.add(Definition::createDef<AstraProNode>("Source", AstraProNode::getTypeStringStatic()));
    defs.add(Definition::createDef<Kinect2Node>("Source", Kinect2Node::getTypeStringStatic()));
    defs.add(Definition::createDef<WebsocketSourceNode>("Source", WebsocketSourceNode::getTypeStringStatic()));

    defs.add(Definition::createDef<QRCodeNode>("RGB", QRCodeNode::getTypeStringStatic()));

    defs.add(Definition::createDef<TransformNode>("Point Cloud", TransformNode::getTypeStringStatic()));
    defs.add(Definition::createDef<CropBoxNode>("Point Cloud", CropBoxNode::getTypeStringStatic()));
    defs.add(Definition::createDef<VoxelGridNode>("Point Cloud", VoxelGridNode::getTypeStringStatic()));
    defs.add(Definition::createDef<PlaneSegmentationNode>("Point Cloud", PlaneSegmentationNode::getTypeStringStatic()));
    defs.add(Definition::createDef<MergeNode>("Point Cloud", MergeNode::getTypeStringStatic()));
    defs.add(Definition::createDef<RecorderNode>("Point Cloud", RecorderNode::getTypeStringStatic()));

    defs.add(Definition::createDef<EuclideanClusterNode>("Clusters", EuclideanClusterNode::getTypeStringStatic()));
    defs.add(Definition::createDef<TrackingNode>("Clusters", TrackingNode::getTypeStringStatic()));
    defs.add(Definition::createDef<OneEuroFilterNode>("Clusters", OneEuroFilterNode::getTypeStringStatic()));
    defs.add(Definition::createDef<PredictionNode>("Clusters", PredictionNode::getTypeStringStatic()));
    defs.add(Definition::createDef<MergeClustersNode>("Clusters", MergeClustersNode::getTypeStringStatic()));


    defs.add(Definition::createDef<WebsocketOutputNode>("Output", WebsocketOutputNode::getTypeStringStatic()));

}
