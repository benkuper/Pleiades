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
    defs.add(Definition::createDef<PlaneSegmentationNode>("Filter", PlaneSegmentationNode::getTypeStringStatic()));
    defs.add(Definition::createDef<CropBoxNode>("Filter", CropBoxNode::getTypeStringStatic()));
    defs.add(Definition::createDef<WebsocketOutputNode>("Output", WebsocketOutputNode::getTypeStringStatic()));
}
