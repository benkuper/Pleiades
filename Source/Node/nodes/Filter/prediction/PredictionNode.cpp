/*
  ==============================================================================

	PredictionNode.cpp
	Created: 5 Apr 2022 10:44:58am
	Author:  bkupe

  ==============================================================================
*/

PredictionNode::PredictionNode(var params) :
	Node(getTypeString(), OUTPUT, params)
{
	addInOutSlot(&in, &out, CLUSTERS);

	strength = addFloatParameter("Strength", "Prediction coefficient", 1, 0, 5);
	velocityThreshold = addFloatParameter("Velocity Treshold", "Threshold at which the prediction is effective", 1, 0, 2);
}

PredictionNode::~PredictionNode()
{
}


void PredictionNode::processInternal()
{
	Array<ClusterPtr> sources = slotClustersMap[in];

	pleiades::copyClusters(sources, trackedClusters);

	float val = strength->floatValue();
	float thresh = velocityThreshold->floatValue();

	for (auto& c : trackedClusters)
	{
		float vel = c->velocity.length();
		if (vel > thresh)
		{
			c->centroid += c->velocity.normalised() * (vel - 1) * deltaTime * val;
		}
	}


	sendClusters(out, trackedClusters);
}