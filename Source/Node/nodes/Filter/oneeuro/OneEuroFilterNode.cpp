/*
  ==============================================================================

	OneEuroFilterNode.cpp
	Created: 7 Apr 2022 8:35:50am
	Author:  bkupe

  ==============================================================================
*/

OneEuroFilterNode::OneEuroFilterNode(var params) :
	Node(getTypeString(), FILTER, params)
{
	addInOutSlot(&in, &out, CLUSTERS);

	minCutOff = addFloatParameter("Min Cutoff", "Minimum cutoff", 1, 0);
	beta = addFloatParameter("Beta", "Beta", 10, 0);
	derivativeCutOff = addFloatParameter("Derivative CutOff", "", 0, 0);

	affectCentroid = addBoolParameter("Affect Centroid", "", true);
	affectBoundingBox = addBoolParameter("Affect Bouding Box", "", true);
}

OneEuroFilterNode::~OneEuroFilterNode()
{
}


void OneEuroFilterNode::processInternal()
{
	Array<ClusterPtr> sources = slotClustersMap[in];

	for (auto& cf : filters) cf->hasProcessed = true;

	bool affectC = affectCentroid->boolValue();
	bool affectB = affectBoundingBox->boolValue();
	for (auto& s : sources)
	{
		if (!idFilterMap.contains(s->id))
		{
			ClusterFilter* newF = new ClusterFilter({ s->id });
			updateFilterParams(newF);
			filters.add(newF);
			idFilterMap.set(s->id, newF);
		}

		ClusterFilter* cf = idFilterMap[s->id];
		if(affectC) s->centroid = (&cf->filters[0])->filter(s->oldCentroid, s->centroid, deltaTime);
		if (affectB)
		{
			s->boundingBoxMin = (&cf->filters[1])->filter(s->oldBoundingBoxMin, s->boundingBoxMin, deltaTime);
			s->boundingBoxMax = (&cf->filters[2])->filter(s->oldBoundingBoxMax, s->boundingBoxMax, deltaTime);
		}

		cf->hasProcessed = true;
	}


	//Clean unused filters
	Array<int> idToRemove;
	for (auto& cf : filters) if (!cf->hasProcessed) idToRemove.add(cf->associatedID);
	for (auto& i : idToRemove)
	{
		filters.removeObject(idFilterMap[i]);
		idFilterMap.remove(i);
	}

	sendClusters(out, sources);
}

void OneEuroFilterNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
	if (p == minCutOff || p == beta || p == derivativeCutOff) updateFilterParams();
}

void OneEuroFilterNode::updateFilterParams(ClusterFilter* f)
{
	if (f == nullptr)
	{
		for (auto& f : filters) updateFilterParams(f);
		return;
	}

	for (int i = 0; i < f->numFilters; i++)
	{
		(&f->filters[i])->minCutOff = minCutOff->floatValue();
		(&f->filters[i])->beta = beta->floatValue();
		(&f->filters[i])->derivativeCutOff = derivativeCutOff->floatValue();
	}
}
