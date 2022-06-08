/*
  ==============================================================================

	MergeNode.cpp
	Created: 3 May 2022 6:10:52pm
	Author:  bkupe

  ==============================================================================
*/

MergeNode::MergeNode(var params) :
	Node(getTypeString(), FILTER, params)
{
	for (int i = 0; i < 8; i++) ins.add(addSlot("In " + String(i + 1), true, POINTCLOUD));
	out = addSlot("Merged", false, POINTCLOUD);

	processOnlyOnce = true;
	processOnlyWhenAllConnectedNodesHaveProcessed = true;
}

MergeNode::~MergeNode()
{
}


void MergeNode::processInternal()
{

	if (!out->isEmpty())
	{
		CloudPtr outC(new Cloud());

		StringArray mergedSlots;
		for (int i = 0; i < ins.size(); i++)
		{
			CloudPtr c = slotCloudMap[ins[i]];
			if (c == nullptr) continue;
			*outC += *c;
			mergedSlots.add(String(i + 1));
		}


		NNLOG("Merged " << mergedSlots.size() << " (" << mergedSlots.joinIntoString(",") << "), total points : " << outC->size());

		sendPointCloud(out, outC);
	}
}

MergeClustersNode::MergeClustersNode(var params) :
	Node(getTypeString(), FILTER, params),
	mergeIDIncrement(0)
{
	for (int i = 0; i < 8; i++) ins.add(addSlot("In " + String(i + 1), true, CLUSTERS));
	out = addSlot("Merged", false, CLUSTERS);

	mergeDistance = addFloatParameter("Merge Distance", "Distance at which 2 clusters will merge into one, in meter.", .5f, 0);
	detachDistance = addFloatParameter("Detach Distance", "Distance at which 2 merged clusters will split", .7f, 0);
	mergeOnEnterOnly = addBoolParameter("Merge On Enter Only", "If checked, this will only check for merge on enter. If no candidate, then it will never merge after", false);

	resetClusters = addTrigger("Reset", "Reset clusters and ids");

	processOnlyOnce = false;
}

MergeClustersNode::~MergeClustersNode()
{
}

void MergeClustersNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
	if (t == resetClusters)
	{
		mergeIDIncrement = 0;
		mergedClusters.clear();
	}
}

void MergeClustersNode::processInternal()
{
	bool _mergeOnlyOnEnter = mergeOnEnterOnly->boolValue();
	float _mergeDist = mergeDistance->floatValue();
	float _detachDist = detachDistance->floatValue();

	for (int i = 0; i < ins.size(); i++)
	{
		Array<ClusterPtr> sourceClusters = slotClustersMap[ins[i]];

		for (auto& c : sourceClusters)
		{
			SourceClusterPtr newSource(new SourceCluster{ i, c });

			auto mergedSource = getMergedSourceClusterForNewSource(newSource); //we could maybe use it more to avoid looping through sources in merged clusters during add / update / remove source

			if (mergedSource == nullptr)
			{
				//create a new one
				MergedCluster* mCluster = new MergedCluster(mergeIDIncrement++, newSource);
				mergedClusters.add(mCluster);
				//DBG("Create new Merged, new size : " << mergedClusters.size());
			}
			else
			{
				MergedCluster* mCluster = mergedSource->parent;
				jassert(mCluster != nullptr);

				bool shouldRemove = false;

				if (newSource->cluster->state == Cluster::WILL_LEAVE)
				{
					shouldRemove = true;
				}
				else
				{
					float dist = (newSource->cluster->centroid - mCluster->centroid).length();
					if (dist > _detachDist)shouldRemove = true;
					else mCluster->updateSource(newSource);
				}

				if (shouldRemove)
				{
					//DBG("Remove merged cluster");
					mCluster->removeSource(newSource);
					if (mCluster->sourceClusters.size() == 0)
					{
						//DBG("Merged cluster is empty, before remove : " << mergedClusters.size());
						mergedClusters.removeObject(mCluster);
						//DBG("Merged cluster is empty, remove. New num clusters " << mergedClusters.size());
					}
				}
			}
		}
	}

	//merging
	HashMap<MergedCluster*, MergedCluster*> mergeMap;
	for (int i = 0; i < mergedClusters.size(); i++)
	{
		MergedCluster* mg1 = mergedClusters[i];

		if (_mergeOnlyOnEnter && mg1->state != Cluster::ENTERED) continue;
		if (mergeMap.containsValue(mg1)) continue;

		float closestDist = INT32_MAX;
		MergedCluster* closestMerged = nullptr;

		for (int j = i + 1; j < mergedClusters.size(); j++)
		{
			MergedCluster* mg2 = mergedClusters[j];
			if (mergeMap.containsValue(mg2)) continue;
			if (mg1->hasCommonSourceWith(mg2)) continue;

			float dist = (mg1->centroid - mg2->centroid).length();
			if (dist < _mergeDist && dist < closestDist)
			{
				closestDist = dist;
				closestMerged = mg2;
			}
		}

		if (closestMerged != nullptr) mergeMap.set(mg1, closestMerged);
	}


	HashMap<MergedCluster*, MergedCluster*>::Iterator it(mergeMap);
	Array<MergedCluster*> clustersToRemove;
	while (it.next())
	{
		//DBG("Merge " << it.getKey()->id << " with " << it.getValue()->id);
		HashMap<int, SourceClusterPtr>::Iterator mit(it.getValue()->sourceClusters);
		while (mit.next()) it.getKey()->addSource(mit.getValue());
		it.getValue()->state = Cluster::WILL_LEAVE;
		clustersToRemove.add(it.getValue());
	}

	for (auto& c : mergedClusters) if (c->state != Cluster::WILL_LEAVE) c->update();

	//sending
	outClusters.clear();
	for (auto& c : mergedClusters) outClusters.add(ClusterPtr(new Cluster(*c))); //Copy to new ClusterPtr for sending through connection

	NNLOG("Sending " << outClusters.size() << " merged clusters");
	sendClusters(out, outClusters);

	//clearing will_leave
	for (auto& c : clustersToRemove) mergedClusters.removeObject(c);

	//clear slots
	clearSlotMaps();
}

MergeClustersNode::SourceClusterPtr MergeClustersNode::getMergedSourceClusterForNewSource(SourceClusterPtr newSource)
{
	//DBG("Get merged for new source, source ID :" << newSource->sourceID << ", num merged : " << mergedClusters.size());
	for (auto& m : mergedClusters) if (SourceClusterPtr s = m->getSourceForCluster(newSource)) return s;
	return nullptr;
}

MergeClustersNode::MergedCluster::MergedCluster(int id, MergeClustersNode::SourceClusterPtr firstSource) :
	Cluster(id, CloudPtr(new Cloud()))
{
	addSource(firstSource);
	state = ENTERED; //force here because addSource will make an update that will remove the entered state
}

MergeClustersNode::MergedCluster::~MergedCluster()
{
	sourceClusters.clear();
}

//
void MergeClustersNode::MergedCluster::addSource(SourceClusterPtr newSource)
{
	if (SourceClusterPtr source = getSourceForCluster(newSource))
	{
		jassertfalse;
	}
	else
	{
		newSource->parent = this;
		sourceClusters.set(newSource->sourceID, newSource);
	}

	updateSource(newSource);
}

void MergeClustersNode::MergedCluster::updateSource(SourceClusterPtr newSource)
{
	if (SourceClusterPtr source = getSourceForCluster(newSource))
	{
		newSource->parent = this;
		sourceClusters.set(source->sourceID, newSource);
		source.swap(newSource);
	}
}

void MergeClustersNode::MergedCluster::removeSource(SourceClusterPtr newSource)
{
	SourceClusterPtr source = getSourceForCluster(newSource);
	if (source->sourceID != -1)
	{
		source->parent = nullptr;
		sourceClusters.remove(source->sourceID);
	}
}

void MergeClustersNode::MergedCluster::update()
{
	if (sourceClusters.size() == 0) return;

	CloudPtr newCloud(new Cloud());
	ClusterPtr newCluster(new Cluster(id, newCloud));


	newCluster->boundingBoxMin = Vector3D<float>(INT32_MAX, INT32_MAX, INT32_MAX);
	newCluster->boundingBoxMax = Vector3D<float>(INT32_MIN, INT32_MIN, INT32_MIN);

	HashMap<int, SourceClusterPtr>::Iterator it(sourceClusters);
	while (it.next())
	{
		SourceClusterPtr source = it.getValue();
		*newCloud += *source->cluster->cloud;

		newCluster->boundingBoxMin = Vector3D<float>(jmin(newCluster->boundingBoxMin.x, source->cluster->boundingBoxMin.x), jmin(newCluster->boundingBoxMin.y, source->cluster->boundingBoxMin.y), jmin(newCluster->boundingBoxMin.z, source->cluster->boundingBoxMin.z));
		newCluster->boundingBoxMax = Vector3D<float>(jmax(newCluster->boundingBoxMax.x, source->cluster->boundingBoxMax.x), jmax(newCluster->boundingBoxMax.y, source->cluster->boundingBoxMax.y), jmax(newCluster->boundingBoxMax.z, source->cluster->boundingBoxMax.z));
		newCluster->centroid += source->cluster->centroid;
		newCluster->velocity += source->cluster->velocity;
	}

	newCluster->centroid /= sourceClusters.size();
	newCluster->velocity /= sourceClusters.size();

	Cluster::update(newCluster);
}

MergeClustersNode::SourceClusterPtr MergeClustersNode::MergedCluster::getSourceForCluster(SourceClusterPtr newSource)
{
	//DBG(" >  get source for new cluster, num sources = " << sourceClusters.size());
	if (sourceClusters.contains(newSource->sourceID) && sourceClusters[newSource->sourceID]->isSameAs(newSource)) return sourceClusters[newSource->sourceID];
	return nullptr;
}

bool MergeClustersNode::MergedCluster::hasClusterWithSourceID(int sourceID)
{
	return sourceClusters.contains(sourceID);
}

bool MergeClustersNode::MergedCluster::hasCommonSourceWith(MergedCluster* other)
{
	HashMap<int, SourceClusterPtr>::Iterator it(sourceClusters);
	while (it.next()) if (other->sourceClusters.contains(it.getKey())) return true;
	return false;
}
