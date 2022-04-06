/*
  ==============================================================================

	TrackingNode.cpp
	Created: 5 Apr 2022 10:45:07am
	Author:  bkupe

  ==============================================================================
*/

TrackingNode::TrackingNode(var params) :
	Node(getTypeString(), OUTPUT, params),
	curTrackingID(0)
{
	in = addSlot("In", true, CLUSTERS);
	out = addSlot("Out", false, CLUSTERS);

	searchDist = addFloatParameter("Search Distance","in meter", .1f,0,1);
	enableGhosting = addBoolParameter("Enable Ghosting", "", false);
	ghostSearchDist = addFloatParameter("Ghost Search Distance", "in meter", .1f);
	minAgeForGhost = addFloatParameter("Min Ghost Age", "in seconds.",1); // Minimum required age to become a ghost, default 1
	maxGhostAge = addFloatParameter("Max Ghost Age","in seconds.", 1); // Maximum time in sec a ghost can remain, default 0.5f

	clearClusters = addTrigger("Clear clusters","");
}

TrackingNode::~TrackingNode()
{
}


void TrackingNode::processInternal()
{
	Array<ClusterPtr> newClusters = slotClustersMap[in];

	//Removed WILL_LEAVE clusters

	//NNLOG("Start of tracking, new clusters << " << newClusters.size() << ", num clusters " << trackedClusters.size());
	int numRemoved = trackedClusters.removeIf([](ClusterPtr c) { return c->state == Cluster::WILL_LEAVE; });

	//if(numRemoved > 0) NNLOG(numRemoved << " clusters removed, remaining " << trackedClusters.size());

	//Hungarian Matching

	Array<int> matchedClusterIndices;
	matchedClusterIndices.resize(trackedClusters.size());
	matchedClusterIndices.fill(-1);

	Array<int> newClusterIndices;
	for (int i = 0; i < newClusters.size(); i++) newClusterIndices.add(i);

	double curTime = Time::getMillisecondCounterHiRes() / 1000.0;

	float sDist = searchDist->floatValue();
	float ghostSDist = ghostSearchDist->floatValue();

	Array<Array<double >> distanceMatrix;
	distanceMatrix.resize(trackedClusters.size());

	//Fill matrix of distances between current and new objects for the Hungarian matching
	for (int i = 0; i < trackedClusters.size(); i++) {

		Array<double> tDistMatrix;
		tDistMatrix.resize(newClusters.size());

		ClusterPtr cluster = trackedClusters[i];

		auto timeDiff = curTime - cluster->lastUpdateTime;

		Vector3D<float> predictedCentroid = cluster->centroid + cluster->velocity * timeDiff; // Dumb prediction

		for (int j = 0; j < newClusters.size(); j++) {

			ClusterPtr newCluster = newClusters[j];
			Vector3D<float> centroid = newCluster->centroid;

			float dist = (cluster->centroid - newCluster->centroid).length();

			//float clusterSize = (cluster->boundingBoxMax - cluster->boundingBoxMin).length();
			// Should we reintegrate size-base search ? something like  dist > 4 * clusterSize
			if (cluster->state != Cluster::GHOST && dist > sDist) tDistMatrix.set(j, INT32_MAX);
			else if (cluster->state == Cluster::GHOST && dist > ghostSDist) tDistMatrix.set(j, INT32_MAX);
			else tDistMatrix.set(j, dist);
		}

		distanceMatrix.set(i, tDistMatrix);
	}


	String matDbg;
	for (int i = 0; i < distanceMatrix.size(); i++)
	{
		matDbg += String(i) + " [";
		for (int j = 0; j < distanceMatrix[i].size(); j++) matDbg += String(distanceMatrix[i][j]) + ",";
		matDbg += "]\n";
	}

	DBG(matDbg);

	// Weight distance to reduce new object, potential false-positive to "steal" ids, see doc.
	// https://docs.google.com/document/d/1iQvuF4_4AHZbrWNMZPK_ZoerLovdRzTVwKP8m-xlYKM/edit
	// Is it necessary ?

	//float mini = 0.2; // the smaller mini is the more old objects are advantaged
	//float mu = 0;
	//float sigma = 2; // the higher sigma is the slower the coef decreases at the begining (function of the age)
	//float amplitude = 25;

	//Array<Array<double>> costMatrix;
	//costMatrix.resize(trackedClusters.size());

	//for (int i = 0; i < trackedClusters.size(); i++) {

	//	costMatrix[i].resize(newClusters.size());

	//	for (int j = 0; j < newClusters.size(); j++) {

	//		PCloud& obj = trackedClusters[i];
	//		int age = cluster.age;
	//		// ghost have a bigger scope, the costMatrix function gives them an advantage
	//		if (cluster.state == PCloud::GHOST) age += 20;
	//		float coef = 2 * amplitude * (1 - mini) / (1 + exp((age - mu) / sigma)) + mini;
	//		costMatrix[i].set(j, coef * (distanceMatrix[i][j] + 10)); // add a fixed distance 
	//	}
	//}

	hungarian.Solve(distanceMatrix, matchedClusterIndices);


	for (size_t i = 0; i < trackedClusters.size(); i++)
	{

		ClusterPtr cluster = trackedClusters[i];
		int matchedID = matchedClusterIndices[i];

		if (matchedID != -1)
		{
			NNLOG("Found matching cluster for id " << cluster->id);
			cluster->update(newClusters[matchedID]);
			newClusterIndices.removeAllInstancesOf(matchedID);
		}
		else
		{
			NNLOG("No match for id " << cluster->id);
			if (enableGhosting->boolValue())
			{
				// we did not find a match for this tracked object in the incoming detected objects, apply the ghosting algorithm 

				/* #region GHOSTING */
				cluster->velocity = Vector3D<float>();

				if (cluster->state != Cluster::GHOST) {
					// this object was not a ghost
					if (cluster->age > minAgeForGhost->floatValue()) {
						// if it is old enough, we turn it into a ghost
						cluster->state = Cluster::GHOST;
						cluster->ghostAge = 0;
					}
					else {
						// otherwise if it had a brief lifetime, it has good chances to be some unwanted noise
						cluster->state = Cluster::WILL_LEAVE;
					}
				}
				else {
					// this object was already a ghost
					if (cluster->ghostAge > maxGhostAge->floatValue()) {
						cluster->state = Cluster::WILL_LEAVE;
					}
					else
					{
						double curT = Time::getMillisecondCounterHiRes() / 1000.0;
						cluster->ghostAge += curT - cluster->lastUpdateTime;
					}
				}
				/* #endregion */
			}
			else
			{
				cluster->state = Cluster::WILL_LEAVE;
			}

		}
	}

	// add the new objects at the end of the tracked object vector
	for (int i = 0; i < newClusterIndices.size(); i++) {
		ClusterPtr cluster = newClusters[newClusterIndices[i]];
		cluster->id = curTrackingID++;
		cluster->state = Cluster::ENTERED;
		NNLOG("Add new cluster width id " << cluster->id);
		//cluster->oid = newClustersIndexStart + i;
		trackedClusters.add(cluster);
	}

	//NNLOG("End of tracking, num clusters " << trackedClusters.size());

	sendClusters(out, trackedClusters);
}

void TrackingNode::onContainerTriggerTriggered(Trigger* t)
{
	if (t == clearClusters) trackedClusters.clear();
}