/*
  ==============================================================================

	PCLHelpers.cpp
	Created: 5 Apr 2022 1:57:51pm
	Author:  bkupe

  ==============================================================================
*/

#include "PCLHelpers.h"

Cluster::Cluster(int id, CloudPtr cloud) :
	id(id),
	cloud(cloud)
{
	state = ENTERED;
	lastUpdateTime = Time::getMillisecondCounterHiRes() / 1000.0;
}

Cluster::Cluster(std::shared_ptr<Cluster>& other)
{
	cloud.reset(new Cloud());
	pcl::copyPointCloud(*other->cloud, *cloud); //simple reaffectation ? may cause problem in the node chain

	id = other->id;
	age = other->age;
	ghostAge = other->ghostAge;
	oldBoundingBoxMin = other->oldBoundingBoxMin;
	oldBoundingBoxMax = other->oldBoundingBoxMax;
	oldCentroid = other->oldCentroid;
	oldVelocity = other->oldVelocity;
	boundingBoxMin = other->boundingBoxMin;
	boundingBoxMax = other->boundingBoxMax;
	centroid = other->centroid;
	velocity = other->velocity;

	lastUpdateTime = other->lastUpdateTime;
	state = other->state;
}

void Cluster::update(std::shared_ptr<Cluster> newData)
{
	double curT = Time::getMillisecondCounterHiRes() / 1000.0;
	double delta = curT - lastUpdateTime;

	pcl::copyPointCloud(*newData->cloud, *cloud);

	age += delta;

	oldBoundingBoxMin = Vector3D<float>(boundingBoxMin);
	oldBoundingBoxMax = Vector3D<float>(boundingBoxMax);
	oldCentroid = Vector3D<float>(centroid);
	oldVelocity = Vector3D<float>(velocity);

	boundingBoxMin = newData->boundingBoxMin;
	boundingBoxMax = newData->boundingBoxMax;
	centroid = newData->centroid;

	if (delta > 0) velocity = (centroid - oldCentroid) / delta;

	lastUpdateTime = curT;

	state = UPDATED;
}

namespace pleiades
{
	void copyClusters(Array<ClusterPtr>& source, Array<ClusterPtr>& dest)
	{
		dest.clear();
		for (int i = 0; i < source.size(); i++) dest.add(ClusterPtr(new Cluster(*source[i])));
	}
}

