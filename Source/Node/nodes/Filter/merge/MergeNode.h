/*
  ==============================================================================

	MergeNode.h
	Created: 3 May 2022 6:10:52pm
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class MergeNode :
	public Node
{
public:
	MergeNode(var params = var());
	~MergeNode();

	Array<NodeConnectionSlot*> ins;
	NodeConnectionSlot* out;

	void processInternal() override;

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Merge"; }
};

class MergeClustersNode :
	public Node
{
public:
	MergeClustersNode(var params = var());
	~MergeClustersNode();

	Array<NodeConnectionSlot*> ins;
	NodeConnectionSlot* out;

	FloatParameter* mergeDistance;
	FloatParameter* detachDistance;
	BoolParameter* mergeOnEnterOnly;

	FloatParameter* autoClearTime;
	Trigger* resetClusters;

	int mergeIDIncrement;


	class MergedCluster;
	struct SourceCluster
	{
		int sourceID;
		ClusterPtr cluster;
		MergedCluster* parent = nullptr;
		uint32 timeAtLastUpdate = 0; //auto clear

		bool isSameAs(std::shared_ptr<SourceCluster> other) { return other != nullptr && sourceID == other->sourceID && cluster->id == other->cluster->id; }
	};

	typedef std::shared_ptr<SourceCluster> SourceClusterPtr;

	class MergedCluster :
		public Cluster
	{
	public:
		MergedCluster(int id, SourceClusterPtr firstSource, uint32 autoClearTime);
		~MergedCluster();

		HashMap<int, SourceClusterPtr> sourceClusters;
		uint32 autoClearTime = 0;

		virtual void addSource(SourceClusterPtr newSource);
		virtual void updateSource(SourceClusterPtr newSource);
		virtual void removeSource(SourceClusterPtr newSource);

		void update();

		SourceClusterPtr getSourceForCluster(SourceClusterPtr newSource);
		bool hasClusterWithSourceID(int sourceID);
		bool hasCommonSourceWith(MergedCluster* other);
	};

	//typedef std::shared_ptr<MergedCluster> MClusterPtr;

	OwnedArray<MergedCluster, CriticalSection> mergedClusters;
	Array<ClusterPtr> outClusters;

	void onContainerTriggerTriggered(Trigger* t) override;

	void processInternal() override;

	SourceClusterPtr getMergedSourceClusterForNewSource(SourceClusterPtr newSource);


	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "Merge Clusters"; }
};