/*
  ==============================================================================

	OneEuroFilterNode.h
	Created: 7 Apr 2022 8:35:50am
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class OneEuroFilterNode :
	public Node
{
public:
	OneEuroFilterNode(var params = var());
	~OneEuroFilterNode();

	NodeConnectionSlot* in;
	NodeConnectionSlot* out;

	struct ClusterFilter
	{
		//keeping track
		int associatedID = 0;
		bool hasProcessed = false; //flag to check if should be removed 
		const int numFilters = 3;
		OneEuroFilter filters[3];
	};

	OwnedArray<ClusterFilter, CriticalSection> filters;
	HashMap<int, ClusterFilter*> idFilterMap;

	FloatParameter* minCutOff;
	FloatParameter* beta;
	FloatParameter* derivativeCutOff;

	void processInternal() override;

	void onContainerParameterChangedInternal(Parameter* p) override;
	void updateFilterParams(ClusterFilter* f = nullptr);

	String getTypeString() const override { return getTypeStringStatic(); }
	static String getTypeStringStatic() { return "One Euro Filter"; }
};