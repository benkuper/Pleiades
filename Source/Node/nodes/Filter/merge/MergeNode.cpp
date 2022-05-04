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
	for (int i = 0; i < 4; i++) ins.add(addSlot("In " + String(i + 1), true, POINTCLOUD));
	out = addSlot("Merged", false, POINTCLOUD);

}

MergeNode::~MergeNode()
{
}


void MergeNode::processInternal()
{
	if (!out->isEmpty())
	{
		CloudPtr outC = slotCloudMap[ins[0]];
		if (outC == nullptr) return;


		for (int i = 1; i < ins.size() - 1; i++)
		{
			CloudPtr c = slotCloudMap[ins[i]];
			if (c == nullptr) continue;
			*outC += *c;
		}
		sendPointCloud(out, outC);
	}
}