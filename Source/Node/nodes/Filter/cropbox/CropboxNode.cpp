/*
  ==============================================================================

	CropboxNode.cpp
	Created: 5 Apr 2022 10:45:43am
	Author:  bkupe

  ==============================================================================
*/

CropBoxNode::CropBoxNode(var params) :
	Node(getTypeString(), FILTER, params),
	boxes("Boxes")
{
	saveAndLoadRecursiveData = true;

	addInOutSlot(&in, &out, POINTCLOUD);

	keepOrganized = addBoolParameter("Keep Organized", "If checked, this will keep the 2D structure of the input cloud", false);
	cleanUp = addBoolParameter("Clean up", "If checked, this will remove any bad points", false);

	boxes.selectItemWhenCreated = false;

	addChildControllableContainer(&boxes);
	if(!Engine::mainEngine->isLoadingFile) boxes.addItem();
}

CropBoxNode::~CropBoxNode()
{
}


void CropBoxNode::processInternal()
{

	CloudPtr source = slotCloudMap[in];
	if (source->empty()) return;


	pcl::IndicesPtr indices(new pcl::Indices);
	if (cleanUp->boolValue())
	{
		pcl::removeNaNFromPointCloud(*source, *indices);
		//NNLOG("Clean cloud size : " << (int)sCloud->size());
	}

	{
		GenericScopedLock(boxes.items.getLock());
		for (auto& b : boxes.items)
		{
			if (!b->enabled->boolValue()) continue;

			Vector3D minV = b->minPoint->getVector();
			Vector3D maxV = b->maxPoint->getVector();
			CropMode m = b->cropMode->getValueDataAsEnum<CropMode>();

			pcl::CropBox<pcl::PointXYZ> filter;

			filter.setInputCloud(source);
			filter.setKeepOrganized(keepOrganized->boolValue());
			filter.setMin(Eigen::Vector4f(minV.x, minV.y, minV.z, 1));
			filter.setMax(Eigen::Vector4f(maxV.x, maxV.y, maxV.z, 1));


			pcl::IndicesPtr newIndices(new pcl::Indices);
			if (m == SUBTRACT || m == INTERSECT)
			{
				filter.setIndices(indices);
				filter.setNegative(m == SUBTRACT);

				filter.filter(*newIndices);
				*indices = *newIndices;
			}
			else if (m == ADD)
			{
				if (indices->size() == 0)
				{
					filter.filter(*indices);
				}
				else
				{
					pcl::IndicesPtr removedIndices(new pcl::Indices);
					pcl::ExtractIndices<PPoint> extract;
					extract.setIndices(indices);
					extract.setInputCloud(source);
					extract.setNegative(true);
					extract.filter(*removedIndices);

					filter.setIndices(removedIndices);
					filter.filter(*newIndices);
					indices->insert(indices->end(), newIndices->begin(), newIndices->end());
				}
			}

			NNLOG(" > " << b->cropMode->getValueKey() << " : " << indices->size());
		}
	}


	CloudPtr cloud(new Cloud());
	pcl::ExtractIndices<PPoint> extract;
	extract.setInputCloud(source);
	extract.setIndices(indices);
	extract.filter(*cloud);

	NNLOG("After crop : " << (int)cloud->size());
	sendPointCloud(out, cloud);
}

void CropBoxNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void CropBoxNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
}

CropBoxNode::CBox::CBox() :
	BaseItem(getTypeString())
{
	showInspectorOnSelect = false;

	cropMode = addEnumParameter("Crop Mode", "Mode to use for this box.");
	cropMode->addOption("Add", ADD)->addOption("Subtract", SUBTRACT)->addOption("Intersect", INTERSECT);

	minPoint = addPoint3DParameter("Min", "Min Point");
	minPoint->setVector(-1, -1, -1);

	maxPoint = addPoint3DParameter("Max", "Max Point");
	maxPoint->setVector(1, 1, 1);

}

CropBoxNode::CBox::~CBox()
{
}
