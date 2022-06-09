/*
  ==============================================================================

	QRCodeNode.cpp
	Created: 12 Apr 2022 12:46:58am
	Author:  bkupe

  ==============================================================================
*/

QRCodeNode::QRCodeNode(var params) :
	Node(getTypeString(), FILTER, params),
	findOnNextProcess(false)
{
	angle = 0;

	reprojMat = Eigen::Matrix4f::Identity();
	cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);

	addInOutSlot(&inDepth, &out, POINTCLOUD, "In", "Out");
	inColor = addSlot("Color", true, NodeConnectionType::RGB);
	inMatrix = addSlot("Camera Matrix", true, MATRIX);

	planeReferenceSlot = addSlot("Plane Reference", false, VECTOR);
	planeOffsetSlot = addSlot("Plane Offset", false, VECTOR);
	planeNormalSlot = addSlot("Plane Normal", false, VECTOR);
	planeRotationSlot = addSlot("Plane Rotation", false, VECTOR);

	calibrateCamera = addBoolParameter("Calibrate Camera", "", false);

	continuous = addBoolParameter("Continuous Search", "If checked, search always for the plane. Otherwise, it will only search when triggering", false);
	findPlane = addTrigger("Find Plane", "Find the plane. Now.");

	transformPlane = addBoolParameter("Transform Plane", "If checked, this will tranform the plane", false);
	rotOffset = addFloatParameter("Rot Offset", "", 0, -3.14, 3.14);
	cleanUp = addBoolParameter("Clean Up", "If checked, this will clean bad points (i.e. points at 0,0,0) before transformation", true);

}

QRCodeNode::~QRCodeNode()
{
}


void QRCodeNode::processInternal()
{
	CloudPtr source = slotCloudMap[inDepth];
	Image img = slotImageMap[inColor];

	if (source == nullptr || source->empty() || !img.isValid()) return;

	if (calibrateCamera->boolValue())
	{
		calibCam(img);
		return;
	}

	if (continuous->boolValue() || findOnNextProcess)
	{
		detectQR(source, img);
		findOnNextProcess = false;
	}

	if (!out->isEmpty())
	{
		transformAndSend(source);
	}

	sendVector(planeReferenceSlot, planeReference);
	sendVector(planeOffsetSlot, planeOffset);
	sendVector(planeRotationSlot, planeRotation);
}

void QRCodeNode::calibCam(Image& img)
{
#if USE_QR
	std::vector<cv::Point2f> pointBuf;
	int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH || cv::CALIB_CB_FAST_CHECK;

	Image::BitmapData bmd(img, Image::BitmapData::readOnly);
	cv::Mat image(img.getHeight(), img.getWidth(), CV_8UC3, bmd.data);

	cv::Size boardSize(7, 9);
	float squareSize = 0.021f; //21mm;

	bool found = cv::findChessboardCorners(image, boardSize, pointBuf, chessBoardFlags);

	if (found)                // If done with success,
	{
		NNLOG("Found  corners");
		// improve the found corners' coordinate accuracy for chessboard
		cv::Mat viewGray;
		cv::cvtColor(image, viewGray, cv::COLOR_BGR2GRAY);
		cornerSubPix(viewGray, pointBuf, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.0001));



		std::vector<cv::Point3f> objectPoints;
		for (int i = 0; i < boardSize.height; ++i)
			for (int j = 0; j < boardSize.width; ++j)
				objectPoints.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));

		//double rms = calibrateCameraRO(objectPoints, pointBuf, imageSize, iFixedPoint, cameraMatrix, distCoeffs, rvecs, tvecs, newObjPoints, s.flag | CALIB_USE_LU);


		//generate preview
		qrImage = img.createCopy();
		Graphics g(qrImage);
		int i = 0;
		Point<float> prevP;
		for (auto& p : pointBuf)
		{
			g.setColour(Colour::fromHSV(i++ * 1.0 / (boardSize.width * boardSize.height), 1, 1, 1));
			g.fillEllipse(Rectangle<float>(0, 0, 10, 10).withCentre(Point<float>(p.x, p.y)));
			if (!prevP.isOrigin()) g.drawLine(prevP.x, prevP.y, p.x, p.y, 2);
			prevP.setXY(p.x, p.y);
		}


		NNLOG("Found calibration : ");
	}
#endif
}

void QRCodeNode::detectQR(CloudPtr source, Image& img)
{
#if USE_QR
	NNLOG("Finding plane..");

	Image::BitmapData bmd(img, Image::BitmapData::readOnly);
	cv::Mat image(img.getHeight(), img.getWidth(), CV_8UC3, bmd.data);

	//to fill
	cv::QRCodeDetector detector;

	std::vector<std::string> decodedInfo;
	std::vector<cv::Point2d> points;
	detector.detectAndDecodeMulti(image, decodedInfo, points);


	NNLOG("Found " << decodedInfo.size() << " tags");

	qrImage = img.createCopy();
	Graphics g(qrImage);

	//if (!inMatrix->isEmpty())
	//{
	//	cameraMatrix = slotMatrixMap[inMatrix];
	//}

	//float gap = 0.11f;
	//std::vector<cv::Point3d> objectPoints;
	//objectPoints.push_back(cv::Point3d(0, 0, 0));
	//objectPoints.push_back(cv::Point3d(gap, 0, 0));
	//objectPoints.push_back(cv::Point3d(gap, 0, gap));
	//objectPoints.push_back(cv::Point3d(0, 0, gap));


	//cameraMatrix = cv::Mat::zeros(3,3, CV_64FC1);    // vector of distortion coefficients
	//distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients
	//cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
	//cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
	//cv::Mat rotMtx = cv::Mat::zeros(3, 3, CV_64FC1);    // vector of distortion coefficients

	//bool solverResult = cv::solvePnP(objectPoints, points, cameraMatrix, distCoeffs, rvec, tvec);

	//if (solverResult)
	//{
	//	LOG("Found pose");

	//	cv::Rodrigues(rvec, rotMtx);
	//	cv::Matx44f T_inv = cv::Matx44f(
	//		(float)rotMtx.at<double>(0, 0), (float)rotMtx.at<double>(0, 1), (float)rotMtx.at<double>(0, 2), (float)tvec.at<double>(0, 0),
	//		(float)rotMtx.at<double>(1, 0), (float)rotMtx.at<double>(1, 1), (float)rotMtx.at<double>(1, 2), (float)tvec.at<double>(1, 0),
	//		(float)rotMtx.at<double>(2, 0), (float)rotMtx.at<double>(2, 1), (float)rotMtx.at<double>(2, 2), (float)tvec.at<double>(2, 0),
	//		0.0F, 0.0F, 0.0F, 1.0F
	//	);

	//	cv::cv2eigen(T_inv, reprojMat);

	//}

	for (int i = 0; i < decodedInfo.size(); i++)
	{
		if (i == 0)
		{
			StringArray sSplit;
			sSplit.addTokens(decodedInfo[i], ",");
			DBG(decodedInfo[i]);
			planeOffset.x() = sSplit[0].getFloatValue();
			planeOffset.y() = sSplit[1].getFloatValue();
			planeOffset.z() = sSplit[2].getFloatValue();

			cv::Point p = points[0];
			int tx = p.x * source->width / image.cols;
			int ty = p.y * source->height / image.rows;

			PPoint pp = source->at(tx, ty);

			NNLOG(p.x << "," << p.y << " > " << pp.x << "," << pp.y << "," << pp.z);

			planeReference.x() = pp.x;
			planeReference.y() = pp.y;
			planeReference.z() = pp.z;

			cv::Point px = points[1];
			int txx = px.x * source->width / image.cols;
			int txy = px.y * source->height / image.rows;
			PPoint ppx = source->at(txx, txy);

			cv::Point pz = points[3];
			int tzx = pz.x * source->width / image.cols;
			int tzy = pz.y * source->height / image.rows;
			PPoint ppz = source->at(tzx, tzy);

			cv::Point3f cvPP(pp.x, pp.y, pp.z);
			cv::Point3f cvPX(ppx.x, ppx.y, ppx.z);
			cv::Point3f cvPZ(ppz.x, ppz.y, ppz.z);
			cv::Point3f relPX(cvPX - cvPP);
			cv::Point3f relPZ(cvPZ - cvPP);


			cv::Point3f dir = relPX.cross(cvPZ - cvPP);

			planeNormal = Eigen::Vector3f(dir.x, dir.y, dir.z).normalized();


			Eigen::Vector3f planeX(relPX.x, relPX.y, relPX.z);
			Eigen::Vector3f planeZ(relPZ.x, relPZ.y, relPZ.z);
			rot = Eigen::Quaternionf::FromTwoVectors(planeX, planeZ);
			rotAA = Eigen::AngleAxisf(rot);

			//float angle = std::atan2(planeX.cross(right).norm(), planeX.dot(right));
			planeRotation = rot.toRotationMatrix().eulerAngles(0, 1, 2);//  Eigen::Vector3f(0, angle, 0);

			angle = rotAA.angle();


			Eigen::Vector3f rotAxis = rotAA.axis();

			reproj = Eigen::Quaternionf::FromTwoVectors(planeNormal, Eigen::Vector3f(0, -1, 0));
		}

		int index = i * 4;
		for (int j = 0; j < 4; j++)
		{
			g.setColour(Colour::fromHSV((index + j) / 4.0, 1, 1, 1));
			g.drawEllipse(Rectangle<float>(0, 0, 10, 10).withCentre(Point<float>(points[index + j].x, points[index + j].y)), 2);
			g.drawLine(points[index + j].x, points[index + j].y, points[index + (j + 1) % 4].x, points[index + (j + 1) % 4].y, 2);
		}
	}
#endif
}

void QRCodeNode::transformAndSend(CloudPtr source)
{
	if (cleanUp->boolValue())
	{
		source->erase(std::remove_if(source->points.begin(), source->points.end(), [](PPoint p) { return p.x == 0 && p.y == 0 && p.z == 0; }), source->points.end());
	}

	if (transformPlane->boolValue())
	{
		CloudPtr transformedCloud(new Cloud(source->width, source->height));

		Eigen::Affine3f transform = Eigen::Affine3f::Identity();
		transform.translate(planeOffset);
		transform.rotate(reproj);
		transform.rotate(Eigen::AngleAxisf(rotOffset->floatValue(), planeNormal));
		transform.translate(-planeReference);
		pcl::transformPointCloud(*source, *transformedCloud, transform);
		sendPointCloud(out, transformedCloud);
	}
	else
	{
		sendPointCloud(out, source);
	}
}


void QRCodeNode::onContainerParameterChangedInternal(Parameter* p)
{
	Node::onContainerParameterChangedInternal(p);
}

void QRCodeNode::onContainerTriggerTriggered(Trigger* t)
{
	Node::onContainerTriggerTriggered(t);
	if (t == findPlane) findOnNextProcess = true;
}

Image QRCodeNode::getPreviewImage()
{
	return qrImage;
}



var QRCodeNode::getJSONData()
{
	var data = Node::getJSONData();
	var planeData;
	planeData.append(planeReference.x());
	planeData.append(planeReference.y());
	planeData.append(planeReference.z());

	planeData.append(planeOffset.x());
	planeData.append(planeOffset.y());
	planeData.append(planeOffset.z());

	planeData.append(planeNormal.x());
	planeData.append(planeNormal.y());
	planeData.append(planeNormal.z());

	planeData.append(planeRotation.x());
	planeData.append(planeRotation.y());
	planeData.append(planeRotation.z());

	planeData.append(reproj.w());
	planeData.append(reproj.x());
	planeData.append(reproj.y());
	planeData.append(reproj.z());


	data.getDynamicObject()->setProperty("planeData", planeData);
	return data;
}

void QRCodeNode::loadJSONDataItemInternal(var data)
{
	Node::loadJSONDataItemInternal(data);
	var planeData = data.getProperty("planeData", var());
	if (planeData.size() >= 16)
	{
		planeReference = Eigen::Vector3f(planeData[0], planeData[1], planeData[2]);
		planeOffset = Eigen::Vector3f(planeData[3], planeData[4], planeData[5]);
		planeNormal = Eigen::Vector3f(planeData[6], planeData[7], planeData[8]);
		planeRotation = Eigen::Vector3f(planeData[9], planeData[10], planeData[11]);
		reproj = Eigen::Quaternionf(planeData[12], planeData[13], planeData[14], planeData[15]);
	}
}
