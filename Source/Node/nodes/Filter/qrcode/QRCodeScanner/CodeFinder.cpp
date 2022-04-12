#include <utility> //needed by qr code
#include "CodeFinder.hpp"

/**
 * \brief Used for sorting lines along an axis.
 * \param left The float is the axis intersection and the vector describes the line.
 * \param right The float is the axis intersection and the vector describes the line.
 * \return True if left is smaller than right. False otherwise.
 */
bool compareLineAlongAxis(std::pair<float, cv::Vec4f> left, std::pair<float, cv::Vec4f> right)
{
	return left.first < right.first;
}

/**
 * \brief Constructs a CodeFinder that will operate on the passed image.
 * \param image Image that will be searched for a code.
 * \param hasCode Currently no function.
 */
CodeFinder::CodeFinder(cv::Mat image, bool hasCode)
	: hasCode(hasCode) {

	image = image.clone();
	if (image.cols > 2000 || image.rows > 2000) {
		DBG("Resizing Image, because it is too large: " << image.rows << "x" << image.cols << ". ");

		int x = image.rows;
		int y = image.cols;
		while (x > 1500 || y > 1500)
		{
			x *= 0.9;
			y *= 0.9;
		}

		cv::Mat resizedImage(x, y, image.type());
		resize(image, resizedImage, resizedImage.size(), 0.25, 0.25, cv::INTER_LINEAR);
		image = resizedImage;
		DBG("New size: " << image.rows << "x" << image.cols << ".");
	}
	// Saving original image.
	originalImage = image.clone();

	// Initializing default debug draw colors.
	debuggingColors.push_back(cv::Scalar(0, 0, 255));
	debuggingColors.push_back(cv::Scalar(0, 255, 0));
	debuggingColors.push_back(cv::Scalar(255, 0, 0));
	debuggingColors.push_back(cv::Scalar(255, 0, 255));
	debuggingColors.push_back(cv::Scalar(255, 255, 0));
}

/**
 * \brief Execute the search function that will anaylze the associated image for a code.
 * \return The best fitting code that could be detected or a 1x1 image if none could be found.
 */
cv::Mat CodeFinder::find() {
	cv::Mat image = originalImage.clone();

	DBG("Converting image to binary image...");
	cv::Mat grayscaleImage;
	cvtColor(image, grayscaleImage, cv::COLOR_RGB2GRAY);

	ImageBinarization binarizer;
	int thresholdMethod = -1;
	int maxThresholdMethod = binarizer.getMaxThresholdMethod();

	//Try different Thresholdmethods until one is successfull or out of thresholdmethods
	do {
		thresholdMethod++;
		binarizedImage = binarizer.run(grayscaleImage, thresholdMethod);

		DBG("	Finding all contours...");
		findAllContours();

		DBG("	Finding all finder pattern candidates...");
		findPatternContours();
		DBG("	Number of detected patterns: " << allFinderPatterns.size());

		//If less than 3 Patterns found, try different threshold method
		if (allFinderPatterns.size() < 3) {
			if (thresholdMethod < maxThresholdMethod) {
				DBG("	Try different ThresholdMethod.");
				continue;
			}
			DBG("Could not find any valid Patterns.");
			return drawNotFound();
		}

		DBG("	Finding all edge lines for finder patterns...");;
		findPatternLines();
		DBG("	Number of detected valid patterns: " << validFinderPatterns.size());

		//If less than 3 Patterns are valid, try different threshold method
		if (validFinderPatterns.size() < 3) {
			if (thresholdMethod < maxThresholdMethod) {
				DBG("	Try different ThresholdMethod.");
				continue;
			}
			DBG("Could not find any valid Patterns.");
			return drawNotFound();
		}

		//More than 3 valid Patterns were found. End Loop.
		break;
	} while (true);
	DBG("Successfully located FinderPatterns.");

	DBG("Iterating all combinations of detected finder patterns...");
	bool isQRCode = false;
	for (int a = 0; a < validFinderPatterns.size() - 2 && !isQRCode; a++) {
		for (int b = a + 1; b < validFinderPatterns.size() - 1 && !isQRCode; b++) {
			for (int c = b + 1; c < validFinderPatterns.size() && !isQRCode; c++) {
				DBG("Examining combination (" << a + 1 << ", " << b + 1 << ", " << c + 1 << ")...");
				QRCode code;
				// TODO: Maybe don't copy here but instead use references.
				code.topLeft = validFinderPatterns[a];
				code.topRight = validFinderPatterns[b];
				code.bottomLeft = validFinderPatterns[c];

				DBG("Finding clockwise pattern order...");
				findClockwiseOrder(code);

				DBG("Finding top left corner pattern...");
				findTopLeftPattern(code);

				DBG("Finding merged lines...");
				if (!findMergedLines(code)) {
					DBG("Invalid merged lines. Combination is not a QRCode.");
					continue;
				}

				DBG("Finding corners...");
				findCorners(code);

				DBG("Finding perspective transform...");
				try
				{
					findPerspectiveTransform(code);
				}
				catch (...)
				{
					DBG("Invalid perspective transform. Combination is not a QRCode.");
					continue;
				}

				DBG("Finding number of modules...");
				if (!findNumberOfModules(code))
				{
					DBG("Invalid module values. Combination is not a QRCode.");
					continue;
				}

				DBG("Finding resized image...");
				normalize(code);

				if (verifyQRCode(code)) {
					if (code.verifyPercentage < 85)
					{
						alternativeNormalize(code);
						DBG("Final Percentage:" << code.verifyPercentage);
					}
					allCodes.push_back(code);
				}
				else
				{
					if (alternativeNormalize(code))
					{
						allCodes.push_back(code);
					}

					DBG("Final Percentage:" << code.verifyPercentage);
					if (code.verifyPercentage < 65)
					{
						DBG("Value below 65%. Combination is not a QRCode.");
					}
				}
			}
		}
	}

	float verify = 0.0;
	cv::Mat result = drawNotFound();
	for (QRCode& code : allCodes)
	{
		if (code.verifyPercentage > verify)
		{
			result = code.qrcodeImage;
			verify = code.verifyPercentage;
		}
	}

	return result;
}

/**
 * \brief Identify all contours in the binarized image.
 */
void CodeFinder::findAllContours() {
	// Clone image, because it will be directly manipulated by findAllContours.
	cv::Mat image = binarizedImage.clone();

	// Image values will be either 0 or 1.
	image /= 255;
	allContours.clear();
	hierarchy.clear();
	findContours(image, allContours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE);
}

/**
 * \brief Approximates a contour and verifys if it's a trapezoid.
 * \param polygon A set of points describing a contour.
 * \return True if the approximation of the polygon is a trapezoid.
 */
bool CodeFinder::isTrapez(std::vector<cv::Point> polygon) {
	std::vector<cv::Point> approximatedPolygon;
	double epsilon = 0.1 * cv::arcLength(polygon, true);
	approxPolyDP(polygon, approximatedPolygon, epsilon, true);
	bool ret = (approximatedPolygon.size() == 4);
	return ret;
}


/**
 * \brief Identify all contours which could be a finder pattern and save them in allFinderPatterns.
 */
void CodeFinder::findPatternContours() {
	// Thresholds for ignoring too small or too large contours.
	float minArea = 8.00;
	float maxArea = 0.2 * binarizedImage.cols * binarizedImage.rows;

	for (int i = 0; i < hierarchy.size(); i++)
	{
		// If contour has no children, no neighbours and one parent.
		if (hierarchy[i][0] < 0 && hierarchy[i][1] < 0 &&
			hierarchy[i][2] < 0 && hierarchy[i][3] >= 0)
		{
			// If contour is neither too large nor too small.
			double area = contourArea(allContours[i]);
			if (minArea > area || area > maxArea) {
				continue;
			}

			// If parent contour has a parent as well and no neighbours.
			int outter1 = hierarchy[i][3];
			if (hierarchy[outter1][0] < 0 && hierarchy[outter1][1] < 0 &&
				hierarchy[outter1][3] >= 0)
			{
				int outter2 = hierarchy[outter1][3];
				// If it's a trapez.
				if (isTrapez(allContours[outter2]))
				{
					// Then add it's second parent as a possible pattern candidate.
					FinderPattern pattern;
					pattern.contour = allContours[outter2];
					allFinderPatterns.push_back(pattern);
				}
			}
		}
	}
}

/**
 * \brief Approximate the edge lines for each finder pattern and the segments they're based on.
 */
void CodeFinder::findPatternLines() {
	for (FinderPattern& pattern : allFinderPatterns) {
		// Approximate the contour with a polygon.
		double epsilon = 0.1 * arcLength(pattern.contour, true);
		std::vector<cv::Point> approximatedPolygon;
		approxPolyDP(pattern.contour, approximatedPolygon, epsilon, true);

		// Use the approximated points as cuts for segmenting the contour.
		// Find the indices of the cuts within the contour for that.
		std::vector<int> approxCut;
		for (cv::Point& approxPoint : approximatedPolygon) {
			bool found = false;
			for (int index = 0; index < pattern.contour.size(); index++) {
				if (approxPoint == pattern.contour[index]) {
					approxCut.push_back(index);
					found = true;
					break;
				}
			}
			if (!found) {
				// TODO: Verify that this assumption is always true.
				// Assumption that the approximation always lies on a part of the contour did not hold true.
				break;
			}
		}
		if (approxCut.size() != 4) {
			// Assumption that the approximation is a quad did not hold true.
			continue;
		}
		sort(approxCut.begin(), approxCut.end());

		// Percentage of segment that will be ignored to avoid taking the corners into the segment.
		float marginPercentage = 0.1;

		// Cut contour into segments using the approximated cutting points.
		for (int j = 0; j < 3; j++) {
			std::vector<cv::Point> segment;
			int margin = (approxCut[j + 1] - approxCut[j]) * marginPercentage;
			for (int index = approxCut[j] + margin; index < approxCut[j + 1] - margin; index++) {
				segment.push_back(pattern.contour[index]);
			}
			pattern.segments.push_back(segment);
		}

		// Special case for wraping around the end of the vector.
		{
			std::vector<cv::Point> segment;
			int margin = (approxCut[0] + pattern.contour.size() - approxCut[3]) * marginPercentage;
			for (int index = approxCut[3] + margin; index < approxCut[0] + pattern.contour.size() - margin; index++) {
				if (index >= pattern.contour.size())
					segment.push_back(pattern.contour[index - pattern.contour.size()]);
				else
					segment.push_back(pattern.contour[index]);
			}
			pattern.segments.push_back(segment);
		}

		// Use each cut segment for approximating a line fit.
		cv::Vec4f line;
		for (std::vector<cv::Point> segment : pattern.segments) {
			fitLine(segment, line, fitType, 0, fitReps, fitAeps);
			pattern.lines.push_back(line);
		}
		validFinderPatterns.push_back(pattern);
	}
}

/**
 * \brief Takes three finder patterns and deduces if they're ordered clockwise.
 *		  If not, reverses the ordering.
 * \param code The QRCode structure with the necessary fields filled out.
 */
void CodeFinder::findClockwiseOrder(QRCode& code) {
	// Use shoelace algorithm (Gaußsche Trapezformel) to find winding order.
	int area = 0;

	cv::Point& pa = code.topLeft.contour[0];
	cv::Point& pb = code.topRight.contour[0];
	cv::Point& pc = code.bottomLeft.contour[0];

	area += pa.x * (pb.y - pc.y);
	area += pb.x * (pc.y - pa.y);
	area += pc.x * (pa.y - pb.y);

	// If area is less than zero, reverse pattern order.
	if (area < 0) {
		FinderPattern temp = code.topLeft;
		code.topLeft = code.topRight;
		code.topRight = temp;
	}
}

/**
 * \brief Identify the top left pattern, and subsequently top right and bottom left.
 * \param code Code holding three patterns.
 */
void CodeFinder::findTopLeftPattern(QRCode& code) {
	// Compare the distances between each line for each finder pattern.
	std::vector<double> distanceA;
	std::vector<double> distanceB;
	std::vector<double> distanceC;

	patternPatternLineDistances(code.topLeft, code.topRight, distanceA, distanceB);
	patternPatternLineDistances(code.topLeft, code.bottomLeft, distanceA, distanceC);
	patternPatternLineDistances(code.topRight, code.bottomLeft, distanceB, distanceC);

	sort(distanceA.begin(), distanceA.end());
	sort(distanceB.begin(), distanceB.end());
	sort(distanceC.begin(), distanceC.end());

	// Every distance got essantially pushed twice so we have eight small values for
	// the top left pattern. Therefore compare the eighth value of each pattern.
	if (distanceA[7] < distanceB[7] && distanceA[7] < distanceC[7]) {
		// A is the top left pattern.
	}
	if (distanceB[7] < distanceA[7] && distanceB[7] < distanceC[7]) {
		// B is the top left pattern.
		FinderPattern temp = code.topLeft;
		code.topLeft = code.topRight;
		code.topRight = code.bottomLeft;
		code.bottomLeft = temp;
	}
	if (distanceC[7] < distanceA[7] && distanceC[7] < distanceB[7]) {
		// C is the top left pattern.
		FinderPattern temp = code.topRight;
		code.topRight = code.topLeft;
		code.topLeft = code.bottomLeft;
		code.bottomLeft = temp;
	}
}

/**
 * \brief Merges two vectors into a single one by appending the second to the first vector.
 * \tparam T Type of the data contained in the vector.
 * \param a First vector.
 * \param b Second vector.
 * \return The merged vector.
 */
template<typename T>
std::vector<T> merge(std::vector<T>& a, std::vector<T>& b) {
	std::vector<T> result;
	result.reserve(a.size() + b.size());
	result.insert(result.end(), a.begin(), a.end());
	result.insert(result.end(), b.begin(), b.end());
	return result;
}

/**
 * \brief Merge all lines of the top left finder pattern with two lines each of one of the
 *		  other patterns.
 * \param code Code with three correctly identified patterns.
 * \return True on success. False if the merge function detected that the code is not a valid code.
 */
bool CodeFinder::findMergedLines(QRCode& code) {
	FinderPattern& tl = code.topLeft;
	FinderPattern& tr = code.topRight;
	FinderPattern& bl = code.bottomLeft;

	// Keeps track of how the lines for the top left pattern look after merging.
	std::vector<cv::Vec4f> mergedLines;
	// Keeps track of which lines already have been used.
	std::vector<cv::Vec4f> usedLines;

	mergedLines.reserve(tl.lines.size());
	usedLines.reserve(tl.lines.size() * 2);

	// Merge each line of the top left pattern with one line of either top right or bottom left.
	for (int a = 0; a < tl.lines.size(); a++) {
		// Find the line with the smallest distance to the current line.
		double minDistance = lineLineDistance(tl.lines[a], tr.lines[0]);
		int index = 0;
		bool isTopRight = true;

		// Search in top right.
		for (int i = 1; i < tr.lines.size(); i++)
		{
			double currentDistance = lineLineDistance(tl.lines[a], tr.lines[i]);
			if (currentDistance < minDistance)
			{
				minDistance = currentDistance;
				index = i;
				isTopRight = true;
			}
		}
		// Search in bottom left.
		for (int i = 0; i < bl.lines.size(); i++)
		{
			double currentDistance = lineLineDistance(tl.lines[a], bl.lines[i]);
			if (currentDistance < minDistance)
			{
				minDistance = currentDistance;
				index = i;
				isTopRight = false;
			}
		}

		// Once found, merge the underlying segments and fit a new line.
		cv::Vec4f line;
		std::vector<cv::Point> mergedSegment;
		if (isTopRight) {
			mergedSegment = merge(tl.segments[a], tr.segments[index]);
			fitLine(mergedSegment, line, fitType, 0, fitReps, fitAeps);

			code.hLines.push_back(line);
			mergedLines.push_back(line);
			usedLines.push_back(tl.lines[a]);
			usedLines.push_back(tr.lines[index]);
		}
		else {
			mergedSegment = merge(tl.segments[a], bl.segments[index]);
			fitLine(mergedSegment, line, fitType, 0, fitReps, fitAeps);

			code.vLines.push_back(line);
			mergedLines.push_back(line);
			usedLines.push_back(tl.lines[a]);
			usedLines.push_back(bl.lines[index]);
		}
	}

	// If not exactly two lines have been merged for top and bottom, this is not a qrcode.
	if (code.hLines.size() != 2 || code.vLines.size() != 2)
		return false;

	// Add all lines we have not used for merging yet.
	for (cv::Vec4f& line : tr.lines) {
		bool used = false;
		for (cv::Vec4f trLine : usedLines) {
			used = line == trLine;
			if (used)
				break;
		}
		if (!used)
			code.vLines.push_back(line);
	}
	for (cv::Vec4f& line : bl.lines) {
		bool used = false;
		for (cv::Vec4f& blLine : usedLines) {
			used = line == blLine;
			if (used)
				break;
		}
		if (!used)
			code.hLines.push_back(line);
	}

	if (code.hLines.size() != 4 || code.vLines.size() != 4)
		return false;

	// Now sort the lines within qrcode coordinate system from top left to bottom right.
	sortLinesAlongAxis(code.vLines, code.hLines[0]);
	sortLinesAlongAxis(code.hLines, code.vLines[0]);

	bool isReversed = true;
	for (cv::Vec4f& line : mergedLines) {
		if (code.hLines[0] == line) {
			isReversed = false;
			break;
		}
	}
	if (isReversed) {
		reverse(code.hLines.begin(), code.hLines.end());
	}

	isReversed = true;
	for (cv::Vec4f& line : mergedLines) {
		if (code.vLines[0] == line) {
			isReversed = false;
			break;
		}
	}
	if (isReversed) {
		reverse(code.vLines.begin(), code.vLines.end());
	}

	return true;
}

/**
 * \brief Calculate all line intersections between vertical and horizontal lines.
 *		  The result will be all corner of the code and the finder patterns.
 * \param code Code containing merged lines.
 */
void CodeFinder::findCorners(QRCode& code) {
	code.corners = cv::Mat(4, 4, CV_32FC2);
	for (int a = 0; a < code.hLines.size(); a++) {
		for (int b = 0; b < code.vLines.size(); b++) {
			cv::Point2f result;
			if (lineIntersection(code.hLines[a], code.vLines[b], result))
				code.corners.at<cv::Point2f>(a, b) = result;
		}
	}
}

/**
 * \brief Use corners to calculate a perspective transform matrix and extract the code.
 * \param code Code containing corners.
 */
void CodeFinder::findPerspectiveTransform(QRCode& code)
{
	// The corners of the code within the original image.
	std::vector<cv::Point2f> sourceQuad;
	sourceQuad.reserve(4);
	sourceQuad.push_back(code.corners.at<cv::Point2f>(0, 0));
	sourceQuad.push_back(code.corners.at<cv::Point2f>(3, 0));
	sourceQuad.push_back(code.corners.at<cv::Point2f>(0, 3));
	sourceQuad.push_back(code.corners.at<cv::Point2f>(3, 3));

	//Check if points are negative
	for (cv::Point2f p : sourceQuad) {
		if (p.x < 0 || p.y < 0) {
			DBG("The Points for perspectiveTransform have negative values.");
			throw std::exception();
		}
	}
	//Check if points are equal
	for (int i = 0; i < 3; i++) {
		for (int j = i + 1; j < 4; j++) {
			if (sourceQuad[i] == sourceQuad[j]) {
				DBG("At least two Points for perspectiveTransform are equal.");
				throw std::exception();
			}
		}
	}

	// Find the longest distance between corners and use it as the target size.
	double distance = 0.0f;
	for (cv::Point2f& p1 : sourceQuad)
	{
		for (cv::Point2f& p2 : sourceQuad)
		{
			cv::Point2f d = p2 - p1;
			double distanceCurrent = sqrt(d.x * d.x + d.y * d.y);
			if (distance < distanceCurrent)
				distance = distanceCurrent;
		}
	}

	// Mutliply by two for increased quality of extraction result.
	distance = distance * 2;

	// Define target transformation area.
	std::vector<cv::Point2f> targetQuad;
	targetQuad.reserve(4);
	targetQuad.push_back(cv::Point2f(0, 0));
	targetQuad.push_back(cv::Point2f(0, distance));
	targetQuad.push_back(cv::Point2f(distance, 0));
	targetQuad.push_back(cv::Point2f(distance, distance));

	code.transform = getPerspectiveTransform(sourceQuad, targetQuad);

	warpPerspective(binarizedImage, code.extractedImage, code.transform,
		cv::Size(distance, distance), cv::INTER_NEAREST);

	// Also warp get the positions of the corners in the new image.
	perspectiveTransform(code.corners, code.transformedCorners, code.transform);
}

/**
 * \brief Calculate the number of modules the code has, the version and the step size for the extracted image.
 * \param code Code containing an extracted image.
 * \return True on success. False if the function detected illogical step sizes.
 */
bool CodeFinder::findNumberOfModules(QRCode& code)
{
	cv::Point2f bottomLeftCorner = code.transformedCorners.at<cv::Point2f>(1, 1);

	// Average the step size for a single module.
	code.gridStepSize.x = bottomLeftCorner.x / 7;
	code.gridStepSize.y = bottomLeftCorner.y / 7;

	// Calculate how many cells there would be with the current step size.
	double cellsX = code.extractedImage.cols / code.gridStepSize.x;
	double cellsY = code.extractedImage.rows / code.gridStepSize.y;

	DBG("Grid Step Size (x,y): " << "(" << code.gridStepSize.x << "," << code.gridStepSize.y << ")");
	DBG("Approximated cell counts for X: " << cellsX << " and Y: " << cellsY);
	if (cellsX < 0 || cellsY < 0 || abs(cellsX - cellsY) > 8) {
		return false;
	}

	// Take average of both cellscounts
	float cells = (cellsX + cellsY) / 2;

	int version;
	if (cells <= 23) {
		version = 1;
	}
	else if (cells > 175) {
		version = 40;
	}
	else {
		//Calculate versionnumber by supposing cells = #modules, then rounding to nearest integer
		version = cvRound((cells - 17) / 4);
	}

	if (version <= 0 || version > 40) {
		DBG("The versionnumber " << version << " is invalid.");
		return false;
	}
	code.version = version;

	// The result is the version, the number of modules and the step size for the qrcode.
	code.modules = 17 + 4 * code.version;
	code.gridStepSize.x = float(code.extractedImage.cols) / float(code.modules);
	code.gridStepSize.y = float(code.extractedImage.rows) / float(code.modules);
	return true;
}

/**
 * \brief Walk through the extracted image and take the median of each module cell to create
 *		  a true size code image.
 * \param code Code containing an extracted image and step size.
 */
void CodeFinder::normalize(QRCode& code)
{
	code.qrcodeImage = cv::Mat(code.modules, code.modules, CV_8UC1, cv::Scalar(255));

	for (int a = 0; a < code.modules; a++)
	{
		for (int b = 0; b < code.modules; b++)
		{
			int beginX = a * code.gridStepSize.x;
			int endX = (a + 1) * code.gridStepSize.x;

			int beginY = b * code.gridStepSize.y;
			int endY = (b + 1) * code.gridStepSize.y;

			int blackCount = 0;
			for (int x = beginX; x < endX && x < code.extractedImage.cols; x++)
			{
				for (int y = beginY; y < endY && y < code.extractedImage.rows; y++)
				{
					uint8_t pixel = code.extractedImage.at<uint8_t>(x, y);

					if (pixel == 0)
					{
						blackCount++;
					}
				}
			}
			int totalCount = (endX - beginX) * (endY - beginY);

			if (totalCount * 0.5 < blackCount)
			{
				// Set to black.
				code.qrcodeImage.at<uint8_t>(a, b) = 0;
			}
			else
			{
				// Set to white.
				code.qrcodeImage.at<uint8_t>(a, b) = 255;
			}
		}
	}
}

/**
 * \brief Compare the quality of the true size code if the approximated version is set to
 *		  one value higher or lower, and replace the true size code if the quality rises.
 * \param code Code containing a verified true size image.
 * \return True in case the quality has risen by increasing or decreasing the version.
 */
bool CodeFinder::alternativeNormalize(QRCode& code)
{
	float oldPercentage = code.verifyPercentage;
	int oldVersion = code.version;
	int oldModules = code.modules;

	// Check one version higher.
	if (oldVersion < 40)
	{
		code.version = oldVersion + 1;
		code.modules = 17 + 4 * code.version;
		code.gridStepSize.x = float(code.extractedImage.cols) / float(code.modules);
		code.gridStepSize.y = float(code.extractedImage.rows) / float(code.modules);

		normalize(code);

		verifyQRCode(code);
	}

	// Check one version lower.
	if (code.verifyPercentage <= oldPercentage && oldVersion > 1)
	{
		code.version = oldVersion - 1;
		code.modules = 17 + 4 * code.version;
		code.gridStepSize.x = float(code.extractedImage.cols) / float(code.modules);
		code.gridStepSize.y = float(code.extractedImage.rows) / float(code.modules);

		normalize(code);

		verifyQRCode(code);
	}
	else
	{
		return true;
	}

	// Put old version if no improvement has been made.
	if (code.verifyPercentage <= oldPercentage)
	{
		code.verifyPercentage = oldPercentage;
		code.version = oldVersion;
		code.modules = oldModules;
		code.gridStepSize.x = float(code.extractedImage.cols) / float(code.modules);
		code.gridStepSize.y = float(code.extractedImage.rows) / float(code.modules);

		normalize(code);
	}
	else
	{
		return true;
	}

	return false;
}

/**
 * \brief Distance between a point and a line.
 * \param p Point to compare distance to.
 * \param line Vector whose first two elements denote a direction and whose second two
 *		  elements denote a support vector.
 * \return Shortest distance between the point and the line.
 */
double CodeFinder::pointLineDistance(cv::Vec2f p, cv::Vec4f line)
{
	cv::Vec2f supportVector(line[2], line[3]);

	// Translate p so that supportVector moves into origin.
	p -= supportVector;

	// Calculate cross product between p and direction and get absolute distance.
	return abs(p[0] * line[1] - p[1] * line[0]);
}

/**
 * \brief Calculate similarity of lines using custom line similarity measure.
 * \param lineOne First line in the format needed by pointLineDistance.
 * \param lineTwo Second line in the format needed by pointLineDistance.
 * \return Similarity measure.
 */
double CodeFinder::lineLineDistance(cv::Vec4f lineOne, cv::Vec4f lineTwo) {
	double distOne = pointLineDistance(cv::Vec2f(lineTwo[2], lineTwo[3]), lineOne);
	double distTwo = pointLineDistance(cv::Vec2f(lineOne[2], lineOne[3]), lineTwo);

	return distOne + distTwo;
}

/**
 * \brief Calculate similarity measure for each line pair of two patterns.
 * \param one Pattern containg lines.
 * \param two Pattern containg lines.
 * \param distanceOne vector that will hold all similarity measures for pattern one.
 * \param distanceTwo vector that will hold all similarity measures for pattern two.
 */
void CodeFinder::patternPatternLineDistances(FinderPattern& one, FinderPattern& two,
	std::vector<double>& distanceOne, std::vector<double>& distanceTwo) {
	for (cv::Vec4f& lineOne : one.lines) {
		for (cv::Vec4f& lineTwo : two.lines) {
			double dist = lineLineDistance(lineOne, lineTwo);
			distanceOne.push_back(dist);
			distanceTwo.push_back(dist);
		}
	}
}

/**
 * \brief Calculate intersection point of two lines.
 * \param line1 Line in (direction, support vector) format.
 * \param line2 Line in (direction, support vector) format.
 * \param result Output for the found intersection point.
 * \return True if an intersection point was found. False if the lines are parallel.
 */
bool CodeFinder::lineIntersection(cv::Vec4f line1, cv::Vec4f line2, cv::Point2f& result) {
	cv::Point2f x = cv::Point2f(line2[2], line2[3]) - cv::Point2f(line1[2], line1[3]);
	cv::Point2f d1 = cv::Point2f(line1[0], line1[1]);
	cv::Point2f d2 = cv::Point2f(line2[0], line2[1]);

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	result = cv::Point2f(line1[2], line1[3]) + d1 * t1;
	return true;
}

/**
 * \brief Sort the vector of lines along an arbitary axis.
 * \param lines vector with lines that will be sorted.
 * \param axis Axis to sort the lines along.
 */
void CodeFinder::sortLinesAlongAxis(std::vector<cv::Vec4f>& lines, cv::Vec4f axis) {
	std::vector<std::pair<float, cv::Vec4f>> sortedLines;

	cv::Point2f sv = cv::Point2f(axis[2], axis[3]);
	cv::Point2f dir = cv::Point2f(axis[0], axis[1]);
	cv::Point2f xAxis = cv::Point2f(1, 0);

	// Calculate the intersection of each line with axis.
	for (cv::Vec4f& line : lines) {
		cv::Point2f intersect;
		if (lineIntersection(line, axis, intersect)) {
			// Transform intersection into axis coordinate system.
			intersect = intersect - sv;
			float x = intersect.x * dir.dot(xAxis) - intersect.y * dir.cross(xAxis);
			sortedLines.push_back(std::pair<float, cv::Vec4f>(x, line));
		}
		else {
			sortedLines.push_back(std::pair<float, cv::Vec4f>(0.0f, line));
		}
	}

	// Sort lines within the axis system.
	sort(sortedLines.begin(), sortedLines.end(), compareLineAlongAxis);
	lines.clear();

	for (std::pair<float, cv::Vec4f>& pair : sortedLines) {
		lines.push_back(pair.second);
	}
}

/**
 * \brief Verify that well known module values are as expected and measure divergence from this.
 * \param code Code containing a qrcode image.
 * \return True if the Code has more than 65% values that meet the expectations.
 */
bool CodeFinder::verifyQRCode(QRCode& code) {
	cv::Mat QRImage = code.qrcodeImage;
	const int cols = QRImage.cols;
	const int rows = QRImage.rows;
	int count = 0;
	int total = 0;

	//Initialize PerfectPatterns
	cv::Mat perfectPattern = cv::Mat::zeros(7, 7, CV_8UC1);
	for (int i = 1; i < 6; i++) {
		perfectPattern.at<uint8_t>(1, i) = 255;
		perfectPattern.at<uint8_t>(5, i) = 255;
		perfectPattern.at<uint8_t>(i, 1) = 255;
		perfectPattern.at<uint8_t>(i, 5) = 255;
	}

	//Compare to Patterns of QR
	cv::Mat topLeftPattern = QRImage(cv::Rect(0, 0, 7, 7));
	cv::Mat topRightPattern = QRImage(cv::Rect(0, cols - 7, 7, 7));
	cv::Mat bottomLeft = QRImage(cv::Rect(rows - 7, 0, 7, 7));


	for (int y = 0; y < 7; y++) {
		for (int x = 0; x < 7; x++) {
			total = total + 3;
			if (topLeftPattern.at<uint8_t>(x, y) == perfectPattern.at<uint8_t>(x, y))
				count++;
			if (topRightPattern.at<uint8_t>(x, y) == perfectPattern.at<uint8_t>(x, y))
				count++;
			if (bottomLeft.at<uint8_t>(x, y) == perfectPattern.at<uint8_t>(x, y))
				count++;
		}
	}

	//Initialize PerfectAligments
	int timingSize = rows - 7 - 7;
	cv::Mat perfectTimingPattern = cv::Mat::zeros(1, timingSize, CV_8UC1);
	for (int i = 0; i < timingSize; i = i + 2) {
		perfectTimingPattern.at<uint8_t>(0, i) = 255;
	}

	//Compare to alignments of QR
	cv::Mat topTiming = QRImage(cv::Rect(7, 6, timingSize, 1));
	cv::Mat leftTiming = QRImage(cv::Rect(6, 7, 1, timingSize));

	for (int i = 0; i < timingSize; i++) {
		total = total + 2;
		if (topTiming.at<uint8_t>(0, i) == perfectTimingPattern.at<uint8_t>(0, i))
			count++;
		if (leftTiming.at<uint8_t>(i, 0) == perfectTimingPattern.at<uint8_t>(0, i))
			count++;
	}

	float percentage = float(count) / float(total);

	code.verifyPercentage = percentage * 100;
	DBG("Verified QRCode. Percentage is " << code.verifyPercentage << "%.");
	if (code.verifyPercentage > 65)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*****************************************************************************/
/* Drawing function used for debugging. Will output various internal states. */
/*****************************************************************************/

cv::Mat CodeFinder::drawBinaryImage() {
	return binarizedImage;
}

cv::Mat CodeFinder::drawAllContours() {
	return drawContours(allContours);
}

cv::Mat CodeFinder::drawAllContoursBinarized()
{
	cv::Mat image = binarizedImage.clone();
	cvtColor(image, image, cv::COLOR_GRAY2RGB);
	return drawContours(allContours, &image);
}

cv::Mat CodeFinder::drawPatternContours() {
	std::vector<std::vector<cv::Point>> patternContours;
	for (FinderPattern& pattern : validFinderPatterns) {
		patternContours.push_back(pattern.contour);
	}
	return drawContours(patternContours);
}

cv::Mat CodeFinder::drawAllLines() {
	cv::Mat lineImage = originalImage.clone();
	/*
	std::vector<std::vector<cv::Point>> polygons;
	for (FinderPattern &pattern : validFinderPatterns) {
		// Approximate the contour with a polygon.
		double epsilon = 0.1 * arcLength(pattern.contour, true);
		std::vector<cv::Point> approximatedPolygon;
		approxPolyDP(pattern.contour, approximatedPolygon, epsilon, true);
		polygons.push_back(approximatedPolygon);
	}

	std::vector<cv::Scalar> colorsPolygons;
	colorsPolygons.push_back(cv::Scalar(255, 255, 0));

	drawContours(polygons, &lineImage, &colorsPolygons);
	*/
	std::vector<cv::Scalar> colorsLines;
	colorsLines.push_back(cv::Scalar(0, 0, 255));

	for (FinderPattern& pattern : validFinderPatterns) {
		drawLines(pattern.lines, &lineImage, &colorsLines);
	}
	for (FinderPattern& pattern : validFinderPatterns) {
		for (cv::Vec4f& line : pattern.lines)
		{
			circle(lineImage, cv::Point2f(line[2], line[3]), 3, cv::Scalar(255, 255, 0), 2);
		}
	}

	return lineImage;
}

std::vector<cv::Mat> CodeFinder::drawMergedLinesAndIntersections() {
	std::vector<cv::Mat> images;
	for (QRCode& code : allCodes) {
		cv::Mat lineImage = originalImage.clone();

		drawLines(code.hLines, &lineImage);
		drawLines(code.vLines, &lineImage);

		for (int a = 0; a < code.hLines.size(); a++) {
			for (int b = 0; b < code.vLines.size(); b++) {
				cv::Point2f intersection;
				if (lineIntersection(code.hLines[a], code.vLines[b], intersection)) {
					circle(lineImage, intersection, 3, cv::Scalar(255, 255, 0), 2);
				}
			}
		}

		cv::Point2f intersection;
		if (lineIntersection(code.hLines[0], code.vLines[0], intersection)) {
			circle(lineImage, intersection, 5, cv::Scalar(255, 0, 255), 3);
		}
		if (lineIntersection(code.hLines[3], code.vLines[0], intersection)) {
			circle(lineImage, intersection, 5, cv::Scalar(0, 0, 255), 3);
		}
		if (lineIntersection(code.hLines[0], code.vLines[3], intersection)) {
			circle(lineImage, intersection, 5, cv::Scalar(0, 255, 0), 3);
		}
		if (lineIntersection(code.hLines[3], code.vLines[3], intersection)) {
			circle(lineImage, intersection, 5, cv::Scalar(255, 0, 0), 3);
		}

		images.push_back(lineImage);
	}

	return images;
}

std::vector<cv::Mat> CodeFinder::drawExtractedCodes() {
	std::vector<cv::Mat> images;
	for (QRCode& code : allCodes) {
		if (code.extractedImage.data)
			images.push_back(code.extractedImage);
	}

	return images;
}

std::vector<cv::Mat> CodeFinder::drawExtractedCodeGrids() {
	std::vector<cv::Mat> images;
	for (QRCode& code : allCodes) {
		cv::Mat image;

		if (!code.extractedImage.data)
			continue;

		cvtColor(code.extractedImage, image, cv::COLOR_GRAY2RGB);
		for (int a = 0; a < 4; a++) {
			for (int b = 0; b < 4; b++) {
				circle(image, code.transformedCorners.at<cv::Point2f>(a, b), 3, cv::Scalar(0, 0, 255), 2);
			}
		}

		circle(image, code.gridStepSize, 3, cv::Scalar(0, 255, 0), 2);

		if (code.gridStepSize.x <= 0 || code.gridStepSize.y <= 0)
		{
			continue;
		}

		std::vector<cv::Vec4f> lines;
		for (int i = -7; (i * code.gridStepSize.x) < image.cols || (i * code.gridStepSize.y) < image.rows; i++)
		{
			lines.push_back(cv::Vec4f(0, 1, code.gridStepSize.x * i, code.gridStepSize.y * i));
		}
		for (int i = -7; (i * code.gridStepSize.x) < image.cols || (i * code.gridStepSize.y) < image.rows; i++)
		{
			lines.push_back(cv::Vec4f(1, 0, code.gridStepSize.x * i, code.gridStepSize.y * i));
		}

		std::vector<cv::Scalar> colors;
		colors.push_back(cv::Scalar(255, 0, 255));

		drawLines(lines, &image, &colors);

		images.push_back(image);
	}

	return images;
}

std::vector<cv::Mat> CodeFinder::drawResized()
{
	std::vector<cv::Mat> images;
	for (QRCode& code : allCodes)
	{
		if (code.qrcodeImage.data)
			images.push_back(code.qrcodeImage);
	}

	return  images;
}

std::vector<QRCode> CodeFinder::getAllCodes()
{
	return allCodes;
}

cv::Mat CodeFinder::drawAllSegments()
{
	cv::Mat segmentImage = originalImage.clone();
	cv::Scalar color[4];
	color[0] = cv::Scalar(0, 0, 255);
	color[1] = cv::Scalar(0, 255, 0);
	color[2] = cv::Scalar(255, 0, 0);
	color[3] = cv::Scalar(255, 255, 0);

	for (FinderPattern& pattern : validFinderPatterns) {
		for (int i = 0; i < pattern.segments.size(); i++) {
			for (cv::Point& p : pattern.segments[i]) {
				circle(segmentImage, p, 1, color[i % 4]);
			}
		}
	}

	return segmentImage;
}

cv::Mat CodeFinder::drawContours(std::vector<std::vector<cv::Point>>& vecs, cv::Mat* image, std::vector<cv::Scalar>* colors) {
	cv::Mat drawImage;
	if (!image)
		drawImage = originalImage.clone();
	else
		drawImage = *image;

	if (!colors)
		colors = &debuggingColors;

	for (int idx = 0; idx < vecs.size(); idx++) {
		cv::drawContours(drawImage, vecs, idx, (*colors)[idx % colors->size()]);
	}

	return drawImage;
}

cv::Mat CodeFinder::drawLines(std::vector<cv::Vec4f>& lines, cv::Mat* image, std::vector<cv::Scalar>* colors) {
	cv::Mat drawImage;
	if (!image)
		drawImage = originalImage.clone();
	else
		drawImage = *image;

	if (!colors)
		colors = &debuggingColors;

	for (int idx = 0; idx < lines.size(); idx++) {
		cv::Vec4f& line = lines[idx];
		int factor = 10000;
		float x = line[2];
		float y = line[3];
		float dx = line[0];
		float dy = line[1];
		cv::Point p1(x - dx * factor, y - dy * factor);
		cv::Point p2(x + dx * factor, y + dy * factor);
		cv::line(drawImage, p1, p2, (*colors)[idx % colors->size()]);
	}

	return drawImage;
}

cv::Mat CodeFinder::drawNotFound() 
{
	cv::Mat codeNotFound(1, 1, CV_8UC1);
	codeNotFound.setTo(cv::Scalar(0));
	return codeNotFound;
}