/**
 * \brief Smooth the image using gaussian blur.
 */
void ImageBinarization::computeSmoothing() {
	blurredImage = image.clone();
	blurredImage += cv::Scalar(-50, -50, -50);
	GaussianBlur(blurredImage, blurredImage, cv::Size(3, 3), 0, 0);
}

/**
 * \brief Create a color histogram of the whole image.
 */
void ImageBinarization::createHistogram() {
	bool uniform = true;
	bool accumulate = false;

	float range[] = { 0, 256 };
	const float *histRange = range;
	calcHist(&blurredImage, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);
}

/**
 * \brief Calculate the threshold value for global binarization.
 */
void ImageBinarization::setThresholdValue() {
	for (int i = 0; i < histSize && threshold_value < 0.0; i++) {
		bin += cvRound(hist.at<float>(i));
		if (bin > m && threshold_value < 0.0)
			threshold_value = i;
	}
}

/**
 * \return False.
 */
bool ImageBinarization::isLightingUneven() {
	return false;
}

/**
 * \brief Calculate global binarization.
 */
void ImageBinarization::computeGlobalThreshold() {
	threshold(blurredImage, binarizedImage, threshold_value, max_BINARY_value, cv::THRESH_OTSU);
}

/**
 * \brief Calculate local binarization
 * \param adaptiveMethod Method used for kernel.
 * \param blockSize Kernel size
 * \param C Constant subtraction for each pixel.
 */
void ImageBinarization::computeLocalThreshold(int adaptiveMethod, int blockSize, int C) {
	adaptiveThreshold(blurredImage, binarizedImage, max_BINARY_value, adaptiveMethod, cv::THRESH_BINARY, blockSize, C);
}

/**
 * \brief Returns the max index of available threshold methods.
 * \return 2.
 */
int ImageBinarization::getMaxThresholdMethod() {
	return 2;
}

/**
 * \brief Print function for console output.
 * \param thresholdMethod Enumeration of the threshold method.
 */
void ImageBinarization::printThresholdMethod(int thresholdMethod) {
	DBG("Current threshold method: ");
	switch (thresholdMethod) {
	case Global:
		DBG("GlobalThreshold");
		break;

	case LocalMean:
		DBG("LocalThresholdMean");
		break;

	case LocalGaussian:
		DBG("LocalThresholdGaussian");
		break;
	}
}

/**
 * \brief Execute the binarizer on the passed image.
 * \param image Image to binarize.
 * \param thresholdMethod Method to use for binarization.
 * \return The binarized image.
 */
cv::Mat ImageBinarization::run(cv::Mat image, int thresholdMethod) {

	this->image = image;
	computeSmoothing();
	createHistogram();

	printThresholdMethod(thresholdMethod);
	switch (thresholdMethod) {
	case Global:
		setThresholdValue();
		computeGlobalThreshold();
		break;

	case LocalMean:
		computeLocalThreshold(cv::ADAPTIVE_THRESH_MEAN_C, 11, 0);
		break;

	case LocalGaussian:
		computeLocalThreshold(cv::ADAPTIVE_THRESH_GAUSSIAN_C, 11, 0);
		break;
	}
	return binarizedImage;
}

