/*
  ==============================================================================

	OneEuroFilter.h
	Created: 18 Jun 2021 10:45:02am
	Author:  bkupe

  ==============================================================================
*/

#pragma once

class LowPassFilter {

	float y, a, s;
	bool initialized;

	void setAlpha(float alpha) {
		a = jlimit<float>(0, 1, alpha);
	}

public:

	LowPassFilter(float alpha = 0, float initval = 0.0f) {
		y = s = initval;
		setAlpha(alpha);
		initialized = false;
	}

	float filter(float value) {
		float result;
		if (initialized)
			result = a * value + (1.0f - a) * s;
		else {
			result = value;
			initialized = true;
		}
		y = value;
		s = result;
		return result;
	}

	float filterWithAlpha(float value, float alpha) {
		setAlpha(alpha);
		return filter(value);
	}

	bool hasLastRawValue(void) {
		return initialized;
	}

	float lastRawValue(void) {
		return y;
	}
};

class OneEuroFilter
{
public:
	OneEuroFilter();
	~OneEuroFilter();

	
	float freq;
	float minCutOff;
	float beta;
	float derivativeCutOff;

	LowPassFilter x[3];
	LowPassFilter dx[3];

	float alpha(float cutoff);
	Vector3D<float> filter(Vector3D<float> oldPos, Vector3D<float> newPos, double deltaTime);
};

