/*
  ==============================================================================

	OneEuroFilter.cpp
	Created: 18 Jun 2021 10:45:02am
	Author:  bkupe

  ==============================================================================
*/

OneEuroFilter::OneEuroFilter()
{
	freq = 50;
	minCutOff = 1;
	beta = 10;
	derivativeCutOff = 1;
}

OneEuroFilter::~OneEuroFilter()
{
}

float OneEuroFilter::alpha(float cutoff)
{
	float te = 1.0f / freq;
	float tau = 1.0f / (2 * MathConstants<float>::pi * cutoff);
	return 1.0f / (1.0f + tau / te);
}


Vector3D<float> OneEuroFilter::filter(Vector3D<float> oldPos, Vector3D<float> newPos, double deltaTime)
{
	freq = 1.0 / deltaTime;
	float op[3]{ oldPos.x,oldPos.y,oldPos.z };
	float p[3]{ newPos.x,newPos.y,newPos.z };

	for (int i = 0; i < 3; i++)
	{
		float deltaP = (p[i] - op[i]) * freq; 

		float edvalue = dx[i].filterWithAlpha(deltaP, alpha(derivativeCutOff));
		float cutoff = minCutOff + beta * fabs(edvalue);
		p[i] = x[i].filterWithAlpha(p[i], alpha(cutoff));
	}

	return Vector3D<float>(p[0], p[1], p[2]);
}