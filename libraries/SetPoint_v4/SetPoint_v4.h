#ifndef SET_POINT
# define SET_POINT


class SetPoint {

public:
	SetPoint(float, int, int, float*, float*, float*, float*, float*, float*); // constructor

	void LoadSetPoint();
		
	void RadToDeg();

	void DegToRad();

	void InverseKinY();

	void InverseKinX();


private:
	float myHeight;
	int mySetPointSize;
	int mySetPointTime;
	float* myDesiredXArray;
	float* myDesiredYArray;
	float* myDesiredYawArray;
	float* myDesiredPitchArray;
	float* myDesiredX;
	float* myDesiredY;
	float* myDesiredYaw;
	float* myDesiredPitch;
	
	float desiredX;
	float desiredY;
	float desiredYaw;
	float desiredPitch;
	float angleInDeg;
	float angleInRad;
	int signed long currentTime;
	int signed long lastTime;
	int setPointCounter;
};

#endif // !SET_POINT
