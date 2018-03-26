#ifndef SET_POINT
# define SET_POINT


class SetPoint {

public:
	#define SETPOINT_PERIOD 8 // set point period measured in milliseconds


	SetPoint(float, int, float*, float*, float*, float*, float*, float*); // constructor

	void LoadSetPoint();
		
	void RadToDeg();

	void DegToRad();

	void InverseKinY();


private:
	float myHeight;
	int mySetPointSize;
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
