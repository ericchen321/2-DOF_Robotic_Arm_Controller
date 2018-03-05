#ifndef SET_POINT
# define SET_POINT


class SetPoint {

public:
	#define SETPOINT_PERIOD 80 // set point period measured in milliseconds


	SetPoint(double, int, double*, double*, double*, double*, double*, double*); // constructor

	void LoadSetPoint();
	
	void LoadKinParams();
	
	void RadToDeg();

	void DegToRad();

	void InverseKinY();


private:
	double myHeight;
	int mySetPointSize;
	double* myDesiredXArray;
	double* myDesiredYArray;
	double* myDesiredX;
	double* myDesiredY;
	double* myDesiredYaw;
	double* myDesiredPitch;
	
	double desiredX;
	double desiredY;
	double desiredYaw;
	double desiredPitch;
	double angleInDeg;
	double angleInRad;
	unsigned long currentTime;
	unsigned long lastTime;
	int setPointCounter;
};

#endif // !SET_POINT
