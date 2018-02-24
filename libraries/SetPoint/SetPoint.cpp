/* stuff */
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


/* include header */
#include <SetPoint.h>


/* functions */
// cosntructor: create hard links
SetPoint::SetPoint(double InHeight, int InSetPointSize, double* InDesiredXArray, double* InDesiredYArray, double* InDesiredYawArray, double* InDesiredPitchArray, double* InDesiredX, double* InDesiredY, double* InDesiredYaw, double* InDesiredPitch) {
	myHeight = InHeight;
	mySetPointSize = InSetPointSize;
	myDesiredXArray = InDesiredXArray;
	myDesiredYArray = InDesiredYArray;
	myDesiredYawArray = InDesiredYawArray;
	myDesiredPitchArray = InDesiredPitchArray;
	myDesiredX = InDesiredX;
	myDesiredY = InDesiredY;
	myDesiredYaw = InDesiredYaw;
	myDesiredPitch = InDesiredPitch;
	currentTime = 0;
	lastTime = 0;
	setPointCounter = 0;
}


// convert deg to rad
double SetPoint::DegToRad(double angleInDeg) {
	return (angleInDeg * 3.1416 / 180.0);
}

// conveert rad to deg
double SetPoint::RadToDeg(double angleInRad) {
	return (angleInRad * (1.0 / 3.1416) * 180.0);
}

// inverse kin on y axis: y coord -> pitch angle
void SetPoint::InverseKinY() {
	int i = 0;
	for (i = 0; i < mySetPointSize; i++) {
		myDesiredPitchArray[i] = atan2(myDesiredYArray[i], myHeight);
		myDesiredPitchArray[i] = RadToDeg(myDesiredPitchArray[i]);
	}
}

// Checking if the next set point should be loaded. If so load in the next set point from array
void SetPoint::LoadSetPoint() {
	currentTime = millis();
	if (currentTime >= lastTime + SETPOINT_PERIOD) {
		setPointCounter++;
		lastTime = currentTime;
	}

	if (setPointCounter > mySetPointSize) {
		setPointCounter = 0;
	}

	*myDesiredYaw = *(myDesiredYawArray + setPointCounter);
	*myDesiredPitch = *(myDesiredPitchArray + setPointCounter);
}
