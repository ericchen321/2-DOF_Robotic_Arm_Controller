/* take existing Arduino stuff */
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


/* include header */
#include <SetPoint_v4.h>


/* functions */
// cosntructor: create hard links
SetPoint::SetPoint(float InHeight, int InSetPointSize, int InSetPointTime, float* InDesiredXArray, float* InDesiredYArray, float* InDesiredYawArray, float* InDesiredPitchArray, float* InDesiredYaw, float* InDesiredPitch) {
	myHeight = InHeight;
	mySetPointSize = InSetPointSize;
	mySetPointTime = InSetPointTime;
	myDesiredXArray = InDesiredXArray;
	myDesiredYArray = InDesiredYArray;
	myDesiredYawArray = InDesiredYawArray;
	myDesiredPitchArray = InDesiredPitchArray;
	myDesiredYaw = InDesiredYaw;
	myDesiredPitch = InDesiredPitch;
	currentTime = 0;
	lastTime = 0;
	setPointCounter = 0;
}

// convert deg to rad
void SetPoint::DegToRad() {
	angleInRad = angleInDeg * 3.1416 / 180.0;
}

// conveert rad to deg
void SetPoint::RadToDeg() {
	angleInDeg = angleInRad * (1.0 / 3.1416) * 180.0;
}

// inverse kin on y axis: y coord -> pitch angle
void SetPoint::InverseKinY() {
	int i = 0;

	for (i = 0; i < mySetPointSize; i++) {
		angleInRad = atan2(myDesiredYArray[i], myHeight);
		RadToDeg();
		myDesiredPitchArray[i] = angleInDeg;
	}
}

// inverse kin on x axis: x coord -> yaw angle
void SetPoint::InverseKinX() {
	int i = 0;

	for (i = 0; i < mySetPointSize; i++) {
		angleInRad = atan2(myDesiredXArray[i], myHeight);
		RadToDeg();
		myDesiredYawArray[i] = angleInDeg;
	}
}

// Checking if the next set point should be loaded. If so load in the next set point from array
void SetPoint::LoadSetPoint() {
	currentTime = millis();
	if (currentTime >= lastTime + 1000 * mySetPointTime) {
		setPointCounter++;
		lastTime = currentTime;
	}

	if (setPointCounter >= mySetPointSize) {
		setPointCounter = 0;
	}

	*myDesiredYaw = myDesiredYawArray[setPointCounter];
	*myDesiredPitch = myDesiredPitchArray[setPointCounter];
}
