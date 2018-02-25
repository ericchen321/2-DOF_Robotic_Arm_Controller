/* take existing Arduino stuff */
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


/* include header */
#include <SetPoint.h>


/* functions */
// cosntructor: create hard links
SetPoint::SetPoint(double InHeight, int InSetPointSize, double* InDesiredXArray, double* InDesiredYArray, double* InDesiredX, double* InDesiredY, double* InDesiredYaw, double* InDesiredPitch) {
	myHeight = InHeight;
	mySetPointSize = InSetPointSize;
	myDesiredXArray = InDesiredXArray;
	myDesiredYArray = InDesiredYArray;
	myDesiredX = InDesiredX;
	myDesiredY = InDesiredY;
	myDesiredYaw = InDesiredYaw;
	myDesiredPitch = InDesiredPitch;
	currentTime = 0;
	lastTime = 0;
	setPointCounter = 0;
}

// load coordinates and angles into the object
void SetPoint::LoadKinParams() {
	desiredX = *myDesiredX;
	desiredY = *myDesiredY;
	desiredYaw = *myDesiredYaw;
	desiredPitch = *myDesiredPitch;
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
	angleInRad = atan2(desiredY, myHeight);
	RadToDeg();
	desiredPitch = angleInDeg;
	*myDesiredPitch = desiredPitch;
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

	*myDesiredX = *(myDesiredXArray + setPointCounter);
	*myDesiredY = *(myDesiredYArray + setPointCounter);
}
