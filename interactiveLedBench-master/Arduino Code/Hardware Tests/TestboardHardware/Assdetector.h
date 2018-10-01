#include "CapacitiveSensor.h"
#include "SimpleKalmanFilter.h"

class AssDetector{

	int debugLedPin;
	int ledState;

	int currentCoupling;

	int assDetected;
	int thresholdHard;

	unsigned long interval;
	unsigned long previousTime;
	
	CapacitiveSensor cs;
	SimpleKalmanFilter kalmanFilter;

	public:
	AssDetector( int sendPin_, int receivePin_, int debugLedPin_): cs(sendPin_,receivePin_), kalmanFilter(10000,10000,0.05){

		debugLedPin = debugLedPin_;
		
		currentCoupling = 0;

		ledState = LOW;
		assDetected = LOW;	

		//turn the led off
		pinMode(debugLedPin, OUTPUT);
		digitalWrite(debugLedPin, ledState);
		
		//Turn off autocalibration
		cs.set_CS_AutocaL_Millis(0xFFFFFFFF);

	}

	void UpdateHard(){
	
		//calculate the current coupling value filtering using kalman
		currentCoupling = kalmanFilter.updateEstimate(cs.capacitiveSensor(20));

		if(currentCoupling > thresholdHard){
			//detection
			assDetected = HIGH;
			ledState = HIGH;
		}

		else if(currentCoupling < thresholdHard){

			assDetected = LOW;
			ledState = LOW;
		}

		digitalWrite(debugLedPin, ledState);
	}

	int ReturnState(){

		//return currentCoupling;
		return assDetected;
	}

	int DebugValues(){

		return currentCoupling;
	}

	void CalculateTreshold(){
		//threshold should be an increase of 20%
		//fill the kalmanFilter with data

		digitalWrite(debugLedPin, HIGH);
		for(int i=0; i<50; i++){
			currentCoupling = kalmanFilter.updateEstimate(cs.capacitiveSensor(20));
			delay(100);  
		}
		
		//calculate the threshold
		thresholdHard = (0.7*currentCoupling)+currentCoupling;
		//thresholdHard = (currentCoupling + 2000);
		digitalWrite(debugLedPin, LOW);
	}

	void UpdateTreshold(){
		//Recalibrate the threshold

		digitalWrite(debugLedPin, HIGH);
		
		currentCoupling = kalmanFilter.updateEstimate(cs.capacitiveSensor(20));
			
		//calculate the threshold
		thresholdHard = (0.7*currentCoupling)+currentCoupling;
		digitalWrite(debugLedPin, LOW);
	}
};