#include <OctoWS2811.h>

const int  LEDAMOUNT = 900;

//Array to store all the colors
uint32_t currentColors[LEDAMOUNT];

DMAMEM int displayMemory[LEDAMOUNT*6];
int drawingMemory[LEDAMOUNT*6];
const int config = WS2811_GRB | WS2811_800kHz;
OctoWS2811 leds(72, displayMemory, drawingMemory, config);

//Global variables for the framerate
unsigned int frameRate = 10;
unsigned int interval= (1000/frameRate);
unsigned long previousMillis = 0;

//animation variables
int count = 0;

void setup(){
	//Initialize the library
	leds.begin();
	//Update the led's Turning them all to black/off
	leds.show();	
}

void loop(){
	
	for(int i=0; i<LEDAMOUNT; i++){
		//turn the bg greenish 
		leds.setPixel(i,255,0,0);
		leds.show();
		delay(100);
	}
}


void applyColorsToStrip(uint32_t colorArray[], int arrayLength){

	for(int i=0; i<LEDAMOUNT; i++){
		
		//fetch the colors 		
		uint8_t red = colorArray[i] >> 16;
		uint8_t green = colorArray[i] >> 8;
		uint8_t blue = colorArray[i];

		leds.setPixel(i,red, green, blue);
	}
	leds.show();
}


void putColorsInArray(int ledNumber, uint8_t red, uint8_t green, uint8_t blue){

	currentColors[ledNumber] = ((uint32_t)red << 16)| ((uint32_t)green << 8) |((uint32_t) blue);
}

uint8_t returnColor(uint32_t combinedColor, char choice){
	//returns an rgb color value based on the choice given.
	//Only accepts r g b

	if 		( choice == 'r'){
		return combinedColor >>16;
	}
	else if( choice == 'g'){
		return combinedColor >>8;
	}
	else if( choice == 'b'){
		return combinedColor;
	}
	else {
		return 0;
	}
}