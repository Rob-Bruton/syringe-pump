
// Final working code for 300mL syringe with speed settings menu in mL/min for use with pumps using lcd keypad from Sparkfun


// Controls a stepper motor via an LCD keypad shield.
// Accepts triggers and serial commands.
// To run, you will need the LCDKeypad library installed - see libraries dir.

// Serial commands:
// Set serial baud rate to 57600 and terminate commands with newlines.
// Send a number, e.g. "100", to set bolus size.
// Send a "+" to push that size bolus.
// Send a "-" to pull that size bolus.

#include <LiquidCrystal.h>
#include <LCDKeypad.h>
#include <AccelStepper.h>
#include <Adafruit_RGBLCDShield.h>

/* -- Constants -- */
#define SYRINGE_VOLUME_ML 300.0
#define SYRINGE_BARREL_LENGTH_MM 155.0

#define THREADED_ROD_PITCH 1.25
#define STEPS_PER_REVOLUTION 200.0
#define MICROSTEPS_PER_STEP 16.0

#define SPEED_MICROSECONDS_DELAY 450 //longer delay = lower speed (450 = 50mL/min for 300mL syringe) (2250 = 10mL/min for 300mL syringe)

#define mLBolusStep 10.0

float SpeedDisplay = 50.0;


long ustepsPerMM = MICROSTEPS_PER_STEP * STEPS_PER_REVOLUTION / THREADED_ROD_PITCH;
long ustepsPerML = (MICROSTEPS_PER_STEP * STEPS_PER_REVOLUTION * SYRINGE_BARREL_LENGTH_MM) / (SYRINGE_VOLUME_ML * THREADED_ROD_PITCH );

/* -- Pin definitions -- */
int motorDirPin = 2;
int motorStepPin = 3;

int triggerPin = A3;
int bigTriggerPin = A4;

/* -- Keypad states -- */
int adc_key_val[5] ={622, 822, 863, 913, 940};

enum{KEY_SELECT, KEY_RIGHT, KEY_LEFT, KEY_DOWN, KEY_UP, KEY_NONE};
int NUM_KEYS = 5;
int adc_key_in;
int key = KEY_NONE;

/* -- Enums and constants -- */
enum{PUSH,PULL}; //syringe movement direction
enum{MAIN, SPEED_MENU}; //UI states

const int SpeedSettingsLength = 3;
float SpeedSettings[3] = {2250.0, 450.0, 50.0}; //2250 is 10mL/min and 450 is 50mL/min

/* -- Default Parameters -- */
float mLBolus = 300.00; //default bolus size
int SpeedSettingsIdx = 1; //50mL/min default
float SpeedSetting = SpeedSettings[SpeedSettingsIdx];


long stepperPos = 0; //in microsteps
char charBuf[16];

//debounce params
long lastKeyRepeatAt = 0;
long keyRepeatDelay = 400;
long keyDebounce = 125;
int prevKey = KEY_NONE;
        
//menu stuff
int uiState = MAIN;

//triggering
int prevBigTrigger = HIGH;
int prevTrigger = HIGH;

//serial
String serialStr = "";
boolean serialStrReady = false;

/* -- Initialize libraries -- */
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

void setup(){
  /* LCD setup */  
  lcd.begin(16, 2);
  lcd.clear();

  lcd.print("SyringePump");

  /* Triggering setup */
  pinMode(triggerPin, INPUT);
  pinMode(bigTriggerPin, INPUT);
  digitalWrite(triggerPin, HIGH); //enable pullup resistor
  digitalWrite(bigTriggerPin, HIGH); //enable pullup resistor
  
  /* Motor Setup */ 
  pinMode(motorDirPin, OUTPUT);
  pinMode(motorStepPin, OUTPUT);
  
  /* Serial setup */
  //Note that serial commands must be terminated with a newline
  //to be processed. Check this setting in your serial monitor if 
  //serial commands aren't doing anything.
  Serial.begin(57600); //Note that your serial connection must be set to 57600 to work!
}

void loop(){
  //check for LCD updates
  readKey();
  
  //look for triggers on trigger lines
  checkTriggers();
  
  //check serial port for new commands
  readSerial();
	if(serialStrReady){
		processSerial();
	}
}

void checkTriggers(){
		//check low-reward trigger line
    int pushTriggerValue = digitalRead(triggerPin);
    if(pushTriggerValue == HIGH && prevTrigger == LOW){
      bolus(PUSH);
			updateScreen();
    }
    prevTrigger = pushTriggerValue;
}

void readSerial(){
		//pulls in characters from serial port as they arrive
		//builds serialStr and sets ready flag when newline is found
		while (Serial.available()) {
			char inChar = (char)Serial.read(); 
			if (inChar == '\n') {
				serialStrReady = true;
			} 
                        else{
			  serialStr += inChar;
                        }
		}
}

void processSerial(){
	//process serial commands as they are read in
	if(serialStr.equals("+")){
		bolus(PUSH);
		updateScreen();
	}
	else if(serialStr.equals("-")){
		bolus(PULL);
		updateScreen();
	}
        else if(serialStr.toInt() != 0){
          int uLbolus = serialStr.toInt();
          mLBolus = (float)uLbolus / 1000.0;
          updateScreen();
        }
        else{
           Serial.write("Invalid command: ["); 
           char buf[40];
           serialStr.toCharArray(buf, 40);
           Serial.write(buf); 
           Serial.write("]\n"); 
        }
        serialStrReady = false;
	serialStr = "";
}

void bolus(int direction){
        //Move stepper. Will not return until stepper is done moving.        
  
	//change units to steps
	long steps = (mLBolus * ustepsPerML);
	if(direction == PUSH){
                digitalWrite(motorDirPin, HIGH);
		steps = mLBolus * ustepsPerML;
	}
	else if(direction == PULL){
                digitalWrite(motorDirPin, LOW);

	}	

      float usDelay = SpeedSetting;
    
      for(long i=0; i < steps; i++){ 
        digitalWrite(motorStepPin, HIGH); 
        delayMicroseconds(usDelay); 
    
        digitalWrite(motorStepPin, LOW); 
        delayMicroseconds(usDelay); 
      } 

}

void readKey(){
	//Some UI niceness here. 
        //When user holds down a key, it will repeat every so often (keyRepeatDelay).
        //But when user presses and releases a key, 
        //the key becomes responsive again after the shorter debounce period (keyDebounce).

	adc_key_in = analogRead(0);
	key = get_key(adc_key_in); // convert into key press

	long currentTime = millis();
        long timeSinceLastPress = (currentTime-lastKeyRepeatAt);
        
        boolean processThisKey = false;
	if (prevKey == key && timeSinceLastPress > keyRepeatDelay){
          processThisKey = true;
        }
        if(prevKey == KEY_NONE && timeSinceLastPress > keyDebounce){
          processThisKey = true;
        }
        if(key == KEY_NONE){
          processThisKey = false;
        }  
        
        prevKey = key;
        
        if(processThisKey){
          doKeyAction(key);
  	  lastKeyRepeatAt = currentTime;
        }
}

void doKeyAction(unsigned int key){
	if(key == KEY_NONE){
        return;
    }

	if(key == KEY_SELECT){
		if(uiState == MAIN){
			uiState = SPEED_MENU;
		}
		else if(SPEED_MENU){
			uiState = MAIN;
		}
	}

	if(uiState == MAIN){
		if(key == KEY_LEFT){
			bolus(PULL);
		}
		if(key == KEY_RIGHT){
			bolus(PUSH);
		}
		if(key == KEY_UP){
			mLBolus += mLBolusStep;
		}
		if(key == KEY_DOWN){
			if((mLBolus - mLBolusStep) > 0){
			  mLBolus -= mLBolusStep;
			}
			else{
			  mLBolus = 0;
			}
		}
	}
	else if(uiState == SPEED_MENU){
		if(key == KEY_LEFT){
			//nothin'
		}
		if(key == KEY_RIGHT){
			//nothin'
		}
		if(key == KEY_UP){
			if(SpeedSettingsIdx < SpeedSettingsLength-1){
				SpeedSettingsIdx++;
				SpeedSetting = SpeedSettings[SpeedSettingsIdx];
			}
		}
		if(key == KEY_DOWN){
			if(SpeedSettingsIdx > 0){
				SpeedSettingsIdx -= 1;
				SpeedSetting = SpeedSettings[SpeedSettingsIdx];
			}
		}
	}

	updateScreen();
}




void updateScreen(){
  //convert speed delay settings to mL/min values
  if(SpeedSetting == 2250.0){
    SpeedDisplay = 10.0;
  }
  else if(SpeedSetting == 450.0){
    SpeedDisplay = 50.0;
  }
  else if(SpeedSetting == 50.0){
   SpeedDisplay = 400.0;
  }
  
	//build strings for upper and lower lines of screen
	String s1; //upper line
	String s2; //lower line
	
	if(uiState == MAIN){
		s1 = (String("Speed: ") + String(SpeedDisplay, 0) + String(" mL/m"));
		s2 = (String("Volume: ") + String(mLBolus, 0) + String(" mL"));		
	}
	else if(uiState == SPEED_MENU){
		s1 = String("Menu> Speed Set");
		s2 = (String(SpeedDisplay, 0) + String(" mL/m"));
	}

	//do actual screen update
	lcd.clear();

	s2.toCharArray(charBuf, 16);
	lcd.setCursor(0, 1);  //line=2, x=0
	lcd.print(charBuf);
	
	s1.toCharArray(charBuf, 16);
	lcd.setCursor(0, 0);  //line=1, x=0
	lcd.print(charBuf);
}


// Convert ADC value to key number
int get_key(unsigned int input){
  int k;
  for (k = 0; k < NUM_KEYS; k++){
    if (input < adc_key_val[k]){
      return k;
    }
  }
  if (k >= NUM_KEYS){
    k = KEY_NONE;     // No valid key pressed
  }
  return k;
}

String decToString(float decNumber){
	//not a general use converter! Just good for the numbers we're working with here.
	int wholePart = decNumber; //truncate
	int decPart = round(abs(decNumber*1000)-abs(wholePart*1000)); //3 decimal places
        String strZeros = String("");
        if(decPart < 10){
          strZeros = String("00");
        }  
        else if(decPart < 100){
          strZeros = String("0");
        }
	return String(wholePart) + String('.') + strZeros + String(decPart);
}
