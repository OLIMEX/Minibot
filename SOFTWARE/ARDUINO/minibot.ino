/*
Robot version Olimex
ATtiny85, 2 motors, 3 LED's, four sensors

Sensor stuff:
both white  :  no prior  :  random walk
            :  prior     :  turn in direction of last black

one white   :  ideal situation, move straight

both black  :  no prior  :  random walk
            :  prior     :  turn in direction of last white

This code was initialy written by Eric Heisler (shlonkin)
It is in the public domain, so do what you will with it.

Modified for olimex's minibot by Chris B. (d3v1c3nv11)
Hardware Revision A1
*/


//=========================================================================================
// Here are some parameters that you need to adjust for your setup


// Base speeds are the motor pwm values. Adjust them for desired motor speeds.
#define baseLeftSpeed 49
#define baseRightSpeed 49

// some behavioral numbers
// these are milisecond values
#define steplength 300
#define smallturn 200
#define bigturn 500
#define memtime 5000

#define PRESC 2
//==========================================================================================

// the pin definitions
#define lmotorpin 1 // PB1 pin 6
#define rmotorpin 0 // PB0 pin 5
#define lsensepin 3 //ADC3 pin 2
#define rsensepin 1 //ADC1 pin 7
#define ledpin 4 //PB4 pin 3
#define fledpin 5


// variables
uint8_t lspd, rspd, lspeedreq, rspeedreq, prescaler; // motor speeds
uint16_t rambient, lambient, lsenseval, rsenseval,flsenseval, frsenseval, lwhiteval, rwhiteval, lblack, rblack, rmaxwhiteval, lmaxwhiteval; // sensor values

// just for convenience and simplicity (HIGH is on)
#define ledon PORTB |= 0b00010000
#define ledoff PORTB &= 0b11101111

#define fledon digitalWrite(fledpin,0);
#define fledoff digitalWrite(fledpin,1);

void setup(){
  // setup pins
  pinMode(lmotorpin, OUTPUT);
  pinMode(rmotorpin, OUTPUT);
  pinMode(2, INPUT); // these are the sensor pins, but the analog
  pinMode(3, INPUT); // and digital functions use different numbers
  ledoff;
  pinMode(ledpin, OUTPUT);
  pinMode(fledpin, OUTPUT);
  analogWrite(lmotorpin, 0);
  analogWrite(rmotorpin, 0);

  lspd = baseLeftSpeed;
  rspd = baseRightSpeed;

  // give a few second pause to set the thing on a white surface
  // the LED will flash during this time
  lsenseval = 6;
  while(lsenseval){
    lsenseval--;
    flashLED(1);
    delay(989);
  }
  flashLED(4);
  delay(500);
  senseInit();
}

void loop(){
  // followEdge() contains an infinite loop, so this loop really isn't necessary
  followEdge();
}

void followEdge(){
  // now look for an edge
  uint8_t lastMove = 1; //0=straight, 1=left, 2=right
  unsigned long moveEndTime = 0; // the millis at which to stop
  unsigned long randomBits = micros(); // for a random walk

  unsigned long prior = 0; // after edge encounter set to millis + memtime
  uint8_t priorDir = 0; //0=left, 1=right, 2=both
  uint8_t lastSense = 1; //0=edge, 1=both white, 2=both black
  uint8_t i = 0; // iterator

  while(true){

    // only read sensors about once every 20ms
    // it can be done faster, but this makes motion smoother
    delay(8);
    // read the value 4 times and average
    ledon;
    delay(1);
    lsenseval = 0;
    rsenseval = 0;
    for(i=0; i<4; i++){
    	lsenseval += analogRead(lsensepin);
    	rsenseval += analogRead(rsensepin);
    }
    // don't divide by 4 because it is used below
    ledoff;

    delay(1);
    // Get ambient light values
    // read the value 4 times and average
    lambient = 0;
    rambient = 0;
    for(i=0; i<4; i++){
      lambient += analogRead(lsensepin);
      rambient += analogRead(rsensepin);
    }


    // read front light values
    fledon;

    delay(1);

    flsenseval = 0;
    frsenseval = 0;
    for(i=0; i<4; i++){
    	flsenseval += analogRead(lsensepin);
    	frsenseval += analogRead(rsensepin);
    }

    // don't divide by 4 because it is used below
    fledoff;


    // refill the random bits if needed
    if(randomBits == 0){ randomBits = micros(); }

    // Here is the important part
    // There are four possible states: both white, both black, one of each
    // The behavior will depend on current and previous states
    if((lsenseval > lwhiteval) && (rsenseval > rwhiteval)){
    	// both white - if prior turn toward black, else random walk
    	if(lastSense == 2 || millis() < prior){
    		// turn toward last black or left
    		if(priorDir == 0){
    			moveEndTime = millis()+smallturn;
    			move(0, rspd); // turn left
    			lastMove = 1;
    		}else if(priorDir == 1){
    			moveEndTime = millis()+smallturn;
    			move(lspd, 0); // turn right
    			lastMove = 2;
    		}else{
    			moveEndTime = millis()+bigturn;
    			move(0, rspd); // turn left a lot
    			lastMove = 1;
    		}
    	}else{
    		// random walk
    		if(millis() < moveEndTime){
    			// just continue moving
    		}else{
    			if(lastMove){
    				moveEndTime = millis()+steplength;
    				move(lspd, rspd); // go straight
    				lastMove = 0;
    			}else{
    				if(randomBits & 1){
    					moveEndTime = millis()+smallturn;
    					move(0, rspd); // turn left
    					lastMove = 1;
    				}else{
    					moveEndTime = millis()+smallturn;
    					move(lspd, 0); // turn right
    					lastMove = 2;
    				}
    				randomBits >>= 1;
    			}
    		}
    	}
    	lastSense = 1;

    }else if((lsenseval > lwhiteval) || (rsenseval > rwhiteval)){
    	// one white, one black - this is the edge
    	// just go straight
    	moveEndTime = millis()+steplength;
    	move(lspd, rspd); // go straight
    	lastMove = 0;
    	lastSense = 0;
    	prior = millis()+memtime;
    	if(lsenseval > lwhiteval){
    		// the right one is black
    		priorDir = 1;
    	}else{
    		// the left one is black
    		priorDir = 0;
    	}

    }else{
    	// both black - if prior turn toward white, else random walk
    	if(lastSense == 1 || millis() < prior){
    		// turn toward last white or left
    		if(priorDir == 0){
    			moveEndTime = millis()+smallturn;
    			move(lspd, 0); // turn right
    			lastMove = 2;
    		}else if(priorDir == 1){
    			moveEndTime = millis()+smallturn;
    			move(0, rspd); // turn left
    			lastMove = 1;
    		}else{
    			moveEndTime = millis()+bigturn;
    			move(lspd, 0); // turn right a lot
    			lastMove = 2;
    		}
    	}else{
    		// random walk
    		if(millis() < moveEndTime){
    			// just continue moving
    		}else{
    			if(lastMove){
    				moveEndTime = millis()+steplength;
    				move(lspd, rspd); // go straight
    				lastMove = 0;
    			}else{
    				if(randomBits & 1){
    					moveEndTime = millis()+smallturn;
    					move(0, rspd); // turn left
    					lastMove = 1;
    				}else{
    					moveEndTime = millis()+smallturn;
    					move(lspd, 0); // turn right
    					lastMove = 2;
    				}
    				randomBits >>= 1;
    			}
    		}
    	}
    	lastSense = 2;
    }



// check for obstacles front of minibot
    if(flsenseval > lambient+10){

    move(lspd, 0);  //turn left

    //analogWrite(rmotorpin, 0); // just stop and wait
    //delay(smallturn);
    //stop();
    }
    if(frsenseval > rambient+10){

    	move(0, rspd); // turn right
    //analogWrite(lmotorpin, 0); // just stop and wait

    //delay(smallturn);
    //stop();
    }

  }




}

void move(uint8_t lspeed, uint8_t rspeed){
  analogWrite(lmotorpin, lspeed);
  analogWrite(rmotorpin, rspeed);
}
void doMove(){
  analogWrite(lmotorpin, lspeedreq);
  analogWrite(rmotorpin, rspeedreq);
}

void stop(){
  analogWrite(lmotorpin, 0);
  analogWrite(rmotorpin, 0);
}

// stores the average of 16 readings as a white value
void senseInit(){
  lwhiteval = 0;
  rwhiteval = 0;
  // Calibration on black
  ledon;
  delay(1);
  for(uint8_t i=0; i<16; i++){
    lwhiteval += analogRead(lsensepin);
    delay(1);
    rwhiteval += analogRead(rsensepin);
    delay(9);
  }

  rblack = rwhiteval >> 2;
  lblack = lwhiteval >> 2;
 ledoff;
// step forward to White zone
 move(lspd,rspd);
 delay(300);
 stop();

 lwhiteval = 0;
 rwhiteval = 0;
 ledon;
 delay(1);
 for(uint8_t i=0; i<16; i++){
   lwhiteval += analogRead(lsensepin);
   delay(1);
   rwhiteval += analogRead(rsensepin);
   delay(9);
 }


 lwhiteval >>= 2;
 rwhiteval >>= 2;


lwhiteval = (lwhiteval - lblack)/2 + lblack;
rwhiteval = (rwhiteval - rblack)/2 + rblack;
  ledoff;
}


void flashLED(uint8_t flashes){
  while(flashes){
    flashes--;
    ledon;
    delay(100);
    ledoff;
    fledon;
    delay(100);
    fledoff;

    if(flashes){ delay(500); }
  }
}
