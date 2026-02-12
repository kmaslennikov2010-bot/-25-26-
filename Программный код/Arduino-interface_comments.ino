/******************************************************************
 *                           Libraries                            *
 ******************************************************************/ 

#include <Servo.h>


/******************************************************************
 *                    Definition of variables                     *
 ******************************************************************/

/* Pin definition of the board to be used */

#define pinLeftWheel            8   
#define pinRightWheel           9   

/* Definition of the values ​​that can take continuous rotation servo,
 that is, the wheels */
#define wheelStopValue 90
#define leftWheelFordwardValue 0
#define leftWheelBackwardsValue 180
#define rightWheelFordwardValue 180
#define rightWheelBackwardsValue 0

/* Bauderate of the Bluetooth*/
#define MI_PRIMER_KIT_DE_ROBOTICA_BLUETOOTH    38400
#define BQ_ZUM_BLUETOOTH                       19200

/* Size of the received data buffer */
#define bufferSize 5

/* Default delay */
#define defaultDelay        10

/* A object from the Servo class is created for each servo */
Servo leftWheel;                       /*  Values from 0 to 180  */
Servo rightWheel;                      /*  Values from 0 to 180  */

/*  A char buffer to storage the received data from the Bluetooth
    Serial */
char dataBuffer[bufferSize]; 

/* Buffer iterator */
int i = 0;

/* Number of characters availables in the Serial */
int numChar = 0;    


/******************************************************************
 *                     Definition of functions                    *
 ******************************************************************/



void stopWheels() {
  leftWheel.write(wheelStopValue);
  delay(defaultDelay);

  rightWheel.write(wheelStopValue);
  delay(defaultDelay);
}


void goForwards() {

  leftWheel.write(leftWheelFordwardValue);
  delay(defaultDelay);

  rightWheel.write(rightWheelFordwardValue);
  delay(defaultDelay);
}


void goBackwards() {

  leftWheel.write(leftWheelBackwardsValue);
  delay(defaultDelay);

  rightWheel.write(rightWheelBackwardsValue);
  delay(defaultDelay);
}


void goLeft() {

  leftWheel.write(leftWheelBackwardsValue);
  delay(defaultDelay);

  rightWheel.write(rightWheelFordwardValue);
  delay(defaultDelay);
}


void goRight() {

  leftWheel.write(leftWheelFordwardValue);
  delay(defaultDelay);

  rightWheel.write(rightWheelBackwardsValue);
  delay(defaultDelay);
}


void turnLeft() {
  goLeft();
  delay(900);
  stopWheels();
}


void turnRight() {
  goRight();
  delay(900);
  stopWheels();
}


/* Manage the buffer of data */
void checkData(char* data){  
  
   switch(data[0]) {

    case'S':
      /* Stop button pressed */
      stopWheels();
      break;
    
    case 'U':
      /* Up button pressed */
      goForwards();
      break;
    
    case 'D':
      /* Down button pressed */
      goBackwards();
      break;
      
    case 'L':
      /* Left button pressed */ 
      goLeft();
      break;
      
    case 'R':
      /* Right button pressed */ 
      goRight();
      break;
    
  } 

  /* Empty the Serial */   
  Serial.flush();

}



  //delay(800);

    

/******************************************************************
 *                             Setup                              *
 ******************************************************************/

void setup() {
  
  /* Open the Bluetooth Serial and empty it */
  Serial.begin(BQ_ZUM_BLUETOOTH);  
  //Serial.begin(MI_PRIMER_KIT_DE_ROBOTICA_BLUETOOTH); 
  Serial.flush();     
  
  /* Define the appropiate pin to each object */
  leftWheel.attach(pinLeftWheel);
  rightWheel.attach(pinRightWheel);

  /* The robot is stopped at the beginning */
  stopWheels();
}


/******************************************************************
 *                       Main program loop                        *
 ******************************************************************/

void loop() {
 
   /* If there is something in the Bluetooth serial port */
  if (Serial.available() > 0) { 
   
    /* Reset the iterator and clear the buffer */
    i = 0;
    memset(dataBuffer, 0, sizeof(dataBuffer));  
    
    /* Wait for let the buffer fills up. Depends on the length of 
       the data, 1 ms for each character more or less */
    delay(bufferSize); 

    /* Number of characters availables in the Bluetooth Serial */
    numChar = Serial.available();   
    
    /* Limit the number of characters that will be read from the
       Serial to avoid reading more than the size of the buffer */
    if (numChar > bufferSize) {
          numChar = bufferSize;
    }

    /* Read the Bluetooth Serial and store it in the buffer */
    while (numChar--) {
        dataBuffer[i++] = Serial.read();

        /* As data trickles in from your serial port you are 
         grabbing as much as you can, but then when it runs out 
         (as it will after a few bytes because the processor is 
         much faster than a 9600 baud device) you exit loop, which
         then restarts, and resets i to zero, and someChar to an 
         empty array.So please be sure to keep this delay */
        delay(3);
    } 

    /* Manage the data */   
    checkData(dataBuffer);
    
  }

  
}  
  
