

#include <AS5600.h>

AS5600 encoder;


// Options to check address 
// CAN ONLY BE RUN ONCE --- WARNING 

#define INITIAL_ADDRESS 0x40 
#define NEW_ADDRESS 0x50

long revolutions = 0;   // number of revolutions the encoder has made
double position = 0;    // the calculated value the encoder is at

double angle; 

double output;          // raw value from AS5600
long lastOutput;        // last output from AS5600


void setup() {
  delay(100); 
  Serial.begin(115200);
  delay(100);


  Serial.println("Changing reference address..."); 

  encoder.setI2CAddress(INITIAL_ADDRESS); // only setting the internal reference 

  Serial.println("Burning new address to non-volatile memory..."); 
  encoder.changeI2CAddress(NEW_ADDRESS); // actually change it 

  // wait after burn op 

  delay(1000); 

  
  Serial.println("Cycle chip power for address change to take effect!"); 

    
}

void loop() {

}
