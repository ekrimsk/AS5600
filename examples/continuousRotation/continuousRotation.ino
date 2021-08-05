

#include <AS5600.h>

AS5600 encoder;

long revolutions = 0;   // number of revolutions the encoder has made
double position = 0;    // the calculated value the encoder is at

double angle; 


double output;          // raw value from AS5600
long lastOutput;        // last output from AS5600


// Options to check address 



void setup() {
  delay(100); 
  Serial.begin(115200);

  // If testing encoder with another address saved e.g. 0x41, 0x42
  encoder.setI2CAddress(0x50); 

  output = encoder.getPosition();
  lastOutput = output;
  position = output;
}

void loop() {
  output = encoder.getPosition();           // get the raw value of the encoder                      
  
  if ((lastOutput - output) > 2047 )        // check if a full rotation has been made
    revolutions++;
  if ((lastOutput - output) < -2047 )
    revolutions--;
    
  position = revolutions * 4096 + output;   // calculate the position the the encoder is at based off of the number of revolutions

  int agc = encoder.getGain();

  Serial.print("Output : "); Serial.println(output);
  Serial.print("Revolutions: "); Serial.println(revolutions);
  Serial.print("Angle: "); Serial.println(360.0*output/4096.0); 
  Serial.print("AGC : "); Serial.println(agc);

  lastOutput = output;                      // save the last raw value for the next loop 

  delay(10); 
}
