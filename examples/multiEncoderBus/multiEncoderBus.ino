

#include <AS5600.h>


#define ENC1_ADDR 0x40
#define ENC2_ADDR 0x41 


#define ENC3_ADDR 0x42
#define ENC4_ADDR 0x44 

#define ENC5_ADDR 0x48
#define ENC6_ADDR 0x50 


AS5600 encoder1;
AS5600 encoder2;

double output1;          // raw value from AS5600
double output2;          // raw value from AS5600
// double output3;
//double output4; 
//double output5;
//double output6; 

// Options to check address 



void setup() {
  delay(100); 
  Serial.begin(115200);

  encoder1.setI2CAddress(ENC1_ADDR); 
  encoder2.setI2CAddress(ENC2_ADDR); 
  
}

void loop() {
  output1 = encoder1.getPosition();           // get the raw value of the encoder                      
  output2 = encoder2.getPosition();           // get the raw value of the encoder                      

  Serial.print("Angle 1: "); Serial.println(360.0*output1/4096.0); 
  Serial.print("Angle 2: "); Serial.println(360.0*output2/4096.0); 
  Serial.println(""); 
 
  delay(250); 
}
