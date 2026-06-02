
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9,10); 
const byte diachi[6] = "12345"; 

void log(int s) {
  Serial.print(s);
  Serial.print(" ");
}
void setup() 
{
  Serial.begin(9600);

  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);

  if (!radio.begin()) 
  {
    Serial.println("Error...!!");
    while (1) {}
  }   
  radio.openWritingPipe(diachi);
  radio.setPALevel(RF24_PA_MIN);
  radio.setChannel(80);
  radio.setDataRate(RF24_250KBPS);  
  radio.stopListening();
  if (!radio.available())
  {
    Serial.println("Wait...!!");
  }
}

int data[6] = {0,0,0,0,0,0};
int low = 321, mid = 501, high = 680; 
int low2 = 420, mid2 = 503, high2 = 603; 

void loop() 
{
  //////////
  int joy1 = analogRead(A3);
  int joy2 = analogRead(A5);
  data[0] = (joy2-mid)/(1.0*(high-mid))*255;
  data[0] = ((data[0]<50&&data[0]>-50)?0:data[0]);
  data[1] = (joy1-mid2)/(1.0*((joy1<mid2?mid2-low2:high2-mid2)))*255;
  //////////
  int sw1 = digitalRead(5);
  int sw2 = digitalRead(6);
  data[4] = sw1;
  data[5] = sw2;
  //////////
  int vol1 = analogRead(A0);
  int vol2 = analogRead(A1);
  data[2] = vol1;
  data[3] = vol2;
  //////////
  for (int i = 0; i<6; i++) {
    log(data[i]);
  }
  Serial.println("");
  radio.write(&data, sizeof(data));
  
  delay(60);
}