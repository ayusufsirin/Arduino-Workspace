#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

File myFile;

SoftwareSerial GPS(3, 4);  // RX, TX

void setup() {
  Serial.begin(9600);
  GPS.begin(9600);
  SD.begin(10);
  myFile = SD.open("log.txt", FILE_WRITE);

  if (myFile) {
    myFile.println("This is a log file :)");
    // close the file:
    myFile.close();
  } else {
    Serial.println("error opening log.txt");
  }
}

void loop() {
  String data = GPS.readStringUntil('\r');
  
//  if (GPS.available() > 0)
    Serial.print(data);

  if (myFile) {
    myFile.println(data);
  }
}
