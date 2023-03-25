#include <SPI.h>
#include <MFRC522.h>
#include <LiquidCrystal.h>


const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
 
#define SS_PIN 10
#define RST_PIN 9
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
int relay_pin=8;
const int piezo = A0; // the piezo is connected to analog pin 0
const int threshold = 1023;
int LED_STRIP=A4;

 
void setup() 
{
  //Serial.begin(9600);   // Initiate a serial communication
  SPI.begin();      // Initiate  SPI bus
  mfrc522.PCD_Init();   // Initiate MFRC522
  //Serial.println("Approximate your card to the reader...");
  //Serial.println();
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Tap to charge!");
  
  pinMode(relay_pin, OUTPUT);
  pinMode(piezo,INPUT);
  pinMode(LED_STRIP,OUTPUT);
  
  
  
  

}
void loop() 
{


  // if the sensor reading is greater than the threshold:
  if (analogRead(piezo) >= threshold) 
  {
      digitalWrite(LED_STRIP, HIGH);
    
  }
  else
  {
      digitalWrite(LED_STRIP,LOW);
  }
 
  
  
  // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  {
    return;
  }
  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  {
    return;
  }
  //Show UID on serial monitor
  //Serial.print("UID tag :13 DD E1 02");
  String content= "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++) 
  {
     //Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
     //Serial.print(mfrc522.uid.uidByte[i], HEX);
     content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
     content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  //Serial.println();
  //Serial.print("Message : ");
  content.toUpperCase();
  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Tap to charge!");

  
  
  if (content.substring(1) == "97 2B 52 B3") //change here the UID of the card/cards that you want to give access
  {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Authorized access");
    
    //Serial.println("Authorized access");
    //digitalWrite(LED_STRIP, HIGH);
    
    digitalWrite(relay_pin, HIGH);
    delay(500000);
    digitalWrite(relay_pin, LOW);
    
    //Serial.println();
    delay(3000);
  }
 
 else   {
    
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Access denied");
    
    //Serial.println(" Access denied");
    //digitalWrite(relay_pin, LOW); 
    delay(3000);
  }
  
  
  
} 
