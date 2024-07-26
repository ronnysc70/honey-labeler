/*Ettimandl fork


  Copyright (C) 2020 by Martin Zahn
  2020-10-12 Martin Zahn      | initial version V0.1
                              Den Code kann jeder frei verwenden, ändern und hochladen wo er will,
                              solange er nicht seinen eigenen Namen drüber setzt, oder diesen kommerziell verwertet, beispielsweise
                              indem Etikettiermaschinen mit diesem Code versehen und verkauft werden.
  2024-07-01 Ronny Schmiedel  Nutzung Gleichstrommotor (H-Bridge), Setup-Menü für Stempel und Etikettenlänge

  Pinbelegung:
  Bezeichnung         GPIO    Bemerkung  
 ---------------------------------------------------                
  DRV8871        IN1  32      für DC Motor
                 IN2  33
                 
  ROTARY BIG     A    13
                 B    14

  Buttons        1    16      up
                 2    17      down
                 3    4       select
     
  Setupswitch         19
  Glas-Sensor         23      kapazitiver Sensor

  OLED          SDL   22
  SSD1306       SDA   21

  Servo-Stempel       25

 * 
 * Arduino-libraries:
 *                    - U8g2
 *                    - bounce2
 *                    - ESP32encoder
 *                    - ESP32servo
 * 
 */
#include <Arduino.h>
#include <Wire.h>

#include <U8g2lib.h>      //Display
#include <Bounce2.h>      //Entprellung

#include <Preferences.h>  // für EEPROM
// Encoder
#include <ESP32Encoder.h> //optical Encoder
ESP32Encoder encoder;

//Servo
#include <ESP32Servo.h>   //Servo für Stempel
Servo servo;
const byte servo_pin = 25;
const int servoAngleMax = 180;      //max Winkel vom Servo

//DC Motor Einstellungen
#define MOTOR_IN1 32
#define MOTOR_IN2 33
const int PWMfreq=10000;   // 10kHz
const int DCmax=255;        // maximale Geschwindigkeit
const int DCSlowly=128;    // halbe Geschwindigkeit
const int DCsneak=60;      // geringste Geschwindigkeit

//Button Einstellungen
byte button_pins[] = {16, 17, 4}; // button pins, 16,17 = up/down, 4 = select
#define NUMBUTTONS sizeof(button_pins)
Bounce * buttons = new Bounce[NUMBUTTONS];
#define longTime 600       //langer Tastendruck 0,6sek. 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

// Daten in Eeprom lesen/speichern
Preferences preferences;

//Menü Einstellungen
#define MENU_SIZE 3
#define MENUS_SIZE 5
#define MENUC_SIZE 2
#define MENUL_SIZE 2
char *menu[MENU_SIZE] = { "Etikettenlänge", "Stempel", "alles löschen" };
char *menuC[MENU_SIZE] = { "wirklich löschen?", "zurück" };
char *menuL[MENU_SIZE] = { "weiter?", "zurück" };

int cursor=0;
const byte SettingSW = 19;

// Rotary Encoder (Inkrementaler Drehgeber für Etikettenposition)
// Grün = A-Phase, Weiß = B-Phase, Rot = Vcc-Leitung +, Schwarz = GND
const byte RotaryA  = 13;      // A-Phase
const byte RotaryB  = 14;      // B-Phase

//Glassensor 
const byte startPin = 23;

//Ablaufsteuerung
enum MODUS {RUHE, START, MENU, LABEL};
byte MODUS = RUHE;

//Variablen
unsigned long temp = 0;                       // allg. Variable
long Position = 0;                            // Encodercounter
unsigned long buttonPressStartTimeStamp = 0;  // Startpunkt für Erkennung langer Tastendruck
boolean startTimeout = false;                 // für langen Tastendruck
long preferences_chksum = 0;                  // Checksumme, damit wir nicht sinnlos Prefs schreiben
unsigned int Length = 0;                      // Etikettenlänge
char ausgabe[30];                             // Fontsize 12 = 13 Zeichen maximal in einer Zeile
boolean useStamp = false;                     // Stempel Ja/nein
unsigned int stampPark = 0;                   // Stempel Ruheposition
unsigned int stampActive = 0;                 // Stempel aktive Position
unsigned int longpress = 0;                   // für langen Tastendruck

#define blinkTime 500                         // 0,5sek. Blinkzeit für NotAus Anzeige

#define delayCounter 50                       // Verzögerung für Softstart und -stop DC Motor
unsigned long delayCounterTimeStamp = 0;

void setup() {
  Serial.begin(9600);

  // Make input & enable pull-up resistors on pushbuttons
  for (int i=0; i<NUMBUTTONS; i++) {
    buttons[i].attach( button_pins[i], INPUT_PULLUP); // setup the bounce instance for the current button
    buttons[i].interval(25); // interval in ms
  }

  //Servo
  servo.attach(servo_pin,  750, 2500); // erweiterte Initialisierung, steuert nicht jeden Servo an, sonst:
  //servo.attach(servo_pin, 1000, 2000); // default Werte. Achtung, steuert den Nullpunkt weniger weit aus!
  servo.write(0);

  // Rotary Encoder groß
  //----------------
  ESP32Encoder::useInternalWeakPullResistors = puType::up;    //Pullup on

  encoder.attachHalfQuad(RotaryA, RotaryB);
  encoder.clearCount();

  // Schalter
  pinMode(SettingSW, INPUT_PULLUP); // internal pullup
  pinMode(startPin, INPUT_PULLUP);

  //DC Motor
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);

  //Startbildschirm
  display.begin();
  display.enableUTF8Print();
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(35,20);
  display.println("Honey");  
  display.setCursor(15,40);
  display.println("Labeler V0.6");
  display.sendBuffer();
  delay(2000);
  
  //Löschanimation Startbildschirm
  for (int x = 0; x<=128; x=x+2) {
    display.clearBuffer();
    display.setFont(u8g2_font_courB10_tf);
    display.setCursor(35,20);
    display.println("Honey");  
    display.setCursor(15,40);
    display.println("Labeler V0.6");
    display.setDrawColor(0);
    display.drawBox(0,0, x, 64);
    display.setDrawColor(1);
    display.setFont(u8g2_font_unifont_t_animals);
    display.drawGlyph(x, 20, 0x3d);
    display.drawGlyph(x, 40, 0x3d);
    display.sendBuffer();
    //delay(10);
  }

  MODUS=START;
    
  //EEPROM lesen
  getPreferences();
 
  labelPosStart();
  
}

void loop() {
  //Test für encoder
  //----------------------
  Position = encoder.getCount();
  if ( Position != temp ) {    
    temp = Position;
    Serial.print ("Rotary-Pos: ");
    Serial.println (Position);
  }

 
  if (digitalRead(SettingSW) == LOW && (MODUS==RUHE ))    //Menü Einstellungen
  {
     MODUS=MENU;
     showStartMenu();
     servo.write(0);
  }
  
  if (digitalRead(SettingSW) == HIGH && (MODUS==MENU))    //Rückkehr vom Menü
  {
     labelPosStart();
     processStart();
     MODUS=RUHE;
  }
  
  if (digitalRead(startPin) == LOW && MODUS == RUHE)      //Glassensor erkannt
  {
    if (Length != 0) {
       Position = 0; // Position des Rotary Encoders auf 0 setzen
       encoder.clearCount();
       MODUS = LABEL;
       processLabel();
    }
    else {
      MODUS=RUHE;
    }
  }

  if (MODUS==START)             //Start Voreinstellungen
  {
    processStart();                 
    MODUS=RUHE;
  }
  else if (MODUS==MENU)
  {
    processSettingMenu();
  }
  
}

// Motor STOP, Servo auf 0, keine Rückkehr
void notAus()
{
  boolean abbruch = false;
  boolean blinkDisplay = true;
  unsigned long blinkTimeStamp;
  
  blinkTimeStamp = millis();
  digitalWrite(MOTOR_IN1, LOW);     //Spannung vom Motor aus, sleep Mode
  digitalWrite(MOTOR_IN2, LOW);
  servo.write(0);                   //Servo auf 0-Position

  //blinkende Anzeige
  do {
  if ((millis() - blinkTimeStamp) > blinkTime) {
    blinkTimeStamp = millis();
    blinkDisplay = ! blinkDisplay;
    if (blinkDisplay) {
      display.clearBuffer();
      display.setFont(u8g2_font_courB12_tf);
      display.setCursor(2,40);
      display.println(">> NOTAUS <<");
      display.sendBuffer();
    }
    else {
      display.clearBuffer();
      display.sendBuffer();
    }
   }
  }
  while (abbruch == false);
  
}


// Zuerst Startposition des Etiketts festlegen
//--------------------------------------------------------
void labelPosStart()
{
  boolean abbruch = false;
  if (Length != 0) {
    display.clearBuffer();
    display.setFont(u8g2_font_courB10_tf);
    display.setCursor(5,10);
    display.println("Startposition");
    display.setCursor(0,30);
    display.println("Etikett wählen");
    display.setCursor(0,50);
    display.println("und bestätigen");
    display.sendBuffer();

    do {
      // process button press:
      for (int i = 0; i<NUMBUTTONS; i++) 
      {
        buttons[i].update(); // Update the Bounce instance
        if ( buttons[i].fell() ) // If it fell
        {
          switch(i) {
            case 0:                   //forward
                    digitalWrite(MOTOR_IN1, LOW);
                    analogWriteFrequency(MOTOR_IN2, PWMfreq);
                    analogWrite(MOTOR_IN2, DCSlowly);              //halbe Geschwindigkeit

                    //Erkennung  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    longpress = 0;
                    break;    

            case 1:                   //reverse
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit
                    
                    //Erkennung  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    longpress = 1;
                    break;
         
            case 2:                   // select
                    abbruch = true;
                    Position = 0; // Position des Rotary Encoders auf 0 setzen
                    encoder.clearCount();
                    break;
          }   //end switch
        }  //end if fell
        if ( buttons[i].rose() )  //if it rose
        {
          digitalWrite(MOTOR_IN1, LOW);     //Spannung aus, sleep Mode
          digitalWrite(MOTOR_IN2, LOW);
          startTimeout = false;
        } //end if rose

        //Erkennung langer Tastendruck
        if (startTimeout == true && (millis() - buttonPressStartTimeStamp) > longTime)      //Buttons long press
        {
          switch(longpress) {
            case 0:             //forward long
                    digitalWrite(MOTOR_IN1, LOW);
                    analogWriteFrequency(MOTOR_IN2, PWMfreq);
                    analogWrite(MOTOR_IN2, DCSlowly);              //halbe Geschwindigkeit
                    break;
            case 1:             //reverse long
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit
                    break;
          }   //end switch
        } //end long press
      }   //end for        
     } while (abbruch ==false);
  }  // end if Length
  
}

// Start-Anzeige 
//--------------------------------------------------------
void processStart()
{
  if (Length != 0) {
     display.clearBuffer();
     display.setCursor(15,20);
     display.setFont(u8g2_font_courB10_tf);
     display.print("Bitte Glas");
     display.setCursor(15,40);
     display.print("aufstellen");
     display.sendBuffer();
     if (useStamp) {
      servo.write(stampPark);
     }
  }
  else {
     display.clearBuffer();
     display.setCursor(30,10);
     display.setFont(u8g2_font_courB10_tf);
     display.print("keine");
     display.setCursor(13,25);
     display.print("Etiketten-");
     display.sendBuffer();
     display.setCursor(32,40);
     display.print("länge");
     display.setCursor(15,55);
     display.print("eingestellt");
     display.sendBuffer();
    
  }
  
}


//Ettiketiervorgang mit Notaus über alle Buttons
//------------------------------------------------
void processLabel()       
{
  unsigned long progress;
  boolean blinkDisplay = true;
  unsigned long blinkTimeStamp;
  
  blinkTimeStamp = millis();
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(10,30);
  display.print("Glas erkannt");
  display.sendBuffer();

  if (useStamp) {
  servo.write(stampActive);         //stempeln
  delay(300);
  servo.write(stampPark);
  delay(200);
  }
  else {
    delay(500);
  }

  if (digitalRead(startPin) == HIGH) {    //falls Glas wieder entnommen wurde
    delay(20);
    if (digitalRead(startPin) == HIGH) {
      MODUS = START;                      //Abbruch
      return;
    }
  }
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(10,30);
  display.print("Etikettieren");
  display.sendBuffer();

  //Ramp UP DC Motor
  delayCounterTimeStamp = millis();
  temp = DCsneak;
  do {
     Position = encoder.getCount();
     progress = millis() - delayCounterTimeStamp;
     if (progress >= delayCounter) {
        delayCounterTimeStamp = millis();
        digitalWrite(MOTOR_IN1, LOW);
        analogWriteFrequency(MOTOR_IN2, PWMfreq);
        analogWrite(MOTOR_IN2, temp);              
     
        for (int i = 0; i<NUMBUTTONS; i++) 
        {
          buttons[i].update(); // Update the Bounce instance
          if ( buttons[i].fell() ) // If it fell
          {
            notAus();
          }
        }
        temp++;
     }
  }
  while ((temp < DCmax) || (Position <= Length));  //Abbruch bei Etikettenende oder Motor auf volle Geschwindigkeit

  //volle Geschwindigkeit
  do {
     Position = encoder.getCount();
     for (int i = 0; i<NUMBUTTONS; i++) 
     {
       buttons[i].update(); // Update the Bounce instance
       if ( buttons[i].fell() ) // If it fell
       {
         notAus();
       }
     }
  }
  while (( Position < (Length - (Length/4)) )||(Position <= Length)); //Abbruch bei 3/4 der Etikettenlänge oder Etikettenende

  //Ramp DOWN
  delayCounterTimeStamp = millis();
  do {
     Position = encoder.getCount();
     progress = millis() - delayCounterTimeStamp;
     if (progress >= delayCounter) {
        delayCounterTimeStamp = millis();
        digitalWrite(MOTOR_IN1, LOW);
        analogWriteFrequency(MOTOR_IN2, PWMfreq);
        analogWrite(MOTOR_IN2, temp);              
     
        for (int i = 0; i<NUMBUTTONS; i++) 
        {
          buttons[i].update(); // Update the Bounce instance
          if ( buttons[i].fell() ) // If it fell
          {
             notAus();
          }
      }
      temp--;
     }
  }
  while ((temp > DCsneak) || (Position <= Length));  //Abbruch bei Etikettenende oder Motor auf Schleichfahrt

  //Schleichfahrt bis Etikettenende
  do {
     Position = encoder.getCount();
     for (int i = 0; i<NUMBUTTONS; i++) 
     {
       buttons[i].update(); // Update the Bounce instance
       if ( buttons[i].fell() ) // If it fell
       {
         notAus();
       }
     }
  }
  while (Position <= Length); //Abbruch bei Etikettenende

  //DC Motor Stop
  digitalWrite(MOTOR_IN1, LOW);     //Spannung vom Motor aus, sleep Mode
  digitalWrite(MOTOR_IN2, LOW);
  
  //blinkende Anzeige
  do {
  if ((millis() - blinkTimeStamp) > blinkTime) {
    blinkTimeStamp = millis();
    blinkDisplay = ! blinkDisplay;
    if (blinkDisplay) {
      display.clearBuffer();
      display.setFont(u8g2_font_courB10_tf);
      display.setCursor(32,20);
      display.print("Glas");
      display.setCursor(17,40);
      display.print("entfernen");
      display.sendBuffer();
    }
    else {
      display.clearBuffer();
      display.sendBuffer();
    }
   }
  }
  while(digitalRead(startPin) == LOW);      //warten bis Glas entnommen
  MODUS=START;
}
  
// Setup-Menu
//---------------------------------------------------

//Clear display and show the Main-menu.
void showStartMenu() {
  cursor=0;
  display.clearBuffer();
  // show menu items:
  display.drawFrame(3, 3, 120, 16);
  display.setCursor(26,14);
  display.setFont(u8g2_font_courB08_tf);
  display.println("Einstellungen");
  // show menu items:
  for (int i = 0; i<MENU_SIZE; i++) {
    //display.drawString(2,i,menu[i]);
    display.setCursor(10,33 + ((i) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menu[i]);
  }
  //display.setCursor(0,33);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 33, 0x42);
  display.sendBuffer();
}

//Handle Menustruct Main-Menu
void processSettingMenu()
{
  // process button press:
  for (int i = 0; i<NUMBUTTONS; i++) 
  {
    buttons[i].update(); // Update the Bounce instance
    if ( buttons[i].fell() ) // If it fell
    { 
      if (i==2) { // select
         
         executeChoice(cursor);
      }
      else {
        display.clearBuffer();
        if (i==0) { // up
          cursor++;
          if (cursor>(MENU_SIZE-1)) cursor=0;
        }
        else { // down
          cursor--;
          if (cursor<0) cursor=(MENU_SIZE-1);
        }
        // display menu
        display.drawFrame(3, 3, 120, 16);
        display.setCursor(26,14);
        display.setFont(u8g2_font_courB08_tf);
        display.println("Einstellungen"); 
        for (int i = 0; i<MENU_SIZE; i++) {
              display.setCursor(10,33 + ((i) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menu[i]);
        }
        // show cursor at new line:
        //display.setCursor(0,33 + ((cursor) * 13));
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 33 + ((cursor) * 13), 0x42);
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check
}

//Execute the tasks from MainMenu
void executeChoice(int choice) {
  switch(choice) {
      case 0 :
                menuLabel();
                break;
      case 1 :
                menuStempel();
                break;
      case 2 :
                menuClear();
                break;
  }
  
}

//Submenu LabelLength
void menuLabel()  {
  cursor=0;
  boolean abbruch=false;
  display.clearBuffer();
  // show menu items:
  for (int i = 0; i<MENUL_SIZE; i++) {
    display.setCursor(10,10 + ((i) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menuL[i]);
  }
  //display.setCursor(0,10);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 10, 0x42);
  display.sendBuffer();

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
          if (cursor == 1) {
            abbruch = true;
          }
          else {
            setLabelLenght();
            abbruch = true;
          }
        }
      else {
        display.clearBuffer();
        if (i==0) { // up
          cursor++;
          if (cursor>(MENUL_SIZE-1)) cursor=0;
        }
        else { // down
          cursor--;
          if (cursor<0) cursor=(MENUL_SIZE-1);
        }
        // display menu
        for (int i = 0; i<MENUL_SIZE; i++) {
              display.setCursor(10,10 + ((i) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menuL[i]);
        }
        // show cursor at new line:
        //display.setCursor(0,10 + ((cursor) * 13));
        //display.setFont(u8g2_font_courB08_tf);
        //display.print('>');
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 10 + ((cursor) * 13), 0x42);
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == false);
  showStartMenu();
  
}

//submenu clear preferences
void menuClear() {
  cursor=0;
  boolean abbruch=false;
  display.clearBuffer();
  // show menu items:
  for (int i = 0; i<MENUC_SIZE; i++) {
    display.setCursor(10,10 + ((i) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menuC[i]);
  }
  //display.setCursor(0,10);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 10, 0x42);
  display.sendBuffer();

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
          if (cursor == 1) {
            abbruch = true;
          }
          else {
            preferences.begin("EEPROM", false);   //Werte werden gelöscht
            preferences.clear();
            preferences.end();
            getPreferences();       // gelöschte Werte einlesen, sonst bleiben die Variablen erhalten
            
            display.clearBuffer();
            display.setFont(u8g2_font_courB10_tf);
            display.setCursor(30,20);
            display.println("alles");  
            display.setCursor(20,40);
            display.println("gelöscht");
            display.sendBuffer();
            delay(2000);
            abbruch = true;
          }
        }
      else {
        display.clearBuffer();
        if (i==0) { // up
          cursor++;
          if (cursor>(MENUC_SIZE-1)) cursor=0;
        }
        else { // down
          cursor--;
          if (cursor<0) cursor=(MENUC_SIZE-1);
        }
        // display menu
        for (int i = 0; i<MENUC_SIZE; i++) {
              display.setCursor(10,10 + ((i) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menuC[i]);
        }
        // show cursor at new line:
        //display.setCursor(0,10 + ((cursor) * 13));
        //display.setFont(u8g2_font_courB08_tf);
        //display.print('>');
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 10 + ((cursor) * 13), 0x42);
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == false);
  showStartMenu();

}

//submenu stamp
void menuStempel() {
cursor=0;
boolean abbruch = false;
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  if (useStamp) {
     display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
     display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  }
  else {
     display.setCursor(10, 23); display.print("Ruhepos.     ---");
     display.setCursor(10, 36); display.print("Aktivpos.    ---");
  }
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  // show Cursor
  //display.setCursor(0,10);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 10, 0x42);
  display.sendBuffer();

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
        //Auswertung Cursor
          switch (cursor) {
            case 0:
                    submenuUseStamp();
                    break;
            case 1:
                    if (useStamp) {
                      submenuStampPark();
                    }
                    break;
            case 2:
                    if (useStamp) {
                    submenuStampActive();
                    }
                    break;
            case 3: 
                    if (useStamp) {
                    submenuStampTest();
                    }
                    break;
            case 4: 
                    abbruch = true;
                    setPreferences();
                    break;
          }
        }
      else {
        display.clearBuffer();
        if (i==0) { // up
          cursor++;
          if (cursor>(MENUS_SIZE-1)) cursor=0;
        }
        else { // down
          cursor--;
          if (cursor<0) cursor=(MENUS_SIZE-1);
        }
       // show menu items:
        display.setFont(u8g2_font_courB08_tf);
        display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
        if (useStamp) {
          display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
          display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
        }
        else {
          display.setCursor(10, 23); display.print("Ruhepos.     ---");
          display.setCursor(10, 36); display.print("Aktivpos.    ---");
        }
        
        display.setCursor(10, 49); display.print("Stempeltest");
        display.setCursor(10, 62); display.print("beenden");
        // show cursor at new line:
        //display.setCursor(0,10 + ((cursor) * 13));
        //display.print('>');
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 10 + ((cursor) * 13), 0x42);
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == false);
  showStartMenu();
  
}

void submenuUseStamp() 
{
boolean abbruch = false;
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  if (useStamp) {
     display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
     display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  }
  else {
     display.setCursor(10, 23); display.print("Ruhepos.     ---");
     display.setCursor(10, 36); display.print("Aktivpos.    ---");
  }
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  //show cursor
  //display.setCursor(0,10);
  //display.print('*');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil 
  display.drawGlyph(0, 10, 0x46);
  display.sendBuffer();

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
            abbruch = true;   
        }
      else {
        //Use Stamp or not
        useStamp = ! useStamp; //Status tauschen
                
        display.clearBuffer();
        display.setFont(u8g2_font_courB08_tf);
        // show menu items:
        display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
        if (useStamp) {
          display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
          display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
        }
        else {
          display.setCursor(10, 23); display.print("Ruhepos.     ---");
          display.setCursor(10, 36); display.print("Aktivpos.    ---");
        }
        display.setCursor(10, 49); display.print("Stempeltest");
        display.setCursor(10, 62); display.print("beenden");
        // show special cursor
        //display.setCursor(0,10);
        //display.print('*');
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 10, 0x46);
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == false);
  
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  if (useStamp) {
      display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
      display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  }
  else {
      display.setCursor(10, 23); display.print("Ruhepos.     ---");
      display.setCursor(10, 36); display.print("Aktivpos.    ---");
  }
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  // show normal cursor
  //display.setCursor(0,10);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 10, 0x42);
  display.sendBuffer();
   
}


void submenuStampPark() 
{

boolean abbruch = false;
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  //show cursor
  //display.setCursor(0,23);
  //display.print('*'); 
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 23, 0x46);
  display.sendBuffer();

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
            abbruch = true;   
        }
      else {
        if (i==0) { // up
           if (stampPark < servoAngleMax) {
              stampPark++;
              //Erkennung  langer Tastendruck
              buttonPressStartTimeStamp = millis();
              startTimeout = true;
              longpress = 0;
              servo.write(stampPark);
            }
        }
        else { // down
          if (stampPark > 0) {
            stampPark--;
            //Erkennung  langer Tastendruck
            buttonPressStartTimeStamp = millis();
            startTimeout = true;
            longpress = 1;
            servo.write(stampPark);
            }
        }
     
        display.clearBuffer();
        display.setFont(u8g2_font_courB08_tf);
        // show menu items:
        display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
        display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
        display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
        display.setCursor(10, 49); display.print("Stempeltest");
        display.setCursor(10, 62); display.print("beenden");
        // show special cursor
        //display.setCursor(0,23);
        //display.print('*');
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 23, 0x46);
        display.sendBuffer();
      }
    } // end if button fell...
    if ( buttons[i].rose() )  //if it rose
    {
        startTimeout = false;
    } //end if button rose
    //Erkennung langer Tastendruck
        if (startTimeout == true && (millis() - buttonPressStartTimeStamp) > longTime)      //Buttons long press
        {
          delay(100);
          switch(longpress) {
            case 0:             //forward long
                    if (stampPark < servoAngleMax) {
                     stampPark++;
                     
                    }
                    break;
         
            case 1:             //reverse long
                    if (stampPark > 0) {
                     stampPark--;
                    
                    }
                    break;
            
          }   //end switch
          display.clearBuffer();
          display.setFont(u8g2_font_courB08_tf);
          // show menu items:
          display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
          display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
          display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
          display.setCursor(10, 49); display.print("Stempeltest");
          display.setCursor(10, 62); display.print("beenden");
          // show special cursor
          //display.setCursor(0,23);
          //display.print('*');
          display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
          display.drawGlyph(0, 23, 0x46);
          display.sendBuffer();

          servo.write(stampPark);
          
        } //end long press
  } // end for-loop of button check

  } while (abbruch == false);
  
  if (stampPark > stampActive) {
    stampActive = stampPark;  //aktive Position muss größer sein als die Parkposition
    
  }
    
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  // show normal cursor
  //display.setCursor(0,23);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 23, 0x42);
  display.sendBuffer();
  
  servo.write(0);
     
}


void submenuStampActive() 
{
boolean abbruch = false;
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  //show cursor
  //display.setCursor(0,36);
  //display.print('*'); 
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 36, 0x46);
  display.sendBuffer();

  servo.write(stampActive);

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
            abbruch = true;   
        }
      else {
        if (i==0) { // up
          if (stampActive < servoAngleMax) {
            stampActive++;
            servo.write(stampActive);
          }
        }
        else { // down
          if (stampActive > 0) {
            stampActive--;
            if (stampActive < stampPark) {
              stampActive = stampPark;
            }
            servo.write(stampActive);
          }
        }   
        display.clearBuffer();
        display.setFont(u8g2_font_courB08_tf);
        // show menu items:
        display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
        display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
        display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
        display.setCursor(10, 49); display.print("Stempeltest");
        display.setCursor(10, 62); display.print("beenden");
        // show special cursor
        //display.setCursor(0,36);
        //display.print('*');
        display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
        display.drawGlyph(0, 36, 0x46);
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == false);
  
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  // show normal cursor
  //display.setCursor(0,36);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 36, 0x42);
  display.sendBuffer();

  servo.write(0);
   
}

void submenuStampTest()
{
  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest   >>");
  display.setCursor(10, 62); display.print("beenden");
  // show cursor
  //display.setCursor(0,49);
  //display.print('*');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 49, 0x46);
  display.sendBuffer();

  servo.write(stampPark);
  delay(1000);

  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest   >>>>");
  display.setCursor(10, 62); display.print("beenden");
  // show normal cursor
  //display.setCursor(0,49);
  //display.print('*');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 49, 0x46);
  display.sendBuffer();

  servo.write(stampActive);
  delay(500);
  servo.write(stampPark);

  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest   >>");
  display.setCursor(10, 62); display.print("beenden");
  // show normal cursor
  //display.setCursor(0,49);
  //display.print('*');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 49, 0x46);
  display.sendBuffer();

  delay(1000);

  display.clearBuffer();
  display.setFont(u8g2_font_courB08_tf);
  // show menu items:
  display.setCursor(10, 10); sprintf(ausgabe,"Stempel      %3s", (useStamp==false?"aus":"ein")); display.print(ausgabe);
  display.setCursor(10, 23); sprintf(ausgabe,"Ruhepos.     %3d", stampPark);  display.print(ausgabe);
  display.setCursor(10, 36); sprintf(ausgabe,"Aktivpos.    %3d", stampActive); display.print(ausgabe);
  display.setCursor(10, 49); display.print("Stempeltest");
  display.setCursor(10, 62); display.print("beenden");
  // show normal cursor
  //display.setCursor(0,49);
  //display.print('>');
  display.setFont(u8g2_font_open_iconic_arrow_1x_t);      //neuer Pfeil
  display.drawGlyph(0, 49, 0x42);
  display.sendBuffer();

  servo.write(0);
  


  
}

// Functions
//---------------------------------


//Ablaufsteuerung für Festlegung der Etikettenlänge
void setLabelLenght()
{
  boolean abbruch = false;
    display.clearBuffer();
    display.setFont(u8g2_font_courB10_tf);
    display.setCursor(5,10);
    display.println("Startposition");
    display.setCursor(0,30);
    display.println("Etikett wählen");
    display.setCursor(0,50);
    display.println("und bestätigen");
    display.sendBuffer();

    do {
      // process button press:
      for (int i = 0; i<NUMBUTTONS; i++) 
      {
        buttons[i].update(); // Update the Bounce instance
        if ( buttons[i].fell() ) // If it fell
        {
          switch(i) {
            case 0:                   //forward
                    digitalWrite(MOTOR_IN1, LOW);
                    analogWriteFrequency(MOTOR_IN2, PWMfreq);
                    analogWrite(MOTOR_IN2, DCSlowly);              //halbe Geschwindigkeit

                    //Erkennung  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    longpress = 0;
                    break;    

            case 1:                   //reverse
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit
                    
                    //Erkennung  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    longpress = 1;
                    break;
         
            case 2:                   // select
                    abbruch = true;
                    Position = 0; // Position des Rotary Encoders auf 0 setzen
                    encoder.clearCount();
                    break;
          }   //end switch
        }  //end if fell
        if ( buttons[i].rose() )  //if it rose
        {
          digitalWrite(MOTOR_IN1, LOW);     //Spannung aus, sleep Mode
          digitalWrite(MOTOR_IN2, LOW);
          startTimeout = false;
        } //end if rose

        //Erkennung langer Tastendruck
        if (startTimeout == true && (millis() - buttonPressStartTimeStamp) > longTime)      //Buttons long press
        {
          switch(longpress) {
            case 0:             //forward long
                    digitalWrite(MOTOR_IN1, LOW);
                    analogWriteFrequency(MOTOR_IN2, PWMfreq);
                    analogWrite(MOTOR_IN2, DCSlowly);              //halbe Geschwindigkeit
                    break;
            case 1:             //reverse long
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit
                    break;
          }   //end switch
        } //end long press
      }   //end for        
     } while (abbruch == false);

   abbruch = false;

// Position des nächten Etikettenanfangs anfahren und gezählte Schritte speichern
    display.clearBuffer();
    display.setFont(u8g2_font_courB10_tf);
    display.setCursor(5,10);
    display.println("Startposition");
    display.setCursor(9,25);
    display.println("vom nächsten");
    display.setCursor(0,40);
    display.println("Etikett wählen");
    display.setCursor(0,55);
    display.println("und bestätigen");
    
    display.sendBuffer();

    do {
      //Encoder aktualisieren
      Position = encoder.getCount();
      
      // process button press:
      for (int i = 0; i<NUMBUTTONS; i++) 
      {
        buttons[i].update(); // Update the Bounce instance
        if ( buttons[i].fell() ) // If it fell
        {
          switch(i) {
            case 0:                   //forward
                    digitalWrite(MOTOR_IN1, LOW);
                    analogWriteFrequency(MOTOR_IN2, PWMfreq);
                    analogWrite(MOTOR_IN2, DCSlowly);              //halbe Geschwindigkeit

                    //Erkennung  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    longpress = 0;
                    break;    

            case 1:                   //reverse
                    if (Position > 0) {             //bei Position 0 geht es nur vorwärts
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit

                    //Erkennung  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    longpress = 1;
                    }
                    break;
         
            case 2:                   // select
                    abbruch = true;
                    Length = Position;                      //speichern der Länge im EEPROM
                    setPreferences();
                    break;
          }   //end switch
        }  //end if fell
        if ( buttons[i].rose() )  //if it rose
        {
          digitalWrite(MOTOR_IN1, LOW);     //Spannung aus, sleep Mode
          digitalWrite(MOTOR_IN2, LOW);
          startTimeout = false;
        } //end if rose

        //Erkennung langer Tastendruck
        if (startTimeout == true && (millis() - buttonPressStartTimeStamp) > longTime)      //Buttons long press
        
        {
          switch(longpress) {
            case 0:             //forward long
                    digitalWrite(MOTOR_IN1, LOW);
                    analogWriteFrequency(MOTOR_IN2, PWMfreq);
                    analogWrite(MOTOR_IN2, DCSlowly);              //halbe Geschwindigkeit
                    break;
            case 1:             //reverse long
                    if (Position > 0) {     // bei Position 0 geht es nur vorwärts           
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit
                    }
                    break;
          }   //end switch
        } //end long press
      }   //end for        
     } while (abbruch == false);
    
}


void getPreferences() // Daten aus Eeprom lesen
{

  preferences.begin("EEPROM", false);       //Parameter aus eeprom lesen
  Length = preferences.getUInt("Length", 0);
  useStamp = preferences.getBool("useStamp", false);
  stampPark = preferences.getUInt("stampPark", 0);
  stampActive = preferences.getUInt("stampActive", 0);
  preferences.end();

  //Checksumme berechnen durch Addition der Werte
  preferences_chksum = Length + useStamp + stampPark + stampActive;

}



void setPreferences() // Daten in Eeprom schreiben
{
 long preferences_newchksum;

  //Checksumme berechnen durch Addition der Werte
  preferences_newchksum = Length + useStamp + stampPark + stampActive;
  
  if ( preferences_newchksum != preferences_chksum ) {    //nur bei Änderungen in den EEPROM speichern
    preferences.begin("EEPROM", false);
    preferences.putUInt("Length", Length);
    preferences.putBool("useStamp", useStamp);
    preferences.putUInt("stampPark", stampPark);
    preferences.putUInt("stampActive", stampActive);
    preferences.end();
    
    display.clearBuffer();
    display.setFont(u8g2_font_courB10_tf);
    display.setCursor(20,20);
    display.print("Änderungen");
    display.setCursor(15,40);
    display.print("gespeichert");
    display.sendBuffer();
    delay(1000);
  }
   
 
  preferences_chksum = preferences_newchksum;  
  

}
