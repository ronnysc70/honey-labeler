/*Ettimandl fork


  Copyright (C) 2020 by Martin Zahn
  2020-10-12 Martin Zahn      | initial version V0.1
                              Den Code kann jeder frei verwenden, ändern und hochladen wo er will,
                              solange er nicht seinen eigenen Namen drüber setzt, oder diesen kommerziell verwertet, beispielsweise
                              indem Etikettiermaschinen mit diesem Code versehen und verkauft werden.
  2024-07-01 Ronny Schmiedel  Nutzung Gleichstrommotor (H-Bridge), Setup-Menü für Stempel und Etikettenlänge

  Pinbelegung:
                 
  DRV8871        IN1  32      für DC Motor
                 IN2  33
                 
  ROTARY BIG     A    13
                 B    14

  Buttons        1    16
                 2    17
                 3    4
  Setupswitch         19
  Start-Sensor        23

  OLED          SDL   22
                SDA   21

  ServoStempel        25

 * 
 */
#include <U8g2lib.h>
#include <Bounce2.h>
#include <Preferences.h>  // für EEPROM
// Encoder
#include <ESP32Encoder.h>
ESP32Encoder encoder;

//DC Motor Einstellungen
#define MOTOR_IN1 32
#define MOTOR_IN2 33
const int PWMfreq=10000;   // 10kHz
const int DCSlowly=128;    // halbe Geschwindigkeit

//Button Einstellungen
byte button_pins[] = {16, 17, 4}; // button pins, 16,17 = up/down, 4 = select
#define NUMBUTTONS sizeof(button_pins)
Bounce * buttons = new Bounce[NUMBUTTONS];
#define longTime 1000       //langer Tastendruck

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);

// Daten in Eeprom lesen/speichern
Preferences preferences;

//Menü Einstellungen
#define MENU_SIZE 3
#define MENUS_SIZE 5
#define MENUC_SIZE 2
#define MENUL_SIZE 2
char *menu[MENU_SIZE] = { "Etikettenlänge", "Stempel", "alles löschen" };
char *menuS[MENUS_SIZE] = { "Stempel an/aus", "Stempel Ruhe", "Stempel Aktiv", "Stempeltest", "beenden"};
char *menuC[MENU_SIZE] = { "wirklich löschen?", "zurück" };
char *menuL[MENU_SIZE] = { "weiter?", "zurück" };

int cursor=0;
const byte SettingSW = 19;

// Rotary Encoder (Inkrementaler Drehgeber für Etikettenposition)
// Grün = A-Phase, Weiß = B-Phase, Rot = Vcc-Leistung +, Schwarz = V0
const byte RotaryA  = 32;      // A-Phase
const byte RotaryB  = 33;      // B-Phase

//Glassensor 
const byte startPin = 23;

//Ablaufsteuerung
enum MODUS {RUHE, START, MENU, LABELSTART, LABELRUN, LABELEND};
byte MODUS = RUHE;

//Variablen
volatile long temp, Position = 0;           // Encodercounter
unsigned long buttonPressStartTimeStamp;    // Startpunkt für Erkennung langer Tastendruck
boolean startTimeout = false;               // für langen Tastendruck
long preferences_chksum;                    // Checksumme, damit wir nicht sinnlos Prefs schreiben
int Length;                                 // Etikettenlänge
boolean useStamp = false;                   // Stempel Ja/nein
int stampPark;                              // Stempel Ruheposition
int stampActive;                            // Stempel aktive Position
          

void setup() {
  Serial.begin(9600);

  // Make input & enable pull-up resistors on pushbuttons
  for (int i=0; i<NUMBUTTONS; i++) {
    buttons[i].attach( button_pins[i], INPUT_PULLUP); // setup the bounce instance for the current button
    buttons[i].interval(25); // interval in ms
  }


  // Rotary Encoder groß
  //----------------
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

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
  display.println("Labeler V0.2");
  display.sendBuffer();
  delay(2000);


  MODUS=START;
    
  //EEPROM lesen
  getPreferences();
  Length = 10;      //testweise danach wieder löschen

  labelPosStart();
  
}

void loop() {
  Position = encoder.getCount();

  //Test für encoder
  //----------------------
  if ( Position != temp ) {    
    temp = Position;
    Serial.print ("Rotary-Pos: ");
    Serial.println (Position);
  }

 
  if (digitalRead(SettingSW) == LOW && (MODUS==RUHE ))    //Menü Einstellungen
  {
     MODUS=MENU;
     showStartMenu();
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
       MODUS = LABELSTART;
       processLabeling();
    }
    else {
      MODUS=RUHE;
    }
  }
  if (MODUS==START)
  {
    processStart();                 //Start Voreinstellungen
    MODUS=RUHE;
  }
  else if (MODUS==MENU)
  {
    processSettingMenu();
  }
  else if(MODUS != RUHE && MODUS != START && MODUS != MENU)   //Ettiketiervorgang
  {
    processLabeling();
  }

  
  //delay(10);
}

// Zuerst Startposition des Etiketts festlegen
//--------------------------------------------------------
void labelPosStart()
{
  int abbruch = 0;
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

                    //Erkennugn  langer Tastendruck
                    buttonPressStartTimeStamp = millis();
                    startTimeout = true;
                    break;    

            case 1:                   //reverse
                    digitalWrite(MOTOR_IN2, LOW);
                    analogWriteFrequency(MOTOR_IN1, PWMfreq);
                    analogWrite(MOTOR_IN1, DCSlowly);              //halbe Geschwindigkeit
                    break;
         
            case 2:                   // select
                    abbruch = 1;
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
          switch(i) {
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
     } while (abbruch == 0);
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

//TODO
//Ettiketiervorgang
//------------------------------------------------
void processLabeling()       
{
  if (MODUS==LABELSTART)
  {
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(10,30);
  display.print("Glas erkannt");
  display.sendBuffer();
  MODUS=LABELRUN;
  delay(2000);
  }
  else if (MODUS == LABELRUN)
  {
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(10,30);
  display.print("Etikettieren");
  display.sendBuffer();
  MODUS=LABELEND;
  delay(2000);
  }
  else if (MODUS==LABELEND)
  {
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(32,20);
  display.print("Glas");
  display.setCursor(17,40);
  display.print("entfernen");
  display.sendBuffer();
  MODUS=START;
  while(digitalRead(startPin) == LOW);      //warten bis Glas entnommen
  }
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
    display.setCursor(10,33 + (((i)%5) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menu[i]);
  }
  display.setCursor(0,33);
  display.print('>');
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
              display.setCursor(10,33 + (((i)%5) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menu[i]);
        }
        // show cursor at new line:
        display.setCursor(0,33 + (((cursor)%5) * 13));
        display.setFont(u8g2_font_courB08_tf);
        display.print('>');
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

//Submenu Label
void menuLabel()  {
  cursor=0;
  int abbruch=0;
  display.clearBuffer();
  // show menu items:
  for (int i = 0; i<MENUL_SIZE; i++) {
    display.setCursor(10,10 + (((i)%5) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menuL[i]);
  }
  display.setCursor(0,10);
  display.print('>');
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
            abbruch = 1;
          }
          else {
            setLabelLenght();
            abbruch = 1;
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
              display.setCursor(10,10 + (((i)%5) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menuL[i]);
        }
        // show cursor at new line:
        display.setCursor(0,10 + (((cursor)%5) * 13));
        display.setFont(u8g2_font_courB08_tf);
        display.print('>');
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == 0);
  showStartMenu();
  
}

//submenu clear preferences
void menuClear() {
  cursor=0;
  int abbruch=0;
  display.clearBuffer();
  // show menu items:
  for (int i = 0; i<MENUC_SIZE; i++) {
    display.setCursor(10,10 + (((i)%5) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menuC[i]);
  }
  display.setCursor(0,10);
  display.print('>');
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
            abbruch = 1;
          }
          else {
            preferences.begin("EEPROM", false);   //Werte werden gelöscht
            preferences.clear();
            preferences.end();
            getPreferences();       // gelöschte Werte einlesen, sonst bleiben die Variablen erhalten
            
            display.clearBuffer();
            display.setFont(u8g2_font_courB10_tf);
            display.setCursor(30,15);
            display.println("alles");  
            display.setCursor(20,35);
            display.println("gelöscht");
            display.sendBuffer();
            delay(2000);
            abbruch = 1;
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
              display.setCursor(10,10 + (((i)%5) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menuC[i]);
        }
        // show cursor at new line:
        display.setCursor(0,10 + (((cursor)%5) * 13));
        display.setFont(u8g2_font_courB08_tf);
        display.print('>');
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == 0);
  showStartMenu();

  
}

//submenu stamp
void menuStempel() {
cursor=0;
int abbruch = 0;
  display.clearBuffer();
  // show menu items:
  for (int i = 0; i<MENUS_SIZE; i++) {
    display.setCursor(10,10 + (((i)%5) * 13));
    display.setFont(u8g2_font_courB08_tf);
    display.print(menuS[i]);
  }
  display.setCursor(0,10);
  display.print('>');
  display.sendBuffer();

  do {
    // process button press:
    for (int i = 0; i<NUMBUTTONS; i++) 
    {
      buttons[i].update(); // Update the Bounce instance
      if ( buttons[i].fell() ) // If it fell
      { 
        if (i==2) { // select
          if (cursor == 4) {        //Auswertung Cursor, TODO switch(cursor)
            abbruch = 1;
            //setPreferences();
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
        // display menu
        for (int i = 0; i<MENUS_SIZE; i++) {
              display.setCursor(10,10 + (((i)%5) * 13));
              display.setFont(u8g2_font_courB08_tf);
              display.print(menuS[i]);
        }
        // show cursor at new line:
        display.setCursor(0,10 + (((cursor)%5) * 13));
        display.setFont(u8g2_font_courB08_tf);
        display.print('>');
        display.sendBuffer();
      }
    } // end if button fell...
  } // end for-loop of button check

  } while (abbruch == 0);
  showStartMenu();
  
}

// Functions
//---------------------------------

//TODO
//Ablaufsteuerung für Festlegung der Etikettenlänge
void setLabelLenght()
{
  //PLatzhalter
  display.clearBuffer();
  display.setFont(u8g2_font_courB10_tf);
  display.setCursor(10,20);
  display.println("Platzhalter");
  display.setCursor(0,40);
  display.println("Etikettenlänge");
  
  display.sendBuffer();
  delay(2000);

  
}


void getPreferences() // Daten aus Eeprom lesen
{

  preferences.begin("EEPROM", false);       //Parameter aus eeprom lesen
  Length = preferences.getUInt("Length", 0);
  //Checksumme berechnen durch Addition der Werte
  preferences_chksum = Length;

  preferences.end();

}



void setPreferences() // Daten in Eeprom schreiben
{

  long preferences_newchksum;

  //Checksumme berechnen durch Addition der Werte
  preferences_newchksum = Length;
  
  if ( preferences_newchksum != preferences_chksum ) {    //nur bei Änderungen in den EEPROM speichern
    preferences.begin("EEPROM", false);
    preferences.putUInt("Length", Length);
    preferences.end();
    
    display.clearBuffer();
    display.setFont(u8g2_font_courB10_tf);
    display.setCursor(15,15);
    display.print("Änderungen");
    display.setCursor(35,30);
    display.print("gespeichert");
    display.sendBuffer();
    delay(1000);
  }
   
 
  preferences_chksum = preferences_newchksum;  
  

}
