# honey-labeler
## Etikettiergerät für Honiggläser
Das Gerät soll automatisch das MHD-Datum auf das Honigetikett stempeln und anschließend das Etikett auf das Honigglas aufbringen. Der Stempel wird von einem Digitalservo mit Metallgetriebe bewegt. Die Rolle mit den Etiketten und Trägerband ist auf einem gebremsten Rad von einem Paketbandabroller gesteckt. Das Trägerband wird von dem Gleichstrommotor mit Wickeldorn aufgewickelt. Dabei wird die rechte Rolle mit Encoder angetrieben und dabei das Glas gedreht. Als Controller kommt ein ESP32 als ESP32-Dev Board zum Einsatz. Über ein Menü kann die Etikettenlänge und die Servoposition für den Stempel eingestellt werden. Dies wird im EEPROM gespeichert.<br>
Die Grundlage ist der ettimandl: <a href="https://community.hiveeyes.org/t/ettimandl-halbautomatischer-honigglas-etikettierer/3535">https://community.hiveeyes.org/t/ettimandl-halbautomatischer-honigglas-etikettierer/3535</a> <br>

<ul>
<li>mit:</li>
  <ul>- 0,96 OLED 128x64 Display SSD1306</ul>
  <ul>- DC Getriebemotor und H-Bridge DRV8871</ul>
  <ul>- optical Encoder</ul>
  <ul>- 3 Buttons</ul>
  <ul>- 1 Schalter</ul>
  <ul>- Datumsstempel mit Servo</ul>
</ul>
<ul>
<li>benutzte Libaries:</li>
  <ul>- U8g2</ul>
  <ul>- Bounce2</ul>
  <ul>- ESP32encoder</ul>
  <ul>- ESP32servo</ul>
</ul>
im develop-branch ist eine unvollständige Version hinterlegt.<br>

