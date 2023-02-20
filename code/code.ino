#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10

#define MY_RF24_CHANNEL 76
#define MY_NODE_ID 95
#include <MySensors.h>

#define ledPin 6
#define CHILD_ID 6

#define ledPin2 7
#define CHILD_ID2 7
bool ledState = false;
bool ledState2 = false;

MyMessage msg(CHILD_ID, V_LIGHT);
MyMessage msg2(CHILD_ID2, V_LIGHT);

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);

  // Inicialización de MySensors
  sendSketchInfo("TFG Jaime Lopez Marquez", "0.1");
  present(CHILD_ID, S_LIGHT);
  present(CHILD_ID2, S_LIGHT);
}

void loop()
{
  // Comprobación del estado del LED
  bool newState = digitalRead(ledPin);
  if (newState != ledState) {
    ledState = newState;
    send(msg.set(ledState));
  }

  bool newState2 = digitalRead(ledPin2);
  if (newState2 != ledState2) {
    ledState2 = newState2;
    send(msg2.set(ledState2));
  }

  // Procesamiento de los mensajes recibidos
  
}

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.


  if (message.getSensor()==CHILD_ID && message.getType()== V_STATUS) {
     // Change relay state
    digitalWrite(ledPin, message.getBool()?HIGH:LOW);
     // Store state in eeprom
     Serial.print("Incoming change sensor:");
     Serial.print(message.getSensor());
     Serial.print(", New status: ");
     Serial.println(message.getBool());
     
   } 

   if (message.getSensor()==CHILD_ID2 && message.getType()== V_STATUS) {
     // Change relay state
    digitalWrite(ledPin2, message.getBool()?HIGH:LOW);
     // Store state in eeprom
     Serial.print("Incoming change sensor:");
     Serial.print(message.getSensor());
     Serial.print(", New status: ");
     Serial.println(message.getBool());
     
   } 
} 
