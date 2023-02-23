#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_RF24_CE_PIN 9
#define MY_RF24_CS_PIN 10
#define MY_RF24_CHANNEL 76
#define MY_NODE_ID 95
#define DHT_DATA_PIN 5
#define DHTTYPE DHT11

#include <MySensors.h>
#include <DHT.h>

static const uint64_t UPDATE_INTERVAL = 5000;
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define SENSOR_TEMP_OFFSET 0
#define ledPin 6
#define CHILD_ID 6
#define ledPin2 7
#define CHILD_ID2 7
#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define CHILD_ID3 2
#define BUZZER_PIN 2

#define CHILD_ID4 3
#define BUTTON_PIN 3


int oldValue = 0;
int buzzerState = HIGH;
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool ledState = false;
bool ledState2 = false;

MyMessage msg(CHILD_ID, V_LIGHT);
MyMessage msg2(CHILD_ID2, V_LIGHT);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBuzzer(CHILD_ID3, V_STATUS);
MyMessage msgButton(CHILD_ID4, V_STATUS);

DHT dht(DHT_DATA_PIN, DHTTYPE);

void setup()
{
  dht.begin();
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Inicialización de MySensors
  sendSketchInfo("TFG Jaime Lopez Marquez", "0.1");
  present(CHILD_ID, S_LIGHT);
  present(CHILD_ID2, S_LIGHT);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID3, S_BINARY);
  present(CHILD_ID4, S_DOOR);
}

void loop()
{

  int value = digitalRead(BUTTON_PIN);

  if (value != oldValue) {
    send(msgButton.set(value == LOW ? 1 : 0));
    send(msgBuzzer.set(value == LOW ? 1 : 0));
    oldValue = value;
    buzzerState = !buzzerState;
    digitalWrite(BUZZER_PIN, buzzerState);
    
  }
  

  if (millis() % 60000 == 0){
    

  dht.read(true);
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.println(F("Humedad: "));
  Serial.println(humidity);

  Serial.println("Temperatura: ");
  Serial.println(temperature);

  send(msgTemp.set(temperature, 1));
  send(msgHum.set(humidity, 1));

  }

  
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

  if (message.getSensor()==CHILD_ID4 && message.type == V_STATUS) {
    int value = message.getInt();
    buzzerState = value == 1 ? HIGH : LOW;
    digitalWrite(BUZZER_PIN, buzzerState);
  }

  
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
