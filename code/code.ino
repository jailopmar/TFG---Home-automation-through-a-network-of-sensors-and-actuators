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
#include <Servo.h> 

static const uint64_t UPDATE_INTERVAL = 5000;
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define SERVO_DIGITAL_OUT_PIN 11
#define SERVO_MIN 0 // Fine tune your servos min. 0-180
#define SERVO_MAX 180  // Fine tune your servos max. 0-180
#define DETACH_DELAY 900 // Tune this to let your movement finish before detaching the servo
#define CHILD_ID11 11   // Id of the sensor child

#define SENSOR_TEMP_OFFSET 0
#define ledPin 6
#define CHILD_ID 6
#define ledPin2 7
#define CHILD_ID2 7
#define ledPin3 8
#define CHILD_ID5 8

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define CHILD_ID31 31
#define boton1 31
#define boton2 33
#define CHILD_ID33 33
#define CHILD_ID3 2
#define BUZZER_PIN 2

#define CHILD_ID4 3
#define INF_PIN 3

#define CHILD_ID36 36


int oldValue = 0;
int oldValue2 = 0;
int buzzerState = HIGH;
float lastTemp;
float lastHum;
uint8_t nNoUpdatesTemp;
uint8_t nNoUpdatesHum;
bool ledState = false;
boolean lastButtonState = LOW;
boolean lastButtonState2 = LOW;
bool ledState2 = false;
bool ledState3 = false;
bool ledState4 = false;
boolean buttonStateFromController = HIGH;

bool alarmaEncendida = false;

bool buzzer_active = false;
unsigned long buzzer_activation_time = 0;
unsigned long buzzer_duration = 3000;

MyMessage msg(CHILD_ID, V_LIGHT);
MyMessage msg2(CHILD_ID2, V_LIGHT);
MyMessage msg3(CHILD_ID5, V_LIGHT);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBuzzer(CHILD_ID3, V_STATUS);
MyMessage msgBoton1(CHILD_ID31, V_STATUS);
MyMessage msgBoton2(CHILD_ID33, V_STATUS);
MyMessage msgBotonAlarma(CHILD_ID36, V_STATUS);
MyMessage msgServo(CHILD_ID11, V_DIMMER);
Servo myservo;

DHT dht(DHT_DATA_PIN, DHTTYPE);

unsigned long timeOfLastChange = 0;
bool attachedServo = false;

void setup()
{
  dht.begin();
  pinMode(boton1, INPUT_PULLUP);
  pinMode(boton2, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Inicialización de MySensors
  sendSketchInfo("TFG Jaime Lopez Marquez", "0.1");
  present(CHILD_ID, S_LIGHT);
  present(CHILD_ID2, S_LIGHT);
  present(CHILD_ID5, S_LIGHT);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID3, S_BINARY);
  present(CHILD_ID31, S_DOOR);
  present(CHILD_ID33, S_DOOR);
  present(CHILD_ID36, S_DOOR);
  request(CHILD_ID11, V_DIMMER);
  present(CHILD_ID11, S_COVER);

}

void loop()
{

  boolean buttonState = digitalRead(boton1);

  if (buttonState == LOW && lastButtonState == HIGH) {
    //send(msgBoton1.set(!lastButtonState));
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
    digitalWrite(ledPin2, ledState);
    send(msg.set(ledState));
    send(msg2.set(ledState));
    send(msgBoton1.set(ledState));

    // Enviar el nuevo estado del LED al controlador Domoticz
    
  
  }
  lastButtonState = buttonState;

  boolean buttonState2 = digitalRead(boton2);

  if (buttonState2 == LOW && lastButtonState2 == HIGH) {
    //send(msgBoton1.set(!lastButtonState));
    ledState3 = !ledState3;
    digitalWrite(ledPin3, ledState3);
    send(msg3.set(ledState3));
    send(msgBoton2.set(ledState3));

    // Enviar el nuevo estado del LED al controlador Domoticz
    
  
  }
  lastButtonState2 = buttonState2;

  //----------------------------------------------------BUZZER---------------------------------------------------------------------------------
  

  bool ir_detected = digitalRead(INF_PIN);
  
  
  if (!ir_detected && !buzzer_active ) {
    

    if(alarmaEncendida){
      
      buzzer_active = true;
      buzzer_activation_time = millis();
      digitalWrite(BUZZER_PIN, HIGH);
      send(msgBuzzer.set(buzzer_active));
    }
  }
  
  if (buzzer_active && (millis() - buzzer_activation_time) >= buzzer_duration) { // se compara el tiempo transcurrido desde que se activó la alarma (millis() - buzzer_activation_time) con la duración del tiempo que se quiere que el buzzer esté activo (buzzer_duration).
    buzzer_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    send(msgBuzzer.set(buzzer_active));
  }
  
  //--------------------------------------------------------------------------------------------------------------------------------------------

  //-------------------------------------------------- SENSOR TEMP+HUM -------------------------------------------------------------------------

  if (attachedServo && millis() - timeOfLastChange > DETACH_DELAY) {
     myservo.detach();
     attachedServo = false;
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

  //--------------------------------------------------------------------------------------------------------------------------------------------
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

  bool newState3 = digitalRead(ledPin3);
  if (newState3 != ledState3) {
    ledState3 = newState3;
    send(msg3.set(ledState3));
  }


  

  // Procesamiento de los mensajes recibidos
}

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
   if (message.sensor == CHILD_ID11) {
    myservo.attach(SERVO_DIGITAL_OUT_PIN);   
    attachedServo = true;
  if (message.getSensor()==CHILD_ID11 && message.type==V_DIMMER) { // This could be M_ACK_VARIABLE or M_SET_VARIABLE
     int val = message.getInt();
     myservo.write(SERVO_MAX + (SERVO_MIN-SERVO_MAX)/100 * val); // sets the servo position 0-180
     // Write some debug info
     Serial.print("Servo changed. new state: ");
     Serial.println(val);
   } else if (message.getSensor()==CHILD_ID11 && message.type==V_UP) {
     Serial.println("Servo UP command");
     myservo.write(SERVO_MIN);
     send(msgServo.set(100));
   } else if (message.getSensor()==CHILD_ID11 && message.type==V_DOWN) {
     Serial.println("Servo DOWN command");
     myservo.write(SERVO_MAX); 
     send(msgServo.set(0));
   } else if ( message.getSensor()==CHILD_ID11 && message.type==V_STOP) {
     Serial.println("Servo STOP command");
     myservo.detach();
     attachedServo = false;

   }
   timeOfLastChange = millis();
  }
  if (message.getSensor()==CHILD_ID && message.type == V_STATUS) {
    
    digitalWrite(ledPin, message.getBool()?HIGH:LOW);
  } 

  if (message.getSensor()==CHILD_ID36 && message.type == V_STATUS) {
    
    if(message.getBool()) {
      alarmaEncendida = true;
    }else {
      alarmaEncendida = false;
    }
  } 

  if (message.getSensor()==CHILD_ID5 && message.type == V_STATUS) {
    
    digitalWrite(ledPin3, message.getBool()?HIGH:LOW);
  } 
  
  if (message.getSensor()==CHILD_ID31 && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonState = message.getBool();
    digitalWrite(ledPin, lastButtonState ? HIGH : LOW);
    digitalWrite(ledPin2, lastButtonState ? HIGH : LOW);
    //Serial.println(lastButtonState);
  }

  if (message.getSensor()==CHILD_ID33 && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonState = message.getBool();
    digitalWrite(ledPin3, lastButtonState ? HIGH : LOW);
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
