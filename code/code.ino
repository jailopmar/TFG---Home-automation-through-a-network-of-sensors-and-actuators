#define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 48
#define MY_RF24_CHANNEL 90
#define MY_NODE_ID 95


#include <MySensors.h>
#include <DHT.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Stepper.h>


static const uint64_t UPDATE_INTERVAL = 5000;
static const uint8_t FORCE_UPDATE_N_READS = 10;

#define interruptorPasillo 39  // Interruptor de leds del pasillo
#define CHILD_ID_PASILLO 39

#define ledPasillo1 40  //
#define CHILD_ID_LED_PASILLO1 40

#define interruptorPuertaGaraje 42  // Interruptor puerta Garaje
#define CHILD_ID_PUERTA_GARAJE 42

#define interruptorPuerta1 43  // Interruptor puerta 1
#define CHILD_ID_PUERTA1 43

#define interruptorPuerta2 44  // Interruptor puerta 2
#define CHILD_ID_PUERTA2 44

#define SENSOR_TEMP_OFFSET 0

#define CHILD_ID_LIGHT 51
#define photocellPin A1
int threshold = 200;

#define CHILD_ID_MQ2 48
#define pinDigitalMQ2 13
#define pinAnalogMQ2 A0


#define DHT_DATA_PIN 30  //DHT11 PIN
#define DHTTYPE DHT11

#define ledSalon1 29  //Led1 Pin
#define CHILD_ID_LED_SALON1 29

#define ledSalon2 28  //Led2 Pin
#define CHILD_ID_LED_SALON2 28

#define ledJardin1 47  //Led1 Jardin
#define CHILD_ID_LED_JARDIN1 47

#define ledJardin2 46  //Led1 Jardin
#define CHILD_ID_LED_JARDIN2 46

#define ledBath 26  //Led baño
#define CHILD_ID_LED_BATH 26

#define ledAlarma 12  //led rojo de la alarma

#define ledStair 24  // Led escaleras
#define CHILD_ID_LED_STAIR 24
#define infStair1 23
#define infStair2 22


#define interruptorBath 25  // Interruptor del baño
#define CHILD_ID_BATH 25

#define ledGaraje 27  //Led Garaje
#define CHILD_ID_LED_GARAJE 27

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define interruptorSalon 37  //Interruptor Led1 + Led2 del salón
#define CHILD_ID_SALON 37

#define interruptorGaraje 33  //Interruptor Luz Garaje
#define CHILD_ID_GARAJE 33

#define StepAscensor 35  //Interruptor Stepmotor
#define CHILD_ID_ASCENSOR 35

#define BUZZER_PIN 32  //Buzzer Pin
#define CHILD_ID_BUZZER 32

#define INF_PIN2 45
#define INF_PIN 34  //Infrarrojos Buzzer
#define CHILD_ID_INF 34

#define CHILD_ID_ALARMA 36  //Interruptor Alarma

#define redpin 2
#define bluepin 3
#define greenpin 4
#define CHILD_ID_RGB 31

char temp_Msg[20];

LiquidCrystal_I2C lcd(0x27, 16, 2);

int oldValue = 0;
int oldValue2 = 0;
int buzzerState = HIGH;
bool ledStateSalon1 = false;
boolean lastButtonStateSalon = LOW;
boolean lastButtonStatePasillo = LOW;
boolean lastButtonStateGaraje = LOW;
boolean lastButtonStateBath = LOW;
bool ledStateSalon2 = false;
bool ledStatePasillo = false;

bool ledStateGaraje = false;
bool ledStateBath = false;
bool alarmaState = false;

bool buzzer_active = false;
bool ledStairState = false;
unsigned long buzzer_activation_time = 0;
unsigned long buzzer_duration = 3000;
unsigned long stairActivationTime = 0;
unsigned long ledStairDuration = 15000;

uint16_t gasValue;
bool isGas = false;

const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

MyMessage msg(CHILD_ID_LED_SALON1, V_LIGHT);
MyMessage msg2(CHILD_ID_LED_SALON2, V_LIGHT);
MyMessage msg3(CHILD_ID_LED_GARAJE, V_LIGHT);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBuzzer(CHILD_ID_BUZZER, V_STATUS);
MyMessage msgIntSalon(CHILD_ID_SALON, V_STATUS);
MyMessage msgStepBoton(CHILD_ID_ASCENSOR, V_VAR1);
MyMessage msgIntGaraje(CHILD_ID_GARAJE, V_STATUS);
MyMessage msgBotonAlarma(CHILD_ID_ALARMA, V_STATUS);
MyMessage msgBright(CHILD_ID_RGB, V_PERCENTAGE);
MyMessage msgColor(CHILD_ID_RGB, V_RGB);
MyMessage msgLight(CHILD_ID_RGB, V_LIGHT);
MyMessage msgBath(CHILD_ID_LED_BATH, V_LIGHT);
MyMessage msgIntBath(CHILD_ID_BATH, V_STATUS);
MyMessage msgStair(CHILD_ID_LED_STAIR, V_LIGHT);
MyMessage msgMQ2(CHILD_ID_MQ2, V_LEVEL);
MyMessage msgPuerta1(CHILD_ID_PUERTA1, V_STATUS);
MyMessage msgPuerta2(CHILD_ID_PUERTA2, V_STATUS);
MyMessage msgPuertaGaraje(CHILD_ID_PUERTA_GARAJE, V_STATUS);
MyMessage lightMsg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgJardin1(CHILD_ID_LED_JARDIN1, V_LIGHT);
MyMessage msgJardin2(CHILD_ID_LED_JARDIN2, V_LIGHT);
MyMessage msgPasillo1(CHILD_ID_LED_PASILLO1, V_LIGHT);
MyMessage msgIntPasillo(CHILD_ID_PASILLO, V_STATUS);




DHT dht(DHT_DATA_PIN, DHTTYPE);

unsigned long timeOfLastChange = 0;


unsigned long tiempo_funcion1 = 0;
unsigned long tiempo_funcion2 = 0;
const unsigned long intervalo = 30000;



long RGB_values[3] = { 0, 0, 0 };
float valor;

Servo servo1;
Servo servo2;
Servo servoGaraje;

int i = 1;

void setup() {
  servo1.attach(5);
  servo2.attach(6);
  servoGaraje.attach(7);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  dht.begin();
  lcd.setCursor(0, 0);
  lcd.print("Loading...");
  pinMode(interruptorPasillo, INPUT_PULLUP);
  pinMode(interruptorSalon, INPUT_PULLUP);
  pinMode(interruptorGaraje, INPUT_PULLUP);
  pinMode(interruptorBath, INPUT_PULLUP);
  pinMode(interruptorPuerta1, INPUT_PULLUP);
  pinMode(interruptorPuerta2, INPUT_PULLUP);
  pinMode(interruptorPuertaGaraje, INPUT_PULLUP);
  pinMode(StepAscensor, INPUT_PULLUP);
  pinMode(ledSalon1, OUTPUT);
  pinMode(ledSalon2, OUTPUT);
  pinMode(ledPasillo1, OUTPUT);
  pinMode(ledGaraje, OUTPUT);
  pinMode(ledAlarma, OUTPUT);
  pinMode(ledBath, OUTPUT);
  pinMode(ledStair, OUTPUT);
  pinMode(ledJardin1, OUTPUT);
  pinMode(ledJardin2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);
  pinMode(pinDigitalMQ2, INPUT);

  // Inicialización de MySensors
  sendSketchInfo("TFG Jaime Lopez Marquez", "0.1");
  present(CHILD_ID_LED_SALON1, S_LIGHT);
  present(CHILD_ID_LED_SALON2, S_LIGHT);
  present(CHILD_ID_LED_GARAJE, S_LIGHT);
  present(CHILD_ID_HUM, S_HUM);
  present(CHILD_ID_TEMP, S_TEMP);
  present(CHILD_ID_BUZZER, S_BINARY);
  present(CHILD_ID_SALON, S_DOOR);
  present(CHILD_ID_GARAJE, S_DOOR);
  present(CHILD_ID_ALARMA, S_DOOR);
  present(CHILD_ID_ASCENSOR, S_DOOR);
  present(CHILD_ID_RGB, S_RGB_LIGHT);
  present(CHILD_ID_RGB, S_DIMMER);
  present(CHILD_ID_RGB, S_LIGHT);
  present(CHILD_ID_BATH, S_DOOR);
  present(CHILD_ID_LED_BATH, S_LIGHT);
  present(CHILD_ID_LED_STAIR, S_LIGHT);
  present(CHILD_ID_MQ2, S_AIR_QUALITY);
  present(CHILD_ID_PUERTA1, S_DOOR);
  present(CHILD_ID_PUERTA2, S_DOOR);
  present(CHILD_ID_PUERTA_GARAJE, S_DOOR);
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL);
  present(CHILD_ID_LED_JARDIN1, S_LIGHT);
  present(CHILD_ID_LED_JARDIN2, S_LIGHT);
  present(CHILD_ID_LED_PASILLO1, S_LIGHT);
  present(CHILD_ID_PASILLO, S_DOOR);

  request(CHILD_ID_RGB, V_RGB);
  request(CHILD_ID_RGB, V_PERCENTAGE);
  request(CHILD_ID_RGB, V_LIGHT);

  /*
  send(msgBright.set(100));
  send(msgColor.set("000000"));
  send(msgLight.set(1));
  */

  myStepper.setSpeed(10);

  servo1.write(1);
  servo2.write(1);
  servoGaraje.write(10);
}

void sensorLDR() {

  if (analogRead(photocellPin) < threshold) {

    int percent1 = map(analogRead(photocellPin), 0, 1023, 0, 100);
    send(lightMsg.set(percent1));
    digitalWrite(ledJardin1, HIGH);
    digitalWrite(ledJardin2, HIGH);
    send(msgJardin1.set(1));
    send(msgJardin2.set(1));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Luz: ");
    lcd.print(percent1);
    lcd.write('%');
    lcd.setCursor(0, 1);
    lcd.print("Leds >> ON");

  } else {

    int percent2 = map(analogRead(photocellPin), 0, 1023, 0, 100);
    send(lightMsg.set(percent2));
    digitalWrite(ledJardin1, LOW);
    digitalWrite(ledJardin2, LOW);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Luz: ");
    lcd.print(percent2);
    lcd.write('%');
    lcd.setCursor(0, 1);
    lcd.print("Leds >> OFF");
    send(msgJardin1.set(0));
    send(msgJardin2.set(0));
  }
}

void sensorDHT11() {

  lcd.clear();
  dht.read(true);
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.println(F("Humedad: "));
  Serial.println(humidity);

  Serial.println("Temperatura: ");
  Serial.println(temperature);
  lcd.setCursor(0, 0);  // X, Y
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.write(0xDF);
  lcd.print("C");
  lcd.setCursor(0, 1);  // X, Y
  lcd.print("Hum:  ");
  lcd.print(humidity);
  lcd.write('%');
  send(msgTemp.set(temperature, 1));
  send(msgHum.set(humidity, 1));
}

void controlInterruptoresLeds() {

  boolean buttonStatePasillo = digitalRead(interruptorPasillo);

  if (buttonStatePasillo == LOW && lastButtonStatePasillo == HIGH) {
    //send(msgIntSalon.set(!lastButtonStateSalon));
    ledStatePasillo = !ledStatePasillo;
    digitalWrite(ledPasillo1, ledStatePasillo);
    send(msgIntPasillo.set(ledStatePasillo));
    send(msgPasillo1.set(ledStatePasillo));
  }
  lastButtonStatePasillo = buttonStatePasillo;

  boolean buttonStateSalon = digitalRead(interruptorSalon);

  if (buttonStateSalon == LOW && lastButtonStateSalon == HIGH) {
    //send(msgIntSalon.set(!lastButtonStateSalon));
    ledStateSalon1 = !ledStateSalon1;
    digitalWrite(ledSalon1, ledStateSalon1);
    digitalWrite(ledSalon2, ledStateSalon1);
    send(msg.set(ledStateSalon1));
    send(msg2.set(ledStateSalon1));
    send(msgIntSalon.set(ledStateSalon1));
  }
  lastButtonStateSalon = buttonStateSalon;

  boolean buttonStateGaraje = digitalRead(interruptorGaraje);

  if (buttonStateGaraje == LOW && lastButtonStateGaraje == HIGH) {
    //send(msgIntSalon.set(!lastButtonStateSalon));
    ledStateGaraje = !ledStateGaraje;
    digitalWrite(ledGaraje, ledStateGaraje);
    send(msg3.set(ledStateGaraje));
    send(msgIntGaraje.set(ledStateGaraje));
  }
  lastButtonStateGaraje = buttonStateGaraje;

  boolean buttonStateBath = digitalRead(interruptorBath);

  if (buttonStateBath == LOW && lastButtonStateBath == HIGH) {
    ledStateBath = !ledStateBath;
    digitalWrite(ledBath, ledStateBath);
    send(msgBath.set(ledStateBath));
    send(msgIntBath.set(ledStateBath));
  }
  lastButtonStateBath = buttonStateBath;
}

void sistemaLedStair() {

  bool irStair1 = digitalRead(infStair1);
  bool irStair2 = digitalRead(infStair2);

  if (!irStair1 || !irStair2) {

    if (digitalRead(ledStair) == 0) {

      ledStairState = true;
      stairActivationTime = millis();
      digitalWrite(ledStair, HIGH);
      send(msgStair.set(ledStairState));
    }
  }

  if (ledStairState && (millis() - stairActivationTime) >= ledStairDuration) {  // se compara el tiempo transcurrido desde que se activó la alarma (millis() - buzzer_activation_time) con la duración del tiempo que se quiere que el buzzer esté activo (buzzer_duration).
    ledStairState = false;
    digitalWrite(ledStair, LOW);
    send(msgStair.set(ledStairState));
  }
}

void sistemaAlarma() {

  bool ir_detected = digitalRead(INF_PIN);
  bool ir_detected2 = digitalRead(INF_PIN2);

  if (!ir_detected || !ir_detected2 && !buzzer_active) {

    if (alarmaState) {

      buzzer_active = true;
      buzzer_activation_time = millis();
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(ledAlarma, HIGH);
      send(msgBuzzer.set(buzzer_active));
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Alarma!!");
      lcd.setCursor(0, 1);
      lcd.print("Enviando datos...");
    }
  }

  if (buzzer_active && (millis() - buzzer_activation_time) >= buzzer_duration) {  // se compara el tiempo transcurrido desde que se activó la alarma (millis() - buzzer_activation_time) con la duración del tiempo que se quiere que el buzzer esté activo (buzzer_duration).
    buzzer_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(ledAlarma, LOW);
    send(msgBuzzer.set(buzzer_active));
  }
}

void loop() {

  unsigned long tiempo_actual = millis();

  if (millis() % 60000 == 0) {
    i++;

    if (i % 2 == 0) {
      sensorLDR();
    } else {
      sensorDHT11();
    }
  }

  if (i == 3) {
    i = 1;
  }

  //--------------------------------------------------------------------------------------------------------------------------------------------

  //---------------------------------------------------- INTERRUPTORES LEDS ---------------------------------------------------------------------------------

  controlInterruptoresLeds();

  //--------------------------------------------------------------------------------------------------------------------------------------------

  //----------------------------------------------------BUZZER---------------------------------------------------------------------------------

  sistemaAlarma();


  //---------------------------------------------------- LED STAIR---------------------------------------------------------------------------------


  sistemaLedStair();

  //--------------------------------------------------------------------------------------------------------------------------------------------


  //-------------------------------------------------- SENSOR TEMP+HUM + LCD -------------------------------------------------------------------------






  //--------------------------------------------------------------------------------------------------------------------------------------------




  // Comprobación del estado del LED

  bool newStateLedSalon1 = digitalRead(ledSalon1);
  if (newStateLedSalon1 != ledStateSalon1) {
    ledStateSalon1 = newStateLedSalon1;
    send(msg.set(ledStateSalon1));
  }

  bool newStateLedSalon2 = digitalRead(ledSalon2);
  if (newStateLedSalon2 != ledStateSalon2) {
    ledStateSalon2 = newStateLedSalon2;
    send(msg2.set(ledStateSalon2));
  }

  bool newStateLedGaraje = digitalRead(ledGaraje);
  if (newStateLedGaraje != ledStateGaraje) {
    ledStateGaraje = newStateLedGaraje;
    send(msg3.set(ledStateGaraje));
  }

  bool newStateLedBath = digitalRead(ledBath);
  if (newStateLedBath != ledStateBath) {
    ledStateBath = newStateLedBath;
    send(msgBath.set(ledStateBath));
  }

  bool newStateLedPasillo1 = digitalRead(ledPasillo1);
  if (newStateLedPasillo1 != ledStatePasillo) {
    ledStatePasillo = newStateLedPasillo1;
    send(msgPasillo1.set(ledStatePasillo));
  }
}


void receive(const MyMessage &message) {


  if (message.getSensor() == CHILD_ID_RGB && message.type == V_RGB) {

    String hexa = message.getString();
    long n = (long)strtol(&hexa[0], NULL, 16);
    RGB_values[0] = n >> 16;
    RGB_values[1] = n >> 8 & 0xFF;
    RGB_values[2] = n & 0xFF;
  }
  if (message.getSensor() == CHILD_ID_RGB && message.type == V_PERCENTAGE) {
    valor = message.getInt();
    analogWrite(redpin, int(RGB_values[0] * (valor / 100)));
    analogWrite(greenpin, int(RGB_values[1] * (valor / 100)));
    analogWrite(bluepin, int(RGB_values[2] * (valor / 100)));
  }

  if (message.getSensor() == CHILD_ID_RGB && message.type == V_LIGHT) {
    if (message.getInt() == 0) {
      digitalWrite(redpin, 0);
      digitalWrite(greenpin, 0);
      digitalWrite(bluepin, 0);
    }
    if (message.getInt() == 1) {
      analogWrite(redpin, int(RGB_values[0] * (valor / 100)));
      analogWrite(greenpin, int(RGB_values[1] * (valor / 100)));
      analogWrite(bluepin, int(RGB_values[2] * (valor / 100)));
    }
  }


  if (message.getSensor() == CHILD_ID_LED_SALON1 && message.type == V_STATUS) {

    digitalWrite(ledSalon1, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_LED_BATH && message.type == V_STATUS) {

    digitalWrite(ledBath, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_ALARMA && message.type == V_STATUS) {

    if (message.getBool()) {
      alarmaState = true;
    } else {
      alarmaState = false;
    }
  }

  if (message.getSensor() == CHILD_ID_LED_GARAJE && message.type == V_STATUS) {

    digitalWrite(ledGaraje, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_SALON && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateSalon = message.getBool();
    digitalWrite(ledSalon1, lastButtonStateSalon ? HIGH : LOW);
    digitalWrite(ledSalon2, lastButtonStateSalon ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_PASILLO && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStatePasillo = message.getBool();
    digitalWrite(ledPasillo1, lastButtonStatePasillo ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_ASCENSOR && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {


      myStepper.step(stepsPerRevolution * 8.5);

    } else if (message.getBool() == 0) {

      myStepper.step(-stepsPerRevolution * 8.5);
    }
  }


  if (message.getSensor() == CHILD_ID_PUERTA1 && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      servo1.write(1);

    } else if (message.getBool() == 0) {

      servo1.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_PUERTA2 && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      servo2.write(1);

    } else if (message.getBool() == 0) {

      servo2.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_PUERTA_GARAJE && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      servoGaraje.write(10);

    } else if (message.getBool() == 0) {

      servoGaraje.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_GARAJE && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateSalon = message.getBool();
    digitalWrite(ledGaraje, lastButtonStateSalon ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_BATH && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateBath = message.getBool();
    digitalWrite(ledBath, lastButtonStateBath ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_LED_SALON2 && message.getType() == V_STATUS) {

    digitalWrite(ledSalon2, message.getBool() ? HIGH : LOW);
  }
}
