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

#define interruptorPasillo 39  // Interruptor de leds del pasillo
#define CHILD_ID_PASILLO 39

#define ledPasillo 40  //
#define CHILD_ID_LED_PASILLO 40

#define interruptorPuertaGaraje 42  // Interruptor puerta Garaje
#define CHILD_ID_PUERTA_GARAJE 42

#define interruptorPuertaSalon 43  // Interruptor puerta 1
#define CHILD_ID_PUERTA_SALON 43

#define interruptorPuertaDormitorio 44  // Interruptor puerta 2
#define CHILD_ID_PUERTA_DORMITORIO 44

#define CHILD_ID_LIGHT 51
#define photocellPin A1
int threshold = 200;  //Umbral LDR

#define SENSOR_TEMP_OFFSET 0
#define DHT_DATA_PIN 30  //DHT11 PIN
#define DHTTYPE DHT11

#define ledSalon1 29  //Led1 Pin
#define CHILD_ID_LED_SALON1 29

#define ledSalon2 28  //Led2 Pin
#define CHILD_ID_LED_SALON2 28

#define ledExterior1 47  //Led1 Jardin
#define CHILD_ID_LED_EXTERIOR1 47

#define ledExterior2 46  //Led1 Jardin
#define CHILD_ID_LED_EXTERIOR2 46

#define ledBath 26  //Led baño
#define CHILD_ID_LED_BATH 26

#define ledAlarma 12  //led rojo de la alarma

#define ledStair 24  // Led escaleras
#define CHILD_ID_LED_STAIR 24
#define infStair1 23  // IR para led escaleras 1
#define infStair2 22  // IR para led escaleras 2

#define interruptorBath 25  // Interruptor del baño
#define CHILD_ID_BATH 25

#define ledGaraje 27  //Led Garaje
#define CHILD_ID_LED_GARAJE 27

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_INDICECALOR 38

#define interruptorSalon 37  //Interruptor ledSalon1 + ledSalon2
#define CHILD_ID_SALON 37

#define interruptorGaraje 33  //Interruptor Luz Garaje
#define CHILD_ID_GARAJE 33

#define StepAscensor 35  //Interruptor Stepmotor
#define CHILD_ID_ASCENSOR 35

#define BUZZER_PIN 32  //Buzzer Pin
#define CHILD_ID_BUZZER 32

#define INF_PIN2 45  // IR Alarma 2
#define INF_PIN 34   // IR Alarma 21
#define CHILD_ID_INF 34

#define CHILD_ID_ALARMA 36  //Interruptor activar/desactivar Alarma

#define redPin 2
#define bluePin 3
#define greenPin 4
#define CHILD_ID_RGB 31

LiquidCrystal_I2C lcd(0x27, 16, 2);

bool ledStateSalon1 = false;
bool ledStateSalon2 = false;
bool ledStateGaraje = false;
bool ledStateBath = false;
bool ledStatePasillo = false;
bool ledStairState = false;

boolean lastButtonStateSalon = LOW;
boolean lastButtonStatePasillo = LOW;
boolean lastButtonStateGaraje = LOW;
boolean lastButtonStateBath = LOW;

bool mensajeLCD = false;
unsigned long lcdTime = 0;

bool alarmaState = false;
bool buzzer_active = false;
unsigned long buzzer_activation_time = 0;
unsigned long buzzer_duration = 3000;  // Duración en milisegundos del Buzzer

unsigned long stairActivationTime = 0;
unsigned long ledStairDuration = 15000;  // DUración en milisegundos de la duración del ledStair encendido

const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);  // Pines motor de paso

MyMessage msgLedSalon1(CHILD_ID_LED_SALON1, V_LIGHT);
MyMessage msgLedSalon2(CHILD_ID_LED_SALON2, V_LIGHT);
MyMessage msgLedGaraje(CHILD_ID_LED_GARAJE, V_LIGHT);
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
MyMessage msgPuerta1(CHILD_ID_PUERTA_SALON, V_STATUS);
MyMessage msgPuerta2(CHILD_ID_PUERTA_DORMITORIO, V_STATUS);
MyMessage msgPuertaGaraje(CHILD_ID_PUERTA_GARAJE, V_STATUS);
MyMessage lightMsg(CHILD_ID_LIGHT, V_LIGHT_LEVEL);
MyMessage msgLedExterior1(CHILD_ID_LED_EXTERIOR1, V_LIGHT);
MyMessage msgLedExterior2(CHILD_ID_LED_EXTERIOR2, V_LIGHT);
MyMessage msgPasillo1(CHILD_ID_LED_PASILLO, V_LIGHT);
MyMessage msgIntPasillo(CHILD_ID_PASILLO, V_STATUS);
MyMessage msgIndiceCalor(CHILD_ID_INDICECALOR, V_TEMP);

DHT dht(DHT_DATA_PIN, DHTTYPE);

long RGB_values[3] = { 0, 0, 0 };
float valor;

Servo servoPuertaSalon;
Servo servoPuertaDormitorio;
Servo servoPuertaGaraje;

int i = 1;

unsigned long tiempoInicioFuncion1 = 0;
unsigned long tiempoInicioFuncion2 = 0;
unsigned long duracionFuncion1 = 120000; // 2 minutos en milisegundos
unsigned long duracionFuncion2 = 10000; // 10 segundos en milisegundos
bool funcion1Activa = false;

unsigned long startTime;                   // Tiempo en el que se inició la cuenta atrás
const unsigned int countdownTime = 15000;  // 15 segundos en milisegundos
bool cuentaAtras = false;


void presentation() {

  sendSketchInfo("Casa Domótica TFG", "0.1");

  present(CHILD_ID_LED_SALON1, S_LIGHT, "Led Salon 1");
  present(CHILD_ID_LED_SALON2, S_LIGHT, "Led Salon 2");
  present(CHILD_ID_LED_GARAJE, S_LIGHT, "Led Garaje");
  present(CHILD_ID_HUM, S_HUM, "Humedad");
  present(CHILD_ID_TEMP, S_TEMP, "Temperatura");
  present(CHILD_ID_INDICECALOR, S_TEMP, "Índice Calor");
  present(CHILD_ID_BUZZER, S_BINARY, "Buzzer");
  present(CHILD_ID_SALON, S_DOOR, "Interruptor Leds Salon");
  present(CHILD_ID_GARAJE, S_DOOR, "Interruptor Led Garaje");
  present(CHILD_ID_ALARMA, S_DOOR, "Interruptor Alarma");
  present(CHILD_ID_ASCENSOR, S_DOOR, "Ascensor");
  present(CHILD_ID_RGB, S_RGB_LIGHT, "RGB Led");
  present(CHILD_ID_RGB, S_DIMMER, "RGB Intendidad");
  present(CHILD_ID_RGB, S_LIGHT, "Estado RGB");
  present(CHILD_ID_BATH, S_DOOR, "Interruptor Led Baño");
  present(CHILD_ID_LED_BATH, S_LIGHT, "Led Baño");
  present(CHILD_ID_LED_STAIR, S_LIGHT, "Led Escalera");
  present(CHILD_ID_PUERTA_SALON, S_DOOR, "Puerta Salón");
  present(CHILD_ID_PUERTA_DORMITORIO, S_DOOR, "Puerta Dormitorio");
  present(CHILD_ID_PUERTA_GARAJE, S_DOOR, "Puerta Garaje");
  present(CHILD_ID_LIGHT, S_LIGHT_LEVEL, "LDR");
  present(CHILD_ID_LED_EXTERIOR1, S_LIGHT, "Led Exterior 1");
  present(CHILD_ID_LED_EXTERIOR2, S_LIGHT, "Led Exterior 2");
  present(CHILD_ID_LED_PASILLO, S_LIGHT, "Led Pasillo ");
  present(CHILD_ID_PASILLO, S_DOOR, "Interruptor Led Pasillo");

  request(CHILD_ID_RGB, V_RGB, "Led RGB");
  request(CHILD_ID_RGB, V_PERCENTAGE, "RGB Intensidad");
  request(CHILD_ID_RGB, V_LIGHT, "Estado RGB");
}

void setup() {

  dht.begin();

  servoPuertaSalon.attach(5);
  servoPuertaDormitorio.attach(6);
  servoPuertaGaraje.attach(7);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Loading...");

  pinMode(interruptorPasillo, INPUT_PULLUP);
  pinMode(interruptorSalon, INPUT_PULLUP);
  pinMode(interruptorGaraje, INPUT_PULLUP);
  pinMode(interruptorBath, INPUT_PULLUP);
  pinMode(interruptorPuertaSalon, INPUT_PULLUP);
  pinMode(interruptorPuertaDormitorio, INPUT_PULLUP);
  pinMode(interruptorPuertaGaraje, INPUT_PULLUP);
  pinMode(StepAscensor, INPUT_PULLUP);
  pinMode(ledSalon1, OUTPUT);
  pinMode(ledSalon2, OUTPUT);
  pinMode(ledPasillo, OUTPUT);
  pinMode(ledGaraje, OUTPUT);
  pinMode(ledAlarma, OUTPUT);
  pinMode(ledBath, OUTPUT);
  pinMode(ledStair, OUTPUT);
  pinMode(ledExterior1, OUTPUT);
  pinMode(ledExterior2, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  myStepper.setSpeed(10);  //Velocidad StepMotor

  servoPuertaSalon.write(1);
  servoPuertaDormitorio.write(1);
  servoPuertaGaraje.write(10);
}

void sensorLDR() {


  if (analogRead(photocellPin) < threshold) {  // Si el valor leído por el sensor LDR es menor al umbral establecido

    //Convertir el valor leído por el sensor LDR en el pin A1 a un valor de 0 a 100
    int percent1 = map(analogRead(photocellPin), 0, 1023, 0, 100);
    send(lightMsg.set(percent1));

    //Activación Leds Exteriores
    digitalWrite(ledExterior1, HIGH);
    digitalWrite(ledExterior2, HIGH);
    send(msgLedExterior1.set(1));
    send(msgLedExterior2.set(1));

    // Mensaje pantalla LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Luz: ");
    lcd.print(percent1);
    lcd.write('%');
    lcd.setCursor(0, 1);
    lcd.print("Leds >> ON");


  } else {

    //Convertir el valor leído por el sensor LDR en el pin A1 a un valor de 0 a 100
    int percent2 = map(analogRead(photocellPin), 0, 1023, 0, 100);
    send(lightMsg.set(percent2));

    //Desactivación Leds Exteriores
    digitalWrite(ledExterior1, LOW);
    digitalWrite(ledExterior2, LOW);
    send(msgLedExterior1.set(0));
    send(msgLedExterior2.set(0));

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Luz: ");
    lcd.print(percent2);
    lcd.write('%');
    lcd.setCursor(0, 1);
    lcd.print("Leds >> OFF");
  }
}

void sensorDHT11() {

  lcd.clear();

  //Lectura datos del sensor DHT11
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();
  float indiceCalor = dht.computeHeatIndex(temperature, humidity, false);

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

  //Envío de datos
  send(msgTemp.set(temperature, 1));
  send(msgHum.set(humidity, 1));
  send(msgIndiceCalor.set(indiceCalor, 1));
}

void controlInterruptoresLeds() {

  boolean buttonStatePasillo = digitalRead(interruptorPasillo);

  if (buttonStatePasillo == LOW && lastButtonStatePasillo == HIGH) {

    ledStatePasillo = !ledStatePasillo;
    digitalWrite(ledPasillo, ledStatePasillo);

    //Envío de datos
    send(msgIntPasillo.set(ledStatePasillo));
    send(msgPasillo1.set(ledStatePasillo));
  }

  lastButtonStatePasillo = buttonStatePasillo;

  boolean buttonStateSalon = digitalRead(interruptorSalon);

  if (buttonStateSalon == LOW && lastButtonStateSalon == HIGH) {

    ledStateSalon1 = !ledStateSalon1;
    digitalWrite(ledSalon1, ledStateSalon1);
    digitalWrite(ledSalon2, ledStateSalon1);

    //Envío de datos
    send(msgLedSalon1.set(ledStateSalon1));
    send(msgLedSalon2.set(ledStateSalon1));
    send(msgIntSalon.set(ledStateSalon1));
  }

  lastButtonStateSalon = buttonStateSalon;

  boolean buttonStateGaraje = digitalRead(interruptorGaraje);

  if (buttonStateGaraje == LOW && lastButtonStateGaraje == HIGH) {

    ledStateGaraje = !ledStateGaraje;
    digitalWrite(ledGaraje, ledStateGaraje);

    //Envío de datos
    send(msgLedGaraje.set(ledStateGaraje));
    send(msgIntGaraje.set(ledStateGaraje));
  }

  lastButtonStateGaraje = buttonStateGaraje;

  boolean buttonStateBath = digitalRead(interruptorBath);

  if (buttonStateBath == LOW && lastButtonStateBath == HIGH) {
    ledStateBath = !ledStateBath;
    digitalWrite(ledBath, ledStateBath);

    //Envío de datos
    send(msgBath.set(ledStateBath));
    send(msgIntBath.set(ledStateBath));
  }

  lastButtonStateBath = buttonStateBath;
}



void sistemaLedStair() {

  //Lectura del estado de los sensores IR infStair
  bool irStair1 = digitalRead(infStair1);
  bool irStair2 = digitalRead(infStair2);

  if (!irStair1 || !irStair2) {

    if (digitalRead(ledStair) == 0) {

      ledStairState = true;

      //Marca de tiempo cuando ledStair >> ON
      stairActivationTime = millis();

      digitalWrite(ledStair, HIGH);
      send(msgStair.set(ledStairState));

      lcd.clear();
      lcd.setCursor(0, 0);  // X, Y
      lcd.print("Led Escalera: ");
      lcd.setCursor(0, 1);  // X, Y
      lcd.print("Encendida");
    }
  }

  // se compara el tiempo transcurrido desde que se encendio ledStair (millis() - stairActivationTime) con la variable ledStairDuration definida anteriormente
  if (ledStairState && (millis() - stairActivationTime) >= ledStairDuration) {

    ledStairState = false;
    digitalWrite(ledStair, LOW);
    send(msgStair.set(ledStairState));

    lcdTemperature();
  }
}

void lcdTemperature() {

  lcd.clear();

  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  lcd.setCursor(0, 0);  // X, Y
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.write(0xDF);
  lcd.print("C");
  lcd.setCursor(0, 1);  // X, Y
  lcd.print("Hum:  ");
  lcd.print(humidity);
  lcd.write('%');
}

void sistemaAlarma() {

  //Lectura del estado de los sensores IR para el sistema de Alarma
  bool ir_detected = digitalRead(INF_PIN);
  bool ir_detected2 = digitalRead(INF_PIN2);

  if (!ir_detected || !ir_detected2 && !buzzer_active) {

    //Si desde Domoticz hemos activado el botón para conectar Alarma entonces alarmaState = True
    if (alarmaState) {

      //Cambio estado del Buzzer
      buzzer_active = true;

      //Marca de tiempo cuando Buzzer se activa
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

  // se compara el tiempo transcurrido desde que se activa Buzzer (millis() - buzzer_activation_time) con la variable buzzer_duration definida anteriormente
  if (buzzer_active && (millis() - buzzer_activation_time) >= buzzer_duration) {  // se compara el tiempo transcurrido desde que se activó la alarma (millis() - buzzer_activation_time) con la duración del tiempo que se quiere que el buzzer esté activo (buzzer_duration).

    //Cambio estado Buzzer
    buzzer_active = false;

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(ledAlarma, LOW);
    send(msgBuzzer.set(buzzer_active));
    lcdTemperature();
  }
}

void loop() {

  unsigned long tiempoActual = millis();
  
  if (funcion1Activa && tiempoActual - tiempoInicioFuncion1 >= duracionFuncion1) {
    // Si se está ejecutando la función 1 y ha pasado el tiempo necesario, desactiva la función 1 y activa la función 2
    funcion1Activa = false;
    tiempoInicioFuncion2 = tiempoActual;
    sensorDHT11();
  }
  
  if (!funcion1Activa && tiempoActual - tiempoInicioFuncion2 >= duracionFuncion2) {
    // Si no se está ejecutando la función 1 y ha pasado el tiempo necesario, activa la función 1
    funcion1Activa = true;
    tiempoInicioFuncion1 = tiempoActual;
    sensorLDR();
  }

  controlInterruptoresLeds();
  sistemaAlarma();
  sistemaLedStair();

  /*

  bool newStateLedSalon1 = digitalRead(ledSalon1);
  if (newStateLedSalon1 != ledStateSalon1) {
    ledStateSalon1 = newStateLedSalon1;
    send(msgLedSalon1.set(ledStateSalon1));
  }

  bool newStateLedSalon2 = digitalRead(ledSalon2);
  if (newStateLedSalon2 != ledStateSalon2) {
    ledStateSalon2 = newStateLedSalon2;
    send(msgLedSalon2.set(ledStateSalon2));
  }

  bool newStateLedGaraje = digitalRead(ledGaraje);
  if (newStateLedGaraje != ledStateGaraje) {
    ledStateGaraje = newStateLedGaraje;
    send(msgLedGaraje.set(ledStateGaraje));
  }

  bool newStateLedBath = digitalRead(ledBath);
  if (newStateLedBath != ledStateBath) {
    ledStateBath = newStateLedBath;
    send(msgBath.set(ledStateBath));
  }

  bool newStateLedPasillo = digitalRead(ledPasillo);
  if (newStateLedPasillo != ledStatePasillo) {
    ledStatePasillo = newStateLedPasillo;
    send(msgPasillo1.set(ledStatePasillo));
  }
  */
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

    analogWrite(redPin, int(RGB_values[0] * (valor / 100)));
    analogWrite(greenPin, int(RGB_values[1] * (valor / 100)));
    analogWrite(bluePin, int(RGB_values[2] * (valor / 100)));
  }

  if (message.getSensor() == CHILD_ID_RGB && message.type == V_LIGHT) {

    if (message.getInt() == 0) {

      digitalWrite(redPin, 0);
      digitalWrite(greenPin, 0);
      digitalWrite(bluePin, 0);
    }

    if (message.getInt() == 1) {

      analogWrite(redPin, int(RGB_values[0] * (valor / 100)));
      analogWrite(greenPin, int(RGB_values[1] * (valor / 100)));
      analogWrite(bluePin, int(RGB_values[2] * (valor / 100)));
    }
  }

  /*
  if (message.getSensor() == CHILD_ID_LED_SALON1 && message.type == V_STATUS) {

    digitalWrite(ledSalon1, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_LED_SALON2 && message.getType() == V_STATUS) {

    digitalWrite(ledSalon2, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_LED_BATH && message.type == V_STATUS) {

    digitalWrite(ledBath, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_LED_GARAJE && message.type == V_STATUS) {

    digitalWrite(ledGaraje, message.getBool() ? HIGH : LOW);
  }
*/

  if (message.getSensor() == CHILD_ID_ALARMA && message.type == V_STATUS) {

    if (message.getBool()) {

      alarmaState = true;

    } else {

      alarmaState = false;
    }
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
    digitalWrite(ledPasillo, lastButtonStatePasillo ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_ASCENSOR && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      myStepper.step(stepsPerRevolution * 8.5);

    } else if (message.getBool() == 0) {

      myStepper.step(-stepsPerRevolution * 8.5);
    }
  }


  if (message.getSensor() == CHILD_ID_PUERTA_SALON && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      servoPuertaSalon.write(1);

    } else if (message.getBool() == 0) {

      servoPuertaSalon.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_PUERTA_DORMITORIO && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      servoPuertaDormitorio.write(1);

    } else if (message.getBool() == 0) {

      servoPuertaDormitorio.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_PUERTA_GARAJE && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      servoPuertaGaraje.write(10);

    } else if (message.getBool() == 0) {

      servoPuertaGaraje.write(100);
      
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
}
