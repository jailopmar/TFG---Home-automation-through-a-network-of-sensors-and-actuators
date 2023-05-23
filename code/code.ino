#define MY_DEBUG

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
#include <MQUnifiedsensor.h>

#define MQ_PIN A14
#define MQ_PIN_DIGITAL 13
#define CHILD_ID_MQ2_DIGITAL 13
#define CHILD_ID_MQ2_CO 61

#define Board ("Arduino MEGA")
#define Type ("MQ-2")  //MQ2
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (10)
#define RatioMQ2CleanAir (9.83)

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ_PIN, Type);

#define interruptorPasillo 39  // Interruptor para led del pasillo
#define CHILD_ID_PASILLO 39

#define ledPasillo 40  // Led pasillo Pn
#define CHILD_ID_LED_PASILLO 40

#define interruptorPuertaGaraje 42  // Interruptor puerta Garaje
#define CHILD_ID_PUERTA_GARAJE 42

#define interruptorPuertaSalon 43  // Interruptor puerta salón
#define CHILD_ID_PUERTA_SALON 43

#define interruptorPuertaDormitorio 44  // Interruptor puerta dormitorio
#define CHILD_ID_PUERTA_DORMITORIO 44

#define CHILD_ID_LIGHT 51
#define photocellPin A1

#define SENSOR_TEMP_OFFSET 0
#define DHT_DATA_PIN 30  //DHT11 PIN
#define DHTTYPE DHT11

#define ledSalon1 29  //Led Salon 1 Pin
#define CHILD_ID_LED_SALON1 29

#define ledSalon2 28  //Led Salon 2 Pin
#define CHILD_ID_LED_SALON2 28

#define ledExterior1 47  //Led Exterior 1 Pin
#define CHILD_ID_LED_EXTERIOR1 47

#define ledExterior2 46  //Led Exterior 2 Pin
#define CHILD_ID_LED_EXTERIOR2 46

#define ledBath 26  //Led baño Pin
#define CHILD_ID_LED_BATH 26

#define ledAlarma 12  //Led rojo Alarma

#define ledStair 24  // Led escaleras pin
#define CHILD_ID_LED_STAIR 24
#define infStair1 23  // IR para led escaleras 1
#define infStair2 22  // IR para led escaleras 2

#define interruptorBath 25  // Interruptor para led Baño
#define CHILD_ID_BATH 25

#define ledGaraje 27  //Led Garaje Pin
#define CHILD_ID_LED_GARAJE 27

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1
#define CHILD_ID_INDICECALOR 38

#define interruptorSalon 37  //Interruptor para  ledSalon1 + ledSalon2
#define CHILD_ID_SALON 37

#define interruptorGaraje 33  //Interruptor Luz Garaje
#define CHILD_ID_GARAJE 33

#define StepAscensor 35  //Interruptor Motor de paso
#define CHILD_ID_ASCENSOR 35

#define BUZZER_PIN 32  //Buzzer Pin
#define CHILD_ID_BUZZER 32


#define INF_PIN2 45  // IR Alarma 1
#define INF_PIN 34   // IR Alarma 2

#define CHILD_ID_ALARMA 36  //Interruptor activar/desactivar Alarma

#define redPin 2
#define bluePin 3
#define greenPin 4
#define CHILD_ID_RGB 31

//LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

bool ledStairState = false;

boolean lastButtonStateSalon = LOW;
boolean lastButtonStatePasillo = LOW;
boolean lastButtonStateGaraje = LOW;
boolean lastButtonStateBath = LOW;

bool ascensorBajadaState = false;
bool ascensorSubidaState = false;

bool alarmaState = false;
bool buzzer_active = false;
unsigned long buzzer_activation_time = 0;
unsigned long buzzer_duration = 3000;  // Duración en milisegundos del Buzzer

unsigned long stairActivationTime = 0;
unsigned long ledStairDuration = 15000;  // Duración en milisegundos de la luz de la escalera

//Configuración motor de paso
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
MyMessage msgMQ2CO(CHILD_ID_MQ2_CO, V_LEVEL);
MyMessage msgMQ2Digital(CHILD_ID_MQ2_DIGITAL, V_STATUS);

DHT dht(DHT_DATA_PIN, DHTTYPE);
float temperature;
float humidity;
float indiceCalor;

long RGB_values[3] = { 0, 0, 0 };
float valor;

Servo servoPuertaSalon;
Servo servoPuertaDormitorio;
Servo servoPuertaGaraje;

unsigned long tiempoInicioFuncion1 = 0;
unsigned long tiempoInicioFuncion2 = 0;
unsigned long tiempoInicioFuncion3 = 0;
unsigned long duracionFuncion1 = 30000;  // 2 minutos en milisegundos
unsigned long duracionFuncion2 = 10000;  // 10 segundos en milisegundos
unsigned long duracionFuncion3 = 10000;
bool funcion1Activa = true;   // Función 1 activa al inicio
bool funcion2Activa = false;  // Función 2 inactiva al inicio
bool funcion3Activa = false;  // Función 3 inactiva al inicio
bool noBloqueo = true;

float threshold = 204.8;  //Umbral LDR -> 20%
float umbralGas = 1000;   //Umbral alarma Gas


void presentation() {

  sendSketchInfo("Casa Domótica TFG", "1.0");

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
  present(CHILD_ID_MQ2_CO, S_AIR_QUALITY, "Detector Gas");
  present(CHILD_ID_MQ2_DIGITAL, S_BINARY, "Aviso Gas");

  request(CHILD_ID_RGB, V_RGB);
  request(CHILD_ID_RGB, V_PERCENTAGE);
  request(CHILD_ID_RGB, V_LIGHT);
}



void sensorLDR() {

  // Si el valor leído por el sensor LDR es menor al umbral establecido
  if (analogRead(photocellPin) < threshold) {

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
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
  indiceCalor = dht.computeHeatIndex(temperature, humidity, false);

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

      noBloqueo = false;
    }
  }

  if (ledStairState && (millis() - stairActivationTime) >= ledStairDuration) {

    ledStairState = false;
    digitalWrite(ledStair, LOW);
    send(msgStair.set(ledStairState));
    lcdTempHum();
  }
}

void lcdTempHum() {

  lcd.clear();
  lcd.setCursor(0, 0);  // X, Y
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.write(0xDF);
  lcd.print("C");
  lcd.setCursor(0, 1);  // X, Y
  lcd.print("Hum:  ");
  lcd.print(humidity);
  lcd.write('%');

  noBloqueo = true;
}

void ascensorLCD() {

  if (ascensorSubidaState) {

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ascensor ");
    lcd.setCursor(0, 1);
    lcd.print("Subiendo.. ");
  }

  if (ascensorBajadaState) {

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ascensor ");
    lcd.setCursor(0, 1);
    lcd.print("Bajando.. ");
  }
}


void mq2Sensor() {

  MQ2.update();
  float valorGas = MQ2.readSensor();
  send(msgMQ2CO.set(int(valorGas)));

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Gas: ");
  lcd.setCursor(0, 1);
  lcd.print(valorGas);
  lcd.print(" ppm");

  //Se activa la alarma de Gas si se activa el pin digital del MQ2 o se excede el umbral
  if (digitalRead(MQ_PIN_DIGITAL) == LOW || valorGas > umbralGas) {

    digitalWrite(BUZZER_PIN, HIGH);
    digitalWrite(ledAlarma, HIGH);
    send(msgMQ2Digital.set(1));

  } else {

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(ledAlarma, LOW);
    send(msgMQ2Digital.set(0));
  }
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

  if (buzzer_active && (millis() - buzzer_activation_time) >= buzzer_duration) {

    //Cambio estado Buzzer
    buzzer_active = false;

    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(ledAlarma, LOW);
    send(msgBuzzer.set(buzzer_active));
    lcdTempHum();
  }
}

void setup() {

  MQ2.setRegressionMethod(1);
  MQ2.setA(36974);
  MQ2.setB(-3.109);

  /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */

  MQ2.init();

  MQ2.setRL(20);

  Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for (int i = 1; i <= 10; i++) {
    MQ2.update();
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0 / 10);
  Serial.println("  done!.");

  if (isinf(calcR0)) {
    Serial.println("Warning");
    while (1)
      ;
  }
  if (calcR0 == 0) {
    Serial.println("Warning");
    while (1)
      ;
  }

  MQ2.serialDebug(true);

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
  pinMode(MQ_PIN, INPUT);
  pinMode(MQ_PIN_DIGITAL, INPUT);

  myStepper.setSpeed(10);  //Velocidad StepMotor

  servoPuertaSalon.write(1);
  servoPuertaDormitorio.write(1);
  servoPuertaGaraje.write(10);


  dht.begin();
  sensorDHT11();
}

void loop() {

  unsigned long tiempoActual = millis();

  if (funcion1Activa && tiempoActual - tiempoInicioFuncion1 >= duracionFuncion1 && noBloqueo) {

    funcion1Activa = false;
    funcion2Activa = true;
    tiempoInicioFuncion2 = tiempoActual;
    sensorLDR();
  }

  if (funcion2Activa && tiempoActual - tiempoInicioFuncion2 >= duracionFuncion2 && noBloqueo) {

    funcion2Activa = false;
    funcion3Activa = true;
    tiempoInicioFuncion3 = tiempoActual;
    mq2Sensor();
  }

  if (funcion3Activa && tiempoActual - tiempoInicioFuncion3 >= duracionFuncion3 && noBloqueo) {

    funcion3Activa = false;
    funcion1Activa = true;
    tiempoInicioFuncion1 = tiempoActual;
    sensorDHT11();
  }

  sistemaAlarma();
  sistemaLedStair();
}


void receive(const MyMessage& message) {

  /*Procesar mensaje recibido por MySensors que contiene un valor RGB en 
   formato hexadecimal y extrae los componentes de rojo, verde, azul
   almacenandolos en RGB_values.
  */
  if (message.getSensor() == CHILD_ID_RGB && message.type == V_RGB) {

    String hexa = message.getString();
    long n = (long)strtol(&hexa[0], NULL, 16);

    RGB_values[0] = n >> 16;
    RGB_values[1] = n >> 8 & 0xFF;
    RGB_values[2] = n & 0xFF;
  }

  //Ajustar la intensidad de los componentes de color RGB, según el porcentaje recibido
  if (message.getSensor() == CHILD_ID_RGB && message.type == V_PERCENTAGE) {

    valor = message.getInt();

    analogWrite(redPin, int(RGB_values[0] * (valor / 100)));
    analogWrite(greenPin, int(RGB_values[1] * (valor / 100)));
    analogWrite(bluePin, int(RGB_values[2] * (valor / 100)));
  }

  if (message.getSensor() == CHILD_ID_RGB && message.type == V_LIGHT) {

    //Apagar led RGB
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

  //Manejo Individual Led Salon 1
  if (message.getSensor() == CHILD_ID_LED_SALON1 && message.type == V_STATUS) {

    digitalWrite(ledSalon1, message.getBool() ? HIGH : LOW);
  }

  //Manejo Individual Led Salon 2
  if (message.getSensor() == CHILD_ID_LED_SALON2 && message.getType() == V_STATUS) {

    digitalWrite(ledSalon2, message.getBool() ? HIGH : LOW);
  }


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

    send(msgLedSalon1.set(lastButtonStateSalon));
    send(msgLedSalon2.set(lastButtonStateSalon));
  }

  if (message.getSensor() == CHILD_ID_PASILLO && message.type == V_STATUS) {

    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStatePasillo = message.getBool();
    digitalWrite(ledPasillo, lastButtonStatePasillo ? HIGH : LOW);

    send(msgPasillo1.set(lastButtonStatePasillo));
  }

  if (message.getSensor() == CHILD_ID_ASCENSOR && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón

    if (message.getBool() == 1) {

      ascensorSubidaState = true;

      ascensorLCD();

      //Número de vueltas que da el motor de paso para llegar al piso 1
      myStepper.step(stepsPerRevolution * 8.5);

      ascensorSubidaState = false;

      lcdTempHum();

    } else if (message.getBool() == 0) {

      ascensorBajadaState = true;

      ascensorLCD();

      //Número de vueltas en dirección contrario que da el motor de paso para llegar al piso 0
      myStepper.step(-stepsPerRevolution * 8.5);

      ascensorBajadaState = false;

      lcdTempHum();
    }
  }


  if (message.getSensor() == CHILD_ID_PUERTA_SALON && message.type == V_STATUS) {

    //Si se recibe un 1, que será "Puerta Bloqueada" está se cerrará
    if (message.getBool() == 1) {

      servoPuertaSalon.write(1);

      //Si se recibe un 0, que será "Puerta Abierta" está se abrirá 90º
    } else if (message.getBool() == 0) {

      servoPuertaSalon.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_PUERTA_DORMITORIO && message.type == V_STATUS) {

    //Si se recibe un 1, que será "Puerta Bloqueada" está se cerrará
    if (message.getBool() == 1) {

      servoPuertaDormitorio.write(1);

      //Si se recibe un 0, que será "Puerta Abierta" está se abrirá 90º
    } else if (message.getBool() == 0) {

      servoPuertaDormitorio.write(90);
    }
  }

  if (message.getSensor() == CHILD_ID_PUERTA_GARAJE && message.type == V_STATUS) {

    //Si se recibe un 1, que será "Puerta Bloqueada" está se cerrará
    if (message.getBool() == 1) {

      servoPuertaGaraje.write(10);

      //Si se recibe un 0, que será "Puerta Abierta" está se abrirá 100º
    } else if (message.getBool() == 0) {

      servoPuertaGaraje.write(100);
    }
  }

  if (message.getSensor() == CHILD_ID_GARAJE && message.type == V_STATUS) {

    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateGaraje = message.getBool();
    digitalWrite(ledGaraje, lastButtonStateGaraje ? HIGH : LOW);

    send(msgLedGaraje.set(lastButtonStateGaraje));
  }

  if (message.getSensor() == CHILD_ID_BUZZER && message.type == V_STATUS) {

    digitalWrite(BUZZER_PIN, message.getBool() ? HIGH : LOW);
    digitalWrite(ledAlarma, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_MQ2_DIGITAL && message.type == V_STATUS) {

    digitalWrite(BUZZER_PIN, message.getBool() ? HIGH : LOW);
    digitalWrite(ledAlarma, message.getBool() ? HIGH : LOW);
  }

  if (message.getSensor() == CHILD_ID_BATH && message.type == V_STATUS) {

    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateBath = message.getBool();
    digitalWrite(ledBath, lastButtonStateBath ? HIGH : LOW);

    send(msgBath.set(lastButtonStateBath));
  }
}
