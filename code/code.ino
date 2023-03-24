#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_RF24_CE_PIN 49
#define MY_RF24_CS_PIN 48
#define MY_RF24_CHANNEL 76
#define MY_NODE_ID 95


#include <MySensors.h>
#include <DHT.h>
#include <Servo.h> 
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <Stepper.h>


static const uint64_t UPDATE_INTERVAL = 5000;
static const uint8_t FORCE_UPDATE_N_READS = 10;


#define SENSOR_TEMP_OFFSET 0

#define DHT_DATA_PIN 30 //DHT11 PIN
#define DHTTYPE DHT11

#define ledSalon1 29 //Led1 Pin
#define CHILD_ID_LED_SALON1 29

#define ledSalon2 28 //Led2 Pin
#define CHILD_ID_LED_SALON2 28

#define ledBath 26 //Led baño
#define CHILD_ID_LED_BATH 26

#define ledAlarma 12 //led rojo de la alarma

#define ledStair 24 // Led escaleras
#define CHILD_ID_LED_STAIR 24
#define infStair1 23
#define infStair2 22



#define interruptorBath 25 // Interruptor del baño
#define CHILD_ID_BATH 25

#define ledGaraje 27 //Led Garaje
#define CHILD_ID_LED_GARAJE 27

#define CHILD_ID_HUM 0
#define CHILD_ID_TEMP 1

#define interruptorSalon 37 //Interruptor Led1 + Led2 del salón
#define CHILD_ID_SALON 37

#define interruptorGaraje 33 //Interruptor Luz Garaje
#define CHILD_ID_GARAJE 33

#define StepAscensor 35 //Interruptor Stepmotor
#define CHILD_ID_ASCENSOR 35

#define BUZZER_PIN 32 //Buzzer Pin
#define CHILD_ID_BUZZER 32

#define INF_PIN 34 //Infrarrojos Buzzer
#define CHILD_ID_INF 34 

#define CHILD_ID_ALARMA 36 //Interruptor Alarma

#define redpin 2
#define bluepin 3
#define greenpin 4
#define CHILD_ID_RGB 31

LiquidCrystal_I2C lcd(0x27, 16, 2);

int oldValue = 0;
int oldValue2 = 0;
int buzzerState = HIGH;
bool ledStateSalon1 = false;
boolean lastButtonStateSalon = LOW;
boolean lastButtonStateGaraje = LOW;
boolean lastButtonStateBath = LOW;
bool ledStateSalon2 = false;
bool ledStateGaraje = false;
bool ledStateBath = false;
bool alarmaState = false;

bool buzzer_active = false;
bool ledStairState = false;
unsigned long buzzer_activation_time = 0;
unsigned long buzzer_duration = 3000;
unsigned long stairActivationTime = 0;
unsigned long ledStairDuration = 15000;


const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

MyMessage msg(CHILD_ID_LED_SALON1, V_LIGHT);
MyMessage msg2(CHILD_ID_LED_SALON2, V_LIGHT);
MyMessage msg3(CHILD_ID_LED_GARAJE, V_LIGHT);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgBuzzer(CHILD_ID_BUZZER, V_STATUS);
MyMessage msgIntSalon(CHILD_ID_SALON, V_STATUS);
MyMessage msgStepBoton(CHILD_ID_ASCENSOR, V_STATUS);
MyMessage msgIntGaraje(CHILD_ID_GARAJE, V_STATUS);
MyMessage msgBotonAlarma(CHILD_ID_ALARMA, V_STATUS);
MyMessage msgBright(CHILD_ID_RGB, V_PERCENTAGE);
MyMessage msgColor(CHILD_ID_RGB, V_RGB);
MyMessage msgLight(CHILD_ID_RGB, V_LIGHT);
MyMessage msgBath(CHILD_ID_LED_BATH, V_LIGHT);
MyMessage msgIntBath(CHILD_ID_BATH, V_STATUS);
MyMessage msgStair(CHILD_ID_LED_STAIR, V_LIGHT);

DHT dht(DHT_DATA_PIN, DHTTYPE);

unsigned long timeOfLastChange = 0;


long RGB_values[3] = {0, 0, 0};
float valor;

void setup()
{

  lcd.init();
  lcd.backlight();
  lcd.clear();
  dht.begin();
  lcd.setCursor(0, 0);
  lcd.print("Loading...");
  pinMode(interruptorSalon, INPUT_PULLUP);
  pinMode(interruptorGaraje, INPUT_PULLUP);
  pinMode(interruptorBath, INPUT_PULLUP);
  pinMode(StepAscensor, INPUT_PULLUP);
  pinMode(ledSalon1, OUTPUT);
  pinMode(ledSalon2, OUTPUT);
  pinMode(ledGaraje, OUTPUT);
  pinMode(ledAlarma, OUTPUT);
  pinMode(ledBath, OUTPUT);
  pinMode(ledStair, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(bluepin, OUTPUT);

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

  request(CHILD_ID_RGB, V_RGB);
  request(CHILD_ID_RGB, V_PERCENTAGE);
  request(CHILD_ID_RGB, V_LIGHT);

  send(msgBright.set(100));
  send(msgColor.set("000000"));
  send(msgLight.set(1));

  myStepper.setSpeed(10);

  digitalWrite(redpin, 0);
  digitalWrite(greenpin, 255);
  digitalWrite(bluepin, 0);

}

void loop()
{

  //---------------------------------------------------- INTERRUPTORES LEDS ---------------------------------------------------------------------------------
  

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

  //--------------------------------------------------------------------------------------------------------------------------------------------

  //----------------------------------------------------BUZZER---------------------------------------------------------------------------------
  

  bool ir_detected = digitalRead(INF_PIN);
  
  
  if (!ir_detected && !buzzer_active ) {
    

    if(alarmaState){
      
      buzzer_active = true;
      buzzer_activation_time = millis();
      digitalWrite(BUZZER_PIN, HIGH);
      digitalWrite(ledAlarma, HIGH);
      send(msgBuzzer.set(buzzer_active));
    }
  }
  
  if (buzzer_active && (millis() - buzzer_activation_time) >= buzzer_duration) { // se compara el tiempo transcurrido desde que se activó la alarma (millis() - buzzer_activation_time) con la duración del tiempo que se quiere que el buzzer esté activo (buzzer_duration).
    buzzer_active = false;
    digitalWrite(BUZZER_PIN, LOW);
    digitalWrite(ledAlarma, LOW);
    send(msgBuzzer.set(buzzer_active));
  }

  //---------------------------------------------------- LED STAIR---------------------------------------------------------------------------------

  bool irStair1 = digitalRead(infStair1);
  bool irStair2 = digitalRead(infStair2);

  if (!irStair1 || !irStair2) {

    ledStairState = true;
    stairActivationTime = millis();
    digitalWrite(ledStair, HIGH);
    send(msgStair.set(ledStairState));
    
    }

    if (ledStairState && (millis() - stairActivationTime) >= ledStairDuration) { // se compara el tiempo transcurrido desde que se activó la alarma (millis() - buzzer_activation_time) con la duración del tiempo que se quiere que el buzzer esté activo (buzzer_duration).
    ledStairState = false;
    digitalWrite(ledStair, LOW);
    send(msgStair.set(ledStairState));
  }

  
  //--------------------------------------------------------------------------------------------------------------------------------------------

  
  //-------------------------------------------------- SENSOR TEMP+HUM + LCD -------------------------------------------------------------------------


  if (millis() % 60000 == 0){
    
  lcd.clear();
  dht.read(true);
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  Serial.println(F("Humedad: "));
  Serial.println(humidity);

  Serial.println("Temperatura: ");
  Serial.println(temperature);
  lcd.setCursor(0, 0); // X, Y
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.setCursor(0, 1); // X, Y
  lcd.print("Hum:  ");
  lcd.print(humidity);

  send(msgTemp.set(temperature, 1));
  send(msgHum.set(humidity, 1));

  }

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

  


}


void receive(const MyMessage &message) {


  if (message.getSensor()==CHILD_ID_RGB && message.type == V_RGB) {

   String hexa = message.getString(); 
   long n = (long) strtol( &hexa[0], NULL, 16);
   RGB_values[0] = n >> 16;
   RGB_values[1] = n >> 8 & 0xFF;
   RGB_values[2] = n & 0xFF;
 }
 if (message.getSensor()==CHILD_ID_RGB && message.type == V_PERCENTAGE) {
   valor = message.getInt();
   analogWrite(redpin, int(RGB_values[0] * (valor / 100)));
   analogWrite(greenpin, int(RGB_values[1] * (valor / 100)));
   analogWrite(bluepin, int(RGB_values[2] * (valor / 100)));
 }

 if (message.getSensor()==CHILD_ID_RGB && message.type == V_LIGHT) {
   if (message.getInt() == 0) {
     digitalWrite(redpin, 0);
     digitalWrite(bluepin, 0);
     digitalWrite(greenpin, 0);

   }
   if (message.getInt() == 1) {
     analogWrite(redpin, int(RGB_values[0] * (valor / 100)));
     analogWrite(greenpin, int(RGB_values[1] * (valor / 100)));
     analogWrite(bluepin, int(RGB_values[2] * (valor / 100)));
   }
 }
  

  if (message.getSensor()==CHILD_ID_LED_SALON1 && message.type == V_STATUS) {
    
    digitalWrite(ledSalon1, message.getBool()?HIGH:LOW);
  } 

  if (message.getSensor()==CHILD_ID_LED_BATH && message.type == V_STATUS) {
    
    digitalWrite(ledBath, message.getBool()?HIGH:LOW);
  } 

  if (message.getSensor()==CHILD_ID_ALARMA && message.type == V_STATUS) {
    
    if(message.getBool()) {
      alarmaState = true;
    }else {
      alarmaState = false;
    }
  } 

  if (message.getSensor()==CHILD_ID_LED_GARAJE && message.type == V_STATUS) {
    
    digitalWrite(ledGaraje, message.getBool()?HIGH:LOW);
  } 
  
  if (message.getSensor()==CHILD_ID_SALON && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateSalon = message.getBool();
    //Serial.println(message.getBool());
    //digitalWrite(ledSalon1, lastButtonStateSalon ? HIGH : LOW);
    //digitalWrite(ledSalon2, lastButtonStateSalon ? HIGH : LOW);
    if (message.getBool() == 1){

      digitalWrite(ledSalon1, HIGH);
      digitalWrite(ledSalon2, HIGH);

      } else if (message.getBool() == 0){

        digitalWrite(ledSalon1, LOW);
        digitalWrite(ledSalon2, LOW);

        }
    //Serial.println(lastButtonStateSalon);
  }

  if (message.getSensor()==CHILD_ID_ASCENSOR && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    
    if (message.getBool() == 1){

      myStepper.step(stepsPerRevolution * 5);

      } else if (message.getBool() == 0){

        myStepper.step(-stepsPerRevolution * 5);

        }
    //Serial.println(lastButtonStateSalon);
  }

  if (message.getSensor()==CHILD_ID_GARAJE && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateSalon = message.getBool();
    digitalWrite(ledGaraje, lastButtonStateSalon ? HIGH : LOW);
  }

  if (message.getSensor()==CHILD_ID_BATH && message.type == V_STATUS) {
    // Cambiar el estado del LED si se recibe un mensaje del botón
    lastButtonStateBath = message.getBool();
    digitalWrite(ledBath, lastButtonStateBath ? HIGH : LOW);
  }

   if (message.getSensor()==CHILD_ID_LED_SALON2 && message.getType()== V_STATUS) {
     // Change relay state
    digitalWrite(ledSalon2, message.getBool()?HIGH:LOW);
     // Store state in eeprom
     
     
   } 
} 
