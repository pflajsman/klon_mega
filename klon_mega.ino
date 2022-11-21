#include <pt.h>
#include <PubSubClient.h>
#include <Ethernet.h>
#include <SPI.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

void callback(char* topic, byte* payload, unsigned int length);

//----------pin definition----------
//1NP HALL
const int hall1npMainDoorSensor = 22;
const int hall1npFloorTempSensor = 23;
#define hall1npAirTempHumSensor 24
const int hall1npMotionSensor = 25;
const int hall1npLightButton = 26;

//1NP WC
const int wc1npNilanBoostButton = 27;

//1NP kitchen
const int kitchenNilanBoostButton = 28;
const int kitchenMotionSensor = 29;
const int kitchenWindowSensor = 30;

//livingroom/dinningroom
const int livroomLightButton = 31;
const int dinroomLibraryButton = 32;
const int livroomFloorTempSensor = 33;
#define livroomAirTempHumSensor 34
const int livroomMotionSensor = 35;
const int livroomDoorToGardenSensor = 36;
const int dinroomLightButton = 37;
const int stairsLightButton = 38;
const int livingroomHall2npLightButton = 39;

//2np hall
const int hall2npLightButton = 40;
const int hall2npStairsLightButton = 41;

//parents Bedroom
const int parBedroomFloorTempSensor = 42;
const int parBedroomWindowSensor = 21;

//bathroom
const int bathroomFloorTempSensor = 43;
#define bathroomAirTempHumSensor 44
const int bathroomLedButton = 45;
const int bathroomNilanBoostButton = 46;

//child room 1
const int childroom1FloorTempSensor = 47;
#define childroom1AirTempHumSensor 48
const int childroom1WindowSensor = 20;

//child room 2
const int childroom2FloorTempSensor = 49;
#define childroom2AirTempHumSensor 50
const int childroom2WindowSensor = 19;

const int ralayNilanBoost = 51;

//----------end pin definition----------

//sensors
#define typDHT22 DHT22
#define typDHT11 DHT11

DHT hall1npAirTempHumSensorDefinition(hall1npAirTempHumSensor, typDHT22);
DHT livroomAirTempHumSensorDefinition(livroomAirTempHumSensor, typDHT22);
DHT bathroomAirTempHumSensorDefinition(bathroomAirTempHumSensor, typDHT22);
DHT childroom1AirTempHumSensorDefinition(childroom1AirTempHumSensor, typDHT22);
DHT childroom2AirTempHumSensorDefinition(childroom2AirTempHumSensor, typDHT22);


// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire1(hall1npFloorTempSensor);
OneWire oneWire2(livroomFloorTempSensor);
OneWire oneWire3(parBedroomFloorTempSensor);
OneWire oneWire4(bathroomFloorTempSensor);
OneWire oneWire5(childroom1FloorTempSensor);
OneWire oneWire6(childroom2FloorTempSensor);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature hall1npFloorTempSensorDefinition(&oneWire1);
DallasTemperature livroomFloorTempSensorDefinition(&oneWire2);
DallasTemperature parBedroomFloorTempSensorDefinition(&oneWire3);
DallasTemperature bathroomFloorTempSensorDefinition(&oneWire4);
DallasTemperature childroom1FloorTempSensorDefinition(&oneWire5);
DallasTemperature childroom2FloorTempSensorDefinition(&oneWire6);

//----------stateRegister----------
//switch
int hall1npLightButtonState = 0;
int livroomLightButtonState = 0;
int wc1npNilanBoostButtonState = 0;
int kitchenNilanBoostButtonState = 0;
int dinroomLibraryButtonState = 0;
int dinroomLightButtonState = 0;
int stairsLightButtonState = 0;
int livingroomHall2npLightButtonState = 0;
int hall2npLightButtonState = 0;
int hall2npStairsLightButtonState = 0;
int bathroomLedButtonState = 0;
int bathroomNilanBoostButtonState = 0;

//sensors
//DHT22
double stateFloorTemperature1 = 0.0;
double hall1npAirTempSensorState = 0.0;
double livroomAirTempSensorState = 0.0;
double bathroomAirTempSensorState = 0.0;
double childroom1AirTempSensorState = 0.0;
double childroom2AirTempSensorState = 0.0;
double hall1npAirHumSensorState = 0.0;
double livroomAirHumSensorState = 0.0;
double bathroomAirHumSensorState = 0.0;
double childroom1AirHumSensorState = 0.0;
double childroom2AirHumSensorState = 0.0;

//DALAS FLOOR
double hall1npFloorTempSensorState = 0.0;
double livroomFloorTempSensorState = 0.0;
double parBedroomFloorTempSensorState = 0.0;
double bathroomFloorTempSensorState = 0.0;
double childroom1FloorTempSensorState = 0.0;
double childroom2FloorTempSensorState = 0.0;

//motion
int stateMotionSensor_1_1 = 0;
int hall1npMotionSensorState = 0;
int kitchenMotionSensorState = 0;
int livroomMotionSensorState = 0;

//ralay nilan boost state
int ralayNilanBoostState = 0;

//----------endStateRegister----------

//Arduino network settings
byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDA, 0x02 };
//IPAddress ip(192, 168, 88, 251);
EthernetClient mega1;

//Mqtt settings------------------------
#define mqtt_server "192.168.88.250"
#define mqtt_user "hass"
#define mqtt_password "inmate543964"


//Mqtt articles publish----------------
//1np hall
#define np1_predsin_dvere "1np/predsin/dvere"
#define np1_predsin_teplota_podlaha "1np/predsin/teplota_podlaha"
#define np1_predsin_teplota_vzduch "1np/predsin/teplota_vzduch"
#define np1_predsin_vlhkost_vzduch "1np/predsin/vlhkost_vzduch"
#define np1_predsin_pohyb "1np/predsin/pohyb"
#define np1_predsin_vypinac_osvetleni "1np/predsin/vypinac_osvetleni"

//1np wc
#define np1_wc_nilan_boost "1np/wc/nilan_boost"

//kitchen
#define np1_kuchyn_nilan_boost "1np/kuchyn/nilan_boost"
#define np1_kuchyn_pohyb "1np/kuchyn/pohyb"
#define np1_kuchyn_okno "1np/kuchyn/okno"

//livingroom
#define np1_obyvak_vypinac_svetlo_obyvak "1np/obyvak/vypinac_svetlo_obyvak"
#define np1_obyvak_vypinac_svetlo_knihovna "1np/obyvak/vypinac_svetlo_knihovna"
#define np1_obyvak_teplota_podlaha "1np/obyvak/teplota_podlaha"
#define np1_obyvak_teplota_vzduch "1np/obyvak/teplota_vzduch"
#define np1_obyvak_teplota_vlhkost "1np/obyvak/teplota_vlhkost"
#define np1_obyvak_pohyb "1np/obyvak/pohyb"
#define np1_obyvak_dvere "1np/obyvak/dvere"
#define np1_obyvak_vypinac_svetlo_jidelna "1np/obyvak/vypinac_svetlo_jidelna"
#define np1_obyvak_vypinac_svetlo_schodiste "1np/obyvak/vypinac_svetlo_schodiste"
#define np1_obyvak_vypinac_svetlo_2npchodba "1np/obyvak/vypinac_svetlo_2npchodba"

//2np hall
#define np2_chodba_vypinac_svetlo_schodiste "2np/chodba/vypinac_svetlo_schodiste"
#define np2_chodba_vypinac_svetlo_chodba "2np/chodba/vypinac_svetlo_chodba"

//parents Bedroom
#define np2_loznice_teplota_podlaha "2np/loznice/teplota_podlaha"
#define np2_loznice_teplota_vzduch "2np/loznice/teplota_vzduch"
#define np2_loznice_vlhkost_vzduch "2np/loznice/vlhkost_vzduch"
#define np2_loznice_vypinac_led_zaves "2np/loznice/vypinac_led_zaves"
#define np2_loznice_okno "2np/loznice/okno"

//bathroom
#define np2_koupelna_teplota_podlaha "2np/koupelna/teplota_podlaha"
#define np2_koupelna_teplota_vzduch "2np/koupelna/teplota_vzduch"
#define np2_koupelna_vlhkost_vzduch "2np/koupelna/vlhkost_vzduch"
#define np2_koupelna_vypinac_led_pod_umyvadlem "2np/koupelna/vypinac_led_pod_umyvadlem"
#define np2_koupelna_nilan_boost "2np/koupelna/nilan_boost"

//child room 2
#define np2_dp2_teplota_podlaha "2np/dp2/teplota_podlaha"
#define np2_dp2_teplota_vzduch "2np/dp2/teplota_vzduch"
#define np2_dp2_vlhkost_vzduch "2np/dp2/vlhkost_vzduch"
#define np2_dp2_okno "2np/dp2/okno"

//child room 1
#define np2_dp1_teplota_podlaha "2np/dp1/teplota_podlaha"
#define np2_dp1_teplota_vzduch "2np/dp1/teplota_vzduch"
#define np2_dp1_vlhkost_vzduch "2np/dp1/vlhkost_vzduch"
#define np2_dp1_okno "2np/dp1/okno"
#define motion_1_1 "motion/livingroomMotion1_1"

const char* ralay_nilan_boost = "relay/nilan_boost/set";
const char* state_ralay_nilan_boost = "relay/nilan_boost/available";

//Protothread
static struct pt pt1, pt2, pt3;

PubSubClient mqttClient(mqtt_server, 1883, callback, mega1);

void reconnect() {
  // Loop until we're reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID

    if (mqttClient.connect("KlonClient", mqtt_user, mqtt_password)) {
      Serial.println("connected");
      //once connected to MQTT broker, subscribe command if any
      mqttClient.subscribe(ralay_nilan_boost);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}  //end reconnect()

void EthernetConnect() {
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    // no point in carrying on, so do nothing forevermore:
    for (;;)
      ;
  }
  // print your local IP address:
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(Ethernet.localIP()[thisByte], DEC);
    Serial.print(".");
  }
  Serial.println();
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  //motion
  pinMode(hall1npMotionSensor, INPUT);
  digitalWrite(hall1npMotionSensor, LOW);
  pinMode(kitchenMotionSensor, INPUT);
  digitalWrite(kitchenMotionSensor, LOW);
  pinMode(livroomMotionSensor, INPUT);
  digitalWrite(livroomMotionSensor, LOW);
  //motion end
  //binary sensor
  pinMode(hall1npMainDoorSensor, INPUT);
  digitalWrite(hall1npMainDoorSensor, HIGH);
  pinMode(kitchenWindowSensor, INPUT);
  digitalWrite(kitchenWindowSensor, HIGH);
  pinMode(livroomDoorToGardenSensor, INPUT);
  digitalWrite(livroomDoorToGardenSensor, HIGH);
  pinMode(parBedroomWindowSensor, INPUT);
  digitalWrite(parBedroomWindowSensor, HIGH);
  pinMode(childroom1WindowSensor, INPUT);
  digitalWrite(childroom1WindowSensor, HIGH);
  pinMode(childroom2WindowSensor, INPUT);
  digitalWrite(childroom2WindowSensor, HIGH);
  //binary sensor end
  pinMode(ralayNilanBoost, OUTPUT);
  digitalWrite(ralayNilanBoost, LOW);
  // start the Ethernet connection:
  EthernetConnect();
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(callback);
  PT_INIT(&pt1);  // sensor thread
  PT_INIT(&pt2);  // button thread
  PT_INIT(&pt3);  // relay thread
  //buttons
  pinMode(hall1npLightButton, INPUT);
  digitalWrite(hall1npLightButton, LOW);
  pinMode(wc1npNilanBoostButton, INPUT);
  digitalWrite(wc1npNilanBoostButton, LOW);
  pinMode(kitchenNilanBoostButton, INPUT);
  digitalWrite(kitchenNilanBoostButton, LOW);
  pinMode(livroomLightButton, INPUT);
  digitalWrite(livroomLightButton, LOW);
  pinMode(dinroomLibraryButton, INPUT);
  digitalWrite(dinroomLibraryButton, LOW);
  pinMode(dinroomLightButton, INPUT);
  digitalWrite(dinroomLightButton, LOW);
  pinMode(stairsLightButton, INPUT);
  digitalWrite(stairsLightButton, LOW);
  pinMode(livingroomHall2npLightButton, INPUT);
  digitalWrite(livingroomHall2npLightButton, LOW);
  pinMode(hall2npLightButton, INPUT);
  digitalWrite(hall2npLightButton, LOW);
  pinMode(hall2npStairsLightButton, INPUT);
  digitalWrite(hall2npStairsLightButton, LOW);
  pinMode(bathroomLedButton, INPUT);
  digitalWrite(bathroomLedButton, LOW);
  pinMode(bathroomNilanBoostButton, INPUT);
  digitalWrite(bathroomNilanBoostButton, LOW);
  //buttons end
  //sensors dht22 Air
  hall1npAirTempHumSensorDefinition.begin();
  livroomAirTempHumSensorDefinition.begin();
  bathroomAirTempHumSensorDefinition.begin();
  childroom1AirTempHumSensorDefinition.begin();
  childroom2AirTempHumSensorDefinition.begin();

  //sensors DALAS floor
  hall1npFloorTempSensorDefinition.begin();
  livroomFloorTempSensorDefinition.begin();
  parBedroomFloorTempSensorDefinition.begin();
  bathroomFloorTempSensorDefinition.begin();
  childroom1FloorTempSensorDefinition.begin();
  childroom2FloorTempSensorDefinition.begin();
}

//----------Buttons----------
void switchCheck() {

  if (digitalRead(hall1npLightButton) == !hall1npLightButtonState) {
    mqttClient.publish(np1_predsin_vypinac_osvetleni, "ON");
    hall1npLightButtonState = !hall1npLightButtonState;
  }

  if (digitalRead(wc1npNilanBoostButton) == !wc1npNilanBoostButtonState) {
    mqttClient.publish(np1_wc_nilan_boost, "ON");
    wc1npNilanBoostButtonState = !wc1npNilanBoostButtonState;
  }

  if (digitalRead(kitchenNilanBoostButton) == !kitchenNilanBoostButtonState) {
    mqttClient.publish(np1_kuchyn_nilan_boost, "ON");
    kitchenNilanBoostButtonState = !kitchenNilanBoostButtonState;
  }

  if (digitalRead(livroomLightButton) == !livroomLightButtonState) {
    mqttClient.publish(np1_obyvak_vypinac_svetlo_obyvak, "ON");
    livroomLightButtonState = !livroomLightButtonState;
  }

  if (digitalRead(dinroomLibraryButton) == !dinroomLibraryButtonState) {
    mqttClient.publish(np1_obyvak_vypinac_svetlo_knihovna, "ON");
    dinroomLibraryButtonState = !dinroomLibraryButtonState;
  }

  if (digitalRead(dinroomLightButton) == !dinroomLightButtonState) {
    mqttClient.publish(np1_obyvak_vypinac_svetlo_jidelna, "ON");
    dinroomLightButtonState = !dinroomLightButtonState;
  }

  if (digitalRead(stairsLightButton) == !stairsLightButtonState) {
    mqttClient.publish(np1_obyvak_vypinac_svetlo_schodiste, "ON");
    stairsLightButtonState = !stairsLightButtonState;
  }

  if (digitalRead(livingroomHall2npLightButton) == !livingroomHall2npLightButtonState) {
    mqttClient.publish(np1_obyvak_vypinac_svetlo_2npchodba, "ON");
    livingroomHall2npLightButtonState = !livingroomHall2npLightButtonState;
  }

  if (digitalRead(hall2npLightButton) == !hall2npLightButtonState) {
    mqttClient.publish(np2_chodba_vypinac_svetlo_chodba, "ON");
    hall2npLightButtonState = !hall2npLightButtonState;
  }

  if (digitalRead(hall2npStairsLightButton) == !hall2npStairsLightButtonState) {
    mqttClient.publish(np2_chodba_vypinac_svetlo_schodiste, "ON");
    hall2npStairsLightButtonState = !hall2npStairsLightButtonState;
  }

  if (digitalRead(bathroomLedButton) == !bathroomLedButtonState) {
    mqttClient.publish(np2_koupelna_vypinac_led_pod_umyvadlem, "ON");
    bathroomLedButtonState = !bathroomLedButtonState;
  }

  if (digitalRead(bathroomNilanBoostButton) == !bathroomNilanBoostButtonState) {
    mqttClient.publish(np2_koupelna_nilan_boost, "ON");
    bathroomNilanBoostButtonState = !bathroomNilanBoostButtonState;
  }
}

//thread method
static int protoThreadButtons(struct pt* pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis();
    //buttonCheck();
    switchCheck();
  }
  PT_END(pt);
}
//----------end buttons----------

//----------Sensors----------
void temperatureHumidityDht(DHT sensor, char* publishTemperature, char* publishHumidity, double* stateTemperature, double* stateHumidity) {
  char charBuffer[13];
  String str;
  double temperature = 0.0;
  double humidity = 0.0;
  temperature = sensor.readTemperature();
  humidity = sensor.readHumidity();
  // kontrola, jestli jsou načtené hodnoty čísla pomocí funkce isnan
  if (isnan(temperature) || isnan(humidity)) {
    // při chybném čtení vypiš hlášku
    Serial.println("Chyba při čtení z DHT senzoru!");
  } else {

    Serial.print("Teplota: ");
    Serial.print(temperature);
    Serial.print(" stupnu Celsia, ");
    str = String(temperature);
    str.toCharArray(charBuffer, str.length());
    if (temperature != *stateTemperature) {
      mqttClient.publish(publishTemperature, charBuffer);
      *stateTemperature = temperature;
    }

    Serial.print("vlhkost: ");
    Serial.print(humidity);
    Serial.println("  %");
    str = String(humidity);
    str.toCharArray(charBuffer, str.length());
    if (humidity != *stateHumidity) {
      mqttClient.publish(publishHumidity, charBuffer);
      *stateHumidity = humidity;
    }
  }
  return;
}

void temperatureDalas(DallasTemperature sensor, char* publishTemperature) {
  sensor.requestTemperatures();
  double temperatureC = 0.0;
  temperatureC = sensor.getTempCByIndex(0);
  if (temperatureC > -50) {
    Serial.print(temperatureC);
    Serial.println("ºC");
    char charBuffer[13];
    String str = String(temperatureC);
    str.toCharArray(charBuffer, str.length());
    mqttClient.publish(publishTemperature, charBuffer);
  } else {
    Serial.println("Sensor in error state.");
  }
}

//thread method
static int protoThreadSensors(struct pt* pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis();
    //dht22
    temperatureHumidityDht(hall1npAirTempHumSensorDefinition, np1_predsin_teplota_vzduch, np1_predsin_vlhkost_vzduch, &hall1npAirTempSensorState, &hall1npAirHumSensorState);
    temperatureHumidityDht(livroomAirTempHumSensorDefinition, np1_obyvak_teplota_vzduch, np1_obyvak_teplota_vlhkost, &livroomAirTempSensorState, &livroomAirHumSensorState);
    temperatureHumidityDht(bathroomAirTempHumSensorDefinition, np2_koupelna_teplota_vzduch, np2_koupelna_vlhkost_vzduch, &bathroomAirTempSensorState, &bathroomAirHumSensorState);
    temperatureHumidityDht(childroom1AirTempHumSensorDefinition, np2_dp1_teplota_vzduch, np2_dp1_vlhkost_vzduch, &childroom1AirTempSensorState, &childroom1AirHumSensorState);
    temperatureHumidityDht(childroom2AirTempHumSensorDefinition, np2_dp2_teplota_vzduch, np2_dp2_vlhkost_vzduch, &childroom2AirTempSensorState, &childroom2AirHumSensorState);
    //dalas
    temperatureDalas(hall1npFloorTempSensorDefinition, np1_predsin_teplota_podlaha);
    temperatureDalas(livroomFloorTempSensorDefinition, np1_obyvak_teplota_podlaha);
    temperatureDalas(parBedroomFloorTempSensorDefinition, np2_loznice_teplota_podlaha);
    temperatureDalas(bathroomFloorTempSensorDefinition, np2_koupelna_teplota_podlaha);
    temperatureDalas(childroom1FloorTempSensorDefinition, np2_dp1_teplota_podlaha);
    temperatureDalas(childroom2FloorTempSensorDefinition, np2_dp2_teplota_podlaha);
  }
  PT_END(pt);
}
//----------end sensors----------

//----------Relay----------
void relaySet(char* topic, byte* payload, const char* relayArticle, int relayGpio) {

  String msg = String((char)payload[1]);
  String top = String(topic);
  if (top == relayArticle && msg == "N") {
    digitalWrite(relayGpio, HIGH);
    Serial.print("Relay on");
  }
  if (top == relayArticle && msg == "F") {
    Serial.print("Relay off");
    digitalWrite(relayGpio, LOW);
  }
}

void relayState(const char* relayArticle, int relayGpio) {
  mqttClient.publish("relay/livingroomRelay1_1/available", "online");
  if (digitalRead(relayGpio) == HIGH) {
    mqttClient.publish(relayArticle, "ON");
  } else {
    mqttClient.publish(relayArticle, "OFF");
  }
}

//thread method
static int protoThreadRelay(struct pt* pt, int interval) {
  static unsigned long timestamp = 0;
  PT_BEGIN(pt);
  while (1) {
    PT_WAIT_UNTIL(pt, millis() - timestamp > interval);
    timestamp = millis();
    relayState(state_ralay_nilan_boost, ralayNilanBoost);
  }
  PT_END(pt);
}
//----------end relay----------

//----------Motion sensors----------
void motionSensor(int motionSensorPin, int motionSensorState, const char* motionMqttMessage) {
  if (digitalRead(motionSensorPin) == HIGH) {
    Serial.println("Detekce pohybu pomoci HC-SR501!");
    mqttClient.publish(motionMqttMessage, "ON");
  } else if (digitalRead(motionSensorPin) == LOW) {
    mqttClient.publish(motionMqttMessage, "OFF");
  }
}
//----------end motions----------
//----------Binary sensors----------
void binarySensor(int binarySensorPin, const char* binarySensorMqttMessage) {
  if (digitalRead(binarySensorPin) == LOW) {
    Serial.print("Sensor rozepnut"); 
    mqttClient.publish(binarySensorMqttMessage, "Open");
  } 
  else if (digitalRead(binarySensorPin) == HIGH) {
    Serial.print("Sensor sepnut");
    mqttClient.publish(binarySensorMqttMessage, "Close");
  }
}
//----------Binary motions----------

void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }
  motionSensor(hall1npMotionSensor, hall1npMotionSensorState, np1_predsin_pohyb);
  motionSensor(kitchenMotionSensor, kitchenMotionSensorState, np1_kuchyn_pohyb);
  motionSensor(livroomMotionSensor, livroomMotionSensorState, np1_obyvak_pohyb);
  
  binarySensor(hall1npMainDoorSensor, np1_predsin_dvere);
  binarySensor(kitchenWindowSensor, np1_kuchyn_okno);
  binarySensor(livroomDoorToGardenSensor, np1_obyvak_dvere);
  binarySensor(parBedroomWindowSensor, np2_loznice_okno);
  binarySensor(childroom1WindowSensor, np2_dp1_okno);
  binarySensor(childroom2WindowSensor, np2_dp2_okno);

  protoThreadButtons(&pt2, 20);     // by calling them infinitely
  protoThreadRelay(&pt3, 30);       // by calling them infinitely
  protoThreadSensors(&pt1, 20000);  // by calling them infinitely
  mqttClient.loop();
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("CallBack");
  String top = String(topic);
  Serial.println(top);
  Serial.println(char(payload[1]));
  relaySet(topic, payload, ralay_nilan_boost, ralayNilanBoost);
}  //end callback