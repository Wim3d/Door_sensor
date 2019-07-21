/*
   Door sensor with TPL5111 and TPS73733
   Power on when door is opened (NC contact made)
   Send via ESP NOW, if no success via ESP now, switch to MQTT
   Connect to WiFi and MQTT (if not already connected)
   Send MQTT message when door is closed
*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <credentials.h> //credentials for WiFi, IFTTT, MQTT
extern "C" {
#include <espnow.h>
}

#define WIFIMAXTIME 8000
#define CALLBACKMAXTIME 800
#define MQTTDELAY 100

#define SECOND 1000
#define MINUTE 60*SECOND
#define MAXONTIME MINUTE*1

#define DETECTPIN 0
#define DONEPIN 3
#define ESP_FAILED 1

#define SERIALDEBUG 0
#define VOLT_LIMIT 3.20

#define LENGTH 40 //message length

ADC_MODE(ADC_VCC); //vcc read-mode
boolean message_confirmed = false;
boolean timer = false;
boolean update_state = false;

uint16_t time1, time2;

// MQTT
const char* mqtt_id = "doorsensor1";
char* sensor_topic = "sensor/doorsensor1";
//char* state_topic = "sensor/doorsensor1/state";
char* debug_topic = "sensor/doorsensor1/debug";
char* voltage_topic = "sensor/doorsensor1/voltage";
char* update_topic = "sensor/doorsensor1/update";
char* payload_open = "OPEN";                // MQTT payload
char* payload_closed = "CLOSED";            // MQTT payload
char* payload_null = "NULL";                // MQTT payload

WiFiClient espClient;
PubSubClient client(espClient);

String tmp_str; // String for publishing the int's as a string to MQTT
char buf[5];

// MAC address of the ESP with which it is paired (slave), global defined since used in setup and loop
uint8_t mac_addr[6] = {0x6A, 0xC6, 0x3A, 0xC4, 0xA7, 0x5C}; // MAC address of acces point

uint8_t result = 1;

void setup() {
  delay(100);
  yield();
  pinMode(DONEPIN, OUTPUT);
  pinMode(DETECTPIN, INPUT);
  if (SERIALDEBUG)
    Serial.begin(115200);
  Serial.println(""); Serial.println("");
  if (digitalRead(DETECTPIN) == HIGH)   // GPIO0 should be high, except if the door is not really opened
  {
    init_esp_now(); // see below is not esp_now_init

    // send door open
    if (senddata(sensor_topic, payload_open) == ESP_FAILED) // no succes via ESP now, switch to MQTT
    {
      Serial.println("ESP-now failed");
      setup_wifi();
      connectMQTT();
      client.publish(sensor_topic, payload_open);
      // publish voltage via MQTT
      Serial.println("data sent via MQTT");
      client.publish(debug_topic, "data sent via MQTT");
    }


    while (digitalRead(DETECTPIN) == HIGH && timer == false)  // door is opened, do nothing but connect to WiFi and MQTT
    {
      // connect to WiFi while door is open
      if (!client.connected()) {
        setup_wifi();
        connectMQTT();
      }
      if (millis() > MAXONTIME)
        timer = true;
      yield();
      delay(10);
    }

    // measure and publish voltage
    float Voltage = ESP.getVcc() / (float)1024;
    int volt2 = round(Voltage * 100);
    float volt_round = volt2 / (float)100;
    tmp_str = String(volt_round); //converting voltage to a string
    tmp_str.toCharArray(buf, tmp_str.length() + 1);
    message_confirmed = false;
    client.publish(voltage_topic, buf);
  }
  if (timer)
  {
    client.publish(sensor_topic, payload_null);
    client.publish(debug_topic, "door opened too long");
    message_confirmed = true;
  }

  while (!message_confirmed)
  {
    if (!client.connected()) {
      setup_wifi();
      connectMQTT();
    }
    client.loop();
    if (millis() - time1 > MQTTDELAY)
    {
      client.publish(sensor_topic, payload_closed);
      time1 = millis();
    }
  }
  client.publish(debug_topic, "Endroutine");
  endroutine();
}

void loop() // this will never run
{
  yield();
}

void setup_wifi() {
  time1 = millis();
  delay(10);
  WiFi.mode(WIFI_STA);
  // We start by connecting to a WiFi network
  WiFi.begin(mySSID, myPASSWORD);

  while (WiFi.status() != WL_CONNECTED && time2 < WIFIMAXTIME) {
    delay(250);
    Serial.print(".");
    time2 = millis() - time1;
  }
  if (WiFi.status() != WL_CONNECTED)
    ESP.restart();
  Serial.println("");
  Serial.print("WiFi connected, ");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

boolean connectMQTT()
{
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  Serial.println("Attempting MQTT connection...");
  if (client.connect(mqtt_id)) {
    Serial.println("connected");
    // ... and resubscribe
    client.subscribe(sensor_topic);
    client.subscribe(update_topic);
  }
  Serial.println(client.connected());
  return client.connected();
}


void endroutine(void)
{
  delay(200);
  Serial.println("shut down");
  client.disconnect();
  espClient.stop();
  delay(100);
  Serial.println("shut down");
  // DONE pulse to TPL5111
  digitalWrite(DONEPIN, HIGH);
  delay(200);
  digitalWrite(DONEPIN, LOW);
  ESP.restart();      // if done signal fails
}

void init_esp_now() {
  // Initialize the ESP-NOW protocol
  esp_now_init();
  /*  //this part in enabled is in the normal ESP-NOW program, but results in false restarts 
      //when esp_now_init aparently fails, in our case we use WiFi if ESP-NOW fails.
    if (esp_now_init() != 0) {
    ESP.restart();    
    }
  */
  // *** DECLARATION OF THE ROLE OF THE ESP DEVICE IN THE COMMUNICATION *** //
  // 0 = LOOSE, 1 = MASTER, 2 = SLAVE and 3 = MASTER + SLAVE
  esp_now_set_self_role(1);   // sender

  // *** PAIRING WITH THE SLAVE *** //
  uint8_t role = 2;   // role of receiver = slave
  uint8_t channel = 1;  // WiFi channel of receiver access point
  esp_now_add_peer(mac_addr, role, channel, NULL, 0);   // NULL means there is no key, length of key is 0

  // set up the call-back function for the confirmation of the sent data. This is executed is ESP-NOW is used further on in the program
  esp_now_register_send_cb([](uint8_t *mac,  uint8_t result2) {
    char MACslave[6];
    // in this call back function the result is stored in the "result" variable, which is important in the program
    result = result2;
    sprintf(MACslave, "% 02X:% 02X:% 02X:% 02X:% 02X:% 02X", mac [0], mac [1], mac [2], mac [3], mac [4], mac [5]);

    // display result on serial if serial is initialized
    Serial.print("Data sent to ESP MAC:"); Serial.print(MACslave);
    Serial.print(". Reception(0 = 0K - 1 = ERROR):"); Serial.println(result);
  });
  Serial.println("ESP-now initialized");
}

uint8_t senddata (char * topic_data, char * payload_data)
{
  // prepare the data to send
  time1 = millis();
  char DATA[LENGTH];
  memset(DATA, '\0', LENGTH);    // Initialice or clear the string
  strcat(DATA, topic_data);   // Copy the topic to the array.
  strcat(DATA, "&");   // Copy "&" symbol to the array.
  strcat(DATA, payload_data);   // Copy payload to the array.

  uint8_t data[sizeof(DATA)];
  memcpy(data, &DATA, sizeof(DATA));
  uint8_t len = sizeof(data);

  Serial.print("DATA: "); Serial.println(DATA);
  Serial.print("data: "); Serial.println((char*) data);

  while ((result == 1) && (millis() - time1 < CALLBACKMAXTIME))
  {
    esp_now_send(mac_addr, data, len);
    delay(200);
  }
  return result;
}

void callback(char* topic, byte * payload, unsigned int length)
{
  if ((char)payload[0] == 'C') // CLOSED
  {
    message_confirmed = true; // correct message received on subscribed topic
    client.publish(debug_topic, "message confirmed");
  }
  if ((char)payload[0] == '1') // update = 1
  {
    update_state = true; // correct message received on subscribed topic
    client.publish(debug_topic, "doorsensor 1 in update mode");
  }
}
