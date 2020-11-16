//Cayenne custom Parimeters
//max save temp
//Restart Pin on TO
//Reset (SPIFFS.format(), wm.resetSettings() and ESP.restart(), PIN T2
//3 Temp Sensor DS18B20, bottom, top, ambient on one bus, bus 16
//Safety update. Geyser switch on command (Status=1) on channel 3 will only switch the relay on if both tempbottom and temptop is below save limits
//Allocate sensors in WiFi Manager to make sensor replacement easy
//Auto Mode. Keep bottom temp in certain range
//Faster readings of temp sensors____________________________________________________________
//Basic OTA
//Solving Cayenne connection problems when WiFi was interupted, but restored

#include <FS.h>          // this needs to be first, or it all crashes and burns..., Needed for custom parimeters of WiFiManager
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager, https://github.com/bblanchon/ArduinoJson/issues/1276
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson

#include <ESPmDNS.h> //Used for OTA
#include <WiFiUdp.h> //Used for OTA
#include <ArduinoOTA.h> //Used for OTA

#ifdef ESP32
#include <SPIFFS.h>
#endif

//define your default values here, if there are different values in config.json, they are overwritten.

#define CAYENNE_PRINT Serial
#include <CayenneMQTTESP32.h>

#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into GPIO16 on the Arduino
#define ONE_WIRE_BUS 16

// Setup a oneWire instance to communicate with any OneWire devices
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
float tempbottom ;
float temptop;
float tempambient;
boolean GeyserStatus;
boolean GeyserMode;
int AutoBottomTemp = 45;
float touch_restart = 66; //variable to store exp smoothed value of T0
float touch_resett = 66; //variable to store exp smoothed value of T2
int current = 0;
int oldtimestamp = 0; //need for timing of dallas temp sensors

WiFiManager wm;

char mqtt_username[40];
char mqtt_password[40]  = "MQTT Password";
char mqtt_client_id[40] = "MQTT Clint ID";
char max_save_temp_top[3] = "73";
char max_save_temp_bottom[3] = "55";
char sensorid_bottom[2] = "0";
char sensorid_top[2] = "1";
char sensorid_ambient[2] = "2";

//flag for saving data
bool shouldsaveConfig = false;

//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldsaveConfig = true; //set resolution of the ds18db
}

void readtempsensors()
{
  int newtimestamp = millis();
  if (newtimestamp > oldtimestamp + 5000) //reading dallas temp sensors every 5 seconds
  {
    tempbottom = sensors.getTempCByIndex(atoi(sensorid_bottom));
    temptop = sensors.getTempCByIndex(atoi(sensorid_top));
    tempambient = sensors.getTempCByIndex(atoi(sensorid_ambient));
    sensors.requestTemperatures(); // Send the command to measure temperature readings
    oldtimestamp = newtimestamp;
  }
}


void aoutomode()
{
  if (tempbottom < AutoBottomTemp - 1 and GeyserMode == 1 and checksavetemp() == 1)
  {
    digitalWrite(15, HIGH);//Switch Geyser on
  }
  if (tempbottom > AutoBottomTemp)
  {
    digitalWrite(15, LOW);//Switch Geyser of
  }
  GeyserStatus = digitalRead(15);
  //Serial.println("-------------------------------------------------------------");
  //Serial.println("Geyser Mode (0=Manual, 1=Auto): " + String(GeyserMode) + ", Geyser Status (0=OFF, 1=ON): " + String(GeyserStatus));
  //Serial.println("-------------------------------------------------------------");
}

boolean checksavetemp()
{
  if (tempbottom > atoi(max_save_temp_bottom) or temptop > atoi(max_save_temp_top)) //atoi() transfer char into int. atol() and atof used to convert to long and float
  {
    return 0;
  }
  else
  {
    return 1;
  }
}
void setupSpiffs() {
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonDocument json(ESP.getMaxAllocHeap());
        DeserializationError error = deserializeJson(json, buf.get());
        //JsonObject& json = jsonBuffer.parseObject();
        //json.printTo(Serial);
        serializeJson(json, Serial);
        if (!error) {
          Serial.println("\nparsed json");

          strcpy(mqtt_username, json["mqtt_username"]);
          strcpy(mqtt_password, json["mqtt_password"]);
          strcpy(mqtt_client_id, json["mqtt_client_id"]);
          strcpy(max_save_temp_bottom, json["max_save_temp_bottom"]);
          strcpy(max_save_temp_top, json["max_save_temp_top"]);
          strcpy(sensorid_bottom, json["sensorid_bottom"]);
          strcpy(sensorid_top, json["sensorid_top"]);
          strcpy(sensorid_ambient, json["sensorid_ambient"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
}

void setup() {
  pinMode(15, OUTPUT); //Set the Pin of the SSR to Output
  pinMode(LED_BUILTIN, OUTPUT); //Internal blue led of the ESP32 Wemos Lolin
  digitalWrite(15, LOW); //For safety, the Geyser should start in off mode
  GeyserStatus = LOW;
  sensors.setResolution(12); //Set resolution of DS18B20 (can be 9, 10, 11, 12)
  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println();

  setupSpiffs();

  wm.setConfigPortalTimeout(90); //Make it 180 in production mode

  //set config save notify callback
  wm.setSaveConfigCallback(saveConfigCallback);

  // setup custom parameters
  //
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_username("mqtt_username", "MQTT Username", mqtt_username, 40);
  WiFiManagerParameter custom_mqtt_password("mqtt_password", "MQTT Password", mqtt_password, 40);
  WiFiManagerParameter custom_mqtt_client_id("mqtt_client_id", "MQTT Client ID", mqtt_client_id, 40);
  WiFiManagerParameter custom_max_save_temp_bottom("max_save_temp_bottom", "MAX SAVE TEMPERATURE IN BOTTOM", max_save_temp_bottom, 3);
  WiFiManagerParameter custom_max_save_temp_top("max_save_temp_top", "MAX SAVE TEMPERATURE IN OUTLET PIPE", max_save_temp_top, 3);
  WiFiManagerParameter custom_sensorid_bottom("sensorid_bottom", "BOTTOM SENSOR ID", sensorid_bottom, 2);
  WiFiManagerParameter custom_sensorid_top("sensorid_top", "TOP (Hot Pipe) SENSOR ID", sensorid_top, 2);
  WiFiManagerParameter custom_sensorid_ambient("sensorid_ambient", "AMBIENT SENSOR ID", sensorid_ambient, 2);

  //add all your parameters here
  wm.addParameter(&custom_mqtt_username);
  wm.addParameter(&custom_mqtt_password);
  wm.addParameter(&custom_mqtt_client_id);
  wm.addParameter(&custom_max_save_temp_bottom);
  wm.addParameter(&custom_max_save_temp_top);
  wm.addParameter(&custom_sensorid_bottom);
  wm.addParameter(&custom_sensorid_top);
  wm.addParameter(&custom_sensorid_ambient);

  //reset settings - wipe credentials for testing
  //wm.resetSettings();

  //automatically connect using saved credentials if they exist
  //If connection fails it starts an access point with the specified name
  //here  "AutoConnectAP" if empty will auto generate basedcon chipid, if password is blank it will be anonymous
  //and goes into a blocking loop awaiting configuration
  if (!wm.autoConnect("Slim_Geyser_DEV", "password")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    // if we still have not connected restart and try all over again
    ESP.restart();
    delay(5000);
  }
  //wm.startWebPortal(); // to access the web portal after the ESP connected to WiFi. This does not work, probably becuase of delay in Cayenne loop


  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  //Flicker Blue led 3 times
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);

  //read updated parameters
  strcpy(mqtt_username, custom_mqtt_username.getValue());
  strcpy(mqtt_password, custom_mqtt_password.getValue());
  strcpy(mqtt_client_id, custom_mqtt_client_id.getValue());
  strcpy(max_save_temp_bottom, custom_max_save_temp_bottom.getValue());
  strcpy(max_save_temp_top, custom_max_save_temp_top.getValue());
  strcpy(sensorid_bottom, custom_sensorid_bottom.getValue());
  strcpy(sensorid_top, custom_sensorid_top.getValue());
  strcpy(sensorid_ambient, custom_sensorid_ambient.getValue());

  //save the custom parameters to FS
  if (shouldsaveConfig) {
    Serial.println("saving config");
    DynamicJsonDocument json(ESP.getMaxAllocHeap());
    //JsonObject& json = jsonBuffer.createObject();
    json["mqtt_username"] = mqtt_username;
    json["mqtt_password"]   = mqtt_password;
    json["mqtt_client_id"]   = mqtt_client_id;
    json["max_save_temp_bottom"]   = max_save_temp_bottom;
    json["max_save_temp_top"]   = max_save_temp_top;

    json["sensorid_bottom"]   = sensorid_bottom;
    json["sensorid_top"]   = sensorid_top;
    json["sensorid_ambient"]   = sensorid_ambient;

    json["ip"]          = WiFi.localIP().toString();
    json["gateway"]     = WiFi.gatewayIP().toString();
    json["subnet"]      = WiFi.subnetMask().toString();

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }
    serializeJsonPretty(json, Serial);
    //json.prettyPrintTo(Serial);
    serializeJson(json, configFile);
    //json.printTo(configFile);
    configFile.close();
    //end save
    shouldsaveConfig = false;
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.gatewayIP());
  Serial.println(WiFi.subnetMask());
  Serial.println("-------------------------------");
  Serial.print("MQTT Username: ");
  Serial.println(mqtt_username);
  Serial.print("MQTT Password: ");
  Serial.println(mqtt_password);
  Serial.print("MQTT Client ID: ");
  Serial.println(mqtt_client_id);
  Serial.print("Max Save Temp for Geyser Bottom: ");
  Serial.println(max_save_temp_bottom);
  Serial.print("Max Save Temp for Geyser Top: ");
  Serial.println(max_save_temp_top);

  Serial.print("Bottom Sensor ID: ");
  Serial.println(sensorid_bottom);
  Serial.print("Top Sensor ID: ");
  Serial.println(sensorid_top);
  Serial.print("Ambient Sensor ID: ");
  Serial.println(sensorid_ambient);


  Serial.println("\n----------------------------------------------------------\n");
  Serial.println("LED_BUITIN: 10 pulses=connected to WiFi and Caywnne sever");
  Serial.println("LED_BUITIN: 3 pulses=Received Geyser ON/OFF status changed from Cayenne server");
  Serial.println("LED_BUITIN: 1 pulses=Send Temperature and Geyser ON/OFF Status to Cayenne server (+/- every 15 seconds");
  Serial.println("\n----------------------------------------------------------\n");
  Serial.println("Conection with your WiFi router and Cayenne server will now be tried.");

  Cayenne.begin(mqtt_username, mqtt_password, mqtt_client_id,WiFi.SSID().c_str(), WiFi.psk().c_str());
  Serial.println("It seems like you are connected to Cayenne.");
  Serial.println("\n----------------------------------------------------------\n");
  sensors.begin(); //start the  DallasTemperature sensor
  delay(1000);
  sensors.requestTemperatures(); // Send the command to measure temperature readings.
  delay(1000);

  //OTA Stuff
    ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("OTA Ready");
}


void loop() {
  //wm.process();//to access the web portal after the ESP connected to WiFi. This does not work, probably becuase of delay in Cayenne loop
  //Restart if T0 is touched
  touch_restart = 0.03 * touchRead(T0) + 0.97 * touch_restart;
  if (touch_restart < 55)
  {
    Serial.println("tochread TO: " + String(touch_restart) + ".........ESP32 will restart");
    delay(5000);
    ESP.restart();
    delay(5000);
  }

  touch_resett = 0.02 * touchRead(T2) + 0.98 * touch_resett;
  if (touch_resett < 55)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    Serial.println("tochread T2: " + String(touch_resett) + ".........ESP32 will reset");
    SPIFFS.format();
    wm.resetSettings();
    ESP.restart();
    delay(5000);
  }
  readtempsensors();

  if (checksavetemp() == 0)
  {
    digitalWrite(15, LOW);
  }
  GeyserStatus = digitalRead(15);
  if (GeyserMode == 1)
  {
    aoutomode();
  }
  Cayenne.loop();
  ArduinoOTA.handle();
}


// Default function for sending sensor data at intervals to Cayenne.
// You can also use functions for specific channels, e.g CAYENNE_OUT(1) for sending channel 1 data.
CAYENNE_OUT_DEFAULT()
{
  // Write data to Cayenne here. This example just sends the current uptime in milliseconds on virtual channel 0.
  digitalWrite(LED_BUILTIN, LOW); //Switch internal blue LED ON to indicate DATA will be send to Cayenne server.
  Cayenne.virtualWrite(0, tempbottom); //send bottom temperature to caynne
  Cayenne.virtualWrite(1, temptop); //send bottom temperature to caynne
  Cayenne.virtualWrite(2, tempambient); //send bottom temperature to caynne
  Cayenne.virtualWrite(3, GeyserStatus); //Send Geyser Status
  Cayenne.virtualWrite(4, GeyserStatus); //Send Geyser Status
  Cayenne.virtualWrite(5, max_save_temp_bottom); //Send Max save temp set during AP mode (Adhoc mode)
  Cayenne.virtualWrite(6, max_save_temp_top); //Send Max save temp set during AP mode (Adhoc mode)
  Cayenne.virtualWrite(7, GeyserMode); //0=M anual, 1 Auto Mode
  Cayenne.virtualWrite(8, GeyserMode); //0=M anual, 1 Auto Mode
  Cayenne.virtualWrite(9, AutoBottomTemp); //Auto Mode, bottom temp

  Serial.println("Channel 0: Bottom Temperature send to Caynne Server: " + String(tempbottom));
  Serial.println("Channel 1: Top Temperature send to Caynne Server: " + String(temptop));
  Serial.println("Channel 2: Ambient Temperature send to Caynne Server: " + String(tempambient));
  Serial.println("Channel 3: Geyser Status send to Caynne Server: Status is " + String(GeyserStatus) + " (1=ON, 0=OFF)");
  Serial.println("Channel 5: Max save bottom temperature set during AP Mode is still: :" + String(max_save_temp_bottom));
  Serial.println("Channel 6: Max save Top temperature set during AP Mode is still: :" + String(max_save_temp_top));
  Serial.println("Channel 7: Geyser Mode (1=Auto Mode active, 0=Auto Mode de-active): " + String(GeyserMode));
  Serial.println("Channel 9: Auto Mode, Bottom Temp Target: " + String(AutoBottomTemp));
  Serial.println("-----------------------------------------------------------------------------------------------------------");
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH); //Switch internal blue LED OFF to indicate DATA was send to Cayenne server.
  // Some examples of other functions you can use to send data.
  //Cayenne.celsiusWrite(1, 22.0);
  //Cayenne.luxWrite(2, 700);
  //Cayenne.virtualWrite(3, 50, TYPE_PROXIMITY, UNIT_CENTIMETER);
}

// Default function for processing actuator commands from the Cayenne Dashboard.
// You can also use functions for specific channels, e.g CAYENNE_IN(1) for channel 1 commands.
CAYENNE_IN_DEFAULT()
{
  CAYENNE_LOG("Channel %u, value %s", request.channel, getValue.asString());

  //Process message here. If there is an error set an error message using getValue.setError(), e.g getValue.setError("Error message");
}
CAYENNE_IN(3)
{
  digitalWrite(LED_BUILTIN, LOW); //Switch internal blue LED ON to indicate waiting for CAYNNE Server.
  boolean currentServerStatus = getValue.asInt(); //Get the current status of the geyser according to the cloud
  Serial.println("Data received from Cayenne Server. Updated (Current) status of Geyser on server: " + String(currentServerStatus) + " (1=ON, 0=OFF)");
  if (currentServerStatus == HIGH and checksavetemp() == 1)
  {
    digitalWrite(15, HIGH);
  }
  else
  {
    digitalWrite(15, LOW);
  }
  GeyserStatus = digitalRead(15);
  digitalWrite(LED_BUILTIN, HIGH); //Pulses to indicate data was received from Cayenne
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  GeyserMode = 0; //Manual
  Cayenne.virtualWrite(7, GeyserMode);
  Cayenne.virtualWrite(4, GeyserStatus); //Send Geyser Status
  Cayenne.virtualWrite(8, GeyserMode); //Send Geyser Mode=0
}
CAYENNE_IN(7)
{
  GeyserMode = getValue.asInt();
  if (GeyserMode == 1)
  {
    aoutomode();
  }
  Cayenne.virtualWrite(8, GeyserMode);
  Cayenne.virtualWrite(4, GeyserStatus);
  Serial.println("Data received from Cayenne Server. Updated (Current) Mode of Geyser on server: " + String(GeyserMode) + " (1=Auto Mode Activated, 0=Auto Mode-de activated)");
  digitalWrite(LED_BUILTIN, HIGH); //Pulses to indicate data was received from Cayenne
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
}
CAYENNE_IN(9)
{
  AutoBottomTemp = getValue.asInt();
  if (AutoBottomTemp > atoi(max_save_temp_bottom))
  {
    AutoBottomTemp = atoi(max_save_temp_bottom);
    Cayenne.virtualWrite(9, AutoBottomTemp);
  }
}
CAYENNE_DISCONNECTED()//This is to avoid an invinite loop in the Cayenne.loop()
{
    delay(10000); //This is important for the reset (SPIFS format, WiFiManager reset
    ESP.restart();
    delay(5000);
}
