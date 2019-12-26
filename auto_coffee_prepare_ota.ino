/* *****************************************************************
 *
 * Download latest Blinker library here:
 * https://github.com/blinker-iot/blinker-library/archive/master.zip
 * 
 * 
 * Blinker is a cross-hardware, cross-platform solution for the IoT. 
 * It provides APP, device and server support, 
 * and uses public cloud services for data transmission and storage.
 * It can be used in smart home, data monitoring and other fields 
 * to help users build Internet of Things projects better and faster.
 * 
 * Make sure installed 2.5.0 or later ESP8266/Arduino package,
 * if use ESP8266 with Blinker.
 * https://github.com/esp8266/Arduino/releases
 * 
 * Make sure installed 1.0.2 or later ESP32/Arduino package,
 * if use ESP32 with Blinker.
 * https://github.com/espressif/arduino-esp32/releases
 * 
 * Docs: https://doc.blinker.app/
 *       https://github.com/blinker-iot/blinker-doc/wiki
 * 
 * *****************************************************************
 * 
 * Blinker 库下载地址:
 * https://github.com/blinker-iot/blinker-library/archive/master.zip
 * 
 * Blinker 是一套跨硬件、跨平台的物联网解决方案，提供APP端、设备端、
 * 服务器端支持，使用公有云服务进行数据传输存储。可用于智能家居、
 * 数据监测等领域，可以帮助用户更好更快地搭建物联网项目。
 * 
 * 如果使用 ESP8266 接入 Blinker,
 * 请确保安装了 2.5.0 或更新的 ESP8266/Arduino 支持包。
 * https://github.com/esp8266/Arduino/releases
 * 
 * 如果使用 ESP32 接入 Blinker,
 * 请确保安装了 1.0.2 或更新的 ESP32/Arduino 支持包。
 * https://github.com/espressif/arduino-esp32/releases
 * 
 * 文档: https://doc.blinker.app/
 *       https://github.com/blinker-iot/blinker-doc/wiki
 *  
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * PIN out:
 * D0 --- power signal led
 * D1 --- relay1 for power of coffie machine
 * D2 --- relay2 for grinder power
 * 
 * *****************************************************************/

#define BLINKER_WIFI
#define BLINKER_MIOT_MULTI_OUTLET
//#define BLINKER_ESP_SMARTCONFIG //自动配网

#define MAIN_POWER_PIN D0
#define COFFEE_MACHINE_PIN D1
#define GRINDER_PIN D2

// default delay time(*0.1 second) for grinder
#define DELAY_1PERSON 65
#define DELAY_2PERSON 100
// default delay time(minutes) for auto power off
#define DELAY_POWER_OFF 20

#include <ArduinoOTA.h>
#include <Blinker.h>

bool oState[5] = {false};

char auth[] = "";
char ssid[] = "";
char pswd[] = "";

//{grinder delay1,grinder delay2, auto power off delay}
int Delay_values[3]= {0,0,0}; 

const int GPIO_POWER_PIN = 14; //D5

#define BUTTON_POWER "btn-power"
#define BUTTON_1PERSON "btn-1"
#define BUTTON_2PERSON "btn-2"
#define SLIDER_1PERSON "ran-1p"
#define SLIDER_2PERSON "ran-2p"
#define SLIDER_AUTO_OFF "ran-auto-off"
BlinkerButton btn_power(BUTTON_POWER);
BlinkerButton btn_1(BUTTON_1PERSON);
BlinkerButton btn_2(BUTTON_2PERSON);
BlinkerSlider sldr_1p(SLIDER_1PERSON);
BlinkerSlider sldr_2p(SLIDER_2PERSON);
BlinkerSlider sldr_auto_off(SLIDER_AUTO_OFF);

// Timer: Auxiliary variables
unsigned long now = millis();
//for coffee machine trigger
unsigned long lastTrigger0 = 0;
boolean startTimer0 = false;
//for grinder trigger 1
unsigned long lastTrigger1 = 0;
boolean startTimer1 = false;
//for grinder trigger 2
unsigned long lastTrigger2 = 0;
boolean startTimer2 = false;
//for auto power off 
//unsigned long lastTriggerPower = 0;
//boolean startTimerPower = false; will use power pin status instead


// call this in loop
void check_delay()
{  
    // Current time
    now = millis();
    // Turn off the LED after the number of seconds defined in the timeSeconds variable
    
    if(startTimer0 && (now - lastTrigger0 > (300))) {
      BLINKER_LOG("after delay for btn_power");
      digitalWrite(COFFEE_MACHINE_PIN, HIGH);
      startTimer0 = false;
    }
    if(startTimer1 && (now - lastTrigger1 > (Delay_values[0]*100))) {
      BLINKER_LOG("after delay for btn_1");
      digitalWrite(GRINDER_PIN, HIGH);
      startTimer1 = false;
    }
    if(startTimer2 && (now - lastTrigger2 > (Delay_values[1]*100))) {
      BLINKER_LOG("after delay for btn_2");
      digitalWrite(GRINDER_PIN, HIGH);
      startTimer2 = false;
    }
    // check auto power off
    if(digitalRead(MAIN_POWER_PIN) && (now - lastTrigger0 > (Delay_values[2]*60000))) {
      BLINKER_LOG("time up, auto power off!");
      btn_power_callback(BLINKER_CMD_OFF);
    }
  
}


void btn_power_callback(const String &state)
{
  if(state == "on" || state == "off" ){
    
    bool b_state = state == BLINKER_CMD_ON;
    startTimer0 = true;
    lastTrigger0 = millis();
    digitalWrite(MAIN_POWER_PIN, b_state);
    digitalWrite(COFFEE_MACHINE_PIN, LOW);
    BLINKER_LOG("get btn_power delayed...: ", state);
    btn_power.color(b_state?"#00FF00":"#CCCCCC");
    btn_power.print(state);
    
  }
}

void btn_1_callback(const String &state)
{
    bool b_state = state == BLINKER_CMD_ON;
    startTimer1 = true;
    lastTrigger1 = millis();
    digitalWrite(GRINDER_PIN, LOW);
    BLINKER_LOG("get btn_1 delayed...: ", state);
    btn_1.print(state);
}
void btn_2_callback(const String &state)
{
    bool b_state = state == BLINKER_CMD_ON;
    startTimer2 = true;
    lastTrigger2 = millis();
    digitalWrite(GRINDER_PIN, LOW);
    BLINKER_LOG("get btn_2 delayed...: ", state);
    btn_2.print(state);
}
void sldr_1p_callback(int32_t value)
{
    BLINKER_LOG("get sldr_1p value: ", value);
    Delay_values[0] = value;
    EEPwrite(); 
}
void sldr_2p_callback(int32_t value)
{
    BLINKER_LOG("get sldr_2p value: ", value);
    Delay_values[1] = value;
    EEPwrite(); 
}
void sldr_auto_off_callback(int32_t value)
{
    BLINKER_LOG("get sldr_auto_off value: ", value);
    Delay_values[2] = value;
    EEPwrite(); 
}

void miotPowerState(const String &state, uint8_t num)
{
    BLINKER_LOG("will set outlet: ", num, ", power state: ", state);
    bool b_state = state == BLINKER_CMD_ON;
    if (num == 0 && oState[num] != b_state)
    {   
        btn_power_callback(state);
    }
    if(num == 1){
        btn_1_callback(state);
    }
    if(num == 2){
        btn_2_callback(state);
    }

    BlinkerMIOT.powerState(state == BLINKER_CMD_ON? "on":"off", num);
    oState[num] = b_state;
    BLINKER_LOG("MIOT outlet: ", num, ", state: ", state);
    BlinkerMIOT.print();
 
}

void miotQuery(int32_t queryCode, uint8_t num)
{
    BLINKER_LOG("MIOT Query outlet: ", num, ", codes: ", queryCode);
    BlinkerMIOT.powerState(oState[num] ? "on" : "off", num);
    BlinkerMIOT.print();
}

void dataRead(const String &data)
{
    BLINKER_LOG("Blinker readString: ", data);
    EEPwrite();
    Blinker.vibrate();

    uint32_t BlinkerTime = millis();
    Blinker.print("millis", BlinkerTime);
    
    sldr_1p.print(Delay_values[0]);
    sldr_2p.print(Delay_values[1]);
    sldr_auto_off.print(Delay_values[2]);
}

void otaConfig()
{
  //=====================OTA start====================================
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //=====================OTA end====================================
}

void setup()
{
    Serial.begin(115200);
    BLINKER_DEBUG.stream(Serial);

    pinMode(MAIN_POWER_PIN, OUTPUT);
    digitalWrite(MAIN_POWER_PIN, LOW);
    pinMode(COFFEE_MACHINE_PIN, OUTPUT);
    digitalWrite(COFFEE_MACHINE_PIN, HIGH);
    pinMode(GRINDER_PIN, OUTPUT);
    digitalWrite(GRINDER_PIN, HIGH);

//    Blinker.begin(auth);
    Blinker.begin(auth, ssid, pswd);
    Blinker.attachData(dataRead);
    
    Blinker.attachHeartbeat(heartbeat); //心跳包
    
    BlinkerMIOT.attachPowerState(miotPowerState);
    BlinkerMIOT.attachQuery(miotQuery);

    btn_power.attach(btn_power_callback);
    btn_1.attach(btn_1_callback);
    btn_2.attach(btn_2_callback);
    btn_power.color("#CCCCCC");
    
    sldr_1p.attach(sldr_1p_callback);
    sldr_2p.attach(sldr_2p_callback);
    sldr_auto_off.attach(sldr_auto_off_callback);

    btn_power.print("off");
    btn_1.print("off");
    btn_2.print("off");
    BlinkerMIOT.powerState("off", 0);
    BlinkerMIOT.powerState("off", 1);
    BlinkerMIOT.powerState("off", 2);
    BlinkerMIOT.print();
    
    otaConfig();
    
    EEPread();
    
    pinMode(GPIO_POWER_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(GPIO_POWER_PIN), detectsPowerBtnClick, FALLING);
}

void loop()
{
    Blinker.run();
    check_delay();
    ArduinoOTA.handle();
}
  
void heartbeat() //心跳包
{
    BLINKER_LOG("heartbeating...");
    sldr_1p.print(Delay_values[0]);
    sldr_2p.print(Delay_values[1]);
    sldr_auto_off.print(Delay_values[2]);
    btn_power.print(digitalRead(MAIN_POWER_PIN)? "on":"off");
}
  
void EEPread(){
  EEPROM.begin(4096);
  Delay_values[0] = EEPROM.read(3001);
  Delay_values[1] = EEPROM.read(3002);
  Delay_values[2] = EEPROM.read(3003);
  if(Delay_values[0]==0 || Delay_values[0]==255){
    Delay_values[0] = DELAY_1PERSON;
    BLINKER_LOG("called EEPread() , get Delay_values[0](DELAY_1PERSON) from default value:",DELAY_1PERSON);
  }
  if(Delay_values[1]==0 || Delay_values[1]==255){
    Delay_values[1] = DELAY_2PERSON;
    BLINKER_LOG("called EEPread() , get Delay_values[1](DELAY_2PERSON) from default value:",DELAY_2PERSON);
  }

  if(Delay_values[2]==0 || Delay_values[2]==255){
    Delay_values[2] = DELAY_POWER_OFF;
    BLINKER_LOG("called EEPread() , get Delay_values[2](DELAY_POWER_OFF) from default value:",DELAY_2PERSON);
  }
  BLINKER_LOG("called EEPread() , get Delay_values[0]:", Delay_values[0], 
  "  Delay_values[1]:", Delay_values[1], "  Delay_values[2]:", Delay_values[2]);
}
 
void EEPwrite(){
  EEPROM.begin(4096);
  EEPROM.write(3001,Delay_values[0]); 
  EEPROM.write(3002,Delay_values[1]); 
  EEPROM.write(3003,Delay_values[2]); 
  EEPROM.commit();

  BLINKER_LOG("called EEPwrite() , set Delay_values[0]:", Delay_values[0], 
              "  Delay_values[1]:", Delay_values[1],"  Delay_values[2]:", Delay_values[2]);
}


volatile unsigned long power_btn_trigger_time = 0;
ICACHE_RAM_ATTR void detectsPowerBtnClick() {
    //BLINKER_LOG("detectsPowerBtnClick(), COFFEE_MACHINE_PIN:",digitalRead(COFFEE_MACHINE_PIN));
  // read status of COFFEE_MACHINE_PIN to avoid conflict
  if (millis() - power_btn_trigger_time > 200 && digitalRead(COFFEE_MACHINE_PIN))
    {
    bool power_state = digitalRead(MAIN_POWER_PIN);
    BLINKER_LOG("power button clicked! toggle current status ",power_state," to ",!power_state);
    lastTrigger0 = power_btn_trigger_time =millis();
    digitalWrite(MAIN_POWER_PIN, !power_state);
    
    //btn_power.color(power_state? "#CCCCCC":"#00FF00");
    //btn_power.print(power_state? "off":"on");
    }
}
