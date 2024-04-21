/*
V0- Image
V2- Min
V3- Max
V4- Timeinput
V5- date/time
V6- Irms0
V7- String
V8- Mode
V9- BTN B1
*/
#define BLYNK_TEMPLATE_ID "TMPL60IlwNf0R"
#define BLYNK_TEMPLATE_NAME "P APhat"
#define BLYNK_AUTH_TOKEN "nVBF4Z-zCp_D17bgg-cqqGUMM7AguN5o"

#define BLYNK_PRINT Serial
#define BLYNK_FIRMWARE_VERSION "240422"
#define APP_DEBUG

const char* ssid = "Vu Dat";
const char* password = "87888788";
//const char* ssid = "tram bom so 4";
//const char* password = "0943950555";

#include <BlynkSimpleEsp8266.h>
#include <ESP8266WiFi.h>
#include <Wire.h>

#include "EmonLib.h"
EnergyMonitor emon0, emon1;

#include <WidgetRTC.h>
#include "RTClib.h"
RTC_DS3231 rtc_module;

#include <Eeprom24C32_64.h>
#define EEPROM_ADDRESS 0x57
static Eeprom24C32_64 eeprom(EEPROM_ADDRESS);
const word address = 0;
//-----------------------------
#include "PCF8575.h"
PCF8575 pcf8575_1(0x20);
//-----------------------------
#include <ESP8266httpUpdate.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
WiFiClient client;
HTTPClient http;
#define URL_fw_Bin "https://raw.githubusercontent.com/quangtran3110/IOT-Personal/main/A.Phat/build/esp8266.esp8266.nodemcuv2/A.Phat.ino.bin"

const int S0 = 14;
const int S1 = 12;
const int S2 = 13;
const int S3 = 15;
const int RL1 = P7;
const int RL2 = P6;
const int RL3 = P5;
const int RL4 = P4;
const int RL5 = P3;
const int RL6 = P2;
const int RL7 = P1;
const int RL8 = P0;

char daysOfTheWeek[7][12] = { "CN", "T2", "T3", "T4", "T5", "T6", "T7" };
char tz[] = "Asia/Ho_Chi_Minh";

byte z;
byte MonWeekDay, TuesWeekDay, WedWeekDay, ThuWeekDay, FriWeekDay, SatWeekend, SunWeekend;
long startsecondswd;  // weekday start time in seconds
long stopsecondswd;   // weekday stop  time in seconds
long nowseconds;      // time now in seconds
bool blynk_first_connect = false;
float Irms0;
bool trip0 = false, keySet = false;
unsigned long int yIrms0 = 0;
int xSetAmpe = 0;
int status_RL1 = LOW;
int start_h, stop_h, start_m, stop_m;

struct Data {
  byte mode;
  float SetAmpemax, SetAmpemin;
  int save_num;
} data,
  dataCheck;
const struct Data dataDefault = { 0, 0, 0, 0 };

struct Data_ {
  byte MonWeekDay, TuesWeekDay, WedWeekDay, ThuWeekDay, FriWeekDay, SatWeekend, SunWeekend;
} data_,
  dataCheck_;
struct Data_ dataDefault_ = { 0, 0, 0, 0, 0, 0, 0 };

WidgetTerminal terminal(V7);
WidgetRTC rtc_widget;
BlynkTimer timer;
BLYNK_CONNECTED() {
  rtc_widget.begin();
  blynk_first_connect = true;
  terminal.clear();
  
}

void savedata() {
  if (memcmp(&data, &dataCheck, sizeof(dataDefault)) == 0) {
    // Serial.println("structures same no need to write to EEPROM");
  } else {
    // Serial.println("\nWrite bytes to EEPROM memory...");
    data.save_num = data.save_num + 1;
    eeprom.writeBytes(address, sizeof(dataDefault), (byte*)&data);
  }
}
void on_RL1() {
  status_RL1 = HIGH;
  pcf8575_1.digitalWrite(RL1, !status_RL1);
  //Blynk.virtualWrite(V0, 1);
  Blynk.virtualWrite(V9, status_RL1);
}
void off_RL1() {
  status_RL1 = LOW;
  pcf8575_1.digitalWrite(RL1, !status_RL1);
  //Blynk.virtualWrite(V0, 2);
  Blynk.virtualWrite(V9, status_RL1);
}
void rst_module() {
  pcf8575_1.digitalWrite(RL6, LOW);
}
void readcurrent()  // C2
{
  digitalWrite(S0, LOW);
  digitalWrite(S1, HIGH);
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  float rms0 = emon0.calcIrms(2960);
  if (rms0 < 3) {
    Irms0 = 0;
    yIrms0 = 0;
  } else if (rms0 > 3) {
    Irms0 = rms0;
    yIrms0 = yIrms0 + 1;
    if ((yIrms0 > 2) && ((Irms0 > data.SetAmpemax) || (Irms0 < data.SetAmpemin))) {
      xSetAmpe = xSetAmpe + 1;
      if (xSetAmpe >= 3) {
        Blynk.logEvent("error", String("Động cơ lỗi: ") + Irms0 + String(" A"));
        off_RL1();
        trip0 = true;
        xSetAmpe = 0;
      }
    } else {
      xSetAmpe = 0;
    }
  }
  Blynk.virtualWrite(V6, Irms0);
}
BLYNK_WRITE(V2)  // min
{
  if (keySet) {
    data.SetAmpemin = param.asFloat();
    savedata();
  } else {
    Blynk.virtualWrite(V2, data.SetAmpemin);
  }
}
BLYNK_WRITE(V3)  // max
{
  if (keySet) {
    data.SetAmpemax = param.asFloat();
    savedata();
  } else {
    Blynk.virtualWrite(V3, data.SetAmpemax);
  }
}
BLYNK_WRITE(V4)  // Time input
{
  DateTime now = rtc_module.now();
  if (blynk_first_connect == true) {
    if ((now.day() != day()) || (now.hour() != hour()) || ((now.minute() - minute() > 2) || (minute() - now.minute() > 2))) {
      rtc_module.adjust(DateTime(year(), month(), day(), hour(), minute(), second()));
      DateTime now = rtc_module.now();
    }
  }
  if (!trip0) {
    Blynk.virtualWrite(V5, daysOfTheWeek[now.dayOfTheWeek()], ", ", now.day(), "/", now.month(), "/", now.year(), "-", now.hour(), ":", now.minute(), ":", now.second());
  } else Blynk.virtualWrite(V5, "Lỗi động cơ!!!");
  TimeInputParam t(param);
  start_h = t.getStartHour();
  start_m = t.getStartMinute();
  stop_h = t.getStopHour();
  stop_m = t.getStopMinute();
  startsecondswd = (t.getStartHour() * 3600) + (t.getStartMinute() * 60);
  stopsecondswd = (t.getStopHour() * 3600) + (t.getStopMinute() * 60);
  int dayadjustment = -1;
  if (weekday() == 1) {
    dayadjustment = 6;  // needed for Sunday, Time library is day 1 and Blynk is day 7
  }
  if (data.mode == 1) {
    data_.MonWeekDay = t.isWeekdaySelected(1);
    data_.TuesWeekDay = t.isWeekdaySelected(2);
    data_.WedWeekDay = t.isWeekdaySelected(3);
    data_.ThuWeekDay = t.isWeekdaySelected(4);
    data_.FriWeekDay = t.isWeekdaySelected(5);
    data_.SatWeekend = t.isWeekdaySelected(6);
    data_.SunWeekend = t.isWeekdaySelected(7);
    if (memcmp(&data_, &dataCheck_, sizeof(dataDefault_)) != 0) {
      Blynk.syncVirtual(V8);
      dataCheck_.MonWeekDay = data_.MonWeekDay;
      dataCheck_.TuesWeekDay = data_.TuesWeekDay;
      dataCheck_.WedWeekDay = data_.WedWeekDay;
      dataCheck_.ThuWeekDay = data_.ThuWeekDay;
      dataCheck_.FriWeekDay = data_.FriWeekDay;
      dataCheck_.SatWeekend = data_.SatWeekend;
      dataCheck_.SunWeekend = data_.SunWeekend;
      //Serial.println(dataCheck_.SunWeekend);
    }
    if (t.isWeekdaySelected(weekday() + dayadjustment)) {  //Time library starts week on Sunday, Blynk on Monday
      nowseconds = ((hour() * 3600) + (minute() * 60) + second());
      if (startsecondswd > stopsecondswd) {
        if ((nowseconds < stopsecondswd) || (nowseconds > startsecondswd)) {  //run
          if ((Irms0 == 0) && (!trip0)) on_RL1();
        }
        if ((nowseconds > stopsecondswd) && (nowseconds < startsecondswd)) {  //Stop
          if (Irms0 != 0) off_RL1();
        }
      } else if (startsecondswd < stopsecondswd) {
        if ((nowseconds < stopsecondswd) && (nowseconds > startsecondswd)) {  //run
          if ((Irms0 == 0) && (!trip0)) on_RL1();
        }
        if ((nowseconds > stopsecondswd) || (nowseconds < startsecondswd)) {  //Stop
          if (Irms0 != 0) off_RL1();
        }
      }
    } else {
      if (Irms0 != 0) off_RL1();
    }
  }
}
BLYNK_WRITE(V7) {  // String
  String dataS = param.asStr();
  if (dataS == "reset") {
    terminal.clear();
    trip0 = false;
    Blynk.virtualWrite(V7, "Đã RESET lỗi động cơ!\n");
  } else if (dataS == "caidat") {
    terminal.clear();
    keySet = true;
    Blynk.virtualWrite(V7, "ĐÃ MỞ KHÓA CÀI ĐẶT...\n");
  } else if (dataS == "ok") {
    terminal.clear();
    keySet = false;
    Blynk.virtualWrite(V7, "ĐÃ KHÓA CÀI ĐẶT!\n");
  } else if (dataS == "update") {
    terminal.clear();
    Blynk.virtualWrite(V7, "UPDATE FIRMWARE...");
    update_fw();
  } else if (dataS == "save_num") {
    terminal.clear();
    Blynk.virtualWrite(V7, "Số lần ghi EEPROM: ", data.save_num);
  } else if (dataS == "rst") {
    terminal.clear();
    Blynk.virtualWrite(V7, "ESP Khởi động lại sau 3s");
    delay(2500);
    rst_module();
  } else {
    Blynk.virtualWrite(V7, "Mã không hợp lệ!\nVui lòng nhập lại.\n");
  }
}
BLYNK_WRITE(V8) {  // Mode
  if (param.asInt() == 0) {
    data.mode = 1;
    terminal.clear();
    Blynk.virtualWrite(V7, "Chế độ: Tự động\nChạy bơm lúc: ", start_h, ":", start_m, "->", stop_h, ":", stop_m, "\nVào các ngày: ");
    if (data_.MonWeekDay && data_.TuesWeekDay && data_.WedWeekDay && data_.ThuWeekDay && data_.FriWeekDay && data_.SatWeekend && data_.SunWeekend) {
      Blynk.virtualWrite(V7, "Cả tuần");
    } else {
      if (data_.MonWeekDay) {
        Blynk.virtualWrite(V7, "T2 ");
      }
      if (data_.TuesWeekDay) {
        Blynk.virtualWrite(V7, "T3 ");
      }
      if (data_.WedWeekDay) {
        Blynk.virtualWrite(V7, "T4 ");
      }
      if (data_.ThuWeekDay) {
        Blynk.virtualWrite(V7, "T5 ");
      }
      if (data_.FriWeekDay) {
        Blynk.virtualWrite(V7, "T6 ");
      }
      if (data_.SatWeekend) {
        Blynk.virtualWrite(V7, "T7 ");
      }
      if (data_.SunWeekend) {
        Blynk.virtualWrite(V7, "CN ");
      }
    }
  } else if (param.asInt() == 1) {
    data.mode = 2;
    terminal.clear();
    Blynk.virtualWrite(V7, "Chế độ: Thủ công");
  }
  savedata();
}
BLYNK_WRITE(V9) {  // BTN
  if (data.mode == 2) {
    if (param.asInt() == 1) {
      if (!trip0) on_RL1();
    } else off_RL1();
  } else Blynk.virtualWrite(V9, status_RL1);
}
//-------------------------------------------------------------------
void update_started() {
  Serial.println("CALLBACK:  HTTP update process started");
}
void update_finished() {
  Serial.println("CALLBACK:  HTTP update process finished");
}
void update_progress(int cur, int total) {
  Serial.printf("CALLBACK:  HTTP update process at %d of %d bytes...\n", cur, total);
}
void update_error(int err) {
  Serial.printf("CALLBACK:  HTTP update fatal error code %d\n", err);
}
//-------------------------
void update_fw() {
  WiFiClientSecure client_;
  client_.setInsecure();
  Serial.print("Wait...");
  ESPhttpUpdate.onStart(update_started);
  ESPhttpUpdate.onEnd(update_finished);
  ESPhttpUpdate.onProgress(update_progress);
  ESPhttpUpdate.onError(update_error);
  t_httpUpdate_return ret = ESPhttpUpdate.update(client_, URL_fw_Bin);
  switch (ret) {
    case HTTP_UPDATE_FAILED: Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str()); break;
    case HTTP_UPDATE_NO_UPDATES: Serial.println("HTTP_UPDATE_NO_UPDATES"); break;
    case HTTP_UPDATE_OK: Serial.println("HTTP_UPDATE_OK"); break;
  }
}
//-------------------------

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pcf8575_1.begin();
  pcf8575_1.pinMode(RL1, OUTPUT);
  pcf8575_1.digitalWrite(RL1, HIGH);
  pcf8575_1.pinMode(RL2, OUTPUT);
  pcf8575_1.digitalWrite(RL2, HIGH);
  pcf8575_1.pinMode(RL6, OUTPUT);
  pcf8575_1.digitalWrite(RL6, HIGH);

  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Blynk.config(BLYNK_AUTH_TOKEN);

  emon0.current(A0, 110);
  rtc_module.begin();

  eeprom.initialize();
  eeprom.readBytes(address, sizeof(dataDefault), (byte*)&data);

  timer.setInterval(1189, []() {
    readcurrent();
  });
  timer.setInterval(5013, []() {
    Blynk.syncVirtual(V4);
    Blynk.setProperty(V7, "label", BLYNK_FIRMWARE_VERSION, "- RSSI ", WiFi.RSSI());
  });
}

void loop() {
  Blynk.run();
  timer.run();
}