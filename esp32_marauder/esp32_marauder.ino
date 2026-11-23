/* FLASH SETTINGS
Board: LOLIN D32
Flash Frequency: 80MHz
Partition Scheme: Minimal SPIFFS
https://www.online-utility.org/image/convert/to/XBM
*/

#include "configs.h"

#ifndef HAS_SCREEN
  #define MenuFunctions_h
  #define Display_h
#endif

#include <stdio.h>

#ifdef HAS_GPS
  #include "GpsInterface.h"
#endif

#include "Assets.h"
#include "WiFiScan.h"
#ifdef HAS_SD
  #include "SDInterface.h"
#endif
#include "Buffer.h"

#ifdef HAS_FLIPPER_LED
  #include "flipperLED.h"
#elif defined(XIAO_ESP32_S3)
  #include "xiaoLED.h"
#elif defined(MARAUDER_M5STICKC) || defined(MARAUDER_M5STICKCP2)
  #include "stickcLED.h"
#elif defined(HAS_NEOPIXEL_LED)
  #include "LedInterface.h"
#endif

#include "settings.h"
#include "CommandLine.h"
#include "lang_var.h"

#ifdef HAS_BATTERY
  #include "BatteryInterface.h"
#endif

#ifdef HAS_SCREEN
  #include "Display.h"
  #include "MenuFunctions.h"
#endif

#ifdef HAS_BUTTONS
  #include "Switches.h"
  
  #if (U_BTN >= 0)
    Switches u_btn = Switches(U_BTN, 1000, U_PULL);
  #endif
  #if (D_BTN >= 0)
    Switches d_btn = Switches(D_BTN, 1000, D_PULL);
  #endif
  #if (L_BTN >= 0)
    Switches l_btn = Switches(L_BTN, 1000, L_PULL);
  #endif
  #if (R_BTN >= 0)
    Switches r_btn = Switches(R_BTN, 1000, R_PULL);
  #endif
  #if (C_BTN >= 0)
    Switches c_btn = Switches(C_BTN, 1000, C_PULL);
  #endif

#endif

WiFiScan wifi_scan_obj;
EvilPortal evil_portal_obj;
Buffer buffer_obj;
Settings settings_obj;
CommandLine cli_obj;

#ifdef HAS_GPS
  GpsInterface gps_obj;
#endif

#ifdef HAS_BATTERY
  BatteryInterface battery_obj;
#endif

#ifdef HAS_SCREEN
  Display display_obj;
  MenuFunctions menu_function_obj;
#endif

#if defined(HAS_SD) && !defined(HAS_C5_SD)
  SDInterface sd_obj;
#endif

#ifdef MARAUDER_M5STICKC
  AXP192 axp192_obj;
#endif

#ifdef HAS_FLIPPER_LED
  flipperLED flipper_led;
#elif defined(XIAO_ESP32_S3)
  xiaoLED xiao_led;
#elif defined(MARAUDER_M5STICKC) || defined(MARAUDER_M5STICKCP2)
  stickcLED stickc_led;
#elif defined(HAS_NEOPIXEL_LED)
  LedInterface led_obj;
#endif

const String PROGMEM version_number = MARAUDER_VERSION;

#ifdef HAS_NEOPIXEL_LED
  Adafruit_NeoPixel strip = Adafruit_NeoPixel(Pixels, PIN, NEO_GRB + NEO_KHZ800);
#endif

uint32_t currentTime  = 0;
#ifdef MARAUDER_DEBUG_USB_BOOT
static volatile uint32_t boot_probe_csi_frames = 0;
static volatile uint32_t boot_probe_promisc_frames = 0;
uint32_t last_debug_heartbeat = 0;
#endif

// PWM Brightness Control
#ifdef HAS_SCREEN
  #include <Preferences.h>
  #define BL_CHANNEL 0
  #define BL_FREQ 5000
  #define BL_RESOLUTION 8
  const uint8_t BL_LEVELS[] = {26, 51, 77, 102, 128, 153, 179, 204, 230, 255};
  const uint8_t BL_NUM_LEVELS = 10;
  uint8_t bl_level_idx = 9; // default full brightness
  Preferences bl_prefs;
#endif

#ifdef MARAUDER_DEBUG_USB_BOOT
  #define BOOT_DEBUG_PRINTLN(msg) Serial.println(msg)
  #define BOOT_DEBUG_PRINTF(msg, value) do { Serial.print(msg); Serial.println(value); } while(0)
#else
  #define BOOT_DEBUG_PRINTLN(msg) do {} while(0)
  #define BOOT_DEBUG_PRINTF(msg, value) do {} while(0)
#endif

#ifdef MARAUDER_DEBUG_USB_BOOT
static void bootProbePromiscRxCb(void* buf, wifi_promiscuous_pkt_type_t type) {
  (void)type;
  if (!buf) {
    return;
  }

  boot_probe_promisc_frames++;
}

static void bootProbeCsiRxCb(void* ctx, wifi_csi_info_t* info) {
  (void)ctx;
  if (!info || !info->buf) {
    return;
  }

  boot_probe_csi_frames++;
}

static void runBootPassiveProbe(bool verbose = false) {
  auto bootProbePrintln = [verbose](const char* msg) {
    if (verbose) {
      Serial.println(msg);
    }
  };
  auto bootProbePrintValue = [verbose](const char* msg, int value) {
    if (verbose) {
      Serial.print(msg);
      Serial.println(value);
    }
  };

  bootProbePrintln("BOOT_PROBE: begin");
  boot_probe_csi_frames = 0;
  boot_probe_promisc_frames = 0;

  esp_wifi_set_csi(false);
  esp_wifi_set_csi_rx_cb(nullptr, nullptr);
  esp_wifi_set_promiscuous(false);
  esp_wifi_stop();
  esp_wifi_deinit();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_err_t err = esp_wifi_init(&cfg);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_init=", (int)err);
  if (err != ESP_OK) {
    return;
  }

  err = esp_wifi_set_mode(WIFI_MODE_STA);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_mode=", (int)err);
  if (err != ESP_OK) {
    return;
  }

  err = esp_wifi_set_storage(WIFI_STORAGE_RAM);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_storage=", (int)err);

  err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_bandwidth=", (int)err);

  err = esp_wifi_start();
  bootProbePrintValue("BOOT_PROBE: esp_wifi_start=", (int)err);
  if (err != ESP_OK) {
    return;
  }

  err = esp_wifi_set_ps(WIFI_PS_NONE);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_ps=", (int)err);

  err = esp_wifi_set_channel(6, WIFI_SECOND_CHAN_BELOW);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_channel=", (int)err);

  err = esp_wifi_set_promiscuous(true);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_promiscuous=", (int)err);

  bootProbePrintln("BOOT_PROBE: esp_wifi_set_promiscuous_filter=SKIP");
  bootProbePrintln("BOOT_PROBE: esp_wifi_set_promiscuous_rx_cb=SKIP");

  wifi_csi_config_t csi_config = {
      .lltf_en = true,
      .htltf_en = true,
      .stbc_htltf2_en = true,
      .ltf_merge_en = true,
      .channel_filter_en = true,
      .manu_scale = false,
      .shift = false,
  };
  err = esp_wifi_set_csi_config(&csi_config);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_csi_config=", (int)err);

  err = esp_wifi_set_csi_rx_cb(bootProbeCsiRxCb, nullptr);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_csi_rx_cb=", (int)err);

  err = esp_wifi_set_csi(true);
  bootProbePrintValue("BOOT_PROBE: esp_wifi_set_csi=", (int)err);

  uint32_t start_ms = millis();
  uint32_t last_log_ms = start_ms;
  while ((millis() - start_ms) < 5000) {
    if (verbose && (millis() - last_log_ms) >= 1000) {
      last_log_ms = millis();
      Serial.print("BOOT_PROBE: csi_frames=");
      Serial.print((unsigned long)boot_probe_csi_frames);
      Serial.print(" promisc_frames=");
      Serial.println((unsigned long)boot_probe_promisc_frames);
    }
    delay(10);
  }

  if (verbose) {
    Serial.print("BOOT_PROBE: final csi_frames=");
    Serial.print((unsigned long)boot_probe_csi_frames);
    Serial.print(" promisc_frames=");
    Serial.println((unsigned long)boot_probe_promisc_frames);
  }

  esp_wifi_set_csi(false);
  esp_wifi_set_csi_rx_cb(nullptr, nullptr);
  esp_wifi_set_promiscuous(false);
  esp_wifi_stop();
  esp_wifi_deinit();
  bootProbePrintln("BOOT_PROBE: end");
}
#endif

// Helper macros for LEDC API compatibility (2.x vs 3.x board package)
#ifdef HAS_SCREEN
  #ifndef HAS_MINI_SCREEN
    #if ESP_ARDUINO_VERSION_MAJOR >= 3
      #define BL_SETUP()       ledcAttach(TFT_BL, BL_FREQ, BL_RESOLUTION)
      #define BL_SET(duty)     ledcWrite(TFT_BL, (duty))
    #else
      #define BL_SETUP()       do { ledcSetup(BL_CHANNEL, BL_FREQ, BL_RESOLUTION); ledcAttachPin(TFT_BL, BL_CHANNEL); } while(0)
      #define BL_SET(duty)     ledcWrite(BL_CHANNEL, (duty))
    #endif
  #endif
#endif

#ifndef HAS_MINI_SCREEN
  void brightnessInit() {
    #ifdef HAS_SCREEN
      BL_SETUP();
      bl_prefs.begin("backlight", false);
      bl_level_idx = bl_prefs.getUChar("level", 9);
      if (bl_level_idx >= BL_NUM_LEVELS) bl_level_idx = 9;
      BL_SET(BL_LEVELS[bl_level_idx]);
    #endif
  }

  void brightnessCycle() {
    #ifdef HAS_SCREEN
      bl_level_idx = (bl_level_idx + 1) % BL_NUM_LEVELS;
      BL_SET(BL_LEVELS[bl_level_idx]);
      bl_prefs.putUChar("level", bl_level_idx);
      Serial.print(F("[Brightness] Level "));
      Serial.print(bl_level_idx + 1);
      Serial.print(F("/"));
      Serial.print(BL_NUM_LEVELS);
      Serial.print(F(" ("));
      Serial.print(BL_LEVELS[bl_level_idx] * 100 / 255);
      Serial.println(F("%)"));
    #endif
  }

  uint8_t getBrightnessLevel() {
    #ifdef HAS_SCREEN
      return bl_level_idx;
    #else
      return 0;
    #endif
  }

  void brightnessSave(uint8_t level) {
    #ifdef HAS_SCREEN
      if (level >= BL_NUM_LEVELS) level = BL_NUM_LEVELS - 1;
      bl_level_idx = level;
      BL_SET(BL_LEVELS[bl_level_idx]);
      bl_prefs.putUChar("level", bl_level_idx);
    #endif
  }

  void backlightOn() {
    #ifdef HAS_SCREEN
      BL_SET(BL_LEVELS[bl_level_idx]);
    #endif
  }

  void backlightOff() {
    #ifdef HAS_SCREEN
      BL_SET(0);
    #endif
  }
#else
  void backlightOn() {
    #ifdef HAS_SCREEN
      #if defined(MARAUDER_MINI) || defined(MARAUDER_MINI_V3)
        digitalWrite(TFT_BL, LOW);
      #endif
    
      #if !defined(MARAUDER_MINI) && !defined(MARAUDER_MINI_V3)
        digitalWrite(TFT_BL, HIGH);
      #endif
    #endif
  }

  void backlightOff() {
    #ifdef HAS_SCREEN
      #if defined(MARAUDER_MINI) || defined(MARAUDER_MINI_V3)
        digitalWrite(TFT_BL, HIGH);
      #endif
    
      #if !defined(MARAUDER_MINI) && !defined(MARAUDER_MINI_V3)
        digitalWrite(TFT_BL, LOW);
      #endif
    #endif
  }
#endif

#ifdef HAS_C5_SD
  SPIClass sharedSPI(SPI);
  SDInterface sd_obj = SDInterface(&sharedSPI, SD_CS);
#endif

void setup()
{
  randomSeed(esp_random());
  BOOT_DEBUG_PRINTLN("BOOT: setup start");
  
  #ifndef DEVELOPER
    esp_log_level_set("*", ESP_LOG_NONE);
  #endif
  
  #if !defined(HAS_IDF_3) && defined(HAS_PSRAM)
    esp_spiram_init();
  #endif

  Serial.begin(115200);
  BOOT_DEBUG_PRINTLN("BOOT: Serial.begin done");

  uint32_t serial_wait_start = millis();
  while(!Serial && (millis() - serial_wait_start) < 2000)
    delay(10);
  BOOT_DEBUG_PRINTLN("BOOT: serial wait done");

  #if defined(MARAUDER_FLIPPER) && defined(MARAUDER_DEBUG_USB_BOOT)
    runBootPassiveProbe(true);
  #endif

  #ifdef HAS_C5_SD
    sharedSPI.begin(SD_SCK, SD_MISO, SD_MOSI);
    delay(100);
  #endif

  #ifdef defined(MARAUDER_M5STICKC) && !defined(MARAUDER_M5STICKCP2)
    axp192_obj.begin();
  #endif

  #if defined(MARAUDER_M5STICKCP2) // Prevent StickCP2 from turning off when disconnect USB cable
    pinMode(POWER_HOLD_PIN, OUTPUT);
    digitalWrite(POWER_HOLD_PIN, HIGH);
  #endif
  
  #ifdef HAS_SCREEN
    pinMode(TFT_BL, OUTPUT);
  #endif
  
  backlightOff();
  #if BATTERY_ANALOG_ON == 1
    pinMode(BATTERY_PIN, OUTPUT);
    pinMode(CHARGING_PIN, INPUT);
  #endif
  
  // Preset SPI CS pins to avoid bus conflicts
  #ifdef HAS_SCREEN
    digitalWrite(TFT_CS, HIGH);
  #endif
  
  #if defined(HAS_SD) && !defined(HAS_C5_SD)
    pinMode(SD_CS, OUTPUT);

    delay(10);
  
    digitalWrite(SD_CS, HIGH);

    delay(10);
  #endif

  //Serial.begin(115200);

  //while(!Serial)
  //  delay(10);

  Serial.println("ESP-IDF version is: " + String(esp_get_idf_version()));
  BOOT_DEBUG_PRINTLN("BOOT: idf banner done");

  #ifdef HAS_PSRAM
    if (!psramInit()) {
      Serial.println(F("PSRAM not available"));
    }
  #endif

  #ifdef HAS_SIMPLEX_DISPLAY
    #if defined(HAS_SD)
      // Do some SD stuff
      if(!sd_obj.initSD())
        Serial.println(F("SD Card NOT Supported"));

    #endif
  #endif

  #ifdef HAS_SCREEN
    display_obj.RunSetup();
    display_obj.tft.setTextColor(TFT_WHITE, TFT_BLACK);
  #endif

  // Init PWM brightness AFTER display init (so ledcAttach overrides TFT_eSPI's pinMode)
  #ifndef HAS_MINI_SCREEN
    brightnessInit();
    backlightOff();
  #endif

  #ifdef HAS_SCREEN
    #if !defined(MARAUDER_CARDPUTER) && !defined(MARAUDER_CARDPUTER_ADV)
      display_obj.tft.drawCentreString("ESP32 Marauder", TFT_WIDTH/2, TFT_HEIGHT * 0.33, 1);
      display_obj.tft.drawCentreString("JustCallMeKoko", TFT_WIDTH/2, TFT_HEIGHT * 0.5, 1);
      display_obj.tft.drawCentreString(display_obj.version_number, TFT_WIDTH/2, TFT_HEIGHT * 0.66, 1);
    #else
      display_obj.tft.drawCentreString("ESP32 Marauder", TFT_HEIGHT/2, TFT_WIDTH * 0.33, 1);
      display_obj.tft.drawCentreString("JustCallMeKoko", TFT_HEIGHT/2, TFT_WIDTH * 0.5, 1);
      display_obj.tft.drawCentreString(display_obj.version_number, TFT_HEIGHT/2, TFT_WIDTH * 0.66, 1);
    #endif
  #endif


  backlightOn(); // Need this

  #ifdef HAS_SCREEN
    // Do some stealth mode stuff
    #ifdef HAS_BUTTONS
      if (c_btn.justPressed()) {
        display_obj.headless_mode = true;

        backlightOff();
      }
    #endif
  #endif

  settings_obj.begin();
  BOOT_DEBUG_PRINTLN("BOOT: settings begin done");

  const char* type = settings_obj.getSettingType("ChanHop");

  if (type == nullptr || type[0] == '\0') {
    Serial.println(F("Current settings format not supported. Installing new default settings..."));
    settings_obj.createDefaultSettings(SPIFFS);
  }

  buffer_obj = Buffer();
  BOOT_DEBUG_PRINTLN("BOOT: buffer init done");

  #if defined(MARAUDER_FLIPPER) && defined(MARAUDER_DEBUG_USB_BOOT)
    Serial.println("BOOT_PROBE: post-buffer");
    runBootPassiveProbe(true);
  #endif

  #ifndef HAS_SIMPLEX_DISPLAY
    #if defined(HAS_SD)
      // Do some SD stuff
      if(!sd_obj.initSD())
        Serial.println(F("SD Card NOT Supported"));

    #endif
  #endif

  wifi_scan_obj.RunSetup();
  BOOT_DEBUG_PRINTLN("BOOT: wifi scan setup done");

  #if defined(MARAUDER_FLIPPER) && defined(MARAUDER_DEBUG_USB_BOOT)
    Serial.println("BOOT_PROBE: post-wifi-scan-setup");
    runBootPassiveProbe(true);
  #endif

  #ifdef HAS_SCREEN
    display_obj.tft.setTextColor(TFT_GREEN, TFT_BLACK);
    display_obj.tft.drawCentreString("Initializing...", TFT_WIDTH/2, TFT_HEIGHT * 0.82, 1);
  #endif

  evil_portal_obj.setup();
  BOOT_DEBUG_PRINTLN("BOOT: evil portal setup done");

  #ifdef HAS_BATTERY
    battery_obj.RunSetup();
  #endif
  BOOT_DEBUG_PRINTLN("BOOT: battery setup done");

  #ifdef HAS_BATTERY
    battery_obj.battery_level = battery_obj.getBatteryLevel();
  #endif

  // Do some LED stuff
  #ifdef HAS_FLIPPER_LED
    flipper_led.RunSetup();
  #elif defined(XIAO_ESP32_S3)
    xiao_led.RunSetup();
  #elif defined(MARAUDER_M5STICKC)
    stickc_led.RunSetup();
  #elif defined(HAS_NEOPIXEL_LED)
    led_obj.RunSetup();
  #endif

  #ifdef HAS_GPS
    #if defined(MARAUDER_DEBUG_USB_BOOT) && defined(MARAUDER_FLIPPER)
      BOOT_DEBUG_PRINTLN("BOOT: gps begin skipped for usb debug");
    #else
      gps_obj.begin();
    #endif
  #endif
  BOOT_DEBUG_PRINTLN("BOOT: gps begin done");

  #ifdef HAS_SCREEN  
    display_obj.tft.setTextColor(TFT_WHITE, TFT_BLACK);
  #endif

  #ifdef HAS_SCREEN
    #if defined(MARAUDER_CARDPUTER) || defined(MARAUDER_CARDPUTER_ADV)
      display_obj.clearScreen();
    #endif
    menu_function_obj.RunSetup();
  #endif

  /*char ssidBuf[64] = {0};  // or prefill with existing SSID
  if (keyboardInput(ssidBuf, sizeof(ssidBuf), "Enter SSID")) {
    // user pressed OK
    Serial.println(ssidBuf);
  } else {
    Serial.println(F("User exited keyboard"));
  }

  menu_function_obj.changeMenu(menu_function_obj.current_menu);*/

  wifi_scan_obj.StartScan(WIFI_SCAN_OFF);
  BOOT_DEBUG_PRINTLN("BOOT: wifi scan off");

  #if defined(MARAUDER_FLIPPER) && defined(MARAUDER_DEBUG_USB_BOOT)
    Serial.println("BOOT_PROBE: pre-cli");
    runBootPassiveProbe(true);
  #endif
  
  cli_obj.RunSetup();
  BOOT_DEBUG_PRINTLN("BOOT: cli setup done");
}


void loop()
{
  currentTime = millis();
  bool mini = false;

  #ifdef MARAUDER_DEBUG_USB_BOOT
    if ((currentTime - last_debug_heartbeat) > 5000) {
      last_debug_heartbeat = currentTime;
      Serial.println("BOOT: loop heartbeat");
    }
  #endif

  #ifdef SCREEN_BUFFER
    #ifndef HAS_ILI9341
      mini = true;
    #endif
  #endif

  #if (defined(HAS_ILI9341) && !defined(MARAUDER_CYD_2USB))
    #ifdef HAS_BUTTONS
      if (c_btn.isHeld()) {
        if (menu_function_obj.disable_touch)
          menu_function_obj.disable_touch = false;
        else
          menu_function_obj.disable_touch = true;

        menu_function_obj.updateStatusBar();

        while (!c_btn.justReleased())
          delay(1);
      }
    #endif
  #endif

  // Update all of our objects
  cli_obj.main(currentTime);
  wifi_scan_obj.main(currentTime);

  #ifdef HAS_GPS
    gps_obj.main();
  #endif

  // Save buffer to SD and/or serial
  buffer_obj.save();

  #ifdef HAS_BATTERY
    battery_obj.main(currentTime);
  #endif
  if ((wifi_scan_obj.currentScanMode != WIFI_PACKET_MONITOR) ||
      (mini)) {
    #ifdef HAS_SCREEN
      menu_function_obj.main(currentTime);
    #endif
  }
  #ifdef HAS_FLIPPER_LED
    flipper_led.main();
  #elif defined(XIAO_ESP32_S3)
    xiao_led.main();
  #elif defined(MARAUDER_M5STICKC)
    stickc_led.main();
  #elif defined(HAS_NEOPIXEL_LED)
    led_obj.main(currentTime);
  #endif

  #ifdef HAS_SCREEN
    delay(1);
  #else
    delay(50);
  #endif
}
