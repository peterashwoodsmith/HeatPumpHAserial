//
// This ESP32 ARDUINO program is a Zibgee end device that will receive commands to be translated into the IR remote codes for a 
// MISTUBISHI HEATE PUMP. THis is work in progress for a Home Assistant end point that you attach near the IR sensor of the
// Mitsubishi heat pump and that is mains powered. This can be powered by USB, or 5V etc. A battery version that sleeps is also
// possible but there are challanges with the sleep duration required to keep the power use down and Home Assistant timeouts.
//
// The code has two major parts. The first is the IR/Mitsibushi send command that takes all the desired settings as arguments.
// Following that is the zibgee 3.0 Espressif Arduino code to create the end points and their controls (clusters) that select
// the various parameters for the heat pump. This IR code is just from the HVAC IR Control project and credit goes to Vincent Cruvellier
// and the other folks that decoded the Mitsibushi IR packets. Their code is mostly as on their git page however compiler problems
// with passing single byte enums caused me grief so I had to de-enum their code and use good old #defines. 
// The Zibgee code is loosly based on some of the Espressif Arduino examples and lots of trial and error experiments. Sadly I was 
// unable to use better clusters than binary and analog, all the other choices caused problems or did not work. Ideally it should
// use multi value (enum type) clusters for the fans/vanes but thats for future.
//       
//     PARTA     https://github.com/r45635/HVAC-IR-Control ,           (c)  Vincent Cruvellier
//     PARTB     https://github.com/peterashwoodsmith/HeatPumpHA       (c)  Peter Ashwood-Smith Dec 2025 
//
// HARDWARE:
//          Waveshare ESP32-C6 but should work on any ESP32 that supports Zibgee and has an RGB Led for status indication.
//          Push button for the reset option, any momentarily on push button will work.
//          IR 38Khz. Transmitter module - I used a DUTTY 38khz but no doubt anything of proper freq/power will work.
//
//          The IR board is connected to +5V power pin and GND but it works on 3.3V pin also (although range is reduced at 
//          the lower voltage, in any case this device is designed to be right up against the Heat Pump IR sensor). 
//          The signal pin to drive the IR module is GPIO10 but any other pin should work as long as its not used for
//          other purposes by the board.
//          The reset button is connected to GPIO18 and Ground, this is convenient because they are one pin separated
//          so you can just solder a reset button between them but any pin and ground should work. This is used to trigger a 
//          factory reset, normal reset just power cycle the board.
//
// BUILD NOTES:
//          I built this on a Mac and had problems with the USB driver. Waveshare has a nice page describing how to put a new
//          driver on your Mac which worked perfectly. Without it one of my boards refused to load the code via Arduino but 
//          surprise a bunch of other boards worked just fine. Anyway if you get CRC errors downlaoding, try lower speeds and if that
//          fails pop over to Waveshare and lookup the USB driver problem.
//
//          ARDUINO IDE TOOLS SETTINGS:
//          You need to set a number of settings in the Arduino IDE/Tools menu for this to work properly.
//              1 - Tools/USB CDC on boot - enabled (allows serial IO for debugging).
//              2 - Tools/Core debug level (set as desired useful for debugging zibbee attach etc.) start verbose.
//              3 - Tools/Erase all flash before upload - this means each download its a brand new Zibgee end point.
//                  Id erase for first few downloads and always delete/reattach but after its working don't erease the
//                  flash each time. Once its working you can erase the flash and start scratch with a long press on the
//                  reset button anyway.
//              3 - Tools/partition scheme: 4MB with spiffs - seems to be what the Zibgee library wants.
//              4 - Tools/zibgee mode ED (end device) - you can also use the end mode with debug enabled for more tracing.
//
// SOFTWARE:
//          There is a debug variable you can set 1/0 for serial I/O debugging. Start 1 turn off later.
//          On initial startup the code will start a watch dog timer just in case. This will cause a panic reset if the main
//          loop() is not entered frequently enough indicating something got stuck. The main loop() of course 'feeds' this watch
//          dog.
//          Next we read the non volatile storage to see if we have previously saved values for the attributes that need to be 
//          restored. THis is because Zibgee/HA does not seem to refresh them when we restart. Presumably there is a way to force this
//          but I've not found it with a few weeks of playing. 
//          Next we setup interrupt handlers for the reset button. Basically if reset is pressed we isue a reset to the ESP but we
//          also we erase the non volatile memory which will cause the end point to forget any ziggee related information and it
//          will have to rebind. If you do this you need to
//          delete the device from home assistant and go back into zigbee discover mode to refind this end point. For a normal
//          non factory reset just power cycle or use the ESP32 reset button.
//          We use the RGB Led to indicate whats going on. Red flashing as it first boots, orange as its getting ready to attach to the
//          zigbee network and then blue flashing as its trying to bind with the zibgee network and finally green flashing as its in the
//          main loop() waiting for HA to ask it to do something.
//          In the setup() code we will create the zibbee end points one for each of the clusters/attributes we want to control.
//          These are using the Espressif Arduino interface which is high level but not well documented. Essentially you create the
//          objects and a callback to set each parameter and then bind them to zibgee and wait for it to connect. After that its all
//          callbacks.
//          In the callbacks we just remember the attribute changes which are processed later in the main loop().
//          In the main loop we look for any attribute changes and if so call the IR send routine. 
//          After sending the packet with the IR routing we write the attributes to non volatile storage in case of a power interruption
//          Or sleep (in future versions).
//          After we send the IR packet we do a quick white flash of the RGB led then its back to green waiting.
//
//  Home Assistant Notes:
//          As a Zibgee device you need to put the Zibgee in device search mode. If all goes well when the ESP32 boots it will flash 
//          red orange then blue and stay blue until you see Home Assistant show the device has connected etc. at which point it 
//          will flash green. Once its bound you can reset the ESP32 via power etc. and it will reattach fast and remember what ever
//          name you gave it. Initial binding can take a few minutes. Any problems just long press to erase the NVS and delete the 
//          device from HA and go back into device discorvery on HA to let it rebind. Sometimes being close to the HA/Zibbee dongle
//          helps so if its cause problem, erase the NVS, delete the device from HA and move close to your dongle to retry. I suspect 
//          the dongles and HA notice devices trying to attach too frequently and back off a bit so sometimes waiting a few minutes to
//          retry gives you a quik attach.
// 
//  TODO:
//          Need to work on battery version and long duration sleep. Problem with this is the HA timeouts and I need to understand how
//          to make HA not try to send when its clearly sleeping. Probably explained in some document somewhere but have not found it yet.
//          Identify callback should probably flash the LED through rainbow colors or something. Easy enough to do but Arduino Zibgee
//          objects don't seem to support this callback, or I missed it.
//          ESP32-H version (i.e. non Wifi / smaller chip);
//          Will probably look at the serial interface to the Mitsubishi too at some point but direct connect to the Heat Pump is a 
//          warranty violation so this has to be done with caution and after warranty expires. I have no idea if Mistibuish can log 
//          a serial device connecting or not. So caution here. The ESP32-C6 can do wifi too so no reason it cannot do the serial connect
//          via WIFI or via Zibgee in the future. Well see.
//
//  NOTES:
//          I make no claims that this even works, or that your entire house won't explode if you use it. Nobody should ever use this 
//          for anything serious ever. This is for educational purposes only.
//
//          Enjoy. Peter Ashwood-Smith
//
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
#include "Zigbee.h"
#include "HeatPump.h"

//
// Set 1 and you'll get lots of useful info as it runs. For debugging the lower layer Zibgee see the tools settings
// in the Arduino menu for use with the debug enabled library and debug levels in that core.
//
bool debug_g = false;

// 
// Function complete shutdown and restart. Forward declared also a flash sequence for factory reset.
//
extern void ha_restart(); 
extern void rgb_led_set_factory_reset();

//
// Interrupt handler for Reset button. If its pressed we do full factory reset. Normal reset is just done with a power off/on.
//
void isr_resetButtonPress()
{      
     rgb_led_set_factory_reset();                       // Go white so its obvious
     Zigbee.factoryReset(false);                        // This should do the same but not sure it does anyway ...
     ha_restart();                                      // And stop all the Zigbee stuff and just restart the ESP
}

//
// Setup the Interrupt service routine for the reset button. Just call the isr_resetButtonPress routing when the pin goes LOW.
// This triggers a factory reset.
//
void isr_setup()
{    const unsigned int isr_resetButton = 18;               // We use pin 18 connected to one side of button, and ground to other side.   
     pinMode(isr_resetButton, INPUT_PULLUP);                // Pullup so GROUNDing cause FALLING and ungrounding causes RISING interrupts.
     //
     attachInterrupt(digitalPinToInterrupt(isr_resetButton), isr_resetButtonPress,   FALLING);  
}

//
// Serial Interface libarary to the Heat Pump.
//
HeatPump hp;

void hp_sendHvacMitsubishi(
     int HVAC_Mode,            
     int HVAC_Temp,            
     int HVAC_FanMode,         
     int HVAC_VaneMode,        
     int HVAC_powerOff         
)
{    char *s;
     char power[10];
     strcpy(power, HVAC_powerOff ? "OFF" : "ON"); 
     //
     char mode[10];
     strcpy(mode, HVAC_Mode ? "HEAT" : "COOL");
     //
     int temp = HVAC_Temp;
     if (temp < 16) temp = 16;
     if (temp > 31) temp = 31;
     //
     char fan[10]; 
     switch(HVAC_FanMode){
          case  0: s = "QUIET"; break;
          case  1: s = "1";     break;
          case  2: s = "2";     break;
          case  3: s = "3";     break;
          case  4: s = "4";     break;
          default: s = "AUTO";  break;
     }
     strcpy(fan, s);
     //
     char vane[10];
     switch(HVAC_VaneMode){
          case  0: s = "AUTO"; break;
          case  1: s = "1";     break;
          case  2: s = "2";     break;
          case  3: s = "3";     break;
          case  4: s = "4";     break;
          case  5: s = "5";     break;   
          default: s = "SWING"; break;
     }
     strcpy(vane, s);//
     //
     heatpumpSettings settings = {
            power,   /* ON/OFF */
            mode,    /* HEAT/COOL/FAN/DRY/AUTO */
            temp,    /* Between 16 and 31 */
            fan,     /* Fan speed: 1-4, AUTO, or QUIET */
            vane,    /* Air direction (vertical): 1-5, SWING, or AUTO */
            "|"    /* Air direction (horizontal): <<, <, |, >, >>, <>, or SWING */
     };
     //
     hp.setSettings(settings);
     hp.update();
}

//
// Connect to the heat pump over serial port. We will install the callbacks for status changes. We
// Use the enable external updates so that changes made by the remove will show up in the zibgre 
// values in home assistant.
//
void hp_setup() {
     extern void hp_settingsChangedCallback();
     hp.enableExternalUpdate();
     hp.setSettingsChangedCallback(hp_settingsChangedCallback);
     hp.connect(&Serial1, 2400, 17, 16);
}

//
// PART B - Z I G B E E / Home Assistant interface
//

// 
// These are the EP control entities each has one cluser which is a 'knob' that controls an HVAC parameter
//
ZigbeeBinary      zbPower       = ZigbeeBinary(10);      // Power on/off knob
ZigbeeBinary      zbColdHot     = ZigbeeBinary(11);      // Heat or cooling knob
ZigbeeAnalog      zbTemp        = ZigbeeAnalog(12);      // Desired temperature slider
ZigbeeAnalog      zbFanControl  = ZigbeeAnalog(13);      // How the fans operate slider
ZigbeeAnalog      zbVaneControl = ZigbeeAnalog(14);      // Van motion/positions slider

//
// These are the variables that maintain the state of what HA has asked to be set
// set.
//
bool              ha_powerStatus   = 0;    // powered on/off
bool              ha_coldHotStatus = 0;    // heating or cooling mode
unsigned int      ha_fanStatus     = 0;    // fan position or movement
unsigned int      ha_tempStatus    = 0;    // desired temperature
unsigned int      ha_vaneStatus    = 0;    // how the vanes move or don't

//
// Keep HA up to date with any changes that happen on the heat pump from the serial updates.
//
void ha_sync_status()
{
     zbVaneControl.setAnalogOutput(ha_vaneStatus);
     zbFanControl.setAnalogOutput(ha_fanStatus);
     zbTemp.setAnalogOutput(ha_tempStatus);
     zbColdHot.setBinaryOutput(ha_coldHotStatus);
     zbPower.setBinaryOutput(ha_powerStatus);
     //
     zbVaneControl.reportAnalogOutput();
     zbFanControl.reportAnalogOutput();
     zbTemp.reportAnalogOutput();
     zbColdHot.reportBinaryOutput();
     zbPower.reportBinaryOutput();
}

//
// The heat pump has reported a change in settings, likely from the remote. Anyway we read the settings
// set the ha_* variables above so we are up to date, and then reflect them back to zibgee with the sync
// function above.
//
void hp_settingsChangedCallback()
{
     heatpumpSettings s = hp.getSettings();
     //
     if (s.temperature > 0) ha_tempStatus      = s.temperature;
     if (s.power)           ha_powerStatus     = strcmp(s.power, "OFF")  == 0 ? 0 : 1;
     if (s.mode)            ha_coldHotStatus   = strcmp(s.mode,  "COLD") == 0 ? 0 : 1;
     //
     if (s.fan) {
         if (sscanf(s.fan, "%d", &ha_fanStatus) != 1) {
             ha_fanStatus = strcmp(s.fan, "QUIET") == 0 ? 0 : 5;
         }
     }
     //
     if (s.vane) {
         if (sscanf(s.vane, "%d", &ha_vaneStatus) != 1) {
             ha_vaneStatus = strcmp(s.vane, "AUTO") == 0 ? 0 : 6;
         }
     }
     //
     ha_sync_status();
}

//
// Send the Heat Pump the proper commands to synchronize with what HA as asked. Basically convert from the above ha_ variables
// to corresponding hv_ variables and call the send function which will encode the 18 byte frame to the IR transmitter.
//
void ha_syncHeatPump()
{
     int hv_mode;
     switch(ha_coldHotStatus) {
         case 1: hv_mode = 1; break;
         case 0: hv_mode = 0; break;
         default:
                 if (debug_g) Serial.printf("Invalid power status =% d\n", ha_coldHotStatus);
                 return;
     }
     //
     int hv_powerOff  = ha_powerStatus ? 0 : 1;        // its an off flag to the UI so reversed from HA.
     int hv_temp      = ha_tempStatus;
     int hv_fanMode   = ha_fanStatus;
     int hv_vanneMode = ha_vaneStatus;
     //
     if (debug_g) Serial.printf("*** SEND HVAC COMMAND: mode=%d, temp=%d, fan=%d, vane=%d, off=%d ***\n",
                                 hv_mode, hv_temp, hv_fanMode, hv_vanneMode, hv_powerOff);
     //                         
     // Send the serial command to the Mitsubishi.
     //
     hp_sendHvacMitsubishi(hv_mode, hv_temp, hv_fanMode, hv_vanneMode, hv_powerOff);
}

//
// These are just useful debugging functions to display the attributes that HA has given us.
// One for each attributes.
//
void ha_displayPowerStatus()
{
     Serial.printf("POWER STATUS     = %s\n", ha_powerStatus ? "ON" : "OFF"); 
}
//
void ha_displayFanStatus()
{     
     Serial.printf("FAN STATUS       = %d\n", ha_fanStatus);
}
//
void ha_displayVaneStatus()
{
     if ((ha_vaneStatus < 0)||(ha_vaneStatus > 6))
        Serial.printf("VANE STATUS      = %d ????\n", ha_vaneStatus);
     else
        Serial.printf("VANE STATUS      = %d\n", ha_vaneStatus);
}
//
void ha_displayColdHotStatus()
{    
     Serial.printf("COOL/HEAT STATUS = %s (%x)\n", ha_coldHotStatus ? "HEAT" : "COOL", (unsigned int) ha_coldHotStatus);
}
//
void ha_displayTempStatus()
{    
     Serial.printf("TEMP STATUS      = %d\n", ha_tempStatus);
}

//
// This is just a little schedule time for when to do a sync. When we get attribute changes we schedule and update 5s in the future.
// THis just allows us to aggregate multiple quick arriving atribute changes.
//
unsigned long ha_update_t = 0;      // not sure about callback thread if its this or different thread.

//
// These are the callback functions that set each of the attributes. When HA makes a change to the attribute the zibgee library
// will call these so we can record the desired setting. Note that we do not immediately try to send the IR packet to the HVAC
// rather we will wait a bit to make sure that if the user changes a few attributes in a few seconds can just send one command and
// save a little bit of power. One set function per attribute follows.
//
void ha_setFan(float value)   
{
     if (ha_fanStatus != (int) value) {
         ha_fanStatus = value;
         if (debug_g) { Serial.print("HA=> "); ha_displayFanStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setPower(bool value)
{
     if (ha_powerStatus != value) {
         ha_powerStatus = value;
         if (debug_g) { Serial.print("HA=> "); ha_displayPowerStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setColdHot(bool state)
{
     if (ha_coldHotStatus != state) { 
         ha_coldHotStatus = state;
         if (debug_g) { Serial.print("HA=> "); ha_displayColdHotStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setTemp(float value)
{    
     if (ha_tempStatus != (int) value) {
         ha_tempStatus = value;
         if (debug_g) { Serial.print("HA=> "); ha_displayTempStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setVane(float value)
{
     if (ha_vaneStatus != (int) value) {
         ha_vaneStatus = value;
         if (debug_g) { Serial.print("HA=> "); ha_displayVaneStatus(); }
         ha_update_t = millis() + 5000;
     }
}

//
// We use the color RGB LED to indicate state.
//
const uint8_t RGB_LED_OFF    = 0;        // Enums are causing compiler problems when passed as first argument.
const uint8_t RGB_LED_WHITE  = 1;        // so back to old school.
const uint8_t RGB_LED_RED    = 2;
const uint8_t RGB_LED_GREEN  = 3;
const uint8_t RGB_LED_BLUE   = 4;
const uint8_t RGB_LED_ORANGE = 5;
const uint8_t RGB_MAX        = RGB_BRIGHTNESS/8;      // Fairly dim or they keep people awake
const uint8_t RGB_MIN        = 0;
#define       RGB_ORDER        LED_COLOR_ORDER_RGB    // Compiler problems passing enums, have to be explicit
//
void rgb_led_set(int color) {
     switch(color) {                                  //RED     GREEN     BLUE
         case RGB_LED_GREEN : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MAX,  RGB_MIN); break;
         case RGB_LED_WHITE : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MAX, RGB_MAX,  RGB_MAX); break;
         case RGB_LED_RED   : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MAX, RGB_MIN,  RGB_MIN); break;          
         case RGB_LED_BLUE  : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MIN,  RGB_MAX); break;
         case RGB_LED_ORANGE: rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MAX, RGB_MAX/2,RGB_MIN); break;
         case RGB_LED_OFF   : rgbLedWriteOrdered(RGB_BUILTIN, RGB_ORDER, RGB_MIN, RGB_MIN,  RGB_MIN); break;
     }
}
//
//   Simple LED flash routine, should really use a background task to do this.. tbd. Flash the chosen color and
//   then return to the restore color after.
//
void rgb_led_flash(int color, int restore_color)
{
     for(int i = 0; i < 5; i++) {
        rgb_led_set(color);
        delay(50);
        rgb_led_set(RGB_LED_OFF);
        delay(50);
     }
     rgb_led_set(restore_color);
}

//
// We are in an interrupt handler for the reset button and will indicate a factory reset.
// Just use white for now.
//
void rgb_led_set_factory_reset()
{
     rgb_led_set(RGB_LED_WHITE);
}

//
// Called when device is asked to identify itself. We will just flash alternating white and green for 1/2 second or so.
//
void ha_identify(uint16_t x)
{
     if (debug_g) Serial.printf("******** HA => IDENTIFY(%d) ******\n", (int) x);
     //
     rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
     delay(100);
     rgb_led_flash(RGB_LED_GREEN, RGB_LED_GREEN);
     delay(100);
     rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
     delay(100); 
     rgb_led_flash(RGB_LED_GREEN, RGB_LED_GREEN);
     delay(100);
     rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
     delay(100); 
     rgb_led_set(RGB_LED_GREEN);
}

//
// Wait for any pending requests and process them if we find some. Do this for a few seconds and return.
// We may gets updates requests while we are running so we keep going if they are still comming in.
//
void ha_processPending() 
{
     uint32_t end_t = millis() + 1000;   
     do { 
         if (!Zigbee.connected()) {
              if (debug_g) Serial.println("zigbee disconnected while in ha_processPending()- restarting");
              ha_restart();   
        }
        delay(50);
        //
        // If synch required then send the IR now. Wonder if there is problem with mutual exclusion and
        // callback functions. Are they in same thread? If not this variable is volatile.
        //
        if ((ha_update_t > 0) && (millis() > ha_update_t)) {
            ha_update_t = 0;
            if (debug_g) {
                Serial.printf("----------------------------- wait for HA  msgs ------------------------------------\n");
                ha_displayPowerStatus();
                ha_displayColdHotStatus();
                ha_displayTempStatus();
                ha_displayFanStatus();
                ha_displayVaneStatus();
                Serial.println("Synch with HVAC required\n");
            }  
            ha_syncHeatPump();
            rgb_led_flash(RGB_LED_WHITE, RGB_LED_GREEN);   // flash white, return to green
        }
     } while((millis() <= end_t) || (ha_update_t > 0));    // keep going till end time or till end of work
}

//
// Go to sleep for some number of seconds then we will be woken up to start all over again in the
// setup() function. The loop() will never get called. Currently about 45 seconds of sleep and 4.5 seconds
// awake seems to be what actually works, going to longer sleep gives me timeouts in HA that I need to 
// to figure out before extending this.
//
void ha_restart()
{  
     rgb_led_set(RGB_LED_OFF);            // Sometimes gets stuck on, don't know why perhaps timing.      
     delay(100);
     rgb_led_set(RGB_LED_OFF);            // So do it twice .
     delay(100);
     if (debug_g) Serial.println("Restarting..."); 
     Zigbee.closeNetwork();
     Zigbee.stop();
     delay(100);
     ESP.restart();
}

// 
// We woke up, configure zibgee and wait for connection, then process any pending requests
// and go back to sleep. 
//
void setup() {
     //
     // Debug stuff
     //
     if (debug_g) {
         Serial.begin(115200);
         delay(1000);
         Serial.println("RiverView S/W Zibgee 3.0 to Mistubishi IR controller");
     }
     //
     // Watch dog timer on this task to panic if we don't get to main loop regulary.
     //
     esp_task_wdt_deinit();
     esp_task_wdt_config_t wdt_config = {
          .timeout_ms = 30 * 60 * 1000,                                 // 30 minutes max to get back to main loop()
          .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Bitmask of all cores
          .trigger_panic = true };                                      // Enable panic to restart ESP32
     esp_task_wdt_init(&wdt_config);
     esp_task_wdt_add(NULL);                                            //add current thread to WDT watch
     //
     rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
     //
     // enable interrupts for reset buttons.
     //
     isr_setup();
     //
     // Add the zibgee clusters (buttons/sliders etc.)
     //
     if (debug_g) Serial.println("On of Power switch cluster");
     zbPower.setManufacturerAndModel("RiverView", "MitsuIR");
     zbPower.addBinaryOutput();
     zbPower.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbPower.setBinaryOutputDescription("Off => On");
     zbPower.onBinaryOutputChange(ha_setPower);
     //
     if (debug_g) Serial.println("Cold/Hot Switch cluster");
     zbColdHot.setManufacturerAndModel("RiverView", "ZigbeeToHvacIR");
     zbColdHot.addBinaryOutput();
     zbColdHot.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbColdHot.setBinaryOutputDescription("Cool => Heat");
     zbColdHot.onBinaryOutputChange(ha_setColdHot);
     //
     if (debug_g) Serial.println("Temp Selector cluster");
     zbTemp.setManufacturerAndModel("RiverView", "ZigbeeToHvacIR");
     zbTemp.addAnalogOutput();
     zbTemp.setAnalogOutputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
     zbTemp.setAnalogOutputDescription("Temperature C");
     zbTemp.setAnalogOutputResolution(1);
     zbTemp.setAnalogOutputMinMax(0, 30); 
     zbTemp.onAnalogOutputChange(ha_setTemp);
     //
     if (debug_g) Serial.println("Fan Selector cluster");
     zbFanControl.setManufacturerAndModel("RiverView", "ZigbeeToHvacIR");
     zbFanControl.addAnalogOutput();
     zbFanControl.setAnalogOutputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
     zbFanControl.setAnalogOutputDescription("Fan 0-4 (5-auto, 6-silent)");
     zbFanControl.setAnalogOutputResolution(1);
     zbFanControl.setAnalogOutputMinMax(0, 6);  
     zbFanControl.onAnalogOutputChange(ha_setFan);
     //
     if (debug_g) Serial.println("Vane Selector cluster");
     zbVaneControl.setManufacturerAndModel("RiverView", "ZigbeeToHvacIR");
     zbVaneControl.addAnalogOutput();
     zbVaneControl.setAnalogOutputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
     zbVaneControl.setAnalogOutputDescription("Vane (0=Auto,1,2,3,4,5,6=move);");
     zbVaneControl.setAnalogOutputResolution(1);
     zbVaneControl.setAnalogOutputMinMax(0, 6);  
     zbVaneControl.onAnalogOutputChange(ha_setVane);
     //
     if (debug_g) Serial.println("Set mains power");
     zbPower.setPowerSource(ZB_POWER_SOURCE_MAINS); 
     zbPower.onIdentify(ha_identify);
     //
     Zigbee.addEndpoint(&zbPower);
     Zigbee.addEndpoint(&zbColdHot);
     Zigbee.addEndpoint(&zbTemp);
     Zigbee.addEndpoint(&zbFanControl);
     Zigbee.addEndpoint(&zbVaneControl);
     //
     // Create a custom Zigbee configuration for End Device with longer timeouts/keepalive
     //
     esp_zb_cfg_t zigbeeConfig =                             \
       {  .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,              \
          .install_code_policy = false,                      \
          .nwk_cfg = {                                       \
            .zed_cfg =  {                                    \                                                          
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_64MIN, \
                .keep_alive = 60000,                         \
              },                                             \
          },                                                 \
       };
     //
     if (debug_g) Serial.println("Starting Zigbee");
     rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);
     //
     // When all EPs are registered, start Zigbee in End Device mode
     //
     if (!Zigbee.begin(&zigbeeConfig, false)) { 
        if (debug_g) {
            Serial.println("Zigbee failed to start!");
            Serial.println("Rebooting ESP32!");
        }
        rgb_led_flash(RGB_LED_RED, RGB_LED_RED);  // Sometimes it stays orange
        rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
        ha_restart();
     }
     //
     // Now connect to network.
     //
     if (debug_g) Serial.println("Connecting to network");   
     int tries = 0;      
     while (!Zigbee.connected()) {
        rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);         // the led sets have delays built in
        delay(5000);
        if (debug_g) Serial.println("connecting..\n");
        if (tries ++ > 360) {                              // Maximum 30 minutes trying    
           if (debug_g) {
               Serial.println("Zigbee failed to connect!");
               Serial.println("Rebooting ESP32!");
           }
           rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);  // We tried for 30 minutes, restart.
           rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
           ha_restart();   
        }
     }
     rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);   
     if (debug_g) Serial.println("Successfully connected to Zigbee network");
     //
     // Try to get the heat pump connected via serial port and install callbacks.
     // If it connects we will ge the current configuration and synch it with zibgee to
     // HA.
     //
     hp_setup();
}

//
// In the loop we just process any pending requests, the requests came via the callbacks.
// We use a watch dog timer to make sure we get into this loop at least every 10 minutes or so.
// If we don't get here in time we will get a hardware reset.
//
void loop()
{    static int ix = 0;            // Loop counter 0..4 for LED on/of flash choice.
     esp_task_wdt_reset();         // Feed the watch dog
     //
     if (debug_g) Serial.println("loop()");
     //
     if (!Zigbee.connected()) {
         if (debug_g) Serial.println("zigbee disconnected while in loop()- restarting");
         ha_restart();   
     }
     //
     // if we have comms with heat pump sync anything to/from the heat pump.
     //
     int status_color = RGB_LED_WHITE;
     if (hp.isConnected()) {
         hp.sync();
         status_color = RGB_LED_GREEN;
     }
     //
     // Alternate GREEN|WHITE/BLACK every loop instance. This is green for one second every 10 seconds or so means all good.
     // If we get WHITE it means zigbee is up but serial is not connected.
     //
     rgb_led_set(ix != 0 ? RGB_LED_OFF : status_color);
     ix = (ix + 1) % 5;
     // 
     // Do any pending work for about a second if there is some.
     //
     ha_processPending();
     delay(1000);
}
