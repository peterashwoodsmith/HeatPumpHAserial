//
// This ESP32 ARDUINO program is a Zibgee end device that will receive commands to be translated into serial for the 
// MISTUBISHI HEATE PUMP. THis is work in progress for a Home Assistant end point that you attach with the serial CT105 port
// on the heat pump. I now have a few of these running for several weeks (as of Jan 2026) with no issues so far although
// the Zibgee icons and choice of clusters needs improvement but what you can do through the Zigbee object is quite limited
// on ESP32 and really this should be recoded using the lower level zigbee libraries provided by Espresif. If anybody
// wants to try this by all means do so and I'll happily update this code and use it myself.
//
// The code has two major parts. The first is the excellent serial HeatPump object which comes from another author (see below).
// Following that is the zibgee 3.0 Espressif Arduino code to create the end points and their controls (clusters) that select
// the various parameters for the heat pump. I have copied the HeatPump.cpp and HeatPump.h from the SwiCago repository to 
// avoid any conflicts with future changes and for ease of references.
//       
//     PARTA     https://github.com/SwiCago/HeatPump/tree/master       (c) 2017 Al Betschart
//     PARTB     https://github.com/peterashwoodsmith/HeatPumpHA       (c) 2026 Peter Ashwood-Smith 
//
// HARDWARE:
//          Waveshare ESP32-C6 but should work on any ESP32 that supports Zibgee and has an RGB Led for status indication.
//          You need to add a single push button for the factory reset option, any momentarily on push button will work.
//          Since the HeatPump uses 5V for its serial port and the ESP32 uses 3.3v a level shifter board with at least 
//          two channels (TX/RX) are required. 
//
//          The level shifter board takes 5V and Ground on its High Voltage side and 3.3V and Ground on its low voltage
//          side. The Serial TX and RX are connected to two free pins on the low voltage side, i.e. LV1, LV2 while the
//          corresponding high voltage sides are connected to the CT105 corresponding pins for RX and TX (i.e. TX->RX).
//          Power from the CT105 (5V) can drive the 5V pin on the ESP32-C6 and of course CT105 ground pin to ESP32-C6 Ground.
//          The CT105 12v wire should of course be properly covered off and not used. Some people are able to run without
//          the level shifters but its smarter to use the proper voltages on the serial port for the Heat Pump to avoid
//          lost data or at worse damage.
//
//          The reset button is connected to GPIO18 and Ground, this is convenient because they are one pin separated
//          so you can just solder a reset button between them but any I/O pin and ground should work. This is used to trigger a 
//          factory reset, for a normal reset just power cycle the board.
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
//              2 - Tools/Core debug level (set as desired useful for debugging zibbee attach etc.) start with verbose.
//              3 - Tools/Erase all flash before upload - this means each download its a brand new Zibgee end point factory refresh.
//                  Id erase for first few downloads and always delete/reattach but after its working don't erease the
//                  flash each time. Once its working you can erase the flash and start scratch by simply pressing the
//                  reset button anyway.
//              3 - Tools/partition scheme: 4MB with spiffs - seems to be what the Zibgee library wants.
//              4 - Tools/zibgee mode ED (end device) - you can also use the end mode with debug enabled for more tracing.
//
// SOFTWARE:
//          There is a debug variable you can set true/false for serial I/O debugging. Start true turn off later.
//          On initial startup the code will start a watch dog timer just in case. This will cause a panic reset if the main
//          talk to HeatPump function is not entered frequently enough indicating something got stuck. The main loop() of course 'feeds' this watch
//          dog but only in the main work part of the loop with serial port connected and functioning.
//          Next we setup interrupt handlers for the reset button. Basically if reset is pressed we isue a reset to the ESP but we
//          also we erase the non volatile memory which will cause the end point to forget any ziggee related information and it
//          will have to rebind. If you do this you need to
//          delete the device from home assistant and go back into zigbee discover mode to redescover this end point. For a normal
//          non factory reset just power cycle or use the ESP32 reset button.
//          We use the RGB Led to indicate whats going on. Red flashing as it first boots, orange as its getting ready to attach to the
//          zigbee network and then blue flashing as its trying to bind with the zibgee network and finally green flashing as its in the
//          main loop() waiting for HA to ask it to do something (also use white flashing to indicate zigbee up but no serial comms to HP).
//          In the setup() code we will create the zibbee end points one for each of the clusters/attributes we want to control.
//          These are using the Espressif Arduino interface which is high level but not well documented. Essentially you create the
//          objects and a callback to set each parameter and then bind them to zibgee and wait for it to connect. After that its all
//          callbacks.
//          In the callbacks we just remember the attribute changes which are processed later in the main loop().
//          In the main loop we look for any attribute changes and if so call hp object to send the serial commands. 
//          Note that we also take setting changes from the heap pump and feed them back to zigbee by callback. That way if the
//          IR remote makes changes it shows up in the zibgee cluster correctly. Sadly there is no way for the zigbee changes to 
//          appear on the IR remote but the Zibgee works so well you won't need the remotes anymore.
//
//          We also report a number of debuging values back as 'sensors' such as uptime, previous reboot reason etc. which can mostly
//          be ignored, but the room temperature and operating outputs are useful.
//
//  Home Assistant Notes:
//          As a Zibgee device you need to put the HA Zibgee in device search mode. If all goes well when the ESP32 boots it will flash 
//          red orange then blue and stay blue until you see Home Assistant show the device has connected etc. at which point it 
//          will flash green. Once its bound you can reset the ESP32 via power etc. and it will reattach fast and remember what ever
//          name you gave it. Initial binding can take a few minutes. Any problems just press your reset button to erase the NVS and delete the 
//          device from HA and go back into device discorvery on HA to let it rebind. Sometimes being close to the HA/Zibbee dongle
//          helps so if its cause problem, erase the NVS, delete the device from HA and move close to your dongle to retry. I suspect 
//          the dongles and HA notice devices trying to attach too frequently and back off a bit so sometimes waiting a few minutes to
//          retry gives you a quik attach.
//
//          Sadly the cluster attributes I've chosen are not the best in terms of icons/units etc. I'm sure its possible to pick really
//          perfect cluster attributes so that HA displays perfect icons/colors etc. but I gave up on the zibgee side and just do it in 
//          the YAMAL side. I've included some example YAML code for my system which shows the on/off always but when off most of the 
//          clusters are invisible. The heat cold cluster requires double click to avoid accidently setting the wrong mode. There are no
//          doubt many great ways to do this using HA dashboards that I'm not aware of so by all means improve and feed back.
// 
//  TODO:
//          Recode using the Espressif lower level Zibgee libraries and get better attribute mappings etc.    
//
//  NOTES:
//          I make no claims that this even works, or that your entire house won't explode if you use it. Nobody should ever use this 
//          for anything serious ever. This is for educational purposes only. Serious harm could come to your expensive heat pump if any
//          miswiring, code failure, or other issues occur and this almost certainly voids your warranty plugging anything into the CT105 that 
//          is not officially from the mfgr.
//
//          Enjoy. Peter Ashwood-Smith and with thanks to Al Betschart for the superb HeatPump object code which worked perfectly first time
//          and I've not had to change a single line of code. I include it in the Git for this project for consistency in case his code 
//          changes in the future.
//
#include <esp_task_wdt.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif
#include "Zigbee.h"
#include "HeatPump.h"
#include "esp_log.h"

//
// Output unitless count app type missing so define it.
//
#define ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT  ESP_ZB_ZCL_AO_SET_APP_TYPE_WITH_ID( ESP_ZB_ZCL_AO_APP_TYPE_COUNT_UNITLESS, 0x0000)
//
// Debugging stuff, simple macro to log debug for us.
//
static const char *TAG = "HP2ZS"; 
#define DPRINTF(format, ...)  ESP_LOGD(TAG, format, ##__VA_ARGS__) 

//
// Set 1 and you'll get lots of useful info as it runs. For debugging the lower layer Zibgee see the tools settings
// in the Arduino menu for use with the debug enabled library and debug levels in that core. We can also compile in/out 
// the watch dog timers. 
//
const bool debug_g = false;
const bool wdt_g   = true;

// 
// Non volatile storage for debugging. When we restart etc. we will write the reasons 
// and track last uptime etc. for display via a Zigbee debug cluster sensor.
//
const char       *ha_nvs_name = "_HP2MIS_";                // Unique name for our partition
const char       *ha_nvs_vname= "_vars_";                  // name for our packeed variables
nvs_handle_t      ha_nvs_handle = 0;                       // Once open this is read/write to NVS
uint32_t          ha_nvs_last_uptime = 0;                  // minutes we were up last time before reboot
uint32_t          ha_nvs_last_reboot_reason = 0;           // why we rebooted last time. (0 factory reset)
uint32_t          ha_nvs_last_reboot_count = 0;            // increase each reboot except factory reset

//
// We are looking for persistant values of the last reboot reason and last uptime. We store these two packed
// into a single Uint32 which we depack after reading from the NVS.
//
void ha_nvs_read()
{    
     ha_nvs_last_reboot_reason = 0;
     ha_nvs_last_uptime        = 0;
     ha_nvs_last_reboot_count  = 0;
     //
     esp_err_t err = nvs_flash_init();
     if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        nvs_flash_erase();
        nvs_flash_init();
        if (debug_g) DPRINTF("ha_nvs_read - nvs_flash_init\n", esp_err_to_name(err));
     }
     err = nvs_open(ha_nvs_name, NVS_READWRITE, &ha_nvs_handle);
     if (err != ESP_OK) {
        if (debug_g) DPRINTF("ha_nvs_read - Error (%s) opening NVS name %s!\n", esp_err_to_name(err), ha_nvs_name);
        return;
     }  
     //
     uint32_t vars;
     err = nvs_get_u32(ha_nvs_handle, ha_nvs_vname, &vars);
     if (err != ESP_OK) {
          if (debug_g) DPRINTF("ha_nvs_read - cant get variable name %s\n", ha_nvs_name);
          return;
     }
     ha_nvs_last_reboot_reason  = vars         & 0xff;
     ha_nvs_last_reboot_count   = (vars >> 8)  & 0xff;
     ha_nvs_last_uptime         = (vars >> 16) & 0xffff;
     ha_nvs_last_reboot_reason += esp_reset_reason() * 1000;  // See below for why * 1000
     /* 
      * For convenient reference. We multiply these by 1000 to we can see the 
      * ESPs idea why it rebooted together with our own reboot reason as a single
      * number displayed as a Zigbee cluster. This is taken from the enum so they 
      * start at 0,1,2... 
      *
      * ESP_RST_UNKNOWN,    //!< Reset reason can not be determined
      * ESP_RST_POWERON,    //!< Reset due to power-on event
      * ESP_RST_EXT,        //!< Reset by external pin (not applicable for ESP32)
      * ESP_RST_SW,         //!< Software reset via esp_restart
      * ESP_RST_PANIC,      //!< Software reset due to exception/panic
      * ESP_RST_INT_WDT,    //!< Reset (software or hardware) due to interrupt watchdog
      * ESP_RST_TASK_WDT,   //!< Reset due to task watchdog
      * ESP_RST_WDT,        //!< Reset due to other watchdogs
      * ESP_RST_DEEPSLEEP,  //!< Reset after exiting deep sleep mode
      * ESP_RST_BROWNOUT,   //!< Brownout reset (software or hardware)
      * ESP_RST_SDIO,       //!< Reset over SDIO
      * ESP_RST_USB,        //!< Reset by USB peripheral
      * ESP_RST_JTAG,       //!< Reset by JTAG
      * ESP_RST_EFUSE,      //!< Reset due to efuse error
      * ESP_RST_PWR_GLITCH, //!< Reset due to power glitch detected
      * ESP_RST_CPU_LOCKUP, //!< Reset due to CPU lock up (double exception)
      */
     if (debug_g) {
          DPRINTF("ha_nvs_read got vars=%x, reason %d, count %d, uptime=%d\n", vars,
               ha_nvs_last_reboot_reason, ha_nvs_last_reboot_count, ha_nvs_last_uptime);
     }
}

//
// And here is the write to NVS of the attributes after they have been changed and sent to the Heat Pump
//
void ha_nvs_write(uint32_t reason = 0, uint32_t uptime = 0)
{
     ha_nvs_last_reboot_count = (ha_nvs_last_reboot_count + 1) & 0xff;
     reason &= 0xff;
     uptime &= 0x0000fffff;
     uint32_t vars  = reason | (ha_nvs_last_reboot_count << 8) | (uptime << 16);
     if (debug_g) {
          DPRINTF("ha_nvs_write got vars=%x, reason %d, count %d, uptime=%d\n", vars, reason, ha_nvs_last_reboot_count, uptime);
     }
     esp_err_t err = nvs_set_u32(ha_nvs_handle, ha_nvs_vname, vars);
     if (err != ESP_OK) {
         if (debug_g) DPRINTF("ha_nvs_write  %s can't write, because %s\n", ha_nvs_vname, esp_err_to_name(err));
         return;
     }
     err = nvs_commit(ha_nvs_handle);
     if (err != ESP_OK) {
         if (debug_g) DPRINTF("ha_nvs_write %s can't commit, because %s\n", ha_nvs_vname, esp_err_to_name(err));
     }  
}
// 
// Function complete shutdown and restart. Forward declared also a flash sequence for factory reset.
//
extern void ha_restart(uint32_t reason, uint32_t uptime); 
extern void rgb_led_set_factory_reset();

//
// Interrupt handler for Reset button. If its pressed we do full factory reset. Normal reset is just done with a power off/on.
//
void isr_resetButtonPress()
{      
     rgb_led_set_factory_reset();                       // Go white so its obvious
     Zigbee.factoryReset(false);                        // This should do the same but not sure it does anyway ...
     ha_restart(0, 0);                                  // And stop all the Zigbee stuff and just restart the ESP
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
// If the task watch dog times out, rather than use the system handler it will come here and we do a nice
// controlled reboot and keep track of the reason and how long we were up for better debugging.
//
void esp_task_wdt_isr_user_handler(void)
{
     ha_restart(1, millis()/1000); 
}

//
// Serial Interface libarary to the Heat Pump. The hp_send.. function just translates between the zigbee values for the various clusters and what
// the HeatPump methods take. Sadly its not that consistent because the Zigbee code only seems to be easy to do with numbers and not enumerated types.
// Converting the zibgee clusters to proper enumerations is FFS.
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
            "|"      /* Air direction (horizontal): <<, <, |, >, >>, <>, or SWING */
     };
     //
     // And set the desired settings and request an update to the HP.
     // Avoid doing this if we are not connected.
     //
     if (hp.isConnected()) {
         hp.setSettings(settings);
         hp.update();
     }
}

//
// Connect to the heat pump over serial port. We will install the callbacks for status changes. We
// Use the enable external updates so that changes made by the remove will show up in the zibgre 
// values in home assistant.
//
void hp_setup() {
     extern void hp_settingsChangedCallback();
     extern void hp_statusChangedCallback(heatpumpStatus);
     hp.enableExternalUpdate();
     hp.setSettingsChangedCallback(hp_settingsChangedCallback);
     hp.setStatusChangedCallback(hp_statusChangedCallback);
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
// These are the EP sensors clusters. 
//
ZigbeeBinary      zbSerial      = ZigbeeBinary(15);      // Serial connection status
ZigbeeBinary      zbOperating   = ZigbeeBinary(16);      // HP working to get desired temperature
ZigbeeAnalog      zbRoomTemp    = ZigbeeAnalog(17);      // Temperature of room as seen by HP
//
ZigbeeAnalog      zbRebootReason= ZigbeeAnalog(18);      // reason for last reboot
ZigbeeAnalog      zbLastUptime  = ZigbeeAnalog(19);      // How long it was up last time before reboot
ZigbeeAnalog      zbRebootCount = ZigbeeAnalog(20);      // How many reboots since factory reset
ZigbeeAnalog      zbUptime      = ZigbeeAnalog(21);      // Seconds since last reboot.
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
     zbSerial.setBinaryInput(hp.isConnected());
     zbOperating.setBinaryInput(hp.getOperating());
     zbRoomTemp.setAnalogInput(hp.getRoomTemperature());
     zbRebootReason.setAnalogInput(ha_nvs_last_reboot_reason);
     zbLastUptime.setAnalogInput(ha_nvs_last_uptime);
     zbRebootCount.setAnalogInput(ha_nvs_last_reboot_count);
     zbUptime.setAnalogInput(millis()/1000);
     //
     zbVaneControl.reportAnalogOutput();
     zbFanControl.reportAnalogOutput();
     zbTemp.reportAnalogOutput();
     zbColdHot.reportBinaryOutput();
     zbPower.reportBinaryOutput();
     zbSerial.reportBinaryInput();
     zbOperating.reportBinaryInput();
     zbRoomTemp.reportAnalogInput();
     zbRebootReason.reportAnalogInput();
     zbLastUptime.reportAnalogInput();
     zbRebootCount.reportAnalogInput();
     zbUptime.reportAnalogInput();
}
//
// Keep HA up to date with any changes that happen on the heat pump from the serial updates.
//
void ha_sync_nvs_clusters()
{
     zbRebootReason.setAnalogInput(ha_nvs_last_reboot_reason);
     zbLastUptime.setAnalogInput(ha_nvs_last_uptime);
     zbRebootCount.setAnalogInput(ha_nvs_last_reboot_count);
     zbUptime.setAnalogInput(millis()/1000);
     //
     zbRebootReason.reportAnalogInput();
     zbLastUptime.reportAnalogInput();
     zbRebootCount.reportAnalogInput();
     zbUptime.reportAnalogInput();
}

//
// The heat pump has reported a change in status, probably room temperature reading, or operating reading so we just synch back to
// HA.
//
void hp_statusChangedCallback(heatpumpStatus s)
{
     ha_sync_status();
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
                 if (debug_g) DPRINTF("Invalid power status =%d\n", ha_coldHotStatus);
                 return;
     }
     //
     int hv_powerOff  = ha_powerStatus ? 0 : 1;        // its an off flag to the UI so reversed from HA.
     int hv_temp      = ha_tempStatus;
     int hv_fanMode   = ha_fanStatus;
     int hv_vanneMode = ha_vaneStatus;
     //
     if (debug_g) DPRINTF("*** SEND HVAC COMMAND: mode=%d, temp=%d, fan=%d, vane=%d, off=%d ***\n",
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
     DPRINTF("POWER STATUS     = %s\n", ha_powerStatus ? "ON" : "OFF"); 
}
//
void ha_displayFanStatus()
{     
     DPRINTF("FAN STATUS       = %d\n", ha_fanStatus);
}
//
void ha_displayVaneStatus()
{
     if ((ha_vaneStatus < 0)||(ha_vaneStatus > 6)) {
        DPRINTF("VANE STATUS      = %d ????\n", ha_vaneStatus);
     } else {
        DPRINTF("VANE STATUS      = %d\n", ha_vaneStatus);
     }
}
//
void ha_displayColdHotStatus()
{    
     DPRINTF("COOL/HEAT STATUS = %s (%x)\n", ha_coldHotStatus ? "HEAT" : "COOL", (unsigned int) ha_coldHotStatus);
}
//
void ha_displayTempStatus()
{    
     DPRINTF("TEMP STATUS      = %d\n", ha_tempStatus);
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
         if (debug_g) { DPRINTF("HA=> "); ha_displayFanStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setPower(bool value)
{
     if (ha_powerStatus != value) {
         ha_powerStatus = value;
         if (debug_g) { DPRINTF("HA=> "); ha_displayPowerStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setColdHot(bool state)
{
     if (ha_coldHotStatus != state) { 
         ha_coldHotStatus = state;
         if (debug_g) { DPRINTF("HA=> "); ha_displayColdHotStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setTemp(float value)
{    
     if (ha_tempStatus != (int) value) {
         ha_tempStatus = value;
         if (debug_g) { DPRINTF("HA=> "); ha_displayTempStatus(); }
         ha_update_t = millis() + 5000;
     }
}
//
void ha_setVane(float value)
{
     if (ha_vaneStatus != (int) value) {
         ha_vaneStatus = value;
         if (debug_g) { DPRINTF("HA=> "); ha_displayVaneStatus(); }
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
// We get called 5 or 6 times with x = 5, 4, ... down to 0. We only triggers the flashing on the 0 call.
//
void ha_identify(uint16_t x)
{
     if (debug_g) DPRINTF("******** HA => IDENTIFY(%d) ******\n", (int) x);
     //
     if (x == 0) {
        rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
        delay(500);
        rgb_led_flash(RGB_LED_GREEN, RGB_LED_GREEN);
        delay(500);
        rgb_led_flash(RGB_LED_WHITE, RGB_LED_WHITE);
        delay(500); 
        rgb_led_set(RGB_LED_GREEN);
     }
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
              if (debug_g) DPRINTF("zigbee disconnected while in ha_processPending()- restarting\n");
              ha_restart(2, millis()/1000);   
        }
        delay(50);
        //
        // If synch required then send the IR now. Wonder if there is problem with mutual exclusion and
        // callback functions. Are they in same thread? If not this variable is volatile.
        //
        if ((ha_update_t > 0) && (millis() > ha_update_t)) {
            ha_update_t = 0;
            if (debug_g) {
                DPRINTF("----------------------------- wait for HA  msgs ------------------------------------\n");
                ha_displayPowerStatus();
                ha_displayColdHotStatus();
                ha_displayTempStatus();
                ha_displayFanStatus();
                ha_displayVaneStatus();
                DPRINTF("Synch with HVAC required\n");
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
void ha_restart(uint32_t reason, uint32_t uptime)
{  
     ha_nvs_write(reason, uptime);        // remember why we are restarting so it can be shown in HA next time
     rgb_led_set(RGB_LED_OFF);            // Sometimes gets stuck on, don't know why perhaps timing.      
     delay(100);
     rgb_led_set(RGB_LED_OFF);            // So do it twice .
     delay(100);
     if (debug_g) DPRINTF("Restarting...\n"); 
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
         esp_log_level_set(TAG, ESP_LOG_DEBUG);
         DPRINTF("RiverView S/W Zibgee 3.0 to Mistubishi Serial");
     }
     //
     // We get debug information from last reboot (uptime and reboot reason etc.)
     ha_nvs_read();
     //
     // Get everything back to square one, we don't always power reset and I'm not convinced these get reset
     // as globals when a panic restart happens.
     //
     ha_update_t      = 0;
     ha_powerStatus   = 0;    // powered on/off
     ha_coldHotStatus = 0;    // heating or cooling mode
     ha_fanStatus     = 0;    // fan position or movement
     ha_tempStatus    = 0;    // desired temperature
     ha_vaneStatus    = 0;    // how the vanes move or don't
   
     //
     // Watch dog timer on this task to panic if we don't get to main loop regulary.
     //
     if (wdt_g) {
         static const esp_task_wdt_config_t wdt_config = {                  // MUST BE CONST!!
              .timeout_ms = 10 * 60 * 1000,                                 // 10 minutes max to get back to main loop()
              .idle_core_mask = (1 << CONFIG_FREERTOS_NUMBER_OF_CORES) - 1, // Bitmask of all cores
              .trigger_panic = true };                                      // no panic, just restart
         esp_task_wdt_reconfigure(&wdt_config);
         esp_task_wdt_add(NULL);
         esp_task_wdt_status(NULL);
     }
     //
     rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
     //
     // enable interrupts for reset buttons.
     //
     isr_setup();
     //
     // Add the zibgee clusters (buttons/sliders etc.)
     //
     const char *MFGR = "RiverView";    // Because my home office looks out over the ottwawa river ;)
     const char *MODL = "Z2MS004";      // Zigbeee 2 Mitsubishi Serial - device 001, 002, 003 etc.
     //
     if (debug_g) DPRINTF("On of Power switch cluster\n");
     zbPower.setManufacturerAndModel(MFGR,MODL);
     zbPower.addBinaryOutput();
     zbPower.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbPower.setBinaryOutputDescription("Off => On");
     zbPower.onBinaryOutputChange(ha_setPower);
     //
     if (debug_g) DPRINTF("Cold/Hot Switch cluster\n");
     zbColdHot.setManufacturerAndModel(MFGR,MODL);
     zbColdHot.addBinaryOutput();
     zbColdHot.setBinaryOutputApplication(BINARY_OUTPUT_APPLICATION_TYPE_HVAC_OTHER);
     zbColdHot.setBinaryOutputDescription("Cool => Heat");
     zbColdHot.onBinaryOutputChange(ha_setColdHot);
     //
     if (debug_g) DPRINTF("Temp Selector cluster\n");
     zbTemp.setManufacturerAndModel(MFGR,MODL);
     zbTemp.addAnalogOutput();
     zbTemp.setAnalogOutputApplication(ESP_ZB_ZCL_AO_TEMPERATURE_OTHER);
     zbTemp.setAnalogOutputDescription("Temperature C");
     zbTemp.setAnalogOutputResolution(1);
     zbTemp.setAnalogOutputMinMax(16, 31); 
     zbTemp.onAnalogOutputChange(ha_setTemp);
     //
     if (debug_g) DPRINTF("Fan Selector cluster\n");
     zbFanControl.setManufacturerAndModel(MFGR,MODL);
     zbFanControl.addAnalogOutput();
     zbFanControl.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbFanControl.setAnalogOutputDescription("Fan 0-4 (5-auto, 6-silent)");
     zbFanControl.setAnalogOutputResolution(1);
     zbFanControl.setAnalogOutputMinMax(0, 6);  
     zbFanControl.onAnalogOutputChange(ha_setFan);
     //
     if (debug_g) DPRINTF("Vane Selector cluster\n");
     zbVaneControl.setManufacturerAndModel(MFGR,MODL);
     zbVaneControl.addAnalogOutput();
     
     zbVaneControl.setAnalogOutputApplication(ESP_ZB_ZCL_AO_COUNT_UNITLESS_COUNT);
     zbVaneControl.setAnalogOutputDescription("Vane (0=Auto,1,2,3,4,5,6=move);");
     zbVaneControl.setAnalogOutputResolution(1);
     zbVaneControl.setAnalogOutputMinMax(0, 6);  
     zbVaneControl.onAnalogOutputChange(ha_setVane);
     //
     if (debug_g) DPRINTF("Serial cluster\n");
     zbSerial.setManufacturerAndModel(MFGR,MODL);
     zbSerial.addBinaryInput();
     zbSerial.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_HVAC_UNIT_ENABLE);
     zbSerial.setBinaryInputDescription("Connected");
     //
     if (debug_g) DPRINTF("Operating cluster\n");
     zbOperating.setManufacturerAndModel(MFGR,MODL);
     zbOperating.addBinaryInput();
     zbOperating.setBinaryInputApplication(BINARY_INPUT_APPLICATION_TYPE_HVAC_UNIT_ENABLE);
     zbOperating.setBinaryInputDescription("Operating");
     //
     if (debug_g) DPRINTF("RoomTemp cluster");
     zbRoomTemp.setManufacturerAndModel(MFGR,MODL);
     zbRoomTemp.addAnalogInput();
     zbRoomTemp.setAnalogInputApplication(ESP_ZB_ZCL_AI_TEMPERATURE_OTHER);
     zbRoomTemp.setAnalogInputDescription("Room Temp");
     zbRoomTemp.setAnalogInputResolution(0.1);
     zbRoomTemp.setAnalogInputMinMax(0, 40);
     //
     if (debug_g) DPRINTF("RebootReason cluster\n");
     zbRebootReason.setManufacturerAndModel(MFGR,MODL);
     zbRebootReason.addAnalogInput();
     zbRebootReason.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbRebootReason.setAnalogInputDescription("Last Reboot Reason");
     zbRebootReason.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("LastUptime cluster\n");
     zbLastUptime.setManufacturerAndModel(MFGR,MODL);
     zbLastUptime.addAnalogInput();
     zbLastUptime.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbLastUptime.setAnalogInputDescription("Last Uptime s");
     zbLastUptime.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("RebootCount cluster\n");
     zbRebootCount.setManufacturerAndModel(MFGR,MODL);
     zbRebootCount.addAnalogInput();
     zbRebootCount.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbRebootCount.setAnalogInputDescription("Reboot Count");
     zbRebootCount.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("This Uptime\n");
     zbUptime.setManufacturerAndModel(MFGR,MODL);
     zbUptime.addAnalogInput();
     zbUptime.setAnalogInputApplication(ESP_ZB_ZCL_AI_COUNT_UNITLESS_COUNT);
     zbUptime.setAnalogInputDescription("This Uptime s");
     zbUptime.setAnalogInputResolution(1.0);
     //
     if (debug_g) DPRINTF("Set mains power\n");
     zbPower.setPowerSource(ZB_POWER_SOURCE_MAINS); 
     zbPower.onIdentify(ha_identify);
     //
     Zigbee.addEndpoint(&zbPower);
     Zigbee.addEndpoint(&zbColdHot);
     Zigbee.addEndpoint(&zbTemp);
     Zigbee.addEndpoint(&zbFanControl);
     Zigbee.addEndpoint(&zbVaneControl);
     Zigbee.addEndpoint(&zbSerial);
     Zigbee.addEndpoint(&zbOperating);
     Zigbee.addEndpoint(&zbRoomTemp);
     Zigbee.addEndpoint(&zbRebootReason);
     Zigbee.addEndpoint(&zbLastUptime);
     Zigbee.addEndpoint(&zbRebootCount);
     Zigbee.addEndpoint(&zbUptime);
     //
     // Create a custom Zigbee configuration for End Device with longer timeouts/keepalive
     //
     esp_zb_cfg_t zigbeeConfig =                             \
       {  .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,              \
          .install_code_policy = false,                      \
          .nwk_cfg = {                                       \
            .zed_cfg =  {                                    \                                                          
                .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN, \
                .keep_alive = 3000,                          \
              },                                             \
          },                                                 \
       };
     //
     if (debug_g) DPRINTF("Starting Zigbee\n");
     rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);
     //
     // When all EPs are registered, start Zigbee in End Device mode
     //
     if (!Zigbee.begin(&zigbeeConfig, false)) { 
        if (debug_g) {
            DPRINTF("Zigbee failed to start!\n");
            DPRINTF("Rebooting ESP32!\n");
        }
        rgb_led_flash(RGB_LED_RED, RGB_LED_RED);  // Sometimes it stays orange
        rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
        ha_restart(3, millis()/1000);             // restart and remember why
     }
     //
     // Now connect to network.
     //
     if (debug_g) DPRINTF("Connecting to network\n");   
     int tries = 0;      
     while (!Zigbee.connected()) {
        rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);         // the led sets have delays built in
        delay(5000);
        if (debug_g) DPRINTF("connecting..\n");
        if (tries ++ > 360) {                              // Maximum 30 minutes trying    
           if (debug_g) {
               DPRINTF("Zigbee failed to connect!\n");
               DPRINTF("Rebooting ESP32!\n");
           }
           rgb_led_flash(RGB_LED_ORANGE, RGB_LED_ORANGE);  // We tried for 30 minutes, restart.
           rgb_led_flash(RGB_LED_RED, RGB_LED_RED);
           ha_restart(4, millis()/1000);   
        }
     }
     rgb_led_flash(RGB_LED_BLUE, RGB_LED_BLUE);   
     if (debug_g) DPRINTF("Successfully connected to Zigbee network\n");
     //
     // Try to get the heat pump connected via serial port and install callbacks.
     // If it connects we will ge the current configuration and synch it with zibgee to
     // HA.
     //
     hp_setup();
     //
     // Update the debug related information to HA.
     //
     ha_sync_nvs_clusters();
}

//
// In the loop we just process any pending requests, the requests came via the callbacks.
// We use a watch dog timer to make sure we get into this loop at least every 10 minutes or so.
// If we don't get here in time we will get a hardware reset.
//
void loop()
{    static int ix = 0;            // Loop counter 0..4 for LED on/of flash choice.
     //
     if (debug_g) DPRINTF("loop()\n");
     //
     if (!Zigbee.connected()) {
         if (debug_g) DPRINTF("zigbee disconnected while in loop()- restarting\n");
         ha_restart(5, millis()/1000);   
     }
     //
     // if we have comms with heat pump sync anything to/from the heat pump and feed the watch
     // dog. Otherwise don't feed watch dog. We can stay up and run for 10 minutes to play with
     // zibgee stuff but we'll get restarted to try to reestablish serial comms.
     //
     //
     int status_color = RGB_LED_WHITE;
     if (hp.isConnected()) {
         if (debug_g) DPRINTF("loop - hp.sync\n");
         hp.sync();
         status_color = RGB_LED_GREEN;
         //
         // And feed the watch dog because all is well, zigbee ok and serial comms ok.
         // 
         if (wdt_g) esp_task_wdt_reset();         
     } 
     //
     // Every so often (5 mins) we update the connected state to Home Assistant. Or if the
     // connected state changes update immediately. Hopefully this keeps the end point alive.
     //
     {  const unsigned long  MAX_TIME           = 60*5;
        static unsigned long last_update_time   = 0;
        static bool          last_updated_state = false;
        unsigned long        now_time           = millis() / 1000;
        bool                 now_state          = hp.isConnected();
        //
        if ((now_state != last_updated_state) || ((last_update_time + MAX_TIME) <= now_time)) {
            if (debug_g) DPRINTF("loop - issue connected cluster update %ld %d %ld %d\n", 
                                            last_update_time, last_updated_state, now_time, now_state);
            zbSerial.setBinaryInput(now_state);
            zbSerial.reportBinaryInput();
            last_update_time   = now_time;
            last_updated_state = now_state;
            ha_sync_nvs_clusters();            // Debug stuff
        }
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
     //
     // No need to buzz this loop, little pause is fine.
     //
     delay(1000);
}

/*
   Cluster ID 0x0202 Identifier: 0x0000 -   Value 00 = off, 1 = low, 2=med, 3=high, 4=on, 5=auto, 6=smart   
*/



