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
