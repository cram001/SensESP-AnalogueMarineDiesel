# SensESP-Analogue to Digital converter for marine diesel engine

Designed for a Yanmar 3JH3E, but could easily be adapted to other Yanmar engines (GM, YM) or Beta that have analogue instrumentation.

ESP32 Boat Analogue Diesel Engine ADC (analogue to digital converter).
Working on the progress made by Boatingwiththebaileys (https://github.com/Boatingwiththebaileys/ESP32-code)

Essentially, leverages an existing SignalK installation (Cerbo GX OS Large or OpenPlotter on an RPI) to use a small, efficient ESP32 board (https://www.dfrobot.com/product-2837.html) and a few additional boards to convert the engine's analogue signals to digital signals which can be sent to NMEA2000 (or NMEA0183) via SignalK.

Requires basic soldering skills as a voltage divider must be assembled on a prototype board.

You will need:
- ESP32 board with wifi (4 MB flash min, 16 MB recommended)
- Voltage divider to sense the engine's coolant temp sender
- Op Amp to sense the engine's tach sender
- Proto board to build the voltage divider
- 18-22 AWG wire for connections
- 12VDC to 5 V USB-C power adapter to power the ESP32
- A box to protect the ESP32 and sub-boards
- One wire DS18B20 temp sensors

  All components can be purchased for under USD35.

This iteration provides:
- Sense engine RPM from flywheel sensor (set to 116 teeth for 3JH3E engine, check your service manual to adjust)
    o Isolated from the engine noise via an op-amp
- Sense coolant temperature from the panel's engine coolant thermocouple, without affecting the panel gauge
    o Uses  voltage divider R1 = 220 kOhm and R2 = 100kOhm
    o Uses user provided resitance to temperature (in Kelvins) curve ... currently set for US temp sender resistance
- Optionally uses up to 3 onewire temp sensors (ex: exhaust elbow, engine compartment, etc... with user assignable paths)
- Calculates fuel flow based on RPM with user provided fuel curve (in m3/sec)
- User selectable wifi ssid and passphrase
- Basic config over webui
- OTA updates via webui
- Data input smoothing and out of range input values handling
- Persistence storage of user configured settings and values, along with OneWire sensor ID.
- Updated to use SensESP v3.x and OneWire v3.x
  

Installation:
1. Install VSCode
2. Install platformio
3. Download the code from gitub
4. Update your coolant sender ohm/temp values (change to European sender values if required)
5. Update your RPM to fuel curve (you'll likely need to convert L/hr or Gal/hr to m3/sec)
6. Confirm your voltage divider uses the specified R1 and R2 values
7. Build and install the code
8. After booting up and connecting sensors, Access the ui via web browser by connecting directly the wifi accesspoint created by the ESP32
   
     a. 192.168.4.1 i
   
     b. Configure wifi

     c. Once connected to your wifi access point, the ui should be available via:  https://sensesp.local or myhostname.local if you have changed myhostname

     d. Assign your signalk server IP and port

     e. Assign signalk paths to the connected OneWire senders (it should auto detect and list all connected senders)

     f. Click restart
   
10. Access your SignalK server and assign the paths to NMEA2K variables
11. Setup your instruments to display the desired data collected via the ESP32

Future updates, add compatibiltiy for:
- oil pressure sender
- user selectable alternator or flywheel tach sender, along with number of flywheel tooth via webui
- user selectable pre-defined US or European temp sender curve and US or European oil pressure sender curve
- User input of fuel consumption / rpm curve via web ui


Recommended Hardware

Currently (Nov 2025) the code was developped and tested on a FireBeetle 2 ESP32-E (4MB Flash). However, to make space for future functionality, the 16MB flash version is highly recommended (FireBeetle 2 ESP32-E (N16R2))

