# A NMEA2000 Engine inerface with BLE

This is a NMEA2000 interface to an marine diesel engine built arround a ESP32-C3 and ADS1115 16 bit ADCs.

NB: All paths in this file are relative to the root of the git repo.

The curcuit is defined in pcb/esp32board/esp32board.kicad_sch

# Peripherals

* CAN bus connection is using the ESP32 CAN interface, driven by the NMEA2000 from ttlappalainen and a TWAI driver for the same. CAN-TX GPIO3, CAN-RX GPIO1
* 3x ADC1115 I2C 16bit ADCs  running at 5V via a level shifter.
	* ADC1 Adress 1001000b (Addr pin GND) 
		* AIN0 - NTC1 10K NTC  5V 4K7 Pullup
		* AIN1 - NTC2 10K NTC  5V 4K7 Pullup
		* AIN2 - NTC3 10K NTC  5V 4K7 Pullup
		* AIN3 - Alternator Voltage Divider 10K/4K7
	* ADC2 Address 1001001b (Addr pin Vdd)
		* AIN0 - Oil Pressure 0-5V Sensor
		* AIN1 - Coolant 12V 1K Pullup
		* AIN2 - Engine Block, see below 
		* AIN3 - Engine Battery, Voltage Divider 10K/4K7 Voltage Divider 10K/4K7
	* ADC3 Address 1001011b (Addr pin SCL)
		* AIN0 - NTC4 10K NTC  5V 4K7 Pullup
		* AIN1 - NTC5 10K NTC  5V 4K7 Pullup
		* AIN2 - Fuel 10K NTC  5V 1K Pullup
		* AIN3 - V sense, spare Voltage Divider 10K/4K7

* Flywheel Pulse on GPIO4, 30 pulses per revolution
* Stop Relay Coil 10K/2K2+LED Divider
* Start Relay Coil 10K/2K2+LED Divider
* Glow Relay Coil 10K/2K2+LED Divider
* 1 Wire GPIO10

# GPIO Connections

* GPIO1 CAN-RX
* GPIO3 CAN-TX
* GPIO4 Flywheel Pulse
* GPIO5 StopCoil
* GPIO6 StartCoil
* GPIO7 GlowCoil
* GPIO8 I2C SDA 
* GPIO9 I2C SCL
* GPI10 1 Wire

# ADC Voltage Dividers

The ADS1115 will be running at 64sps, giving a Nyquist frequency of 32Hz due to Sigma Delta, so no RC used in Dividers. 

12V

    12V  ----
            |
           10K
            |---- 2K2 --- ADC Pin
            4K7
            |
     0V -----

NTC

    5V ------
            |
           4K7
            | --- 2K2 --- ADC Pin
            NTC
            |
    0V ------

Coils

    12V Coil ----
                |
                10K
                | --- 2K2 --- ADC Pin
                2k2
                |
                LED
                |
     0V ---------

Engine Block - -ve relative to 0V

     5V ----------
                 |
                10K 
                 | --- 2K2 --- ADC Pin
                10K
                 |
    Engine Block -         


# NTC Mappings

* NTC1 == Exhaust Water trap
* NTC2 == Alternator
* NTC3 == Engine Room
* NTC4 == Gearbox
* NTC5 == Exhaust Elbow

# PGNS

* 127488L, // Rapid engine ideally 0.1s
* 127489L, // Dynamic engine 0.5s  alternator Temperature sent as engineOilTemperature
* 127505L, // Tank Level 2.5s
* 130316L, // Extended Temperature 2.5s
* 127508L, 
