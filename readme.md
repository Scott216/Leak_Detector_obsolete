This project detects water and sends out  Tweet if water is detected.  The main boards is an Arduino Leonardo with an Ethernet Shield.  The moisture sensors are hard-wired to the PCB.  When hard wiring is not practical, it uses a couple of Panstamps which provide wireless communication.  A Panstamp is basically and Arduino Pro-mini with a wireless radio.  

I chose Leonardo because I needed an Arduino without the headers installed.  The Ethernet shield is actually soldered to the Leonardo with the Ethernet Shield's pins sticking through the Leonardo.  They are still plenty long enough so the Leonardo/Ethernet sandwich can be plugged into the headers on the PCB.

To make the water detector sensor, take a dried out sponge and insert two 14 GA solid copper wires, about an inch apart.  I like to cut copper wire at an angle so it forms a sharp point - this makes it easier to insert into the sponge.  Use small gauge stranded wire to connect the sponge to terminal strip. When sponge is dry it appears as an open circuit. When wet it's varies from 30k to 300k ohms.  I have [sample op-amp circuit](https://www.circuitlab.com/circuit/ayhr2v/water-detector-v2/) circuitlab.com.

Use a MCP6004 Op Amp. Need the Op-amp because the impedance of the sponge is high and you get false readings if you wire directly to an Arduino. I'm using the op amp as a comparator.


Main board leak detector BOM

    1x [OKW Enclosure B6504112](http://www.okwenclosures.com/products/okw/railtec-b.htm) Realtec 105 $9.77. 
        PCB Board specs 3.87" x 3.22". position screw terminal holes 0.27" from edge
        2x M2.5 x 6mm Screws to mount PCB to OKW Enclosure
        2x 18 position plug header, PCB Side CTB9358/18AO $2.14
        2x 18 Position screw terminal CTB9208/18 $5.84 
    1x Arduino [Leonardo](http://arduino.cc/en/Main/arduinoBoardLeonardo) - without headers
    1x [Ethernet Shield](http://arduino.cc/en/Main/ArduinoEthernetShield)
    1x [panStamp](http://www.panstamp.com/products/wirelessarduino)
    1x JST PH 5-position header P/N B5B-PH-K-S, Digikey 455-1707-ND for OLED Display, (JST Data Sheet)
    1x JST PH 5-wire cable Sparkfun PRT-09917, JST # PHR-5, Crimp pin SPH-002T-P0.5S
    1x JST PH 4-position header P/N B4B-PH-K-S, Digikey 455-1706-ND for LEDs
    1x JST PH 4-wire cable Sparkfun PRT-09916, JST # PHR-4, Crimp pin SPH-002T-P0.5S
    1x JST PH 2-position header P/N B2B-PH-K-S, Digikey 455-1704-ND for on-off switch
    1x JST PH 2-wire cable Sparkfun PRT-09914, JST # PHR-2, Crimp pin SPH-002T-P0.5S
    1x OLED 128 x 32 I2C Display, Adafruit 931
    1x Reed Relay, Hamlin HE751A0510, Digikey HE124-ND
    3x MCP6004 Op amp Digikey MCP6004-I/P-ND
    3x Resistor network 1MΩ Bourns P/N 4606X-101-105LF, Digikey 4606X-1-105LF-ND. (cut off lead on pin 6)
    2x 330Ω 1/4 watt resistors
    2x 4.7kΩ 1/4 watt resistors
    2x 68Ω 1/4 watt resistors
    1x 150Ω 1/4 watt resistor
    1x 10k trimmer Pot,Digikey 490-2856-ND
    1x On-Off Switch, CW Industries P/N GPB040B05BR, Digikey SW632-ND
    1x Yellow LED 2 volts, 20 mA, Bivar P/N PM3YDW12.0, Digikey 492-1617-ND
    1x Red LED 2.1 volts, 20 mA, Bivar P/N PM3RDW12.0, Digikey 492-1616-ND
    1x Green LED 2.1 volts, 20 mA, Bifar P/N PM3GDW12.0, Digikey 492-1612-ND
    1x 10-pin Header for Leonardo, Digikey S7043-ND
    2x 8-pin Header for Leonardo, Digikey S7041-ND
    1x 6-pin Header for Leonardo, Digikey S7039-ND
    2x 12-pin Headers for panStamp, Digikey S7045-ND 


Remote Wireless Detector BOM

    1 MCP601 Op Amp Digikey MCP601-I/P-ND
    8 pin DIP socket Sparkfun PRT-07937
    2 Battery Holders, Digikey 708-1412-ND
    2 CR123 batteries
    1 470k ohm resistor
    1 Panstamp
    1 slide switch Sparkfun COM-00102
    2 5.6 ohm resistor for green & blue led in RGB LED
    1 68 ohm resistor for for red in RGB LED
    1 RGB LED Sparkfun COM-00105
    1 Pushbutton switch, Sparkfun COM-00097
    1 OneWire temp sensor DS18B20
    2 12-pin headers for PanStamp 