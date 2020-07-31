# fan-controller
A temperature driven fan controller with a Nextion touchscreen display and Arduino reading contact and non-contact temperature sensors.

I used this to control a fan that cools a servo motor used as a spindle motor on a CNC mill.


# Bill of materials

Below are links to the items that I used. Most of these are Amazon Affiliate links.

## Arduino
I prefer the Arduino Uno with a socketed DIP since it's easily repairable, but the SMD version is a little less expensive. The bare shield is a convenient for mounting all.

[Arduino Uno R3, DIP version](https://amzn.to/2Ekh8hU)

[Arduino Uno R3, SMD version](https://amzn.to/3jF4XfJ)

[Arduino Uno R3 Bare Shield](https://amzn.to/3hA0Lfq)

[Straight Headers](https://amzn.to/2ZXLKOt)



## Power conversion
A Murata DC-DC converter supplies 12V to the Arduino Uno from the 24V source that also powers the fan. (The Arduino Uno's regulator supplies the display and sensors.) To reduce height, the converter should be mounted flush with the shield. The W36H model is easiest to mount, but the W36 model can be mounted to a right-angle header.

[Murata DC-DC Converter OKI-78SR-12/1.0-W36H-C](https://www.digikey.com/product-detail/en/murata-power-solutions-inc/OKI-78SR-12-1-0-W36H-C/811-3294-ND/6817699)

or 

[Right-angle Headers](https://amzn.to/39v0EPC)

[Murata DC-DC Converter OKI-78SR-12/1.0-W36-C](https://www.digikey.com/product-detail/en/murata-power-solutions-inc/OKI-78SR-12-1-0-W36-C/811-3293-ND/6817698)



## Display
The GPIO board is not required, but will useful if you wish to use the Nextion display for other projects. The display can generate a PWM signal, which might control the fan. But, reading the fan tach or the temperature sensors may not be possible with the display.

[Nextion Enhanced NX4832K035](https://amzn.to/2BzLn3o)

[Nextion GPIO Breakout](https://amzn.to/300HcXz)



## Temperature sensor
The DS18B20 and OTI-301 were used. The OTI-301 has a response curve in its manual, but the MLX90614ESF has a richer datasheet. The OTI-301 is also harder to find than the MLX90614ESF.

[DS18B20-based Contact Temperature Sensor](https://amzn.to/32WuU4u)

[OTI-301-based Non-contact Temperature Sensor](https://amzn.to/32WzyzE)

[MLX90614ESF-based Non-contact Temperature Sensor](https://amzn.to/2CQswS5)



## Fan
I used a 24V fan because there was an existing 24V supply. Could use a 12V computer fan instead. The Noctua fans come with pain extensions or extensions with built-in resistors. Either can be modified to serve as a pigtail from the Arduino to the fan.

[Noctua NF-F12 industrialPPC-24V-3000 Q100 IP67 PWM](https://amzn.to/3g4mzQ6)
