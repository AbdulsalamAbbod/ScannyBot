# ScannyBot
3d scanning robot based on esp8266, mpu6050, l298d driver, 4 gearbox with motors ,single encoder sensor, two 18650 batteries, and chassi.


/*NOTE*/ :- The signal of the encoder is 5v and the esp8266 is 3.3v voltage level
so make sure to add a voltage divider or a voltage level shifter or an optocupler
before connecting it ot the d1 mini board
I used an Arduino uno to give me anice signal and then used a voltage divider to convert it to 3.3v signal
It is the stupid thing I've ever done but it works :), you just use a voltage level shifter and you'll be fine.

# CREDITS

webgl code from :- https://github.com/bitluni/3DScannerESP8266


