# rotaryPulserInt
rotaryPulser based on ATmega328 using timer interrupt

## Optinal compile options

If you want to connect an OLED screen to the rotaryPulser
you need to 
<pre>#define _HAS_OLED</pre>

If you don't have a OLED screen place two slashes in front of the define
<pre>// #define _HAS_OLED</pre>

If you want to use HW I2C you also has to 
<pre>#define HARDWARE_I2C</pre>

If you want to use Software I2C emulation place two slashes before the define
<pre>// #define HARDWARE_I2C</pre>
