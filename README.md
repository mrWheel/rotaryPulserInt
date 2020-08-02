# rotaryPulserInt
rotaryPulser based on ATmega328 using Timer1 interrupt
<p>Specifications by JDJ Electronics / JD Engeneers
<p>Firmware by Willem Aandewiel

## uses:
<pre>
https://github.com/Chris--A/Keypad
https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
</pre>

## Optinal compile options
<code>#define JDJEPCB</code> for alternative connections

## Functions

### [0]-[9] .. +[A] Set single pulse mode
Set frequency of pulse-A and pulse-B to keyed-in number
<p>example: 1200A -> sets frequency to 1200Hz

### [0]-[9] .. +[B] sets sweep from active frequency to keyed-in number
Set high sweep frequency
<p>example: 5000B -> with first input (1200) will sweep from 1200Hz to 5000Hz in 5 seconds and back to 1200Hz

### [*]+[A] select potmeter as input
Selects potmeter as frequency input, cancelles sweep mode

### [C] Clear input, stop pulse
Pulse will stop, any input will be cleared

### [0]-[9] .. +[D] sets sweep time in seconds
Minimal sweep time is 3 seconds, maximum is 20 seconds
<p>example: 8D -> sweep time set to 8 seconds

<HR>

<center><img src="./images/PCL-Pulser.png"</img></center>

<center><img src="./images/4x4-button-keypad.png"</img></center>

<center><img src="./images/AB10kHz.png"</img></center>

