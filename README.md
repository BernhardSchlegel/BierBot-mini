![BierBot mini](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/BierBot_mini-Logo_1024x138.png)
## Arduino powered beer brewing software

### What this software does

This software runs on any "arduino compatible" board. It gives enthusiastic homebrewers full control over any temperature in the brewing and fermenting process. 

The brewcontrol is controlled using a turn and push incremental encoder using a DS18b20 temperature sensor. Basically the following operations are supported:
- turning the encoder: incrementing or decreasing values (temperatures, times) as well as navigating through the menus
- short press of encoder: confirm the current selection or proceeding to the next step
- long press of encoder (more than 3s): return to main menu

![BierBot mini splash screen](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/01_splash.png)

### How to install the Software on your Arduino

First, you have to make sure, that the used libraries are available on your System:

- **LiquidCrystal**: To avoid compatibility issues please use version 1.2.1, [download here](https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads/LiquidCrystal_V1.2.1.zip) using the green "Clone or Download"-Button and then click on "Download .ZIP" 
- **Dallastemperaturecontrol**: [download here](https://github.com/milesburton/Arduino-Temperature-Control-Library) using the green "Clone or Download"-Button and then click on "Download .ZIP" 
- **OneWire**: Add via "Sketch" > "Include library" > "Manage libraries" > Search for "onewire" - it's the one by Jim Studt et. al.
- **Time**: [download here](https://github.com/PaulStoffregen/Time) using the green "Clone or Download"-Button and then click on "Download .ZIP" > "Install"

After downloading, all ZIP-files can be added using "Sketch" > "Include library" > "Add .ZIP Library..."

### How to connect your hardware to the Arduino


| GPIO    | What                                       |
| --------| ------------------------------------------ | 
| A4      | Display SDA                                | 
| A5      | Display SCL                                | 
| D2      | Rotary Encoder channel INKR_B              |
| D3      | Rotary Encoder channel INKR_A              |
| D4      | Button                                     |
| D5      | Temperature Sensor (Data, usually yellow)  |
| D6      | Relais (heating or cooling on/off)         |
| D7      | Sound for connecting your piezo speaker    |


### How this software is used

A german manual is available [here](https://bierbot.de/data/BierBot_mini_de_v.1.0.pdf).

#### Main Menu

After booting, the menu is displayed (for now, only on german)

![BierBot mini main menu](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/02_main.png)

Menu points are
- "Maischen"/"Mash": Set a recipe and control the mashing process
- "Nachuss"/"Sparging": Pre heat water to be used for sparging (usually 78°C)
- "Kochen"/"Boiling": Set boiling time as well as up to 5 hop timers
- "Kühlen"/"Cooling": Can be used in combination with a fridge (power on correlates with temperature down in this mode)
- "Setup": Adapt the BierBot mini software to your needs

#### Maischen / Mash

![Setting number of rests](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/03_rast.png)
![Setting rests](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/04_mash.png)

This mode enables you to
-	Set the number of up to 5 mash rests
-	Set the temperature for each rest
-	Set the time for each rest (these two steps are repeated until all rests are set)
-	Setting of the lauteringtemperature, when this temperature is reached, the programm is finished

#### Nachuss / Sparging

Set a temperature that is being hold for an infinite amount of time. This mode is for heating only. The display is showing same information as in mashing mode. To exit this mode, push the button for at least 2s.

#### Kochen / Boiling

![Boiling](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/06_hop.png)

This mode enables you to 
- set the total cooking time (in minutes)
- set the number of hops you want to add
- For each hop set the time before the end of the boiling process when the hop is due

#### Kühlen / Cooling

Set a temperature that is being hold for an infinite amount of time. This mode is for cooling only. The display is showing same information as in mashing mode. To exit this mode, push the button for at least 2s.

#### Setup

Adapt the BierBot mini software to your needs using the GUI.

![Settings](https://github.com/BernhardSchlegel/BierBot-mini/blob/master/img/09_settings.png)

The following settings are supported
- "Schwelle"/"Boiling": Set above which temperature time is being counted in boiling mode
- "Hyst": The hysteresis defines a tolerancemargin around the current targettemperature. This enables to set, when a temperature is being regarded as "reached" (an that's the moment where counting the rest-time starts) or (upon leaving the tolerancemargin) when the heating or cooling is turned on again.
- "kd-Heiz"/"kd-Heat": Sets the proportional part of the integrated, pseudo PD-Control for heating. This value defines how the outlet is being switched in dependence of the current temperature changing rate. Larger values result in an earlier turning off. 
- "kd-Kühl"/"kd-Cool": Same like kd-heat - just for cooling
- "ESVHeizen"/"TOD-Heat": Turn on delay for heating. This value defines how much time needs to pass before the outlet is being turned on again (to go easy on your equipment)
- "ESCKühl"/"TOD-Cool": Same as "TOD-Heat" - just for cooling. 

### Open points / TODOs

- Support multi-language
- Resume after power off
- Stabilize pointer movement in main menu (esp. after boot)
- Reduce increments for the rotary encoder when turning fast
- Evaluate limits for times, temperatures and hops (cooking min minutes to 0? hops to 0?
- put rests into structs

### Contributions

Contributions are welcome and may be submitted as pull request.

### Thanks

Thanks to Thanks to Bitter, Borsti84, Joerg from [braumagazin.de](http://braumagazin.de/), Bodo and all the other great and passionate guys from THE german hobbybrauer community [hobbybrauer.de](http://hobbybrauer.de/)

### Liscense


This software is released under the BierBot mini Firmware liscense v.1.0

BierBot mini Firmware v.1.1.
(c) 2014-2015 Bernhard Schlegel, omni Technologie UG & Co. KG
(called omni in the following)

Redistribution and use in source and binary forms, with or without
modification, is permitted free of charge provided that the 
following conditions are met:

1. Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.

3. The name "BierBot" or "BierBot mini" must not be used to endorse 
or promote products derived from this software without prior written 
permission. For written permission, please contact hello@bierbot.com.

4. Products derived from this software may not be called "BierBot" 
or "BierBot mini", nor may "BierBot" or "BierBot mini" appear in 
their name, without prior written permission from hello@bierbot.com.  

5. omni may publish revised and/or new versions of the license from 
time to time. Each version will be given a distinguishing version 
number.
Once covered code has been published under a particular version
of the license, you may always continue to use it under the terms
of that version. You may also choose to use such covered code
under the terms of any subsequent version of the license
published by omni. No one other than omni has
the right to modify the terms applicable to covered code created
under this License.

7. Redistributions of any form whatsoever must retain the BierBot 
mini splash screen. The splash screen includes the black mashing pan 
icon with the bolt in the middle, the text saying "BierBot mini" as
well as the current software version.

6. Redistributions of any form whatsoever must retain the following
acknowledgment as well as the BierBot mini splash screen:
"This product is powered by BierBot mini, freely available from 
<https://bierbot.de/>".

THIS SOFTWARE IS PROVIDED BY THE omni DEVELOPMENT TEAM ``AS IS'' AND
ANY EXPRESSED OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE omni
DEVELOPMENT TEAM OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.

This software consists of voluntary contributions made by many
individuals on behalf of omni.

omni can be contacted via Email at hello@bierbot.com.

For more information on the BierBot please see <https://bierbot.de>.
