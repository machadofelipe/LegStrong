# LegStrong
![](https://github.com/machadofelipe/LegStrong/blob/master/documentation/images/controller.jpg)

## What is LegStrong?
Legstrong is a Open-Source Hardware and Software controller for electric bikes. The motor controller is a Field Oriented Controller (FOC) and communicates with the user using any smartphone device with Bluetooth and an OBD app (e.g. [Torque Lite](https://play.google.com/store/apps/details?id=org.prowl.torquefree&hl=pt_PT)).

### What is an electric bike controller?
Is the middle point of the system, it connects the battery to the motor dosing the power based on input sensor(s).

### History of LegStrong
The ideia of creating such controller came out of a personal frustration with common e-bike controllers. Most of controllers available at low price cannot control the motor torque, they need Hall sensors in the motor and do not provide any user interface. 

At a higher price point there are controllers capable of field oriented control, but again failed to provide the user interface. Some high-end brands, have the system coming with some kind of interface, but is over simplified, almost like a toy. 

The common solution found by the more enthusiastic people is to buy a good controller and a separate device for monitor and control the controller (giving some intelligence to the dummy controllers). However this impacts greatly on the e-bike price, it is complex to setup and still is not 21st century user interface.


## What Problem Does LegStrong intent to Solve?
- Smother control of the motor;
- More simple and reliable controller;
- Very wide range of possibilites for user interface;
- User configurable data to be shown/log easily;
- Built-in motor parameter identification.


## How Does LegStrong Accomplish Its Goals?

- [TI InstaSPIN FOC](http://www.ti.com/ww/en/mcu/instaspin/instaspin-foc.shtml?DCMP=c2x-instaspin&HQS=instaspin-foc):

    Is an advanced motor control technique currently employed in high-end industrial AC machine drives. FOC has been applied to a custom brushless DC motor controller in an effort to prove its inherent superiority over control techniques currently employed in commercially available BLDC motor controllers. Of special concern is the controller’s ability to sensorlessly and stably control brushless DC motors at low speeds and the controller’s dynamic response. Results show that when applied to a small custom built brushless DC motor controller, field oriented control is far superior to commercial control techniques in regards to response times and low speed commutation capability.

- [OBD-II PIDs](https://en.wikipedia.org/wiki/OBD-II_PIDs) via Bluetooh using an [ELM327 emulator](https://en.wikipedia.org/wiki/ELM327): 

    OBD-II PIDs (On-board diagnostics Parameter IDs) are codes used to request data from a vehicle, used as a diagnostic tool. There are many applications available for smartphone/tablet devices in any platform (Android, iOS, Windows, Blackberry, Symbian and probably others) that can connect and receive data using this protocol. Those applications are made for combustion engine vehicles, but are being used with success in many electric vehicles. Torque for Android is a good example, with over 5 Million installs.

- Use of Open-Source platforms in both hardware and software:

    CircuitMaker for PCB design, Code Composer Studio for code development and GitHub for online project hosting using Git.


## LegStrong Features:
- Operating voltage range from 12V to 60V;
- Bluetooth technology to use with any smartphone/tablet (Android, iOS, Windows, Blackberry, Symbian);
- OBD-II PIDs standard codes to request data and change parameters;
- Wide range of data available for the user to keep tracking;
- Works sensor-less and with any brush-less motor;
- Higher motor efficiency at low speed (compared to non-FOC controllers);
- Torque based throttle (multiple mappings);
- PAS Sensor as input using special mappings to match power with cyclist cadence;
- Efficient auxiliar 5V output for USB power devices (needs hardware improvement);
- Fully programmable parameters (under software development);
- Battery State of Charge and Low cutoff (under software development);
- Torque Sensor or Analog Throttle (under software development);
- Field Weakening allows you to run motors faster than normal back-emf limit (under software development);
- Thermal Rollback (under software development).

## External Links
- https://circuitmaker.com/Projects/Details/machado-felipe/LegStrong
- https://endless-sphere.com/forums/viewtopic.php?f=2&t=88285
- https://www.youtube.com/watch?v=HpnT7h7fUU8
