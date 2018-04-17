
# WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad

Example how to connect [SimpleLink™ Wi-Fi® CC3200 LaunchPad™](http://www.ti.com/tool/CC3200-LAUNCHXL) with [WolkAbout IoT platform 2.0](https://wolkabout.com/), send sensor readings, perform actuation and firmware update, using [WolkConnect-C](https://github.com/Wolkabout/WolkConnect-C) library.
![simplelink-wi-fi-cc3200-launchpad](https://user-images.githubusercontent.com/10706360/36902885-4b3dca14-1e2c-11e8-9371-6bce59f33c1a.png)

## Description

The example connects CC3200 LaunchPad to the WolkAbout IoT platform 2.0, sends temperature sensor readings and perform actuation on user’s request from the platform by turning on and off red LED on the board. It also shows the example of usage firmware update functionality on user's request from the platform. All of these functionalities are implemented using WolkConnect-C library, alongside [Texas Instruments CC3200 SDK](http://www.ti.com/tool/CC3200SDK).

## Getting Started

These instructions will lead you through the step-by-step process for setting up this project and trying it on your own CC3200 LaunchPad.

> **Important**: *The repository must be cloned from the command line using*
> 
> <code><b>git clone --recurse </b> https://<span></span>github.com/Wolkabout/WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad.git</code>
### Prerequisites

 - Own [CC3200 LaunchPad](http://www.ti.com/tool/CC3200-LAUNCHXL) board. Here you can find the [User's guide](http://www.ti.com/lit/ug/swru372b/swru372b.pdf).

 - [Texas Instruments Uniflash](http://www.ti.com/tool/UNIFLASH) to flash binary images to the SFLASH memory of LaunchPad.

 - Account on [WolkAbout IoT platform 2.0](https://demo.wolkabout.com).

 - For editing and recompiling this project, you will need [Code Composer Studio (CCS) Integrated Development Environment (IDE) for Wireless Connectivity](http://www.ti.com/tool/CCSTUDIO-WCS), alongside SimpleLink Wi-Fi CC3200 SDK. It will be the best to install this SDK in C:\TI folder for the ease of setup.

## Trying the example without rebuilding the project

* Make sure that jumpers on the LaunchPad board are mounted as follows:

        * J2 in short
        * J3 in short
        * SOP 2 in short

* Connect micro USB to the LaunchPad board and to the PC

* Open the Uniflash tool, and make a new configuration by choosing Connection: CC3x Serial UART interface, under the "Board or Device" CC3200 should be discovered. If it's not discovered, install the FTDI driver which is located in *{SDK_ROOT}/tools/ftdi* folder.

* Next step is to flash to binary images into the devices SFLASH memory from the Uniflash. Add new files:
	* /sys/mcuimg.bin URL:{SDK_ROOT}/example/application_bootloader/ccs/Release
	* /sys/mcuimg1.bin URL:{PROJECT_ROOT}/WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad/WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad-Factory.bin
	* /sys/config.txt URL:{PROJECT_ROOT}/WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad/config.txt

> If you make changes in this project you will find your new .bin file located here {PROJECT_ROOT}/WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad/Release/WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad.bin

### /sys/config.txt description

The file should look like this:
```
wifi_network_name=default_value
wifi_network_security_type=default_value
wifi_network_password=default_value
device_key=default_value
device_password=default_value
```

Change default values with your own. WiFI parameters with your own. For *wifi_network_security_type* insert one of follow WEP, WPA, WPA2 or OPEN. The device_key and device_password you will get when you create your device on WolkAbout IoT platform 2.0.

### Creating device on Wolkabout IoT platform 2.0

While creating a device on WolkAbout IoT platform 2.0 import *manifest.json* file attached to this repo. Here you can find the [WolkAbout™ IoT Tool Web Application User Manual](https://wolkabout.com/assets/User-Manual-for-Web-Application.pdf).

### Testing the example

Remove the SOP 2 jumper from the CC3200 LaunchPad board and the device should connect to the platform and send sensor's readings every 30 seconds. You should also be able to turn on and off LED from the platform by switching the actuator state.

## Building the example

* Open the Code Composer Studio and import the project.

* Open the project properties:
	* Resource
		* Linked Resources
And change the CC3200_SDK_ROOT to your {SDK_ROOT} - folder where you installed SDK
Also, change the ORIGINAL_PROJECT_ROOT to the folder where you downloaded project

* Inside project go to the:
	* WolkConnect-C
		* examples
			* main.c - Exclude from Build

* Now you should be able to build the project

* For trying the example follow the steps from above for **Trying example without rebuilding the project**

## Performing firmware update
* Details can be found in [WolkAbout™ IoT Tool Web Application User Manual](https://wolkabout.com/assets/User-Manual-for-Web-Application.pdf) **- this document will be updated with DFU section soon**
* Upload the file **WolkConnect-C-SimpleLink-Wi-Fi-CC3200-LaunchPad-Factory.bin** or any other .bin that you previously built for CC3200 LaunchPad
* The device should reboot itself and load a new firmware image!
  > **Note:** *The process of downloading and installing can be tracked with serial monitor as described below*

## Debugging the example

* Debugging from the Code Composer:
	* Add taget configuration as described in [Getting started guide](http://www.ti.com/lit/ug/swru376d/swru376d.pdf)
	* Put SOP 2 jumper in short
	* Connect PC and the LaunchPad board

* Debugging just with the serial console
	* Install some serial monitor or open the serial monitor in CCS
	* Set the baud rate to 115200
	* You should be able to see all messages from the board which indicate what's happening
	> **Note:** *This can be done either with SOP 2 mounted or dismounted*

## Built with

* [WolkConnector-C](https://github.com/Wolkabout/WolkConnect-C) - library used to communicate with the Wolkabout IoT platform 2.0
* [CC3200 SDK](http://www.ti.com/tool/CC3200SDK) - Software development kit by Texas Instruments
