# nibegw-esp

This ESP32 firmware, in combination with RS485 capable hardware, can be used to make your non-S series NIBE heatpump speak MODBUS over TCP. You can directly integrate with Home Assistant's "Nibe Heat Pump" addon.

Useful for the non-S indoor units such as VVM225/VVM320/VVM325/VVM310/VVM500.

## Setup

> WARNING: Read the section below, including the links, to get an idea what needs to be done. Make sure you understand the steps. Then judge whether you will be able to manage. Include the possibility of autonomous troubleshooting.

* You need an ESP32 based and RS485 capable device such as the [M5 Stamp PLC ✅](https://shop.m5stack.com/products/m5stamp-plc-controller-with-m5stamps3), the [LilyGO T-CAN485 ❓](https://github.com/Xinyuan-LilyGO/T-CAN485), the [PRODINo ESP32 ❓](https://kmpelectronics.eu/shop/prodino-esp32/) or similar. Make sure you have the schematics or at least know how the RS485 transceiver is connected to the MCU.

* Setup the ESP-IDF. Due to all the different hardware, I can not supply a precompiled firmware. You will have to compile it yourself. Please follow [Espressif's documentation](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#get-started) on how to set it up. Its not too hard.

* Open the project and run the "SDK Configuration Editor", or `idf.py menuconfig`. In the GUI, navigate to "NIBE Gateway Configuration" and configure the project for your hardware.

* Build the firmware and flash it onto your target device. Consult your devices documentation for the latter part.

* Connect your device to your heatpump via RS485 and enable it. Follow the [installer manual (IHB)](https://professional.nibe.eu/document/Installer%20manual%20(IHB)/031725-10.pdf) of the original MODBUS40 accessory where applicable. Roughly like this:
    * Find the correct terminals on the heatpump.
    * Connect the device. GND to GND, A-to-A, B-to-B. No crossing out the lines with  RS485! If voltage compatible, the heatpump should be able to power the hardware via the +12V terminal too.
    * Enable the MODBUS accessory in the service menu under point "5.2 System settings". The service menu reachable after pressing the "back" button for 7 seconds in the home screen. Might depend on the model.
    * Enable option "5.3.11 word swap". Otherwise some data will be read wrong!
    * Create a "LOG.SET" file using the [Modbus Manager](https://www.nibe.eu/no/support/nibe-modbus-manager) from NIBE. The file specifies the 20 most important registers that the heatpump will periodically supply. The others just take longer to be read.
    

    > NOTE:  If the heatpump goes into an alarm state, disable the MODBUS accessory and start troubleshooting. Check for wrong wiring, check the configuration of nibegw-esp, read its logs, ...

* Add the gateway to Home Assistant using the [Nibe Heat Pump](https://www.home-assistant.io/integrations/nibe_heatpump/) integration. Alternatively, control it manually. If you didn't change the default, the gateway should be reachable as "nibegw.local" via MODBUS TCP, Port 502, Address 1.

## Remarks

* Using a static IP is not yet supported and requires code changes.
* Writing registers is disabled per default, but can be enabled in menuconfig.
* Reading a register yields an immediate response, but with a possibly outdated value. In the background, nibegw fetches an up-to-date value which should be available after a few seconds.


## Other projects & helpful resources

* An ESPHome based gateway: 
https://github.com/elupus/esphome-nibe
* An RPiZero based gateway: https://github.com/anerdins/nibepi
* Server side parser of the NIBE protocol:
https://github.com/openhab/openhab-addons/tree/4.3.x/bundles/org.openhab.binding.nibeheatpump/src/main/java/org/openhab/binding/nibeheatpump/internal
* German setup instruction for another gateway variant:
https://loxwiki.atlassian.net/wiki/spaces/LOX/pages/1768030253/Nibe+Gateway+Arduino+Raspberry
* Broad discussion thread, in german, about the NIBE protocol. The linked page contains some info about the need to un/escape 0x5C: https://www.energiesparhaus.at/forum-diy-alternative-zu-nibe-modbus-modul/52722_8

