# HobbyIoT NET End device THR Type 1 
The HobbyIoT NET End device THR Type 1 - Arduino based MQTT-SN 802.15.4 based Temperature/Humidity/Magnetic(Reed) sensor with low power consumption.

An up to date MQTT-SN Client implementation based on MRF24J40 and Arduino

This is the optimized code and project details for the HobbyIoT NET End device THR Type 1 sensor implementing Temperature, Humidity and Magnetic (Reed) sensors within. The software is optimized and structured compared to older versions. Sensor works with the corresponding MQTT-SN to MQTT Gateway available at https://github.com/sivanovbg/MQTT-SN_GW_802154_V1

One can use either Arduino Pro Mini @ 3.3 V or the HobbyIoT THR Sensor V2 device (https://circuitmaker.com/Projects/Details/sivanovbg/HIOTTHRV2)
When Arduino Pro Mini is used the 3.3V regulator has to be removed or disconnected as shown on the schematic for best low power results.

MQTT-SN topics should be predefined on the client and gateway sides and are fixed to 2 positions alpha-numeric string.

Messages supported:

CONNECT

CONNACK

PINGREQ

PINGRESP

PUBLISH

Visit project website at https://sites.google.com/view/hobbyiot/projects/mqtt-sn-802-15-4

Twitter: @sivanovbg

