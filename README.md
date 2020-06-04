# HobbyIoT NET End device THR Type 1 
The HobbyIoT NET End device THR Type 1 is an Arduino based Temperature/Humidity/Magnetic(Reed) sensor with low power consumption.

The sensor periodically wakes up, measures values and sends them over the air to the Gateway using the MQTT-SN (MQTT for Sensor Networks) protocol. It operates over IEEE 802.15.4 wireless network using the ISM Band at 2.4 GHz. Module will immediately wake and send an event message when the magnetic sensor (reed semsor) changes its state. Keepalives are exchanged periodically with the Gateway of the system to supply the sensor registration within the Gateway list according specifications. The software is Arduino based and can be compiled for any compatible board wired with the MRF24J40MA wireless module and DHT11 and reed sensor.

Each device within the network has a unique 2-byte short network address. The Gateway address (2 bytes also) should be known by each device so one to be able to connect to it. MQTT-SN topics should be predefined on the client side and are fixed to 2 positions alpha-numeric string.

Messages supported:
CONNECT
CONNACK
PINGREQ
PINGRESP
PUBLISH

The protocol complies with MQTT-SN specification Version 1.2.

Sensor operates with the corresponding MQTT-SN to MQTT Gateway, part of the HobbyIoT NET project. It acts as a bidirectional MQTT-SN to/from MQTT translator/forwader. The Gateway terminates and manages all the connections coming from the HobbyIoT NET devices. Details available at https://github.com/sivanovbg/MQTT-SN_GW_802154_V1

Hardware is based on Arduino concept plus a Microchip's MRF24J40MA transceiver module and sensors. The MRF24J40 is responsible for both sending and receiving data and also waking up the whole system from sleep by using internal timer system. That allows to lower the power (currently) under 100 uA in sleep mode. Active mode lasts for about 1.5 seconds including keep alive and data read and transmit messages to the Gateway.

Arduino Pro Mini @3.3 V or the HobbyIoT THR Sensor V2 device are recommended to build such a sensor node. When Arduino Pro Mini is used the 3.3V regulator has to be removed or disconnected as shown on the schematic for best low power results.

The work on optimizing design and power consupmtion is in progress!

ToDo (optimizations and bug fixes):

1. Lower the power consumption by optimizing the sleep current and active time;
2. Deal with keepalive skip when frequent reed events;
3. TBA

The HobbyIoT THR Sensor V2 device hardware project is available at https://circuitmaker.com/Projects/Details/sivanovbg/HIOTTHRV2

Visit project website at https://sites.google.com/view/hobbyiot/projects/mqtt-sn-802-15-4

Twitter: @sivanovbg

