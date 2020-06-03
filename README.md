# HobbyIoT NET End device THR Type 1 
The HobbyIoT NET End device THR Type 1 is an Arduino based MQTT-SN Temperature/Humidity/Magnetic(Reed) sensor with low power consumption.

The sensor periodically wakes up, measures values and send them over the air to the gateway using the MQTT-SN protocol. Module will immediately wake and send the information when the magnetic sensor (reed semsor) changes its state. Keepalives are also exchanged periodically with the Gateway of the system to prove the sensor registration with the gateway according specifications. The software is Arduino based and can be compiled for any compatible board.

Each device will have a unique 2-byte short network address. The Gateway address should be known by each device to be able to connect to it. MQTT-SN topics should be predefined on the client and gateway sides and are fixed to 2 positions alpha-numeric string.

Messages supported:
CONNECT
CONNACK
PINGREQ
PINGRESP
PUBLISH

The protocol complies with MQTT-SN specification Version 1.2.

Sensor operates with the corresponding MQTT-SN to MQTT Gateway available at https://github.com/sivanovbg/MQTT-SN_GW_802154_V1

Hardware is generically based on Arduino with a Microchip''s MRF24J40MA transceiver module. It is responsible for both sending and receiving data and also waking up the whole system from sleep by using internal timer system. That leads to the possibility to lower the power under 100 uA in sleep mode. Active mode is currently about 1.5 seconds including keep alive and data exchange. 

Arduino Pro Mini @ 3.3 V or the HobbyIoT THR Sensor V2 device are recommended for using to build a sensor node. When Arduino Pro Mini is used the 3.3V regulator has to be removed or disconnected as shown on the schematic for best low power results.

The work on optimizing design and power consupmtion are in progress.

ToDo (optimizations and bug fixes):

1. Lower the power consumption by optimizing the sleep current and active time;
2. 

The HobbyIoT THR Sensor V2 device hardware project: https://circuitmaker.com/Projects/Details/sivanovbg/HIOTTHRV2

Visit project website at https://sites.google.com/view/hobbyiot/projects/mqtt-sn-802-15-4

Twitter: @sivanovbg

