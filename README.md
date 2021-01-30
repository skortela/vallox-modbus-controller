# vallox-modbus-controller

Vallox air ventilation contoller via modbus and MQTT.

## Requirements

Hardware: 
* PC like Raspberry PI,
* USB-to-RS485 converter for USB port,
* Vallox 121 SE.

To build you have to have following tools

* CMake
* GCC/CLang

1. Install Paho (requires libssl-dev)
(paho requires libssl)

    sudo apt-get install libssl-dev

MQTT C Client:

    git clone https://github.com/eclipse/paho.mqtt.c.git
    cd paho.mqtt.c
    make
    sudo make install

## Build

To build you have to type following commands:

    cd vallox-modbus-controller
    mkdir build
    cd build
    cmake -DCMAKE_INSTALL_PREFIX=/usr ../
    make
    sudo make install
