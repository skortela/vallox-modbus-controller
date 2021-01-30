
#ifndef _VALLOX_MQTT_MAPPINGS_
#define _VALLOX_MQTT_MAPPINGS_

#include "vallox_constants.h"

static std::map<uint8_t, const char*> g_mqttParamTopic = {
    {VX_PARAM_FAN_RELAYS, "fan_current_speed"},
    {VX_PARAM_FAN_SPEED, "fan_speed"},
    {VX_PARAM_TEMP_OUTDOOR, "outdoor"},
    {VX_PARAM_TEMP_EXHAUST, "exhaust"},
    {VX_PARAM_TEMP_INSIDE, "inside"},
    {VX_PARAM_TEMP_INCOMING, "incoming"},
    {VX_PARAM_POST_HEATING, "post_heating_temp"}, // Jälkilämmityksen kohdearvo
    {VX_PARAM_LAST_ERRORCODE, "last_errorcode"},
    {VX_PARAM_TAKKA_COUNTER, "takka_timeleft"}
};

static std::map<uint8_t, std::map<uint8_t, const char*> > g_mqttParamBitTopic = {

    {VX_PARAM_IO_PORT_7, {
        {5, "post_heating"} // bit 5 = jälkilämmitys
    }},
    {VX_PARAM_IO_PORT_8, {
        {1, "peltimoottorin_asento"}, // (read-only)
        {2, "vikatietorele"}, // (read-only)
        {3, "tulopuhallin"},
        {4, "etulammitys"}, // (read-only)
        {5, "poistopuhallin"},
        {6, "8_takka_tehostustoiminto"} // takka/tehostekythkin (read-only)
    }},
    {VX_PARAM_LEDS, {
        {0, "power"},      // Virtanäppäin
        {1, "co2_btn"},    // CO2 –näppäin
        {2, "rh_btn"},     // %RH –näppäin
        {3, "heat_btn"},   // Jälkilämmityksen näppäin
        {4, "filter_led"}, // Suodatinvahdin merkkivalo (read-only)
        {5, "heat_led"},   // Jälkilämmityksen merkkivalo  (read-only)
        {6, "fault_led"},  // vian merkkivalo  (read-only)
        {7, "maintenance_led"} // huoltomuistutin  (read-only)
    }},
    {VX_PARAM_FLAGS_5, {
        {7, "etulammitys"} // bitti 7 = etulämmityksen tilalippu, 0 = päällä, 1 = pois
    }},
    {VX_PARAM_FLAGS_6, {
        // bitti 4 = kaukovalvontaohjaus
        // bitti 5 = takkakytkimen aktivointi (write)
        // bitti 6 = takka/tehostustoiminto
        {6, "6_takka_tehostustoiminto"}
    }}
};

/*
,
VX_PARAM_IO_PORT_8, {
    {1, "peltimoottorin_asento"}, // (read-only)
    {2, "vikatietorele"}, // (read-only)
    {3, "tulopuhallin"},
    {4, "etulammitys"}, // (read-only)
    {5, "poistopuhallin"},
    {6, "8_takka_tehostustoiminto"} // takka/tehostekythkin (read-only)
},
VX_PARAM_LEDS, {
    {0, "power"},      // Virtanäppäin
    {1, "co2_btn"},    // CO2 –näppäin
    {2, "rh_btn"},     // %RH –näppäin
    {3, "heat_btn"},   // Jälkilämmityksen näppäin
    {4, "filter_led"}, // Suodatinvahdin merkkivalo (read-only)
    {5, "heat_led"},   // Jälkilämmityksen merkkivalo  (read-only)
    {6, "fault_led"},  // vian merkkivalo  (read-only)
    {7, "maintenance_led}" // huoltomuistutin  (read-only)
},
VX_PARAM_FLAGS_5, {
    {7, "etulammitys"} // bitti 7 = etulämmityksen tilalippu, 0 = päällä, 1 = pois
},
VX_PARAM_FLAGS_6, {
    // bitti 4 = kaukovalvontaohjaus
    // bitti 5 = takkakytkimen aktivointi (write)
    // bitti 6 = takka/tehostustoiminto
    {6, "6_takka_tehostustoiminto"}
}
*/

#endif // _VALLOX_MQTT_MAPPINGS_
