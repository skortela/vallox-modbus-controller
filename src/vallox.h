
#ifndef _VALLOX_H_
#define _VALLOX_H_

#include <map>
#include <sys/types.h>
#include <MQTTClient.h>

#include "vallox_constants.h"

class CElapseTimer
{
public:
    CElapseTimer()
    {
        m_ts.tv_sec = 0;
        m_ts.tv_nsec = 0;
    };
    ~CElapseTimer(){};

    void start()
    {
        clock_gettime(CLOCK_MONOTONIC, &m_ts);
        //long int ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    }

    // returns elapsed microseconds since last call of start
    unsigned long int elapsedUSec()
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        unsigned long int us = (now.tv_sec - m_ts.tv_sec) * 1000000 + (now.tv_nsec - m_ts.tv_nsec) / 1000;
        return us;
    }
    // returns elapsed milliseconds since last call of start
    unsigned long int elapsed()
    {
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        unsigned long int us = (now.tv_sec - m_ts.tv_sec) * 1000 + (now.tv_nsec - m_ts.tv_nsec) / 1000000;
        return us;
    }

private:
    struct timespec m_ts;
};

class CVallox
{
public:
    CVallox();
    ~CVallox();

    void setMQTTClient(MQTTClient client);

    int openSerial(const char* serial);
    void closev();

    int handleLoop();

    // Set fan speed, (1 to 8)
    int setFanSpeed(int speed);
    int activateTakkakytkin();
    int setPostHeatingTemp(int temperature);

    void dotests();
private:
    int handlePacket(_vallox_packet* packet);
    void onParamReceived(uint8_t param, uint8_t value);
    int checkAndRefresh(uint8_t param);
    int queryParam(uint8_t param, uint8_t& value);
    int sendRequest(uint8_t param, uint8_t data);

    int handle_value_bits_data(uint8_t param, uint8_t data);
    bool has_param_bit_changed(uint8_t param, uint8_t data, uint8_t bitIndex);
    bool hasRecentData(uint8_t param);
    int send_mqtt(const char* topic, const char* text);
    int send_mqtt(const char* topic, int value);

    void create_querybuf(void* pBuf, uint8_t param);
    // Formulates requestpacket from param and data
    // pBuf buffer, size must be at least 6 bytes
    void create_requestbuf(void* pBuf, uint8_t param, uint8_t data);
    int wait_for_idle();

private:
    pthread_mutex_t m_lock; // public methods lock
    int m_serialFd;
    MQTTClient m_mqttClient;
    //long int m_lastQuery;
    CElapseTimer m_lastQuery;
    //long int m_lastSerialActivity;
    CElapseTimer m_lastSerialActivity;
    CElapseTimer m_lastPing;
    std::map<uint8_t, long int> m_cacheTime;
    std::map<uint8_t, uint8_t> m_cacheData;

    std::map<uint8_t, long int> m_lastPublishTime;

};

#endif // _VALLOX_H_
