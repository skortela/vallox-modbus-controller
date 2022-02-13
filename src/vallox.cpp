/*

*/
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
//#include <chrono>
#include <sys/time.h>
#include <pthread.h>

#include "vallox.h"
#include "debug.h"

#include "valloxMqttMappings.h"

const uint8_t client_id = 0x22; // select id from 0x21 to 0x2F, 0x21 is usually reserved for first controller

long int millis()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    long int ms = ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
    return ms;
}

long int elapsed(long int lastMillis)
{
    long int val = millis() - lastMillis;
    if (val < 0)
    {
        DBG("oops: elapsed was negative: %ld", val);
    }
    return abs(val);
}


int set_interface_attribs (int fd, int speed, int parity)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        DBG("error %d from tcgetattr", errno);
        return -1;
    }

    cfsetospeed (&tty, speed);
    cfsetispeed (&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    // disable IGNBRK for mismatched speed tests; otherwise receive break
    // as \000 chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                    // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    //tty.c_cc[VTIME] = 0; // inter-character timer unused
    //tty.c_cc[VMIN]  = 1; // blocking read until 1 character arrives

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag |= parity;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tcflush(fd, TCIFLUSH);  // clean line
    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        DBG("error %d from tcsetattr", errno);
        return -1;
    }
    return 0;
}

void set_blocking (int fd, int should_block)
{
    struct termios tty;
    memset (&tty, 0, sizeof tty);
    if (tcgetattr (fd, &tty) != 0)
    {
        DBG("error %d from tggetattr", errno);
        return;
    }

    tty.c_cc[VMIN]  = should_block ? 1 : 0;
    tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

    if (tcsetattr (fd, TCSANOW, &tty) != 0)
    {
        DBG("error %d setting term attributes", errno);
    }
}

// Get bit status from given byte and index. index starts from 0.
bool get_bit(uint8_t byteval, uint8_t index)
{
    return ((byteval & (1<<index)) != 0);
}

// Set bit to true from given byte and index. index starts from 0.
uint8_t set_bit(uint8_t byteval, uint8_t index)
{
    uint8_t newbyte = byteval;
    newbyte |= 1 << index;
    return newbyte;
}

// Clear bit from given byte and index. index starts from 0.
uint8_t clear_bit(uint8_t byteval, uint8_t index)
{
    uint8_t newbyte = byteval;
    newbyte &= ~(1 << index);
    return newbyte;
}

int getTemp(uint8_t ntcValue)
{
    std::map<uint8_t, int>::const_iterator it = g_ntcMap.find(ntcValue);
    if (it != g_fanSpeed.end())
    {
        return it->second;
    }
    else
        return 0xffffffff;
}
uint8_t getNtcValue(int temperature)
{
    std::map<uint8_t, int>::const_iterator it = g_ntcMap.begin();
    while (it != g_ntcMap.end())
    {
        if (it->second == temperature)
            return it->first;
        else
            it++;
    }
    return 0;
}


int getFanSpeed(uint8_t speedHex)
{
    std::map<uint8_t, int>::const_iterator it = g_fanSpeed.find(speedHex);
    if (it != g_fanSpeed.end())
    {
        return it->second;
    }
    else
        return 0xffffffff;
}
uint8_t getFanSpeedRaw(int speed)
{
    std::map<uint8_t, int>::const_iterator it = g_fanSpeed.begin();
    while (it != g_fanSpeed.end())
    {
        if (it->second == speed)
            return it->first;
        else
            it++;
    }
    return 0;
}

// Caller is responsible to free returned topic
char* getTopicForParam(uint8_t param)
{
    std::map<uint8_t, const char*>::const_iterator it = g_mqttParamTopic.find(param);
    if (it != g_mqttParamTopic.end())
    {
        int len = strlen(KMqttPrefix) + strlen(it->second) + 1;
        char* topic = (char*)malloc(len);
        if (topic)
        {
            int cx = snprintf(topic, len, "%s%s", KMqttPrefix, it->second);
            if (cx > 0)
                return topic;
            else
                free(topic);
        }
    }
    return NULL;
}
// Caller is responsible to free returned topic
char* getTopicForParamBit(uint8_t param, uint8_t bitIndex)
{
    std::map<uint8_t, std::map<uint8_t, const char*> >::const_iterator it = g_mqttParamBitTopic.find(param);
    if (it != g_mqttParamBitTopic.end())
    {
        std::map<uint8_t, const char*>::const_iterator it2 = it->second.find(bitIndex);
        if (it2 != it->second.end())
        {
            int len = strlen(KMqttPrefix) + strlen(it2->second) + 1;
            char* topic = (char*)malloc(len);
            if (topic)
            {
                int cx = snprintf(topic, len, "%s%s", KMqttPrefix, it2->second);
                if (cx > 0)
                    return topic;
                else
                    free(topic);
            }
        }
    }
    return NULL;
}

bool is_param_known_bitfield(uint8_t param)
{
    std::map<uint8_t, std::map<uint8_t, const char*> >::const_iterator it = g_mqttParamBitTopic.find(param);
    if (it != g_mqttParamBitTopic.end())
        return true;
    else
        return false;
}

CVallox::CVallox() : m_serialFd(0), m_mqttClient(NULL)
{
    //m_lastQuery = 0;// millis() - 5000;
    //m_lastSerialActivity = 0;
    //m_lastPing = 0;
    if (pthread_mutex_init(&m_lock, NULL) != 0)
    {
        DBG("mutex init failed!");
    }
}
CVallox::~CVallox()
{
    closev();
    pthread_mutex_destroy(&m_lock);
}

void CVallox::setMQTTClient(MQTTClient client)
{
    m_mqttClient = client;
}

int CVallox::openSerial(const char* serial)
{
    DBG("opening serial: %s", serial);
    if (m_serialFd > 0) {
        DBG("oops, already opened");
        return -1;
    }
    m_serialFd = open(serial, O_RDWR | O_NOCTTY | O_NDELAY);
    if (m_serialFd == -1)
    {
        /*
        * Could not open the port.
        */
        DBG("open_port: Unable to open %s", serial);
        return -1;
    }
    else
    {
        // port opened ok
        DBG("port succesfully opened.");

        //fcntl(fd, F_SETFL, FNDELAY);

        if (set_interface_attribs(m_serialFd, B9600, 0) == -1)  // set speed to 115,200 bps, 8n1 (no parity)
        {
            close(m_serialFd);
            m_serialFd = 0;
            return -1;
        }
        //set_blocking(m_serialFd, 1);  // set blocking
    }
    return 0;
}
void CVallox::closev()
{
    if (m_serialFd)
        close(m_serialFd);
    m_serialFd = 0;
}

uint8_t calculate_checksum(const char* pData, int len)
{
    uint8_t x = 0;
    for (int i=0; i<len; i++) {
        x = (x + pData[i]);// % 256;
    }
    return x;
}

int CVallox::handleLoop()
{
    if (!m_serialFd)
        return -1;

    pthread_mutex_lock(&m_lock);

    char buf[6];
    //long int m = millis();
    int n = read(m_serialFd, buf, sizeof buf);  // read up to x characters if ready to read
    /*long int timetook = millis() - m;
    if (n>0)
    {
        DBG("read timetook: %ld ms. bytes: %d", timetook, n);
    }*/
    if (n == 6) {
        //DBG("readlen: %d", n);
        //DBG("firstbyte: %d", (int)buf[0] );
        m_lastSerialActivity.start();

        _vallox_packet* packet = (_vallox_packet*) &buf[0];
        handlePacket(packet);

    }
    else if (n > 0)
    {
        m_lastSerialActivity.start();
        //usleep(100000); // 100 ms
    }
    /*else {
        usleep(100000); // 100 ms
    }*/

    if (m_lastPing.elapsed() > 60000)
    {
        DBG("alive");
        m_lastPing.start();
    }

    if (m_lastQuery.elapsed() > 1000 && m_lastSerialActivity.elapsed() > 50)
    {
        // do query
        checkAndRefresh(VX_PARAM_LEDS);
        checkAndRefresh(VX_PARAM_FAN_SPEED);
        //checkAndRefresh(VX_PARAM_FAN_RELAYS);
        checkAndRefresh(VX_PARAM_IO_PORT_7);
        checkAndRefresh(VX_PARAM_IO_PORT_8);
        // etulämmityksen tilalippu,
        checkAndRefresh(VX_PARAM_FLAGS_5);
        // takkatoiminto
        checkAndRefresh(VX_PARAM_FLAGS_6);
        checkAndRefresh(VX_PARAM_TEMP_OUTDOOR);
        checkAndRefresh(VX_PARAM_TEMP_EXHAUST);
        checkAndRefresh(VX_PARAM_TEMP_INSIDE);
        checkAndRefresh(VX_PARAM_TEMP_INCOMING);
        checkAndRefresh(VX_PARAM_LAST_ERRORCODE);
        checkAndRefresh(VX_PARAM_TAKKA_COUNTER);
        checkAndRefresh(VX_PARAM_POST_HEATING);
        m_lastQuery.start();
    }

    pthread_mutex_unlock(&m_lock);

    if (n <= 0)
    {
        usleep(100000); // 100 ms
    }

    return 0;
}

int CVallox::setFanSpeed(int speed)
{
    pthread_mutex_lock(&m_lock);
    int retVal = 0;
    uint8_t fanSpeedData = getFanSpeedRaw(speed);
    if (fanSpeedData == 0)
    {
        DBG("setFanSpeed, invalid speed: %d", speed);
        retVal = -1;
    }
    else
        retVal = sendRequest(VX_PARAM_FAN_SPEED, fanSpeedData);

    pthread_mutex_unlock(&m_lock);
    /*
    fan_speed_val = getKeyByValue(VALLOX.FAN_SPEED, value)
    if fan_speed_val == None:
        logging.warning("set_fan_speed: Invalid value: " + str(value))
        return False
    else:
        logging.info("setting fan speed: " + str(value))
        return self.send_request(VALLOX.VX_PARAM_FAN_SPEED, fan_speed_val)
        */
    return retVal;
}
int CVallox::activateTakkakytkin()
{
    pthread_mutex_lock(&m_lock);
    int retVal = 0;
    // read param VX_PARAM_FLAGS_6 and change 5th bit to true
    // bitti 5 = takkakytkimen aktivointi (write)
    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.find(VX_PARAM_FLAGS_6);
    uint8_t cacheval = 0;
    if (it != m_cacheData.end())
    {
        // data found from cache
        cacheval = it->second;
    }
    else
    {
        // Query
        retVal = queryParam(VX_PARAM_FLAGS_6, cacheval);
        if (retVal < 0)
        {
            DBG("activateTakkakytkin() Failed to query VX_PARAM_FLAGS_6");
            //return -1;
        }
    }

    if (retVal == 0)
    {
        uint8_t newVal = set_bit(cacheval, 5);
        retVal = sendRequest(VX_PARAM_FLAGS_6, newVal);

        m_cacheTime.erase(VX_PARAM_TAKKA_COUNTER);
        m_cacheData.erase(VX_PARAM_TAKKA_COUNTER);
    }

    pthread_mutex_unlock(&m_lock);

    return retVal;
}
int CVallox::setPostHeatingTemp(int temperature)
{
    if (temperature < 10)
    {
        DBG("setPostHeatingTemp failed, too low: %d", temperature);
        return -1;
    }
    else if (temperature > 20)
    {
        DBG("setPostHeatingTemp failed, too high: %d", temperature);
        return -1;
    }

    uint8_t ntcVal = getNtcValue(temperature);
    if (ntcVal == 0)
    {
        DBG("setPostHeatingTemp failed, getNtcValue failed!");
        return -1;
    }

    pthread_mutex_lock(&m_lock);
    int retVal = sendRequest(VX_PARAM_POST_HEATING, ntcVal);
    pthread_mutex_unlock(&m_lock);
    return retVal;
}


int CVallox::handlePacket(_vallox_packet* packet)
{

    //DBG("domain: %d", packet->domain);
    //DBG("sender: %d", packet->sender);
    //DBG("receiver: %d", packet->receiver);
    //DBG("param: %d", packet->param);
    //DBG("data: %d", packet->data);
    //DBG("checksum: %d", packet->checksum);

    if (packet->domain != VX_DOMAIN) // domain must be 0x01
        return 0;

    // VX_DOMAIN, CLIENT_ID, VX_MAINBOARD, VX_REQ_QUERY
    /*#ifdef _TRACE_COMMANDS
    if (packet->receiver == VX_MAINBOARD && packet->param != VX_REQ_QUERY)
    {
        param = xb[3]
        data = xb[4]
        logging.debug("traced cmd: " + hex(param) + ", data: " + hex(data))
        return
    }
    #endif*/

    uint8_t chk_calc = calculate_checksum((char*)packet, 5);

    if (chk_calc != packet->checksum)
    {
        DBG("oops, checksum missmatch");
        return 0;
    }

    if (packet->receiver != VX_ALL_CLIENTS && packet->receiver != client_id) // filter common or for us)
        return 0;

    //DBG("handle param: 0x%02X, data: 0x%02X", packet->param, packet->data);

    onParamReceived(packet->param, packet->data);

    m_cacheTime[packet->param] = millis();
    m_cacheData[packet->param] = packet->data;

    return 0;
}

void CVallox::onParamReceived(uint8_t param, uint8_t value)
{
    bool dataChanged(true);
    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.find(param);
    uint8_t cacheData = 0;
    if (it != m_cacheData.end())
    {
        // data found from cache
        cacheData = it->second;
        dataChanged = (value != cacheData);
    }

    long int now = millis();
    bool forceUpdate(false);

    if (param >= VX_PARAM_TEMP_OUTDOOR && param <= VX_PARAM_TEMP_INCOMING)
    {
        // param not changed, check when last updated
        std::map<uint8_t, long int>::const_iterator it2 = m_lastPublishTime.find(param);
        if (it2 != m_lastPublishTime.end())
        {
            if (now - it2->second < 10*60000) // < 10 minutes
            {
                // has recently updated, do nothing
                forceUpdate = false;
            }
            else
                forceUpdate = true;
        }
        else // newer published
            forceUpdate = true;
    }

    if (dataChanged)
    {
        // data changed
        DBG("param value changed: param: 0x%02X, data: 0x%02X", param, value);
    }
    else
    {
        if (!forceUpdate)
            return;
    }


    if (param == VX_PARAM_SENDING_ALLOWED)
    {
        DBG("VX_PARAM_SENDING_ALLOWED");
        return;
    }
    else if (param == VX_PARAM_SENDING_DISALLOED)
    {
        DBG("VX_PARAM_SENDING_DISALLOED");
        return;
    }

    if (is_param_known_bitfield(param))
    {
        //DBG("handle_response_data() handle_value_bits_data");
        handle_value_bits_data(param, value);
        //DBG("handle_response_data() handle_value_bits_data done");
        return;
    }

    char* topic = getTopicForParam(param);

    if (!topic)
        return;

    if (param == VX_PARAM_FAN_SPEED) // fan speed
    {
        //logging.debug("fan speed: " + str(FAN_SPEED.get(data)))
        //logging.debug ("topic: " + topic_for_param(param) + " " + str(FAN_SPEED.get(data)));
        //send_mqtt(topic, str(.FAN_SPEED.get(data)));
        send_mqtt(topic, getFanSpeed(value));
        m_lastPublishTime[param] = now;
    }
    /*else if (param == VX_PARAM_FAN_RELAYS)
    {
        speed = None
        if get_bit(data, 0)
            speed = 1
        elif get_bit(data, 1)
            speed = 2
        elif get_bit(data, 2)
            speed = 3
        elif get_bit(data, 3)
            speed = 4
        elif get_bit(data, 4)
            speed = 5
        elif get_bit(data, 5)
            speed = 6
        elif get_bit(data, 6)
            speed = 7
        elif get_bit(data, 7)
            speed = 8
        if speed != None
            self.send_mqtt(topic, str(speed))
        else
            DBG("unknown speed!");
    }*/
    else if (param >= VX_PARAM_TEMP_OUTDOOR && param <= VX_PARAM_TEMP_INCOMING)
    {
        int temperature = getTemp(value);
        if (forceUpdate || temperature != getTemp(cacheData))
        {
            send_mqtt(topic, temperature); // temperature has changed
            m_lastPublishTime[param] = now;
        }
        //send_mqtt(topic_for_param(param) + "_raw", hex(NTC_MAP.get(data)))
    }
    else if (param == VX_PARAM_POST_HEATING)
    {
        int temperature = getTemp(value);
        if (forceUpdate || temperature != getTemp(cacheData))
        {
            send_mqtt(topic, temperature); // temperature has changed
            m_lastPublishTime[param] = now;
        }
    }
    else if (param == VX_PARAM_LAST_ERRORCODE)
    {
        //logging.debug("VX_PARAM_LAST_ERRORCODE: " + hex(data))
        send_mqtt(topic, value);
        m_lastPublishTime[param] = now;
    }
    else if (param == VX_PARAM_TAKKA_COUNTER)
    {
        //logging.debug("VX_PARAM_TAKKA_COUNTER: " + str(data) + " minutes")
        send_mqtt(topic, value);
        m_lastPublishTime[param] = now;
    }

    free(topic);
}

void CVallox::handle_value_bits_data(uint8_t param, uint8_t data)
{
    for (int i=0; i<8; i++)
    {
        if (!has_param_bit_changed(param, data, i))
            continue;

        char* topic = getTopicForParamBit(param, i);
        if (!topic)
            continue;

        bool bitValue = get_bit(data, i);
        if (param == VX_PARAM_FLAGS_5 && i == 7) // etulammitys is inverted
            bitValue = !bitValue;
        if (bitValue)
            send_mqtt(topic, "On");
        else
            send_mqtt(topic, "Off");
        free(topic);
    }
    //const char* topic = getTopicForParamBit(param, bit);
}

bool CVallox::has_param_bit_changed(uint8_t param, uint8_t data, uint8_t bitIndex)
{
    bool dataChanged(false);
    std::map<uint8_t, uint8_t>::const_iterator it = m_cacheData.find(param);
    if (it == m_cacheData.end()) // not found, changed
        return true;

    bool changed = (get_bit(it->second, bitIndex) != get_bit(data, bitIndex));
    return changed;
}
int CVallox::send_mqtt(const char* topic, const char* text)
{
    if (!m_mqttClient)
    {
        DBG("m_mqttClient not initialized!");
        return -1;
    }
    DBG("send_mqtt: topic: %s, value: %s", topic, text);

    // MQTTClient handle, const char *topicName, int payloadlen, const void *payload, int qos, int retained, MQTTClient_deliveryToken *dt
    int rc = MQTTClient_publish(m_mqttClient, topic, strlen(text), (void*)text, 1, 0, NULL);
    if (rc != MQTTCLIENT_SUCCESS)
    {
        DBG("Failed to publish! topic: %s, value: %s", topic, text);
        return -1;
    }
    return 0;
}
int CVallox::send_mqtt(const char* topic, int value)
{
    char str[12];
    sprintf(str, "%d", value);
    return send_mqtt(topic, str);
}

bool CVallox::hasRecentData(uint8_t param)
{
    long int now = millis();
    std::map<uint8_t, long int>::const_iterator it = m_cacheTime.find(param);
    if (it != m_cacheTime.end())
    {
        // time found from cache
        if ((now - it->second) < 20000)
            return true; // recently updated
    }
    return false;
}
bool isReplyForParam(const _vallox_packet* pPacket, uint8_t param)
{
    if (pPacket->domain == VX_DOMAIN
        && pPacket->sender == VX_MAINBOARD
        && pPacket->receiver == client_id
        && pPacket->param == param)
        {
            // ok, if checksum is ok
            return pPacket->checksum == calculate_checksum((char*)pPacket, 5);
        }
    else
        return false;
}
int CVallox::wait_for_idle()
{
    char buf[1];
    do
    {
        int n = read(m_serialFd, buf, sizeof buf);
        if (n > 0)
            m_lastSerialActivity.start();
        else
            usleep(1000); // wait 1 ms
    } while (m_lastSerialActivity.elapsedUSec() < 5000); // loop until quiet for 5 ms

    return 0;
}
int CVallox::checkAndRefresh(uint8_t param)
{
    if (hasRecentData(param))
        return 0;

    uint8_t value;
    int retVal = queryParam(param, value);
    if (retVal == 0)
    {
        onParamReceived(param, value);
        m_cacheTime[param] = millis();
        m_cacheData[param] = value;
    }
    return retVal;
}
int CVallox::queryParam(uint8_t param, uint8_t& value)
{
    //if (hasRecentData(param))
    //    return 0;

    //DBG("queryParam param: 0x%02X", param);

    char writeBuffer[6];
    create_querybuf(&writeBuffer[0], param);

/*
    DBG("queryParam writeBuffer:");
    for (int i=0; i<6; i++)
    {
        DBG("  0x%02X", (uint8_t)writeBuffer[i]);
    }
*/
    //buf = create_querybuf(param)

    int failurecount = 0;

    wait_for_idle();

    CElapseTimer requestTimer;
    int writed = write(m_serialFd, writeBuffer, sizeof writeBuffer);
    //DBG("write ret: %d", writed);

    m_lastSerialActivity.start();
    requestTimer.start();
    char readBuffer[6];
    while (1)
    {
        if (requestTimer.elapsed() > 20 && m_lastSerialActivity.elapsed() > 3) // Failed if took more than 20 ms, and idle
        {
            failurecount++;
            if (failurecount < 2)
            {
                int writed = write(m_serialFd, writeBuffer, sizeof writeBuffer);
                //DBG("retry, write ret: %d", writed);
                requestTimer.start();
                m_lastSerialActivity.start();
            }
            else
            {
                //DBG("queryParam() failed too many times, abort");
                break;
            }
        }
        int n = read(m_serialFd, readBuffer, sizeof readBuffer);  // read up to x characters if ready to read
        if (n > 0)
            m_lastSerialActivity.start();

        if (n == 6) {
            //DBG("readlen: %d", n);
            //DBG("firstbyte: %d", (int)buf[0] );

            const _vallox_packet* packet = (_vallox_packet*) &readBuffer[0];
            if (isReplyForParam(packet, param))
            {
                // got reply
                long timetook = requestTimer.elapsedUSec();
                //DBG("got reply, timetook %ld us", timetook);

                value = packet->data;
                return 0;
            }
        }
        else if (n <= 0)
        {
            // no data, wait 500 us
            usleep(500);
        }
    }

    // failed
    DBG("queryparam failed");
    return -1;

}
int CVallox::sendRequest(uint8_t param, uint8_t data)
{
    DBG("sendRequest() param: 0x%02X, data: 0x%02X", param, data);

    char writeBuffer[6];
    create_requestbuf(&writeBuffer[0], param, data);

    // TODO: send request to serial

    /*DBG("sendRequest data:");
    for (int i=0; i<6; i++)
    {
        DBG("  0x%02X", (uint8_t)writeBuffer[i]);
    }*/

    wait_for_idle();


    int writed = write(m_serialFd, writeBuffer, sizeof writeBuffer);
    if (writed < 0)
        return -1;
    /*DBG("write ret: %d", writed);
    if (writed < 0){
        DBG("sendRequest() write failed!");
        return -1;
    }*/

    CElapseTimer requestTimer;
    requestTimer.start();

    int failurecount = 0;
    int retVal = -1;
    char readBuffer[1];
    do
    {
        if (requestTimer.elapsed() > 20 && m_lastSerialActivity.elapsed() > 3) // Failed if took more than 20 ms, and idle
        {
            failurecount++;
            if (failurecount < 2)
            {
                int writed = write(m_serialFd, writeBuffer, sizeof writeBuffer);
                //DBG("retry, write ret: %d", writed);
                requestTimer.start();
                m_lastSerialActivity.start();
            }
            else
            {
                DBG("sendRequest() failed too many times, abort");
                break;
            }
        }
        int n = read(m_serialFd, readBuffer, sizeof readBuffer);  // read up to x characters if ready to read
        if (n > 0)
        {
            m_lastSerialActivity.start();
            // we expect first byte == request checksum
            if (readBuffer[0] == writeBuffer[5])
            {
                retVal = 0;
                break;
            }
        }
        else
        {
            // no data, wait 500 us
            usleep(500);
        }

    } while(1);

    DBG("sendRequest() done, retVal: %d", retVal);

    // Invalidate cache
    m_cacheTime.erase(param);
    m_cacheData.erase(param);


    return retVal;
}

void CVallox::create_querybuf(void* pBuf, uint8_t param)
{
    _vallox_packet* pPacket = (_vallox_packet*)pBuf;
    pPacket->domain = VX_DOMAIN;
    pPacket->sender = client_id;
    pPacket->receiver = VX_MAINBOARD;
    pPacket->param = VX_REQ_QUERY;
    pPacket->data = param;
    pPacket->checksum = calculate_checksum((char*)pPacket, 5);
}

void CVallox::create_requestbuf(void* pBuf, uint8_t param, uint8_t data)
{
    _vallox_packet* pPacket = (_vallox_packet*)pBuf;
    pPacket->domain = VX_DOMAIN;
    pPacket->sender = client_id;
    pPacket->receiver = VX_MAINBOARD;
    pPacket->param = param;
    pPacket->data = data;
    pPacket->checksum = calculate_checksum((char*)pPacket, 5);
}

void CVallox::dotests()
{

    /*
    char buf[6];
    _vallox_packet* packet = (_vallox_packet*) &buf[0];

    packet->domain = 0x01;
    packet->sender = VX_MAINBOARD;
    packet->receiver = VX_ALL_CLIENTS;
    packet->param = VX_PARAM_TEMP_OUTDOOR;
    packet->data = 0x52;
    packet->checksum = calculate_checksum((char*)packet, 5);

    handlePacket(packet);

    packet->data = 0x53;
    packet->checksum = calculate_checksum((char*)packet, 5);
    handlePacket(packet);
*/
    CElapseTimer el;

    long int m = millis();
    el.start();
    //usleep(15276);
sleep(20);
    long long elapsed = el.elapsedUSec();
    long int m_elaps = millis() - m;
    DBG("m_elaps  : %ld", m_elaps);
    DBG("u elapsed: %ld", elapsed);


}
