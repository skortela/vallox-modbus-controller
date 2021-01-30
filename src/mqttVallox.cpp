/*
 *
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdarg.h>
//#include <syslog.h>
#include <signal.h>
#include <getopt.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <time.h>

#include <ctype.h>
#include <limits.h>

#include <MQTTClient.h>
#include <MQTTClientPersistence.h>


#include "vallox.h"
#include "debug.h"

const char* KDefaultConfFile = "/etc/mqttVallox/mqttVallox.conf";

#define UNUSED(x) (void)(x)

struct Config {
    char* mqttHost;
    char* mqttUsername;
    char* mqttPassword;

    char* clientId;
    char* topic;

    char* serial;
    int vallox_clientID;
} config;


#define QOS         1
#define TIMEOUT 10000L

static int running = 0;
//static int counter = 0;
static char *conf_file_name = NULL;
static char *pid_file_name = NULL;
static int pid_fd = -1;
static char *app_name = NULL;

static CVallox m_vallox;

int isConfigValid()
{
    if (config.mqttHost
        && config.mqttUsername
        && config.mqttPassword
        && config.clientId
        && config.topic
        && config.serial
        && config.vallox_clientID > 0)
        return 1;
    else
        return 0;
}

// Note: This function returns a pointer to a substring of the original string.
// If the given string was allocated dynamically, the caller must not overwrite
// that pointer with the returned value, since the original pointer must be
// deallocated using the same allocator with which it was allocated.  The return
// value must NOT be deallocated using free() etc.
char* trim(char *str)
{
  char* end;

  // Trim leading space
  while(isspace((unsigned char)*str)) str++;

  if(*str == 0)  // All spaces?
    return str;

  // Trim trailing space
  end = str + strlen(str) - 1;
  while(end > str && isspace((unsigned char)*end)) end--;

  // Write new null terminator
  *(end+1) = 0;

  return str;
}
/**
 * \brief This function tries to test config file
 */
int test_conf_file(char *_conf_file_name)
{
	FILE *conf_file = NULL;
	int ret = EXIT_SUCCESS;

	conf_file = fopen(_conf_file_name, "r");

	if (conf_file == NULL) {
		fprintf(stderr, "Can't read config file %s\n",
			_conf_file_name);
		return EXIT_FAILURE;
	}

    const int KBufferSize = 1024;
    char buf[KBufferSize];
    int linenum = 0;
    char* currentTag = NULL;

    while(fgets(buf, KBufferSize, conf_file)!=NULL) {
        linenum++;
        //printf("%s", buf);
        char* pTxt = trim(buf);
        if (strlen(pTxt) == 0)
            continue;
        if (pTxt[0] == '#')
            continue;

        if (pTxt[0] == '[') {
            char * pch = strchr(pTxt, ']');
            if (!pch) {
                // log failure
                ret = EXIT_FAILURE;
                DBG("Failed to parse configuretion file!");
                break;
            }
            pch[0] = '\0';
            if (currentTag)
                free(currentTag);
            currentTag = strdup(pTxt+1);
            //printf("currentTag: '%s'\n", currentTag);
            continue;
        }


        char * pch = strchr(pTxt, '=');
        if (pch) {
            char* key = pTxt;
            key[pch-pTxt] = '\0';
            char* value = pch+1;

            key = trim(key);
            value = trim(value);
            printf("key: '%s'\n", key);
            printf("value: %s\n", value);

            if (currentTag && strcmp(currentTag, "MAIN") == 0) {
                if (strcmp(key, "clientid") == 0) {
                    config.clientId = strdup(value);
                }
                else if (strcmp(key, "listentopic") == 0)
                    config.topic = strdup(value);
            }
            else if (currentTag && strcmp(currentTag, "MQTT") == 0) {
                if (strcmp(key, "host") == 0)
                    config.mqttHost = strdup(value);

                else if (strcmp(key, "username") == 0)
                    config.mqttUsername = strdup(value);
                else if (strcmp(key, "password") == 0)
                    config.mqttPassword = strdup(value);
            }
            else if (currentTag && strcmp(currentTag, "VALLOX") == 0) {
                if (strcmp(key, "serial") == 0) {
                    config.serial = strdup(value);
                }
                else if (strcmp(key, "clientid") == 0) {
                    config.vallox_clientID = (int)strtol(value, NULL, 0);
                }
            }
            /*else if (currentTag && strcmp(currentTag, "DATABASE") == 0) {
                if (strcmp(key, "host") == 0) {
                    config.dbHost = strdup(value);
                }
                else if (strcmp(key, "port") == 0) {
                   config.dbPort = atoi(value);
                   if (config.dbPort == 0) {
                       ret = EXIT_FAILURE;
                       DBG("Failed to parse integer from 'port'");
                   }
                }
                else if (strcmp(key, "username") == 0)
                    config.dbUsername = strdup(value);
                else if (strcmp(key, "password") == 0)
                    config.dbPassword = strdup(value);
                else if (strcmp(key, "database") == 0)
                    config.dbName = strdup(value);
            }
            else if (currentTag && strcmp(currentTag, "FILEUPLOAD") == 0) {
                if (strcmp(key, "handlerurl") == 0)
                    config.fileUploadHandler = strdup(value);
                else if (strcmp(key, "extraheader") == 0)
                    config.fileUploadKeyHeader = strdup(value);
            }*/
        }
    }

    if (currentTag) {
        free(currentTag);
    }

	fclose(conf_file);

    if (!isConfigValid()) {
        printf("config is not valid!\n");
        ret = EXIT_FAILURE;
    }

/*
    printf("Testing db connection..\n");
    MYSQL* sql = SqlConnect();
    if (sql) {
        printf(" SUCCEEDED\n");
        mysql_close(sql);
    }
    else {
        ret = EXIT_FAILURE;
        printf(" FAILED!\n");
    }
*/

	return ret;
}
/**
 * \brief Read configuration from config file
 */
int read_conf_file(int reload)
{
	FILE *conf_file = NULL;
	int ret = 0;

    const char* filename = conf_file_name;

	if (!filename) {
        DBG("Config not defined, using defauft: %s", KDefaultConfFile);
        filename = KDefaultConfFile;
    }

	conf_file = fopen(filename, "r");

	if (conf_file == NULL) {
		DBG("Can not open config file: %s, error: %s", filename, strerror(errno));
		return -1;
	}

    const int KBufferSize = 1024;
    char buf[KBufferSize];
    int linenum = 0;
    char* currentTag = NULL;

    while(fgets(buf, KBufferSize, conf_file)!=NULL) {
        linenum++;
        //printf("%s", buf);
        char* pTxt = trim(buf);
        if (strlen(pTxt) == 0)
            continue;
        if (pTxt[0] == '#')
            continue;

        if (pTxt[0] == '[') {
            char * pch = strchr(pTxt, ']');
            if (!pch) {
                // log failure
                ret = EXIT_FAILURE;
                DBG("Error detected on config file on line: %d", linenum);
                break;
            }
            pch[0] = '\0';
            if (currentTag)
                free(currentTag);
            currentTag = strdup(pTxt+1);
            continue;
        }


        char * pch = strchr(pTxt, '=');
        if (pch) {
            char* key = pTxt;
            key[pch-pTxt] = '\0';
            char* value = pch+1;

            key = trim(key);
            value = trim(value);
            //printf("key: '%s'\n", key);
            //printf("value: %s\n", value);
            if (currentTag && strcmp(currentTag, "MAIN") == 0) {
                if (strcmp(key, "clientid") == 0) {
                    config.clientId = strdup(value);
                }
                else if (strcmp(key, "listentopic") == 0)
                    config.topic = strdup(value);
                else if (strcmp(key, "logfile") == 0) {
                    setLogFilename(value);
                    //if (!log_file_name)
                    //    log_file_name = strdup(value);
                }
            }
            else if (currentTag && strcmp(currentTag, "MQTT") == 0) {
                if (strcmp(key, "host") == 0)
                    config.mqttHost = strdup(value);

                else if (strcmp(key, "username") == 0)
                    config.mqttUsername = strdup(value);
                else if (strcmp(key, "password") == 0)
                    config.mqttPassword = strdup(value);
            }
            else if (currentTag && strcmp(currentTag, "VALLOX") == 0) {
                if (strcmp(key, "serial") == 0) {
                    config.serial = strdup(value);
                }
                else if (strcmp(key, "clientid") == 0) {
                    config.vallox_clientID = (int)strtol(value, NULL, 0);
                }
            }
            /*else if (currentTag && strcmp(currentTag, "DATABASE") == 0) {
                if (strcmp(key, "host") == 0) {
                    config.dbHost = strdup(value);
                }
                else if (strcmp(key, "port") == 0) {
                   config.dbPort = atoi(value);
                   if (config.dbPort == 0) {
                       ret = EXIT_FAILURE;
                       DBG("Line: %d: Failed to parse integer from 'port'", linenum);
                   }
                }
                else if (strcmp(key, "username") == 0)
                    config.dbUsername = strdup(value);
                else if (strcmp(key, "password") == 0)
                    config.dbPassword = strdup(value);
                else if (strcmp(key, "database") == 0)
                    config.dbName = strdup(value);
            }
            else if (currentTag && strcmp(currentTag, "FILEUPLOAD") == 0) {
                if (strcmp(key, "handlerurl") == 0)
                    config.fileUploadHandler = strdup(value);
                else if (strcmp(key, "extraheader") == 0)
                    config.fileUploadKeyHeader = strdup(value);
            }*/
        }
    }

    if (currentTag)
        free(currentTag);

    fclose(conf_file);

	if (ret == 0) {
		if (reload == 1) {
			DBG("Reloaded configuration file %s", filename);
		}
	}

    if (!isConfigValid()) {
        DBG("Config is not valid!");
        ret = -1;
    }

	return ret;
}


/**
 * \brief Callback function for handling signals.
 * \param	sig	identifier of signal
 */
void handle_signal(int sig)
{
	if (sig == SIGINT || sig == SIGTERM) {
        if (sig == SIGINT) {
    		DBG("SIGINT: stopping mqttVallox ...");
        }
        else if (sig == SIGTERM) {
    		DBG("SIGTERM: stopping mqttVallox ...");
        }
		/* Unlock and close lockfile */
		if (pid_fd != -1) {
			int rc = lockf(pid_fd, F_ULOCK, 0);
            UNUSED(rc);
			close(pid_fd);
		}
		/* Try to delete lockfile */
		if (pid_file_name != NULL) {
			unlink(pid_file_name);
		}
		running = 0;
		/* Reset signal handling to default behavior */
		signal(SIGINT, SIG_DFL);
        signal(SIGTERM, SIG_DFL);
	} else if (sig == SIGHUP) {
		DBG("SIGHUP: reloading mqttVallox config file ...");
		read_conf_file(1);
	} else if (sig == SIGCHLD) {
		DBG("SIGCHLD: received SIGCHLD signal");
	}
}

/**
 * \brief This function will daemonize this app
 */
static void daemonize()
{
	pid_t pid = 0;
	int fd;

	/* Fork off the parent process */
	pid = fork();

	/* An error occurred */
	if (pid < 0) {
		exit(EXIT_FAILURE);
	}

	/* Success: Let the parent terminate */
	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}

	/* On success: The child process becomes session leader */
	if (setsid() < 0) {
		exit(EXIT_FAILURE);
	}

	/* Ignore signal sent from child to parent process */
	signal(SIGCHLD, SIG_IGN);

	/* Fork off for the second time*/
	pid = fork();

	/* An error occurred */
	if (pid < 0) {
		exit(EXIT_FAILURE);
	}

	/* Success: Let the parent terminate */
	if (pid > 0) {
		exit(EXIT_SUCCESS);
	}

	/* Set new file permissions */
	umask(0);

	/* Change the working directory to the root directory */
	/* or another appropriated directory */
	int rc = chdir("/");
    UNUSED(rc);

	/* Close all open file descriptors */
	for (fd = sysconf(_SC_OPEN_MAX); fd > 0; fd--) {
		close(fd);
	}

	/* Reopen stdin (fd = 0), stdout (fd = 1), stderr (fd = 2) */
	stdin = fopen("/dev/null", "r");
	stdout = fopen("/dev/null", "w+");
	stderr = fopen("/dev/null", "w+");

	/* Try to write PID of daemon to lockfile */
	if (pid_file_name != NULL)
	{
		char str[256];
		pid_fd = open(pid_file_name, O_RDWR|O_CREAT, 0640);
		if (pid_fd < 0) {
			/* Can't open lockfile */
            DBG("Failed to open lockfile %s", pid_file_name);
			exit(EXIT_FAILURE);
		}
		if (lockf(pid_fd, F_TLOCK, 0) < 0) {
			/* Can't lock file */
            DBG("Can't lock file %s", pid_file_name);
			exit(EXIT_FAILURE);
		}
		/* Get current PID */
		sprintf(str, "%d\n", getpid());
		/* Write PID to lockfile */
		rc = write(pid_fd, str, strlen(str));
        UNUSED(rc);
	}
}


/*void printErr(MYSQL *con) {
    DBG("SQL error: %s", mysql_error(con));
}*/

int isNumeric(const char* value) {
    size_t i;
    for (i = 0; i < strlen(value); i++) {
        if (i == 0 && (value[i] == '-' || value[i] == '+'))
            continue;
        if (value[i] == '.')
            continue;
        if (!isdigit(value[i]))
            return 0;
    }
    return 1;
}
/*



struct dataStruct {
    void* dataPtr;
    size_t bytesLeft;
};
*/

/*
static size_t read_callback(void *ptr, size_t size, size_t nmemb, void *userp)
{
    size_t dataLen = size*nmemb;

    struct dataStruct *data = (struct dataStruct *)userp;

    if (dataLen > data->bytesLeft)
        dataLen = data->bytesLeft;


    memcpy(ptr, data->dataPtr, dataLen);

    data->dataPtr = (char*)data->dataPtr + dataLen;
    data->bytesLeft -= dataLen;

    return dataLen;
}*/


void delivered(void* context, MQTTClient_deliveryToken dt)
{
    UNUSED(context);
    UNUSED(dt);
    //printf("Message with token value %d delivery confirmed\n", dt);
    //deliveredtoken = dt;
}

int msgarrvd(void* context, char *topicName, int topicLen, MQTTClient_message *message)
{
    UNUSED(context);
    UNUSED(topicLen);
    // ignore retained messages
    if (message->retained == 0)
    {
        //printf("Message arrived\n");
        //printf("     topic: %s\n", topicName);
        DBG("Message arrived, topic: %s", topicName);

        const char* cmd = strrchr(topicName, '/');
        if (cmd != NULL)
        {
            cmd++; // adjust one byte
            DBG("cmd: %s", cmd);
        }

        if (message->payloadlen > 512)
        {
            //sendFile(topicName, message->payload, message->payloadlen);
        }
        else
        {
            char buffer[message->payloadlen + 1];
            memcpy( &buffer, message->payload, message->payloadlen);
            buffer[message->payloadlen] = '\0';

            DBG("payload: %s", &buffer);

            if (strcmp(cmd, "fan_speed") == 0)
            {
                long int speed = (int)strtol(buffer, NULL, 0);
                if (speed == 0 || speed == LONG_MAX || speed == LONG_MIN)
                {
                    DBG("Failed to parse fan_speed value!");
                }
                else
                {
                    int retVal = m_vallox.setFanSpeed(speed);
                    if (retVal != 0)
                    {
                        DBG("setFanSpeed [ %d ] failed!", speed);
                    }
                }
            }
            else if (strcmp(cmd, "takkakytkin") == 0)
            {
                if (strcmp(&buffer[0], "enable") == 0
                    || strcmp(&buffer[0], "activate") == 0
                    || strcmp(&buffer[0], "on") == 0
                    || strcmp(&buffer[0], "1") == 0)
                    {
                        int retVal = m_vallox.activateTakkakytkin();
                        if (retVal != 0)
                        {
                            DBG("activateTakkakytkin failed!");
                        }
                    }
                else {
                    DBG("Invalid payload on 'takkakytkin' cmnd");
                }
            }
            else if (strcmp(cmd, "post_heating_temp") == 0)
            {
                long int temp = (int)strtol(buffer, NULL, 0);
                if (temp == 0 || temp == LONG_MAX || temp == LONG_MIN)
                {
                    DBG("Failed to parse post_heating_temp value!");
                }
                else
                {
                    int retVal = m_vallox.setPostHeatingTemp(temp);
                    if (retVal != 0)
                    {
                        DBG("setPostHeatingTemp [ %d ] failed!", temp);
                    }
                }
            }
            else
            {
                DBG("unsupported cmnd received: %s", cmd);
            }

            //printf("     message: %s\n", &buffer);

            //MYSQL* sql = SqlConnect();
            //sqlAddTopicValue(sql, topicName, buffer);
            //mysql_close(sql);
        }
    }

    MQTTClient_freeMessage(&message);
    MQTTClient_free(topicName);
    return 1;
}

void connlost(void* context, char *cause)
{
    UNUSED(context);
    if (cause) {
        DBG("Connection lost, cause: %s", cause);
    }
    else {
        DBG("Connection lost");
    }
}

/*void finish_with_error(MYSQL *con)
{
    DBG("SQL error: %s", mysql_error(con));
    mysql_close(con);
    exit(1);
}

MYSQL* SqlConnect() {
    MYSQL *mysql = mysql_init(NULL);
    if (mysql == NULL)
    {
        DBG("mysql_init failed: %s", mysql_error(mysql));
        exit(1);
    }

    if (mysql_real_connect(mysql, config.dbHost, config.dbUsername, config.dbPassword,
        config.dbName, config.dbPort, NULL, 0) == NULL)
    {
        DBG("mysql connect failed: %s", mysql_error(mysql));
        mysql_close(mysql);
        exit(1);
    }
    mysql_set_character_set(mysql, "utf8");
    return mysql;
}
*/




void run()
{

    MQTTClient client;
    MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
    int rc;

    MQTTClient_create(&client, config.mqttHost, config.clientId,
      MQTTCLIENT_PERSISTENCE_NONE, NULL);

    conn_opts.keepAliveInterval = 20;
    conn_opts.cleansession = 1;
    conn_opts.username = config.mqttUsername;
    conn_opts.password = config.mqttPassword;

    MQTTClient_setCallbacks(client, NULL, connlost, msgarrvd, delivered);

    /*rc = MQTTClient_connect(client, &conn_opts);
    if (rc != MQTTCLIENT_SUCCESS)
    {
      //printf("Failed to connect, return code %d\n", rc);
        DBG("MQTTClient_connect: Failed to connect\n");
        exit(-1);
    }
    else {
        DBG("MQTTClient_connect: Succeeded\n");
    }*/
    //printf("Subscribing to topic %s\nfor client %s using QoS%d\n\n", TOPIC, CLIENTID, QOS);

    int wasConnected = 0;
    int connectionErrPrinted = 0;
    int serialErr = -1;


    m_vallox.setMQTTClient(client);

    //m_vallox.dotests();
    //running = false;

    while(running)
    {
        if (!MQTTClient_isConnected(client)) {
            if (wasConnected) {
                DBG("Disconnected");
            }
            wasConnected = 0;
            rc = MQTTClient_connect(client, &conn_opts);
            if (rc == MQTTCLIENT_SUCCESS)
            {
                DBG("MQTTClient_connect: Succeeded");
                MQTTClient_subscribe(client, config.topic, QOS);
                wasConnected = 1;
                connectionErrPrinted = 0;
            }
            else if (!connectionErrPrinted) {
                DBG("MQTTClient_connect: Failed to connect");
                connectionErrPrinted = 1;
                sleep(10);
            }
        }

        //m_vallox.dotests();
        //break;

        if (serialErr != 0) {
            serialErr = m_vallox.openSerial(config.serial);
            if (serialErr != 0)
                break;
        }

        if (!serialErr) {
            int err = m_vallox.handleLoop();
            if (err < 0) {
                DBG("handleLoop failed, terminate");
                break;
            }
        }

        //sleep(1);
    }

    m_vallox.setMQTTClient(NULL);
    m_vallox.closev();
    MQTTClient_disconnect(client, 10000);
    MQTTClient_destroy(&client);
    DBG("MQTTClient terminated");
}
/**
 * \brief Print help for this application
 */
void print_help(void)
{
	printf("\n Usage: %s [OPTIONS]\n\n", app_name);
	printf("  Options:\n");
	printf("   -h --help                 Print this help\n");
	printf("   -c --conf_file filename   Read configuration from the file\n");
	printf("   -t --test_conf filename   Test configuration file\n");
	printf("   -l --log_file  filename   Write logs to the file\n");
	printf("   -d --daemon               Daemonize this application\n");
	printf("   -p --pid_file  filename   PID file used by daemonized app\n");
	printf("\n");
}

/* Main function */
int main(int argc, char *argv[])
{
	static struct option long_options[] = {
		{"conf_file", required_argument, 0, 'c'},
		{"test_conf", required_argument, 0, 't'},
		{"log_file", required_argument, 0, 'l'},
		{"help", no_argument, 0, 'h'},
		{"daemon", no_argument, 0, 'd'},
		{"pid_file", required_argument, 0, 'p'},
		{NULL, 0, 0, 0}
	};
	int value;
    int option_index = 0;
	int start_daemonized = 0;

	app_name = argv[0];

	/* Try to process all command line arguments */
	while ((value = getopt_long(argc, argv, "c:l:t:p:dh", long_options, &option_index)) != -1) {
		switch (value) {
			case 'c':
				conf_file_name = strdup(optarg);
				break;
			case 'l':
				//log_file_name = strdup(optarg);
                setLogFilename(optarg);
				break;
			case 'p':
				pid_file_name = strdup(optarg);
				break;
			case 't':
				return test_conf_file(optarg);
			case 'd':
				start_daemonized = 1;
				break;
			case 'h':
				print_help();
				return EXIT_SUCCESS;
			case '?':
				print_help();
				return EXIT_FAILURE;
			default:
				break;
		}
	}

	/* When daemonizing is requested at command line. */
	if (start_daemonized == 1) {
		/* It is also possible to use glibc function deamon()
		 * at this point, but it is useful to customize your daemon. */
		daemonize();
	}

	/* Open system log and write message to it */
	//openlog(argv[0], LOG_PID|LOG_CONS, LOG_DAEMON);
	//syslog(LOG_INFO, "Started %s", app_name);

	/* Daemon will handle two signals */
	signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
	signal(SIGHUP, handle_signal);



	/* Read configuration from config file */
	int ret = read_conf_file(0);

    if (ret == 0) {
    	/* This global variable can be changed in function handling signal */
    	running = 1;

        /*if (log_file_name) {
            // Test log file writing
            FILE* file = fopen(log_file_name, "a+");
            if (!file) {
                // Failed to open log file
                char* filename = log_file_name;
                log_file_name = NULL;
                DBG("Failed to open logfile %s", filename);
                free(filename);
            }
            else
                fclose(file);
        }*/

        DBG("Started.");
        run();
    }
    else {
        DBG("Failed to parse config file!");
    }

    DBG("Stopped.");

	/* Free allocated memory */
	if (conf_file_name != NULL) free(conf_file_name);
	//if (log_file_name != NULL) free(log_file_name);
    closeLogging();
	if (pid_file_name != NULL) free(pid_file_name);

	return EXIT_SUCCESS;
}
