
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <math.h>

#include "debug.h"

static char* g_log_file_name = NULL;

void setLogFilename(const char* filename)
{
    g_log_file_name = strdup(filename);
}
void DBG(const char *str,...)
{
    FILE* file = NULL;
    FILE* stream = NULL;
    if (g_log_file_name)
		file = fopen(g_log_file_name, "a+");
    if (file)
        stream = file;
    else
        stream = stdout;

    char buffer[25];
    int millisec;
    struct tm* tm_info;
    struct timeval tv;

    gettimeofday(&tv, NULL);

    millisec = lrint(tv.tv_usec/1000.0); // Round to nearest millisec
    if (millisec >= 1000) { // Allow for rounding up to nearest second
        millisec -= 1000;
        tv.tv_sec++;
    }

    tm_info = localtime(&tv.tv_sec);
    size_t len = strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S.", tm_info);
    sprintf(buffer + len, "%03d ", millisec);

    len = fwrite(buffer, 1, strlen(buffer), stream);
    if (len > 0) {
        va_list arglist;
        va_start(arglist, str);
        vfprintf(stream, str, arglist);
        va_end(arglist);
        fprintf(stream, "\n");

        fflush(stream);
    }
    if (file)
        fclose(file);
}

void closeLogging()
{
    free(g_log_file_name);
    g_log_file_name = NULL;
}
