
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>

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

    char buff[30];
    struct tm my_time;
    time_t now;
    now = time(NULL);
    my_time = *(localtime(&now));
    size_t len = strftime(buff, sizeof(buff),"%d-%m-%Y %H:%M:%S ", &my_time);

    len = fwrite(buff, len, 1, stream);
    if (len > 0) {
        //fprintf(stream,"%d : ",time(NULL));
        va_list arglist;
        va_start(arglist,str);
        vfprintf(stream,str,arglist);
        va_end(arglist);
        fprintf(stream," \n");

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
