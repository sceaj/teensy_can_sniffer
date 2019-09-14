



#include <sdlogger.h>
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "stdio.h"

#define LOG_BUFFER_SIZE 2048
#define LOG_BUFFER_AVAIL (LOG_BUFFER_SIZE - (g_dataBufferNext - g_dataBuffer))
#define LOG_BUFFER_MIN_FREE 80

static FATFS g_fileSystem; /* File system object */
static FIL g_fileObject;   /* File object */
static const char* g_loggerRootpath = "2:/LOGGER";
static char g_loggerPattern[13];
static char g_currentPathname[22];
static char g_dataBuffer[LOG_BUFFER_SIZE];
static char* g_dataBufferNext;

status_t mountSDfilesystem(void) {
	FRESULT fresult = f_mount(&g_fileSystem, "2:/", 1);
	if (fresult == FR_OK) {
		PRINTF("SD filesystem mounted.\r\n");
		return kStatus_Success;
	} else {
		PRINTF("f_mount failed! [%d]\r\n", fresult);
		return kStatus_Fail;
	}
}

status_t listSDfiles(char* path)
{
	FRESULT res;
	DIR dir;
	static FILINFO fno;

	res = f_opendir(&dir, path);                       /* Open the directory */
	if (res == FR_OK) {
		for (;;) {
			res = f_readdir(&dir, &fno);                   /* Read a directory item */
			if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
			PRINTF("%02x %s/%s\r\n", fno.fattrib, path, fno.fname);
		}
		f_closedir(&dir);
	}

	return (res == FR_OK) ? kStatus_Success : kStatus_Fail;
}

status_t dumpSDfile(char* pathname)
{
	FRESULT res;
	FIL fil;

    /* Open a text file */
    res = f_open(&fil, pathname, FA_READ);
    if (res) return kStatus_Fail;

    /* Read all lines and display it */
    while (f_gets((char*)g_dataBuffer, sizeof g_dataBuffer, &fil)) {
        PRINTF((char*)g_dataBuffer);
    }

    /* Close the file */
    f_close(&fil);

    return (res == FR_OK) ? kStatus_Success : kStatus_Fail;
}


status_t openLogger(const char* prefix)
{
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory search object */
    static FILINFO fno;    /* File information */

    static char filename[13];
    memset(filename, 0, sizeof filename);
    // Check that the logging directory exists, create it if not
    fr = f_stat(g_loggerRootpath, &fno);
    // May need to check FR_NO_PATH
    if (fr == FR_NO_FILE) {
    	fr = f_mkdir(g_loggerRootpath);
    }
    if (fr != FR_OK) {
    	return kStatus_LoggerNoDir;
    }
    // Search existing files of requested prefix
    strncpy(g_loggerPattern, prefix, 5);
    strcat(g_loggerPattern, "*.LOG");
    int fileno = 0;

    fr = f_findfirst(&dj, &fno, g_loggerRootpath, g_loggerPattern);  /* Start to search for photo files */
    while (fr == FR_OK && fno.fname[0]) { /* Repeat while an item is found */
    	strcpy(filename, fno.fname);
    	*strrchr(filename, '.') = 0;
    	if (atoi(&filename[strlen(prefix)]) > fileno) {
    		fileno = atoi(&filename[strlen(prefix)]);
    	}
        fr = f_findnext(&dj, &fno); /* Search for next item */
    }
    fr = f_closedir(&dj);
    if (fr == FR_OK) {
		// Create the next filename
		sprintf(filename, "%s%03d", prefix, ++fileno);
		strcpy(g_currentPathname, g_loggerRootpath);
		strcat(g_currentPathname, "/");
		strcat(g_currentPathname, filename);
		strcat(g_currentPathname, ".log");
		// Open the file
		fr = f_open(&g_fileObject, g_currentPathname, FA_CREATE_NEW | FA_WRITE);
		PRINTF("Opening: %s [%s]\r\n", g_currentPathname, fr == FR_OK ? "OK" : "Failed");
		g_dataBufferNext = g_dataBuffer;
    }

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

status_t checkRollLogger(void) {

    FRESULT fr;     /* Return value */
    FILINFO fno;    /* File information */
    char filename[13];
    char filenostr[4];

    // Check the size of the current log
    fr = f_stat(g_currentPathname, &fno);
    if (fr == FR_OK) {
    	// 4MB max log size
    	if (fno.fsize > (4 * 1024 * 1024)) {
    		// Roll file...
    		strcpy(filename, strrchr(g_currentPathname, '/') + 1);
    		*strrchr(filename, '.') = 0;
    		int fileno = atoi(&filename[strlen(filename) - 3]);
    		sprintf(filenostr, "%03d", ++fileno);
    		strcpy(&filename[strlen(filename) - 3], filenostr);
    		strcat(filename, ".log");
    		f_close(&g_fileObject);
    		strcpy(strrchr(g_currentPathname, '/') + 1, filename);
    		fr = f_open(&g_fileObject, g_currentPathname, FA_CREATE_NEW | FA_WRITE);
    		PRINTF("Opening: %s [%s]\r\n", g_currentPathname, fr == FR_OK ? "OK" : "Failed");
    	} else {
    		fr = f_sync(&g_fileObject);
    	}
    }

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

status_t closeLogger(void)
{
	if (strlen(g_dataBuffer)) writeLog(g_dataBuffer);
	FRESULT fr = f_close(&g_fileObject);
	return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

void _formatLog(const char* fmt, va_list args)
{
	vsprintf(g_dataBufferNext, fmt, args);
	g_dataBufferNext += strlen(g_dataBufferNext);
	if (LOG_BUFFER_AVAIL < LOG_BUFFER_MIN_FREE) {
		flushLog();
	}
}

void appendLog(const char* fmt, ...)
{
	va_list argsp;
	va_start(argsp, fmt);
	_formatLog(fmt, argsp);
	va_end(argsp);
}

void putLog(const char* str)
{
	strcpy(g_dataBufferNext, str);
	g_dataBufferNext += strlen(g_dataBufferNext);
	if (LOG_BUFFER_AVAIL < LOG_BUFFER_MIN_FREE) {
		flushLog();
	}

}

void flushLog(void) {
	writeLog(g_dataBuffer);
	g_dataBufferNext = g_dataBuffer;
	checkRollLogger();
}

