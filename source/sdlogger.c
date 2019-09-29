



#include <sdlogger.h>
#include "diskio.h"
#include "fsl_sd_disk.h"
#include "fsl_debug_console.h"
#include "ff.h"
#include "stdio.h"

#define LOG_BUFFER_SIZE 8192
#define LOG_BUFFER_OVERFLOW 128
#define LOG_BUFFER_AVAIL ((LOG_BUFFER_SIZE + LOG_BUFFER_OVERFLOW) - (g_dataBufferNext - g_dataBuffer))
#define LOG_BUFFER_MIN_FREE 128

static FATFS g_fileSystem; /* File system object */
static FIL g_fileObject;   /* File object */
static const char* g_loggerRootpath = "2:/LOGGER";
static char g_loggerPrefix[6];
static int g_fileNum;
static int g_fileSize;
static char g_loggerPattern[13];
static char g_currentPathname[22];
static char g_dataBuffer[LOG_BUFFER_SIZE + LOG_BUFFER_OVERFLOW];
static char* g_dataBufferNext;
static char* g_dataBufferOverflow;

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


void setPathname(int filenum)
{
	static char filename[13];

	sprintf(filename, "%s%03d", g_loggerPrefix, filenum);
	strcpy(g_currentPathname, g_loggerRootpath);
	strcat(g_currentPathname, "/");
	strcat(g_currentPathname, filename);
	strcat(g_currentPathname, ".LOG");
}

status_t openLogger(const char* prefix)
{
    FRESULT fr;     /* Return value */
    DIR dj;         /* Directory search object */
    static FILINFO fno;    /* File information */

    strcpy(g_loggerPrefix, prefix);

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
    int fileNum = 0;

    fr = f_findfirst(&dj, &fno, g_loggerRootpath, g_loggerPattern);  /* Start to search for logger files */
    while (fr == FR_OK && fno.fname[0]) { /* Repeat while an item is found */
    	strcpy(filename, fno.fname);
    	*strrchr(filename, '.') = 0;
    	int currentFileNum = atoi(&filename[strlen(prefix)]);
    	if (currentFileNum > fileNum) {
    		fileNum = currentFileNum;
    	}
        fr = f_findnext(&dj, &fno); /* Search for next item */
    }
    fr = f_closedir(&dj);

    g_fileNum = ++fileNum;
    g_dataBufferNext = g_dataBuffer;
    g_dataBufferOverflow = g_dataBuffer + LOG_BUFFER_SIZE;

    if (fr == FR_OK) {
		// Create the next filename
    	setPathname(g_fileNum);
		// Open the file
		fr = f_open(&g_fileObject, g_currentPathname, FA_CREATE_NEW | FA_WRITE);
		PRINTF("Opening: %s [%s]\r\n", g_currentPathname, fr == FR_OK ? "OK" : "Failed");
		g_fileSize = 0;
    }

    return (fr == FR_OK) ? kStatus_Success : kStatus_Fail;
}

status_t checkRollLogger(void) {

    FRESULT fr;     /* Return value */
	// 32MB max log size
	if (g_fileSize > (32UL * 1024UL * 1024UL)) {
		// Roll file...
		f_close(&g_fileObject);
		setPathname(++g_fileNum);
		fr = f_open(&g_fileObject, g_currentPathname, FA_OPEN_ALWAYS | FA_WRITE);
		PRINTF("Opening: %s [%s]\r\n", g_currentPathname, fr == FR_OK ? "OK" : "Failed");
		g_fileSize = 0;
	} else {
		fr = f_sync(&g_fileObject);
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
	unsigned writeSize;
	f_write(&g_fileObject, g_dataBuffer, LOG_BUFFER_SIZE, &writeSize);
	g_fileSize += writeSize;
	strcpy(g_dataBuffer, g_dataBufferOverflow);
	g_dataBufferNext = g_dataBuffer + strlen(g_dataBuffer);
	*g_dataBufferOverflow = 0;
	checkRollLogger();
}

