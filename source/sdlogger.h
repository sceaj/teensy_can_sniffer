

#ifndef __SDLOGGER_H__
#define __SDLOGGER_H__

#include <stdarg.h>
#include "fsl_common.h"

#define readLog(buf,bufsz) f_gets(buf, bufsz, &g_fileObject);
#define writeLog(buf) f_puts(buf, &g_fileObject);

enum _logger_status
{
    kStatus_LoggerNoDir = MAKE_STATUS(121, 1),
	kStatus_LoggerDirError = MAKE_STATUS(121,2)
};


status_t waitForSDcard(void);

status_t testSDcardAccess(void);

status_t mountSDfilesystem(void);

status_t listSDfiles(char* path);

status_t dumpSDfile(char* pathname);

status_t openLogger(const char* prefix);

status_t checkRollLogger(void);

status_t closeLogger(void);

void appendLog(const char* fmt, ...);

void putLog(const char* str);

void flushLog(void);

#endif
