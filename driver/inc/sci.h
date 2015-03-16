#ifndef _SCI_H_
#define _SCI_H_

#include "includes.h"

#define CMDBUFSIZE (50)
#define HW_VERSION (10.1)
#define CommandNameLength (10)
typedef struct 
{
	char cmd;
	long value;
}sciObj;
typedef struct
{
	int head;
	char BufOverErrorFlag;
	char FinishFlag;
	char String[CMDBUFSIZE];
}cmdType;
extern xQueueHandle bMessage;

void sciConfiguration( void );
void sciSendTask ( void *pvParameters );
void sciSendChar( unsigned char Data, xQueueHandle messageQueue );
void sciSendHalfword( short halfword, xQueueHandle messageQueue );
void sciSendWord ( long word, xQueueHandle messageQueue );
void sciSendString( char * addr, xQueueHandle messageQueue );
void CLITask ( void *pvParameters );
char* float2String( float number, unsigned char resolution, char* stringBuf );
char* int2String( int number, char* stringBuf );
void sendSerialMessage( int data );
#endif




				    

