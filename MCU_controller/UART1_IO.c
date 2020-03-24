/*
 * UART1_IO.c
 *
 * Created: 11-Jun-19 9:27:07 AM
 *  Author: teamv
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include "UART1_IO.h"
//#include "UART_debug_v2.h"

//variabels
/*
enum RESULT{ERROR, 
			SUCCES,
			TO_SHORT,
			NO_MSG,
			NO_STARTBYTE,
			MSG_LENGHT_ERROR,
			CHECK_SUM_ERROR};*/

volatile int bytesInBuffer=0;
volatile int rxWritePos=0;
int rxReadPos=0;
volatile char rxBuffer[RX_BUFFER_SIZE];

//functions for recieve
void fixError(char bufData[], int length, int startBytePos);
enum RESULT checkSum(char data[], int bufLength, int msgLength, int startPos);
enum RESULT ZERO16(char MSB, char LSB);
int getDataFromBuffer(char data[]);
int findStartBytes(char data[], int length);
int getLengthOfMsg(char data[], int startPos, int length);
void setLRC8( unsigned char firstFive[], unsigned char data[], unsigned int length, unsigned char checkBytes[2] );
void setLRC8_pointer(unsigned char firstFive[], unsigned char *data, unsigned int length, unsigned char checkBytes[2] );

//functions for transmit
void setPackLength(unsigned char first5Bytes[], unsigned int dataLength);
void setZERO16(unsigned char checkBytes[2]);

void initUART1(){
	//set baudrate
	UBRR1H=(BRC_LABVIEW>>8);
	UBRR1L=BRC_LABVIEW;
	
	//full duplex
	UCSR1A=(1<<U2X1);
	
	//set data bit length
	UCSR1C=(1<<UCSZ11)|(1<<UCSZ10); //8 bit
	
	//enable tx and rx
	UCSR1B|=(1<<TXEN1)|(1<<RXEN1);
	
	//enable rx interrupt
	UCSR1B|=(1<<RXCIE1);
}

/*
	If msg from labView available, return the type and data 
*/
enum RESULT getMsgUART1(unsigned char data[]){
	if (bytesInBuffer<9)//smallest possible packet length
		return NO_MSG;
		
	//get data from buffer
	char bufData[RX_BUFFER_SIZE]={'\0'};
	int dataSize=getDataFromBuffer(bufData);

	//find start bytes
	int startBytePos=findStartBytes(bufData,dataSize);
	if(startBytePos==-1)
		return NO_STARTBYTE;

	//check length
	int msgLength=getLengthOfMsg(bufData,startBytePos,dataSize);
	if(msgLength>11){//hvorfor 11 og ikke ==-1?????
		fixError(bufData, dataSize, startBytePos);
		return MSG_LENGHT_ERROR;
	}
	
	//check checksum
	enum RESULT checkSumResult=checkSum(bufData, dataSize, msgLength, startBytePos);
	if(checkSumResult==CHECK_SUM_ERROR){
		fixError(bufData, dataSize, startBytePos);
		return CHECK_SUM_ERROR;
	}else if(checkSumResult==TO_SHORT){
		return TO_SHORT;
	}
	
	//get data and return
	//data[0]=bufData[startBytePos+4];
	
	uint8_t cnt=0;
	for (uint8_t i=(startBytePos+5); i<msgLength+startBytePos-2; i++)
	{
		data[cnt++]=bufData[i];
	}
	
	//move pointers
	uint8_t movePos=msgLength+startBytePos;
	rxReadPos=(rxReadPos+movePos)%RX_BUFFER_SIZE;
	bytesInBuffer=bytesInBuffer-movePos;
	return SUCCES;
}

/*
	If error in recieved packet, then remove it from buffer
*/
void fixError(char bufData[], int length, int startBytePos){
	
	uint8_t startByteFound=0;
	int i=startBytePos+1;
	int cnt=1;

	while(i<length-1 && startByteFound==0){
		if((bufData[i]==0x55 && bufData[i+1]==0xAA )||startByteFound==1){
			startByteFound=1;
		}else{
			i++;
			cnt++;
		}
	}
	rxReadPos=(rxReadPos+cnt)%RX_BUFFER_SIZE;
	bytesInBuffer-=cnt;
	
}

/*
	calculate check sum
*/
enum RESULT checkSum(char data[], int bufLength, int msgLength, int startPos){
	if((startPos+msgLength)>bufLength){
		return TO_SHORT;
	}
	
	uint8_t sum=0;
	
	for (int i=startPos; i<(msgLength+startPos-2); i++)
	{
		sum=sum^data[i];
	}
	
	if(data[startPos+msgLength-1]==sum){
		return SUCCES;
	}
	return CHECK_SUM_ERROR;
	
	//return ZERO16(data[startPos+msgLength-2], data[startPos+msgLength-1]);
}

/*
	check zero 16 algoritme
*/
enum RESULT ZERO16(char MSB, char LSB){
	if(MSB||LSB)
		return CHECK_SUM_ERROR;
	else
		return SUCCES;
}



/*
	move data between head and tail in circular buffer to data, and return size of data 
*/
int getDataFromBuffer(char data[]){
	int cnt=0;
	int read=rxReadPos;
	
	while (read!=rxWritePos){
		data[cnt++]=rxBuffer[read];
		read=(read+1)%RX_BUFFER_SIZE;
	}
	return cnt;
}

/*
Returns position of first start byte.
if no start byte, return -1
*/
int findStartBytes(char data[], int length){
	for (int i=0; i<length; i++)
	{
		if(data[i]==0x00 && data[i+1]==0x00 && data[i+2]==0x00 && data[i+3]==0x00)
		return i;
	}
	return -1;
}

/*
	find length of packet
*/
int getLengthOfMsg(char data[], int startPos, int length){
	if (startPos+5<length) //4 startbytes before packet length
		return (data[startPos+4]<<8)|(uint8_t) data[startPos+5];
	else
		return -1;
}

/*
Transmit data
type is one char type 0x01, 0x02, 0x03
data is a char array with the length defined in the integer dataLength
*/
void transmitLVData(unsigned char type, unsigned char data[], unsigned int dataLength){
	 unsigned char first5Bytes[5]={0x55, 0xAA, 0x00, 0x00, type};
	setPackLength(first5Bytes, dataLength);
	
	unsigned char checkBytes[2]={0x00};
	//setZERO16(checkBytes);
	setLRC8(first5Bytes, data, dataLength, checkBytes);
	
	//transmit sync, length and type
	for (uint8_t i=0; i<5; i++)
	{
		while(!(UCSR1A &(1<<UDRE1)));
		UDR1=(first5Bytes[i]);
	}
	
	//transmit data
	for (int i=0; i<dataLength; i++)
	{
		while(!(UCSR1A &(1<<UDRE1)));
		UDR1=(data[i]);
	}
	
	//transmit check sum
	for (int i=0; i<2; i++)
	{
		while(!(UCSR1A &(1<<UDRE1)));
		UDR1=(checkBytes[i]);
	}
}

void transmitLVOscilloscope( char type, uint8_t *data, unsigned int dataLength){
	unsigned char first5Bytes[5]={0x55, 0xAA, 0x00, 0x00, type};
	setPackLength(first5Bytes, dataLength);
	
	unsigned char checkBytes[2]={0x00};
	//setZERO16(checkBytes);
	setLRC8_pointer(first5Bytes, data, dataLength, checkBytes);
	
	//transmit sync, length and type
	for (uint8_t i=0; i<5; i++)
	{
		while(!(UCSR1A &(1<<UDRE1)));
		UDR1=(first5Bytes[i]);
	}
	
	//transmit data
	for (unsigned int i=0; i<dataLength; i++)
	{
		while(!(UCSR1A &(1<<UDRE1)));
		UDR1=*(data+i);
		//sendInt(*(data+i));
	}
	
	//transmit check sum
	for (int i=0; i<2; i++)
	{
		while(!(UCSR1A &(1<<UDRE1)));
		UDR1=(checkBytes[i]);
	}
}


/*
	calculate the length of the packet, and set the length bytes
*/
void setPackLength(unsigned char first5Bytes[], unsigned int dataLength){
	int totalLength=dataLength+7;
	first5Bytes[2]=(totalLength>>8);
	first5Bytes[3]=totalLength;
}

/*
	calculate and sets check sum bytes 
*/
void setZERO16(unsigned char checkBytes[2]){
	checkBytes[0]=0x00;
	checkBytes[1]=0x00;
}

void setLRC8(unsigned char firstFive[], unsigned char data[], unsigned int length, unsigned char checkBytes[2] ){
	uint8_t res=0x00;
	for (uint8_t i=0 ; i<5 ; i++)
	{
		res=res^firstFive[i];
	}
	for (int i=0; i< length; i++)
	{
		res=res^data[i];
	}
	checkBytes[0]=0x00;
	checkBytes[1]=res;
}

void setLRC8_pointer(unsigned char firstFive[], unsigned char *data, unsigned int length, unsigned char checkBytes[2] ){
	uint8_t res=0x00;
	for (uint8_t i=0 ; i<5 ; i++)
	{
		res=res^*(firstFive+i);
	}
	for (int i=0; i< length; i++)
	{
		res=res^(*(data+i));
	}
	checkBytes[0]=0x00;
	checkBytes[1]=res;
}

uint8_t getBytesInBuffer(){
	return bytesInBuffer;
}

/************************************************************************/
/* ISR                                                                     */
/************************************************************************/
ISR(USART1_RX_vect){
	rxBuffer[rxWritePos++] = UDR1;
	bytesInBuffer++;
	if(rxWritePos >= RX_BUFFER_SIZE)
	{
		rxWritePos = 0;
	}
}

