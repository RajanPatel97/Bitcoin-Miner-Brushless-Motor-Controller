#include "mbed.h"
#include "Crypto_light/hash/SHA256.h"
#include "mbed-rtos/rtos/rtos.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8  

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Enum for putMessage message types
#define MSG_HASHCOUNT 0
#define MSG_NONCE_OK 1
#define MSG_OVERFLOW 2

//FIFO constant definitions
#define MAX_ARRAY_SIZE 256 //check the variable fifo position if this is changed

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/

//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};  
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

RawSerial pc(SERIAL_TX, SERIAL_RX); 
 
//structure for Mail Class
typedef struct {
     uint8_t code;
     uint32_t data;
} message_t ;
 
//Mail class allowing 16 messages to be stored up in the FIFO
Mail<message_t,16> outMessages;


void putMessage(uint8_t code, uint32_t data)
{
    message_t *pMessage = outMessages.alloc();
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
}

Thread commOutT;
 
void commOutFn()
{  
    while(1) {
        osEvent newEvent = outMessages.get();
        message_t *pMessage = (message_t*)newEvent.value.p;
        pc.printf("Message %d with data 0x%016x\r\n",
                    pMessage->code,pMessage->data);
        outMessages.free(pMessage);
     }
}
//Global varible for the Bitcoin Key
volatile uint64_t newKey = 0; //check initialise value? ****
 
Mutex newKey_mutex;//for mutex locking 

//Queue class
Queue<void, 8> inCharQ;
//serial port ISR to take individual chars
void serialISR(){
 uint8_t newChar = pc.getc();
 inCharQ.put((void*)newChar);
}
 
 //decode commands
Thread decodeT;

//set the global NewKey
void setNewCmd(char newCmd[MAX_ARRAY_SIZE]){
     
    //K 
    if (newCmd[0] == 'K'){
        newKey_mutex.lock();
        sscanf(newCmd, "K%x", &newKey); //Decode the command
        newKey_mutex.unlock();
    }
}


//decode char's function
void decodeFn(){
    pc.attach(&serialISR);
    char newCmd[MAX_ARRAY_SIZE] = "";
    uint32_t bufferPosition = 0;                //change this variable type if the max buffer/fifio size is found to be different
    bool exit = false;
    while(!exit) { 
    
        //get new char
        osEvent newEvent = inCharQ.get();
        uint8_t newChar = (uint8_t)newEvent.value.p;
        
        //check for carriage return "\r"
        if(newChar == 'r'){
            if(bufferPosition != 0){
                if(newCmd[bufferPosition - 1] == '\\'){
                    //carriage found 
                    newChar = '0';  //replace character
                    
                    //add to array
                    newCmd[bufferPosition] = newChar; 
                    
                    //reset buffer
                    bufferPosition = 0;
                    //send char array to decoder ***
                    setNewCmd(newCmd);
                }
            }
        }
        //Add new char to array
        else{
            //add character at current position 
            newCmd[bufferPosition] = newChar; 
            bufferPosition ++;
        }
      
        //------error for overflow-------------------------// 
        if(bufferPosition >= MAX_ARRAY_SIZE ){ 
            exit = true; 
            putMessage(MSG_OVERFLOW, bufferPosition); //
        } 
        //-------------------------------------------------//
      
     }//end of : while(!exit){} 
        
        //iii. Test the first character to determine which command was sent.
        //iv. Decode the rest of the command
} 

//Set a given drive state
void motorOut(int8_t driveState){
    
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
    }
    
volatile uint16_t hashcount = 0;
 
void do_hashcount() 
{
    putMessage(MSG_HASHCOUNT, hashcount);
    hashcount = 0;
}
    
    //Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];
    }

//Basic synchronisation routine    
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);
    
    //Get the rotor state
    return readRotorState();
}

int8_t orState;
int8_t intState;

void spinmotorISR() {
    intState = readRotorState();
    //Run the motor synchronisation
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
}


//Main
int main() {
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    
    //orState is subtracted from future rotor state inputs to align rotor and motor states
    I1.rise(&spinmotorISR);
    I1.fall(&spinmotorISR);
    I2.rise(&spinmotorISR);
    I2.fall(&spinmotorISR);
    I3.rise(&spinmotorISR);
    I3.fall(&spinmotorISR);
    spinmotorISR();
    
    Ticker hashcounter;
    hashcounter.attach(&do_hashcount, 1.0);
    
    commOutT.start(&commOutFn);
    decodeT.start(&decodeFn);
    
    uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,
                            0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,
                            0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,
                            0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,
                            0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,
                            0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,
                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    
    uint64_t* key = (uint64_t*)((int)sequence + 48);
    uint64_t* nonce = (uint64_t*)((int)sequence + 56);
    uint8_t hash[32];
    
    while (1) {
        SHA256::computeHash(hash, sequence, 64);
        
        if (hash[0] == 0 && hash[1] == 0) {
            putMessage(MSG_NONCE_OK, *nonce);
        }
        (*nonce)++;
        hashcount++;
    }
}
