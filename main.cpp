#include "mbed.h"
#include "SHA256.h"
#include "rtos.h"

#define char_len_max 18

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
int8_t lead = 2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
InterruptIn I1(I1pin);
InterruptIn I2(I2pin);
InterruptIn I3(I3pin);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Initialise the serial port
RawSerial pc(SERIAL_TX, SERIAL_RX);

//***********Initialisation Our Variables************//

//Message IDs
enum message_code {
    ERROR_C = 0, //Error message ID
    HASH = 1, //Hash frequency ID
    NONCE = 2, //correct nonce ID
    POSITION = 3, //Starting Rotor ID
    NEWTORQUE = 4, //Torque Value ID
    KEY = 5,        //Key ID
    ROTATION = 6,   //Rotation ID
    MAXVELOCITY = 7, //Max Velocity ID
    VELREP = 8,
    POSREP = 9
};

//message structure
typedef struct{
    uint8_t code; //ID
    uint32_t data; //Data
    } message_t;

Mail<message_t,16> outMessages; //Output message queue
Queue<void, 8> inCharQ; //character inputs

int8_t orState; //starting state of the rotor

uint8_t sequence[] = {0x45,0x6D,0x62,0x65,0x64,0x64,0x65,0x64,0x20,0x53,0x79,0x73,0x74,0x65,0x6D,0x73,0x20,0x61,0x72,0x65,0x20,0x66,0x75,0x6E,0x20,0x61,0x6E,0x64,0x20,0x64,0x6F,0x20,0x61,0x77,0x65,0x73,0x6F,0x6D,0x65,0x20,0x74,0x68,0x69,0x6E,0x67,0x73,0x21,0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint64_t* key = (uint64_t*)((int)sequence + 48); //Key generation
uint64_t* nonce = (uint64_t*)((int)sequence + 56); //Nonce
uint8_t hash[32]; //Hash output

char commInChar[char_len_max]; //array 32 characters length
uint8_t ptr; //char array pointer

volatile uint64_t newKey; //means value can change between thread calls
Mutex newKey_mutex; //Stops the value from being changed during use

Thread commOutT(osPriorityNormal, 1024);; //Output Thread
Thread commInT (osPriorityNormal, 1024);; //Input Thread

int pwm_period = 2000;  //PWM_initialisation
int32_t m_torque = 1000; // m_torque initialistion, protected as it acts as an atomi variable
uint32_t old_time = 0;  //old_time initialisation

volatile int32_t targetVelocity = 0x100; //256
volatile float newRotation = 0;   //Writes newRotation to a memory address immediately, and reads from the memory location dircectly every time it is called


//Coefficient setup for PD parameters
int8_t kp1 = 10;  
int8_t kp2 = 20;
int8_t kd = 10;

void putMessage(uint8_t code, uint32_t data){
    message_t *pMessage = outMessages.alloc(); //allocated the recieved message to  outmessages
    pMessage->code = code;
    pMessage->data = data;
    outMessages.put(pMessage);
    }


void commOutFn(){ //Function for outputting messages to the access terminal 
    while(1){ 
        osEvent newEvent = outMessages.get(); //pulls the message
        message_t *pMessage = (message_t*)newEvent.value.p; //assigns the values to pmessage

        switch(pMessage->code){ //finds correct ID for message
            case ERROR_C:
                if(pMessage->data == 0){ //Input message was too large
                    pc.printf("Input command too large\n\r");
                }
                break;
            case HASH:
                pc.printf("Hash frequency %d Hz \n\r",pMessage->data); //outputs the hash frequency
                break;
            case NONCE:
                pc.printf("Found a nonce 0x%016x\n\r", pMessage->data); //outputs correct nonce
                break;
            case POSITION:
                pc.printf("Rotor Starting Position: %d\n\r", pMessage->data); //outputs starting position
                break;
            case NEWTORQUE:
                pc.printf("Decoded as T %d\n\r", pMessage->data);  //outputs torque
                break;
            case ROTATION:
                pc.printf("Decoded as R %d\n\r",  pMessage->data); //outputs rotation
                break;
            case MAXVELOCITY:
                pc.printf("Decoded max_velocity %d\n\r", pMessage->data); //outputs maxVelocity
                break;
            case KEY:
                newKey_mutex.lock(); //locks the newKey value while the message is being outputted so it can't be changed
                pc.printf("Decoded New Key 0x%x\n\r", pMessage->data);  //outputs the key input 
                newKey_mutex.unlock(); //unlocks, allowing it to be updated again
                break;
            case VELREP:
                pc.printf("Velocity:\t%.1f\n\r",(float)((int32_t)pMessage->data / 6));  //outputs velocity rep
                break;
            case POSREP:
                pc.printf("Motor Position %d\n\r", pMessage->data);  //outputs position rep
                break;
        }
        outMessages.free(pMessage); //removes the message
    }
}

void serialISR(){
    uint8_t newChar = pc.getc(); //gets value from serial port
    pc.putc(newChar);   
    inCharQ.put((void*)newChar); //places into newChar
    }

void decode_char(char* buffer, uint8_t index){

    if(buffer[index] == 'R'){ //if first value is R rotate cretain number of times
        sscanf(buffer, "R%f", &newRotation);  
        putMessage(ROTATION, newRotation);

    }
    else if(buffer[index] == 'r'){ //if first value is R rotate cretain number of times
         sscanf(buffer, "r%f", &newRotation);
        putMessage(ROTATION, newRotation);

    }
    else if(buffer[index] == 'V'){ //if first value is V set speed of rotation
        sscanf(buffer, "V%d", &targetVelocity);
        putMessage(MAXVELOCITY, targetVelocity);
    }
    else if(buffer[index] == 'v'){ //if first value is V set speed of rotation
        sscanf(buffer, "v%d", &targetVelocity);
        putMessage(MAXVELOCITY, targetVelocity);
    }
    else if (buffer[index] == 'K'){ //if char is K set key to value input
        newKey_mutex.lock();
        sscanf(buffer, "K%10llx", &newKey);
        newKey_mutex.unlock();
        putMessage(KEY, newKey);
    }
    else if (buffer[index] == 'k'){ //if char is K set key to value input
        newKey_mutex.lock();
        sscanf(buffer, "k%10llx", &newKey);
        newKey_mutex.unlock();
        putMessage(KEY, newKey);
    }
    else if (buffer[index] == 'T') { //torque test if T is char
        newKey_mutex.lock();
        sscanf(buffer, "T%d", &m_torque);
        newKey_mutex.unlock();
        putMessage(NEWTORQUE, m_torque);                
        
    }
    else if (buffer[index] == 't') { //torque test if t is char
        newKey_mutex.lock();
        sscanf(buffer, "t%d", &m_torque);
        newKey_mutex.unlock();
        putMessage(NEWTORQUE, m_torque);   
    }

}

void commInFn(){  //function for parsing input functions read from the access terminal
    pc.printf("Enter your command:\n\r"); //Tells the person to input their message
    pc.attach(&serialISR); //looks for the serialISR to get message
    while(1){
        if(ptr >= char_len_max){   
            putMessage(ERROR_C,0); //if gone over the buffer length, cancel and restart for next input
            ptr = 0; //reset pointer
            break;
        }
        osEvent newEvent = inCharQ.get(); //get next character
        uint8_t newChar = (uint8_t)newEvent.value.p;  //processes a char of the input stream
        if(newChar != '\r' && newChar != '\n'){
            commInChar[ptr] = newChar; //place values into buffer
            ptr++; //increment pointer
        }
        else{
            commInChar[ptr] = '\0'; //defines the end of the command
            ptr = 0; //resets the pointer
            decode_char(commInChar,ptr); //sends array to decoding function
        }
    }
}

//Set a given drive state
void motorOut(int8_t driveState, uint32_t t){
    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];
      
    //Turn off first
    if (~driveOut & 0x01) L1L.pulsewidth_us(0);
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L.pulsewidth_us(0);
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L.pulsewidth_us(0);
    if (~driveOut & 0x20) L3H = 1;
    
    //Then turn on
    if (driveOut & 0x01) L1L.pulsewidth_us(t);
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L.pulsewidth_us(t);
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L.pulsewidth_us(t);
    if (driveOut & 0x20) L3H = 0;
    }

//Convert photointerrupter inputs to a rotor state
inline int8_t readRotorState(){
    return stateMap[I1 + 2*I2 + 4*I3];  
}

//Basic synchronisation routine
int8_t motorHome() {
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0, pwm_period);  
    wait(2.0);
    
    //Get the rotor state
    return readRotorState();
}

int32_t motorPosition;  
void ISR(){
    static int8_t oldRotorState;  
    int8_t rotorState = readRotorState(); //reads motor position
    motorOut((rotorState-orState+lead+6)%6, m_torque); //+6 to make sure the remainder is positive (6 states in a revolution)
    if (rotorState - oldRotorState == 5) motorPosition--;  //moves the motor forwards towards the intended position
    else if (rotorState - oldRotorState == -5) motorPosition++; //moves the motor back if the intended position is overshot, or if the intended position is back
    else motorPosition += (rotorState - oldRotorState);  
    oldRotorState = rotorState;  //updates oldrotor state, to initialise it for the next loop
}

volatile uint16_t hashcount = 0;  //instantly assigns hashcount a memory location and always read directly from the memory address

void do_hashcount()  //outputs the hashcount then resets, to begin counting operations again
{
    putMessage(HASH, hashcount);
    hashcount = 0;
}

void init() {       //initialisation of timings 
    L1L.period_us(pwm_period);
    L2L.period_us(pwm_period);
    L3L.period_us(pwm_period);
}

Thread motorCtrlT (osPriorityNormal, 1536);  //allocation of thread to a memory space, at normal priority

void motorCtrlTick(){
    motorCtrlT.signal_set(0x1);    //tickcount value for motorcontrol 
}

void motorCtrlFn(){   //function for motorcontrol
    int32_t ys, yr;    
    int32_t  velocity = 0;   //initialising velocity
    
    //putMessage(err, 0x5);
    Ticker motorCtrlTicker;    //initialising the ticker
    motorCtrlTicker.attach_us(&motorCtrlTick, 100000);  
    
    static int32_t oldMotorPosition = 0;    //initialising oldMotorposition, updated in ISR after initialisation
    uint8_t iterations = 0;       //initialising iteration counts
    uint32_t ten_iter_time = 0;   
    float err_old = 0.0f;   //setting starting error, and t-1 error to 0
    float err;   
    float dedt;   
    int32_t torque;  
    
    //Timer timer;
    //timer.start();
    
    while(1){    //main motorcontrol loop
        motorCtrlT.signal_wait(0x1);    //waits one tick cycle
        
        //uint32_t current_time = timer.read();
       // ten_iter_time = current_time - old_time;
        //old_time = current_time;
        
        int32_t currPosition = motorPosition;    //stores motor position into currPosition
        velocity = (currPosition - oldMotorPosition)*10;    //calulates velocity by 'distance travelled' in ticks
        oldMotorPosition = currPosition;   //updates the t-1 value to the t value to initialise for next loop
       
        iterations = (iterations + 1)% 10;  //updates iterations
        if (!iterations) {
            putMessage(VELREP, velocity);
            putMessage(POSREP, currPosition);
            putMessage(NEWTORQUE, m_torque);   //pushes through current values for torque, velocity and position to the access terminal
            }
       
        // Proportional control with k_p = 25
        err = newRotation - currPosition/6.0f;   //calculates and stores error at t
        dedt = err - err_old;   //works out error change between error and t and t-1
        err_old = err;    //updates error at t-1 with t ready for next loop
       
        int32_t s = targetVelocity * 6;  
        
        if (s == 0) {                       // set to max if V0
            ys = pwm_period;    // theoretical max speed: 66.7
        } else {                            // calculate as normal
            ys = (int32_t)(kp1 * ( s - abs(velocity)));  //torque calculated from the motor velocity
        }
        if (err < 0) ys = -ys;            // multiply by sgn(err)
        
        
        yr = (int32_t) (kp2 * err) + (kd * dedt);    //torque calulated from motor position
       
         /* torque: choose y_r or y_s
           y = max(y_s, y_r), v <  0
               min(y_s, y_r), v >= 0 */
        if (((velocity < 0) && (ys > yr)) || ((velocity >= 0) && (ys < yr))) {
            torque = ys;   //chooses torque based on the minimum of the two torque values calculated
        } else {
            torque = yr;
        }
        
        // direction
        if (torque > 0) {
            lead = 2;
        } else {
            torque = -torque;  // torque should be positive
            lead = -2;         // reverse direction
        }
        if (torque > pwm_period)
            torque = pwm_period; // set max value
            
        // output
        m_torque = torque;   // access motorPower once
        
        if (velocity == 0)ISR(); // give jolt if velocity is 0
    }
}


//Main
int main(){
    pc.printf("Hello\n\r"); //outputs hello when turned on
    init();
    commOutT.start(commOutFn); //starts the output and input threads
    commInT.start(commInFn);
    //Run the motor synchronisation
    orState = motorHome(); //finds staring position
    putMessage(POSITION,orState);   //pushs out the state 

    //orState is subtracted from future rotor state inputs to align rotor and motor states
    //Poll the rotor state and set the motor outputs accordingly to spin the motor
    I1.rise(&ISR); //looks for rising edge to trigger the motor change
    I2.rise(&ISR);
    I3.rise(&ISR);
    I1.fall(&ISR); //looks for rising edge to trigger the motor change
    I2.fall(&ISR);
    I3.fall(&ISR);
    ISR(); //runs the ISR to begin with to start the motor running
    
    Ticker hashcounter;  //initialises a ticker based on hashcounter
    hashcounter.attach(&do_hashcount, 1.0); //references back and push hashcount out to access terminal
    motorCtrlT.start(motorCtrlFn);  //initialises and starts the motorcontrol thread

    while (1) {
        newKey_mutex.lock(); //stops value from being changed
        *key = newKey; //changed key
        newKey_mutex.unlock(); //releases the value

        SHA256::computeHash(&hash[0],&sequence[0],sizeof(sequence)); //computes the hash
       
        if((hash[0] == 0) && (hash[1] == 0)){
            putMessage(NONCE,*nonce); //when hash is correct print the nonce
        }

        *nonce += 1; //increments nonce
        hashcount++;  //adds a calculation comleted to hash count
    }
}
