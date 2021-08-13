#include <Arduino.h>
#include <Wire.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <SPI.h>
#include <SD.h>

/*Constants*/
#define SERVICE_UUID   "ab0828b1-198e-4351-b779-901fa0e0371e"
#define CHARACTERISTIC_UUID_RX  "4ac8a682-9736-4e5d-932b-e9b31405049c"
#define CHARACTERISTIC_UUID_TX  "0972EF8C-7613-4075-AD52-756F33D4DA91"

/*Pins*/
//Sensors
byte S_ZERO = 18;   //Initial sensor
byte S_30 = 23;     //30m sensor
byte S_100 = 22;    //100m sensor
byte S_C1 = 21;     //1st sensor on the corner
byte S_C2 = 19;     //2nd sensor on the corner
//SD CS Pin
byte SD_CS = 5;
//Debug Pins
byte LED = 2;       //LED for debugging
byte extLED = 4;    //External LED for debugging

/*Data Structures*/
typedef struct
{
    volatile unsigned long time_in_30;
    volatile unsigned long time_c_start;
    volatile unsigned long time_c_end;
    volatile unsigned long time_in_100;
    float tempCVT;
    float tempMTR;
    
} packet_ble;       //Package type to send by bluetooth

/*Variables*/
BLECharacteristic *characteristicTX; //Object to send data to client
bool deviceConnected = false; //Autoral support boolean to check if device is connected 
typedef enum {IDLE,RUN, SAVE}  state;     //Main states
typedef enum {START_, WAIT_30, WAIT_C1, WAIT_C2, WAIT_100, END_RUN} run_state;  //Secondary/run states

state ss = IDLE;       //Initial main state setting
run_state ss_r = START_;    //Initial secondary/run state setting

packet_ble data;            //Package to send by bluetooth
bool interrupt = false;     //Autoral support boolean to interrupt functions
unsigned long t_curr = 0;   //Current initial time, updated every run
volatile unsigned long t_30 = 0, t_c1 = 0, t_c2 = 0, t_100 = 0;     //Time variables
bool saveFlag = false;
bool saveComm = false;
bool saveNotify = false;
char run_fileName[64];      //Filename string

/*Function Prototypes*/
void isr_30m();     //30m interrupt
void isr_c1();      //Start of corner interrupt
void isr_c2();      //End of corner interrupt
void isr_100m();    //100m interrupt
void setup_ble();   //Configures the BLE
void ble_Send();  //Send a packet via bluetooth


/*Classes*/
class CharacteristicCallbacks: public BLECharacteristicCallbacks 
{
    void onWrite(BLECharacteristic *characteristic) 
    {
        std::string rxValue = characteristic->getValue(); //Return pointer to the register that contains actual value of characteristics
        if (rxValue.length() > 0) //Verify if data exists
        {
            for (int i = 0; i < rxValue.length(); i++) 
            {
                Serial.print(rxValue[i]);
            }
            Serial.println();
              
            if (rxValue.find("STR") != -1) 
            { 
                ss = RUN;
                ss_r = START_;
            }
            else if (rxValue.find("RST") != -1) 
          {
                ss_r = START_;
          }

            else if (rxValue.find("SVE") != -1) 
            { 
                saveFlag = true;
                saveComm = true;
            }
            else if (rxValue.find("NSV") != -1) 
            {
                saveFlag = false;
                saveComm = true;    
            }
            else if (rxValue.find("TCV") != -1) 
            {
              String tempCVT;
              for (int i = 0; i < rxValue.length(); i++)
                {
                    tempCVT[i] = rxValue[i];
                }
                data.tempCVT = tempCVT.toFloat();
            }
            else if (rxValue.find("TMT") != -1) 
            {
              String tempMTR;
              for (int i = 0; i < rxValue.length(); i++)
                {
                  tempMTR[i] = rxValue[i];
                }
                data.tempMTR = tempMTR.toFloat();
            }
            else if (rxValue.find("NOF") != -1) 
            {
                for (int i = 3; i < rxValue.length(); i++)
                {
                    run_fileName[i-3] = rxValue[i];
                } 
            }
        }
    }//onWrite
};

class ServerCallbacks: public BLEServerCallbacks 
{
    void onConnect(BLEServer* pServer) 
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) 
    {
        deviceConnected = false;
    };
};


void setup()
{
    Serial.begin(9600);     //Serial with 9600 baud rate
    //Pin setups
    pinMode(LED,OUTPUT);
    pinMode(extLED,OUTPUT);
    pinMode(S_ZERO, INPUT_PULLDOWN);
    pinMode(S_30, INPUT_PULLDOWN);
    pinMode(S_100, INPUT_PULLDOWN);
    pinMode(S_C1, INPUT_PULLDOWN);
    pinMode(S_C2, INPUT_PULLDOWN);
    digitalWrite(LED, HIGH);    //LED is at high state

    //SD initialization
    SD.begin(SD_CS);
    setup_ble();
}

void loop()     //Main loop
{   
    switch (ss)     //Switch for main states
    {
        case IDLE:
            Serial.println("idle\n");
            if(deviceConnected)
            {
                Serial.println("\n\n\nconnected\n\n\n");
                char txString[50];
                sprintf(txString,"IDLE State, waiting for START command...");
                characteristicTX->setValue(txString); //Set the value for the characteristics will notify(send) 
                characteristicTX->notify(); // Send the value to the smartphone
                delay(1000);
            }
        break;

        case RUN:
            switch (ss_r)   //Switch for secondary/run states
            {               
                case START_:    //Start of RUN
                    Serial.println("START\n");
                    saveNotify = false; //Reset save notifier
                    //Reset all variables
                    t_30 = 0;
                    t_c1 = 0;
                    t_c2 = 0;
                    t_100 = 0;
                    strcpy(run_fileName, "");
                    if(digitalRead(S_ZERO))     //If the car isn't in front of sensor anymore
                    {
                        ss_r = WAIT_30;         //Trigger the secondary state for waiting 30m sensor   
                        t_curr = millis();      //Initial time
                    }
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ble_Send();
                    break;
                case WAIT_30:   //Waiting for the car to get trough 30m sensor
                    Serial.println("WAIT_30\n");
                    if (!interrupt && (ss_r == WAIT_30))    //If interrupt isn't active and this is the current state
                    {
                        Serial.println("wait30if\n");
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_30), isr_30m, RISING); //Attach interrupt
                    }
                    else
                    {
                        //While in this state these variables will be updated
                        t_30 = millis() - t_curr;
                        t_c1 = millis() - t_curr;
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ble_Send();
                    break;
                case WAIT_C1:   //Waiting for the car to get trough 1st sensor of the corner
                    Serial.println("WAIT_C1\n");
                    if (((millis()- t_curr) - t_30 >  1000) && !interrupt && (ss_r == WAIT_C1)) //If at least one second has passed and interrupt isn't active and this is the current state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_C1), isr_c1, RISING);   //Attach interrupt
                    }
                    else
                    {
                        //While in this state these variables will be updated
                        t_c1 = millis() - t_curr;
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ble_Send();
                    break;
                case WAIT_C2:   //Waiting for the car to get trough 2nd sensor of the corner
                    Serial.println("WAIT_C2\n");
                    if (((millis()- t_curr) - t_c1 >  1000) && !interrupt && (ss_r == WAIT_C2)) //If at least one second has passed and interrupt isn't active and this is the current state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_C2), isr_c2, RISING);   //Attach interrupt
                    }
                    else
                    {   
                        //While in this state these variables will be updated
                        t_c2 = millis() - t_curr;
                        t_100 = millis() - t_curr;
                    }
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ble_Send();
                    break;
                case WAIT_100:  //Waiting for the car to get trough 100m sensor
                    Serial.println("WAIT_100\n");
                    if (((millis()- t_curr) - t_c2 >  1000) && !interrupt && (ss_r == WAIT_100))    //If at least one secont has passed and interrupt isn't active and this is the current state
                    {
                        interrupt = true;   //Set the support boolean
                        attachInterrupt(digitalPinToInterrupt(S_100), isr_100m, RISING);    //Attach interrupt
                    }
                    else
                    {
                        //While in this state this variables will be updated
                        t_100 = millis() - t_curr;
                    }
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ble_Send();
                    break;
                case END_RUN:   //The car has passed by 100m sensor, run is ended
                    //Update all variables in the data package
                    data.time_in_30 = t_30;
                    data.time_c_start = t_c1;
                    data.time_c_end = t_c2;
                    data.time_in_100 = t_100;
                    ble_Send();
                    ss = SAVE;     //Trigger the Bluetooth main state
                    break;
            }
            break;
        case SAVE:      //This main state is to save a file in SD that contains some data packages
            if(saveFlag == false && saveComm == false && saveNotify == false && deviceConnected == true)
            {
                char txString[60];
                sprintf(txString,"Entering SAVE State, did you want to save this run?");
                characteristicTX->setValue(txString); //Set the value for the characteristics will notify(send) 
                characteristicTX->notify(); // Send the value to the smartphone
                saveNotify = true;
            }
            if (saveFlag == true && saveComm == true) 
            {
                if (!SD.exists("/AR_runs/")) 
                {
                    SD.mkdir("/AR_runs/"); //Create directory to save the runs
                }
                File f = SD.open(run_fileName);        //Open file to save data
                f.print(data.time_in_30); f.print(", "); f.print(data.time_c_start); f.print(", "); f.print(data.time_c_end); f.print(", "); f.print(data.time_in_100); f.print(", "); f.print(data.tempCVT); f.print(", "); f.print(data.tempMTR);//Print data to file
                f.close();
                saveComm = false;
                ss = IDLE;
            }
            if (saveFlag == false && saveComm == true) 
            {
                saveComm = false;
                ss = IDLE;
            }
            break;
    }
}

void isr_30m()  //30m interrupt function
{
    ss_r = WAIT_C1;     //Trigger the secondary state for waiting 1st sensor of the corner

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_30));   //Detach Interrupt
}

void isr_c1()   //Start of corner interrupt function
{
    ss_r = WAIT_C2;     //Trigger the secondary state for waiting 2nd sensor of the corner

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_C1));   //Detach Interrupt
}

void isr_c2()   //End of corner interrupt function
{
    ss_r = WAIT_100;    //Trigger the secondary state for waiting 100m sensor

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_C2));   //Detach Interrupt
}

void isr_100m() //100m interrupt function
{
    ss_r = END_RUN;     //Trigger the secondary state for end the RUN

    interrupt = false;  //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_100));  //Detach Interrupt
}

void setup_ble()
{
    // Create the BLE Device
    BLEDevice::init("ACELERACAO_RETOMADA"); //Bluetooth device name

    // Create the BLE Server
    BLEServer *server = BLEDevice::createServer(); //Create a BLE server 

    server->setCallbacks(new ServerCallbacks()); //Sets the server callback

    // Create the BLE Service
    BLEService *service = server->createService(SERVICE_UUID);

    // Create a BLE Characteristic for data send
    characteristicTX = service->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

    characteristicTX->addDescriptor(new BLE2902());

    // Create a BLE Characteristic for data receive
    BLECharacteristic *characteristic = service->createCharacteristic(
                                                      CHARACTERISTIC_UUID_RX,
                                                      BLECharacteristic::PROPERTY_WRITE
                                                    );

    characteristic->setCallbacks(new CharacteristicCallbacks());

    // Start the service
    service->start();

    // Start advertising (ESP32 is visible to other devices)
    server->getAdvertising()->start();
    Serial.println("Waiting for some device to connect...");
}

void ble_Send()
{
    if (deviceConnected) 
    {
        //Serial.printf("erro aqui");
        char txString[sizeof(packet_ble)];
        snprintf(txString, sizeof(packet_ble), "%lu,%lu,%lu,%lu", data.time_in_30, data.time_c_start, data.time_c_end, data.time_in_100);
        characteristicTX->setValue(txString); //Set the value for the characteristics will notify(send) 
        characteristicTX->notify(); // Send the value to the smartphone
    }
}