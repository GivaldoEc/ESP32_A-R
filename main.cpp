#include <Wire.h>
#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <string.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>


/*Pins*/
//Sensors
const byte S_ZERO = 27; //Initial sensor
const byte S_30 = 16;   //30m sensor
const byte S_100 = 17;  //100m sensor
const byte S_C1 = 32;   //1st sensor on the corner
const byte S_C2 = 33;   //2nd sensor on the corner
//SD CS Pin
const byte SD_CS = 5; //Pin used on SD module
//Debug Pins
const byte LED = 34;   //LED for debugging
const byte extLED = 4; //External LED for debugging
//LCD Pin
const byte POT = 13;    //Pot signal to LCD cursor
const byte B_SEL = 25;  //Button select to LCD
const byte B_CANC = 26; //Button cancel to LCD

/*Data Structures*/
typedef struct
{
    volatile unsigned long time_in_30;
    volatile unsigned long time_c_start;
    volatile unsigned long time_c_end;
    volatile unsigned long time_in_100;
    float tempCVT;
    float tempMTR;

} packet_ble; //Package type to send by bluetooth

/*Variáveis*/
//LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); //initial lcd
//Variables to be show at LCD
float vel = 0;
String str_30 = "00:000",
       str_100 = "00:000",
       str_101 = "00:000",
       str_vel = "00.00 km/h";
//Bluetooth
BLECharacteristic *characteristicTX; //Object to send data to client
bool deviceConnected = false;        //Autoral support boolean to check if device is connected

class ServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *server) { deviceConnected = true; }
    void onDisconnect(BLEServer *server) { deviceConnected = false; }
};

//states
typedef enum
{
    INIT,
    AV,
    AR,
    MENU,
    RUN,
    SAVE
} state; //main states
typedef enum
{
    START_,
    WAIT_30,
    WAIT_C1,
    WAIT_C2,
    WAIT_100,
    END_RUN
} run_state; //states from the run mode
typedef enum
{
    RESET,
    SIM,
    NAO
} sv_state; //states from the save mode

state ss = INIT;                  //Variable to be use at function GlobalMenu to choose between RUN or SAVE mode
volatile run_state ss_r = START_; //Initial secondary/run state setting
state controlmode = INIT;         //Control if it is at AV or AR mode, will change the behavior of RunMode
sv_state ss_sv;                   //Control the save machine state

packet_ble data;                                                //Package to send by bluetooth
bool interrupt = false;                                         //Autoral support boolean to interrupt functions
unsigned long t_curr = 0;                                       //Current initial time, updated every run
volatile unsigned long t_30 = 0, t_c1 = 0, t_c2 = 0, t_100 = 0; //Time variables
bool saveFlag = false;
bool saveComm = false;
bool saveNotify = false;
char run_fileName[64]; //Filename string

//Potentiometer
byte pot_sel = 0;
byte curr_pos = 0;
byte pos[4]; //Number of positions the potentiometer must go through

/*Funções*/
char potSelect(byte pin, byte num_options); //Show the LCD cursor position and give the control about which option it is
void RunMode(void);                         //AR and AV function
void isr_30m();                             //30m interrupt
void isr_c1();                              //Start of corner interrupt
void isr_c2();                              //End of corner interrupt
void isr_100m();                            //100m interrupt
String format_time(unsigned long int t1);   //Format the millis variable at seconds
void printRun(void);                        //Print the speed data at LCD in real time
void GlobalMenu(void);                      //The LCD menu to AR and AV modes
void SaveRun();                             //Function to save the data to SD card
void btSetup();
void ble_Send();

void setup()
{
    Serial.begin(9600);
    pinMode(B_SEL, INPUT_PULLUP);
    pinMode(B_CANC, INPUT_PULLUP);
    pinMode(POT, INPUT);
    pinMode(S_ZERO, INPUT_PULLDOWN);
    pinMode(S_30, INPUT_PULLUP);
    pinMode(S_100, INPUT_PULLUP);
    pinMode(S_C1, INPUT_PULLUP);
    pinMode(S_C2, INPUT_PULLUP);
    SD.begin(SD_CS); //SD initialize
    lcd.init(21, 22);     //initialize the lcd ------------- changed from init()
    lcd.backlight(); //open the backlight
}

void loop()
{

    switch (ss)
    {

        noInterrupts();
    case INIT: //Welcome menu, only is active one time
        noInterrupts();
        lcd.setCursor(4, 0);
        lcd.print("Bem Vindo");
        lcd.setCursor(2, 1);
        lcd.print("CAPIBAJA 1.0");
        delay(2000);
        ss = MENU;
        lcd.clear();
        interrupts();
        break;

    case MENU: //Global Menu, where the user chooses which module go through
        noInterrupts();
        pos[0] = 20;
        pos[1] = 24;
        lcd.setCursor(0, 0);
        lcd.print("Escolha o modulo:");
        lcd.setCursor(4, 1);
        lcd.print(" AV  AR");
        pot_sel = potSelect(POT, 2);
        lcd.setCursor(pos[pot_sel] % 16, (int)pos[pot_sel] / 16); //Show the LCD cursor
        lcd.write('>');
        delay(50);

        if (!digitalRead(B_SEL))
        { //Check if the button is pressed
            delay(10);
            while (!digitalRead(B_SEL))
            {
                ss = static_cast<state>(pot_sel + 1); //Change the state of "ss" based on the LCD cursor, the static_cast is to convert an INT type on an ENUM
                lcd.clear();
            }
        }
        interrupts();
        break;

    case AV: //AV module
        noInterrupts();
        controlmode = AV;
        GlobalMenu();
        interrupts();
        break;

    case AR: //AR module
        noInterrupts();
        controlmode = AR;
        GlobalMenu();
        interrupts();
        break;

        interrupts();

    case RUN: //Run mode
        while ((ss == RUN) && (controlmode == AR))
        {
            RunMode();
        }

        while ((ss == RUN) && (controlmode == AV))
        {
            RunMode();
        }

        break;

    case SAVE: // Save on SD
        pos[0] = 19;
        pos[1] = 24;
        lcd.setCursor(0, 0);
        lcd.print("Salvar dados?");
        lcd.setCursor(3, 1);
        lcd.print(" SIM  NAO");
        pot_sel = potSelect(POT, 2);
        lcd.setCursor(pos[pot_sel] % 16, (int)pos[pot_sel] / 16);
        Serial.println("Pot1:");
        Serial.println(pot_sel);
        lcd.write('>');
        delay(50);

        if (!digitalRead(B_SEL))
        {
            delay(10);
            while (!digitalRead(B_SEL))
            { //Check if the button is pressed
                ss_sv = static_cast<sv_state>(pot_sel + 1);
                Serial.println("potsave:");
                Serial.println(ss_sv);
                lcd.clear();
            }
        }

        switch (ss_sv)
        {

        case RESET:
            break;

        case SIM:
            SaveRun();
            ss_sv = RESET;
            ss = MENU;
            lcd.clear();
            break;

        case NAO:
            ss_sv = RESET;
            ss = MENU;
            Serial.println("SAVE STATE: NAO");
            Serial.println(ss);
            lcd.clear();
            break;
        }

        break;
    }
}

char potSelect(byte pin, byte num_options)
/*Mapeia o curso do potenciometro para percorrer 2 vezes as opções de seleção,
  retorna o valor da opção escolhida*/
{
    int read_val = analogRead(pin);
    byte option = map(read_val, 0, 4095, 0, 2 * num_options - 1); //ESP32 has ADC 12 bit resolution, so 0 to 4095
    return option % num_options;
}

void RunMode(void)
{
    if (!digitalRead(B_CANC))
    {
        delay(10);
        while (!digitalRead(B_CANC))
        {
            ss = MENU;
            ss_r = START_;
            lcd.clear();
            return;
        }
    }
    switch (ss_r)
    { //Switch for secondary/run states

    case START_: //Start of RUN
        Serial.println("START\n");
        //Start the variables to show on LCD and will be save at SD card
        vel = 0;
        str_vel = "00.00 km/h";
        saveNotify = false; //Reset save notifier
        //Reset all variables
        t_30 = 0;
        t_c1 = 0;
        t_c2 = 0;
        t_100 = 0;
        printRun();
        //strcpy(run_fileName, "");
        if (digitalRead(S_ZERO)) //If the car isn't in front of sensor anymore
        {
            ss_r = WAIT_30;    //Trigger the secondary state for waiting 30m sensor
            t_curr = millis(); //Initial time
        }
        //Update all variables in the data package
        data.time_in_30 = t_30;
        data.time_c_start = t_c1;
        data.time_c_end = t_c2;
        data.time_in_100 = t_100;
        ble_Send(t_30, t_c1, t_c2, t_100);
        break;

    case WAIT_30: //Waiting for the car to get trough 30m sensor
        Serial.println("WAIT_30\n");
        if (!interrupt && (ss_r == WAIT_30)) //If interrupt isn't active and this is the current state
        {
            Serial.println("wait30if\n");
            interrupt = true;                                              //Set the support boolean
            attachInterrupt(digitalPinToInterrupt(S_30), isr_30m, RISING); //Attach interrupt
        }
        else
        {
            //While in this state these variables will be updated
            t_30 = millis() - t_curr;
            t_c1 = millis() - t_curr;
            t_c2 = millis() - t_curr;
            t_100 = millis() - t_curr;
            printRun();
        }
        //Update all variables in the data package
        data.time_in_30 = t_30;
        data.time_c_start = t_c1;
        data.time_c_end = t_c2;
        data.time_in_100 = t_100;
        ble_Send(t_30, t_c1, t_c2, t_100);
        printRun();
        break;

    case WAIT_C1: //Waiting for the car to get trough 1st sensor of the corner
        Serial.println("WAIT_C1\n");
        if (((millis() - t_curr) - t_30 > 1000) && !interrupt && (ss_r == WAIT_C1)) //If at least one second has passed and interrupt isn't active and this is the current state
        {
            interrupt = true;                                             //Set the support boolean
            attachInterrupt(digitalPinToInterrupt(S_C1), isr_c1, RISING); //Attach interrupt
        }
        else
        {
            //While in this state these variables will be updated
            t_c1 = millis() - t_curr;
            t_c2 = millis() - t_curr;
            t_100 = millis() - t_curr;
            printRun();
        }
        //Update all variables in the data package
        data.time_in_30 = t_30;
        data.time_c_start = t_c1;
        data.time_c_end = t_c2;
        data.time_in_100 = t_100;
        ble_Send(t_30, t_c1, t_c2, t_100);
        printRun();
        break;

    case WAIT_C2: //Waiting for the car to get trough 2nd sensor of the corner
        Serial.println("WAIT_C2\n");
        if (((millis() - t_curr) - t_c1 > 1000) && !interrupt && (ss_r == WAIT_C2)) //If at least one second has passed and interrupt isn't active and this is the current state
        {
            interrupt = true;                                             //Set the support boolean
            attachInterrupt(digitalPinToInterrupt(S_C2), isr_c2, RISING); //Attach interrupt
        }
        else
        {
            //While in this state these variables will be updated
            t_c2 = millis() - t_curr;
            t_100 = millis() - t_curr;
            printRun();
        }
        //Update all variables in the data package
        data.time_in_30 = t_30;
        data.time_c_start = t_c1;
        data.time_c_end = t_c2;
        data.time_in_100 = t_100;
        ble_Send(t_30, t_c1, t_c2, t_100);
        printRun();
        break;

    case WAIT_100: //Waiting for the car to get trough 100m sensor
        Serial.println("WAIT_100\n");
        if (((millis() - t_curr) - t_c2 > 1000) && !interrupt && (ss_r == WAIT_100)) //If at least one secont has passed and interrupt isn't active and this is the current state
        {
            interrupt = true;                                                //Set the support boolean
            attachInterrupt(digitalPinToInterrupt(S_100), isr_100m, RISING); //Attach interrupt
        }
        else
        {
            //While in this state this variables will be updated
            t_100 = millis() - t_curr;
            printRun();
        }
        //Update all variables in the data package
        data.time_in_30 = t_30;
        data.time_c_start = t_c1;
        data.time_c_end = t_c2;
        data.time_in_100 = t_100;
        ble_Send(t_30, t_c1, t_c2, t_100);
        printRun();
        break;

    case END_RUN: //The car has passed by 100m sensor, run is ended
        //Update all variables in the data package
        Serial.println("Fim!");
        if (vel == 0)
        {
            //vel = (double) (t_100 - t_30);
            //vel = (double) (t_100)/1000.0;
            vel = (double)3.6 * (100000.0 / t_100);
            str_vel = String(vel, 2) + " km/h";
        }
        printRun();
        data.time_in_30 = t_30;
        data.time_c_start = t_c1;
        data.time_c_end = t_c2;
        data.time_in_100 = t_100;
        ble_Send(t_30, t_c1, t_c2, t_100); // ss = SAVE;     //Trigger the Bluetooth main state
        ss = SAVE;
        ss_r = START_;
        delay(10000);
        lcd.clear();
        break;
    }
}

void isr_30m() //30m interrupt function
{
    ss_r = WAIT_C1; //Trigger the secondary state for waiting 1st sensor of the corner

    if (controlmode == AV) //Check if it is on AV mode to ignore the corner sensors
    {
        ss_r = WAIT_100; //Trigger the secondary state for waiting 100m sensor
    }

    interrupt = false;                            //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_30)); //Detach Interrupt
}

void isr_c1() //Start of corner interrupt function
{
    ss_r = WAIT_C2; //Trigger the secondary state for waiting 2nd sensor of the corner

    interrupt = false;                            //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_C1)); //Detach Interrupt
}

void isr_c2() //End of corner interrupt function
{
    ss_r = WAIT_100; //Trigger the secondary state for waiting 100m sensor

    interrupt = false;                            //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_C2)); //Detach Interrupt
}

void isr_100m() //100m interrupt function
{
    ss_r = END_RUN; //Trigger the secondary state for end the RUN

    interrupt = false;                             //Reset support boolean
    detachInterrupt(digitalPinToInterrupt(S_100)); //Detach Interrupt
}

String format_time(unsigned long int t1)
{
    if (t1 < 10000)
        return '0' + String(t1 / 1000) + ':' + String(t1 % 1000);
    else
        return String(t1 / 1000) + ':' + String(t1 % 1000);
}

void printRun()
{
    // str_30 = format_time(t_30);
    str_100 = format_time(t_100);
    //str_101 = format_time(t_101);
    lcd.setCursor(0, 0);
    lcd.print("  " + str_100 + "    ");
    //lcd.print(' ' + str_30 + "  " + str_100 + "    ");
    lcd.setCursor(0, 1);
    lcd.print("   " + str_vel + "        ");
}

void GlobalMenu()
{
    pos[0] = 17;
    pos[1] = 23;
    lcd.setCursor(0, 0);
    if (controlmode == AV)
    { //Check if it is on AV mode
        lcd.print("   Modulo AV");
    }
    else
    {
        lcd.print("   Modulo AR");
    }
    lcd.setCursor(0, 1);
    lcd.print("  RUN   CONFIG ");
    pot_sel = potSelect(POT, 2);
    lcd.setCursor(pos[pot_sel] % 16, (int)pos[pot_sel] / 16);
    lcd.write('>');
    delay(50);

    if (!digitalRead(B_CANC))
    { //See if the cancel button is active
        delay(10);
        while (!digitalRead(B_CANC))
        {
            ss = MENU;
            lcd.clear();
        }
    }

    if (!digitalRead(B_SEL))
    { //See if the select button is active
        delay(10);
        while (!digitalRead(B_SEL))
        {
            ss = static_cast<state>(pot_sel + RUN); //Static_cast is a function to change a INT (byte in this case) variable on an ENUM
            lcd.clear();
        }
    }
    Serial.println(ss);
}

void SaveRun()
{

    if (!SD.begin(SD_CS))
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Nao ha SD");
        lcd.setCursor(0, 1);
        lcd.print("Conecte um SD");
        delay(3000);
        return;
    }
    else
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Salvando. . .");
        lcd.setCursor(0, 1);
        lcd.print("Um momento");
        delay(5000);
    }

    if (controlmode == AR)
    {
        if (!SD.exists("/AR_runs/"))
        {
            SD.mkdir("/AR_runs/"); //Create directory to save the runs
        }
        File f = SD.open("/RunAR.txt"); //Open file to save data
        f.print(data.time_in_30);
        f.print(", ");
        f.print(data.time_c_start);
        f.print(", ");
        f.print(data.time_c_end);
        f.print(", ");
        f.print(data.time_in_100);
        f.print(", ");
        f.print(data.tempCVT);
        f.print(", ");
        f.print(data.tempMTR); //Print data to file
        f.close();
        saveComm = false;
        return;
    }
    else
    {
        if (!SD.exists("/AV_runs/"))
        {
            SD.mkdir("/AV_runs/"); //Create directory to save the runs
        }
        File f = SD.open("/RunAV.txt", FILE_WRITE); //Open file to save data
        f.print(data.time_in_30);
        f.print(", "); /*f.print(data.time_c_start); f.print(", "); f.print(data.time_c_end); f.print(", "); */
        f.print(data.time_in_100);
        f.print(", ");
        f.print(data.tempCVT);
        f.print(", ");
        f.print(data.tempMTR); //Print data to file
        f.close();
        saveComm = false;
        return;
    }
}

// bluetooth - 0x1801

//characteristics
BLECharacteristic time_in_30(
    BLEUUID((uint16_t)0x1801),
    BLECharacteristic::PROPERTY_READ);
BLECharacteristic time_c_start(
    BLEUUID((uint16_t)0x1801),
    BLECharacteristic::PROPERTY_READ);
BLECharacteristic time_c_end(
    BLEUUID((uint16_t)0x1801),
    BLECharacteristic::PROPERTY_READ);
BLECharacteristic time_in_100(
    BLEUUID((uint16_t)0x1801),
    BLECharacteristic::PROPERTY_READ);

void btSetup()
{
    BLEDevice::init("CAPIBAJA"); //server name

    BLEServer *server = BLEDevice::createServer(); //BLE server creation
    server->setCallbacks(new ServerCallbacks());

    //services
    BLEService *newData = server->createService(BLEUUID((uint16_t)0x1700));

    //adding characteristics
    newData->addCharacteristic(&time_in_30);
    newData->addCharacteristic(&time_c_start);
    newData->addCharacteristic(&time_c_end);
    newData->addCharacteristic(&time_in_100);

    /*setting advertisement*/
    server->getAdvertising()->start();

    Serial.println("Set up! Waiting for the moment...");
}

void ble_Send(long t_30, long t_c1, long t_c2, long t_100)
{

    char t1[8];
    char t2[8];
    char t3[8];
    char t4[8];

    dtostrf(t_30,8,3,t1);

        if (deviceConnected)
    {
        time_in_30.setValue(t1);
        time_c_start.setValue(t2);
        time_c_end.setValue(t3);
        time_in_100.setValue(t4);
    }
    else
    {
        //TO DO: Something that notifies, visually, the disconnection.
    }
}