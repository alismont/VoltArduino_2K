 // DATE 18/06/16
// application mhp avec Processing



//#include "MegunoLink.h"
//#include "CommandHandler.h"


#include <Time.h>
#include <Timer.h>
#include <DS1302RTC.h>
#include <Wire.h>
//#include <LiquidCrystal_I2C.h>
#include "TimerOne.h"

// set time format 2015/10/02 10:02:00
// Set pins:  CE, IO,CLK
DS1302RTC RTC(36, 34, 32);
// Optional connection for RTC module
#define DS1302_GND_PIN 30
#define DS1302_VCC_PIN 28
//********************************************************************************gsm
#include <GSM.h>
#include <EEPROM.h>

// INIT VARIABLES BLUETOOTH
// PIN Number
#define PINNUMBER "5559"//"5559"//"1576"  //"2913"
// initialize the library instance
GSM gsmAccess = false; // include a 'true' parameter for debug enabled
GSM_SMS sms;
char remoteNumber[20];  // Holds the emitting number
String Message = "";
String In = "";
int MessSMS = 0;
//String *InStringVARProc;
int InR;
bool Processing = false;
bool AutorisationSMS = true;
int SMSNbrCycle = 10;
int StatusOndul = 0;
//***********************************************************************************
int SensorValueCpt = 0;
bool CheckBox1;
unsigned long T1SetUp = 0;
unsigned long cycleTime = 0;
unsigned long CyclePrescedent = 0;
unsigned long Cycle = 0;
int GSMPasAPas = 0;
int MemoGSMPas = 0;

//LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
String Pointeur;
int Heure;
int Minute;
int Seconde;
int Jour;
int Mois;
int Annee;
int DEF1_Heure;
int DEF1_Minute;
int DEF1_Seconde;
int DEF1_Jour;
int DEF1_Mois;
int DEF1_Annee;

int DEF2_Heure;
int DEF2_Minute;
int DEF2_Seconde;
int DEF2_Jour;
int DEF2_Mois;
int DEF2_Annee;

int DEF3_Heure;
int DEF3_Minute;
int DEF3_Seconde;
int DEF3_Jour;
int DEF3_Mois;
int DEF3_Annee;

float TestScam = 0;
char senderNumber[20];
//*************** PLC ******************************
int N7[200];
float F8[255];
float  AI7AUX1;
float  AI7AUX2;
float  AI7AUX3;
float  AI7Calculer = 800.0;

int PWM4 = 0;
int PWM5 = 0;
int PWM6 = 0;
int PWM7 = 0;


int PWM8 = 0;
int PWM9 = 0;
int PWM11 = 0;
int PWM12 = 0;

int PWM49 = 0;
int PWM51 = 0;

float N7M60 = 0;
float F8M105 = 0;
float F8M161 = 0;
float F8M125 = 0;
float F8M132 = 0;

float FN7M60 = 0;
float FF8M105 = 0;
float FF8M161 = 0;
float FF8M125 = 0;
float FF8M132 = 0;

float F8I = 10.0;
float N7I = 100.0;
bool Ons1;
bool Ons2;
bool B3[600];
int IndexDebug = 0; // POUR LE TEST
int IndexDebugN = 0; // POUR LE TEST
int IndexDebugF = 0; // POUR LE TEST
int IndexDebugB = 0; // POUR LE TEST
int SetHeure,SetMin,SetJour,SetMois,SetAnnee;
int IndexDebugProc = 0; // POUR LE TEST
int IndexDebugNProc = 0; // POUR LE TEST
int IndexDebugFProc = 0; // POUR LE TEST
int IndexDebugBProc = 0; // POUR LE TEST
int IndexDebugTProc = 0; // POUR LE TEST
int IndexDebugTDProc = 0; // POUR LE TEST
int IndexDebugTTProc = 0; // POUR LE TEST
//*********************************************
int SeqVersTBL = 1;

int analogPin0 = 0;
int analogPin1 = 1;
int analogPin2 = 2;
int analogPin3 = 3;
int analogPin4 = 4;
int analogPin5 = 5;
int analogPin6 = 6;
int analogPin7 = 7;
int analogPin8 = 8;
int analogPin9 = 9;
int analogPin10 = 10;
int analogPin11 = 11;
int analogPin12 = 12;
int analogPin13 = 13;
int analogPin14 = 14;
int analogPin15 = 15;

//int LED3 = 3;
int LED4 = 4;
int LED5 = 5;
int LED6 = 6;
int LED7 = 7;
int LED8 = 8;
int LED9 = 9;
int LED11 = 11;
int LED12 = 12;

//int LED41 = 41;
//int LED43 = 43;
//int LED45 = 45;
//int LED47 = 47;

int LED49 = 49;
int LED51 = 51;
int LED53 = 53;

int LED13 = 13;

char octetReception;
char caractereReception;
char octetReceptionProc;
char caractereReceptionProc;
String chaineReception, Tram;
String chaineReceptionProc, TramProc;

int val = 0;
float val1 = 0.00;

int buttonPin22 = 22;
int buttonPin22Num = 0;
int buttonPin23 = 23;
int buttonPin23Num = 0;
int buttonPin25 = 25;
int buttonPin25Num = 0;
int buttonPin27 = 27;
int buttonPin27Num = 0;
int buttonPin29 = 29;
int buttonPin29Num = 0;
int buttonPin31 = 31;
int buttonPin31Num = 0;
int buttonPin33 = 33;
int buttonPin33Num = 0;
int buttonPin35 = 35;
int buttonPin35Num = 0;
int buttonPin37 = 37;
int buttonPin37Num = 0;
int buttonPin39 = 39;
int buttonPin39Num = 0;

int buttonPin38 = 38;
int buttonPin38Num = 0;
int buttonPin40 = 40;
int buttonPin40Num = 0;
int buttonPin41 = 41;
int buttonPin41Num = 0;
int buttonPin42 = 42;
int buttonPin42Num = 0;
int buttonPin43 = 43;
int buttonPin43Num = 0;
int buttonPin44 = 44;
int buttonPin44Num = 0;
int buttonPin45 = 45;
int buttonPin45Num = 0;
int buttonPin46 = 46;
int buttonPin46Num = 0;
int buttonPin47 = 47;
int buttonPin47Num = 0;
int buttonPin48 = 48;
int buttonPin48Num = 0;
int buttonPin50 = 50;
int buttonPin50Num = 0;
int buttonPin52 = 52;
int buttonPin52Num = 0;

int buttonState = 0;

int SYNCHRO = 0;
int LU = 0;
int NUM = 0;
int FirstScan = 1;

//Affichage lcd
float floatVal = 123.234;
char charVal[10];               //temp||arily holds data from vals
String stringVal = "";     //data on buff is copied to this string
int CptOffset;
int MessInMemoCpt = 0;
String test;
int B3_8_MEMO = 0;
int B3_9_MEMO = 0;
int B3_2_MEMO = 0;
int F8_60_LIM = 0;

int BPOnOff = 0;
float CSG;
float Value;

bool notConnected = true;
int ValnotConnected = 1;
bool GSMChecked = true;

//Messages vers Tablette
int MessTbl[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int MessIn1 = 0;
int MessIn2 = 0;
int MessOut = 0;
int MessIn = 0;
int MessPoint = 1;
int MessInMemo1 = 0;
int MessInMemo2 = 0;

//eeprom
int eeAdress = 0;
float F10[250];
float data;

//Tempo
int T1 = 0;
int ST1 = 1;
int T2 = 0;
int ST2 = 1;
int T3 = 0;
int ST3 = 1;
int T4 = 0;
int ST4 = 1;
long T5 = 0;
int ST5 = 1;
int T6 = 0;
int ST6 = 1;
int T7 = 0;
int ST7 = 1;
int T8 = 0;
int ST8 = 1;
int T9 = 0;
int ST9 = 1;
int T10 = 0;
int ST10 = 1;
int T11 = 0;
int ST11 = 1;

int T1SecCpt = 0;
int T1DSecCpt = 0;
int T[100], CptT[100];
float T_35;
bool TDN[100];
bool T1Sec,  T1SecSet;
bool T1Cent, T1Cent1, T1CentSet;
bool T1DSec,  T1DSecSet;

bool I104;
bool I102;

int T58PRE = 100;
int T59PRE = 100;
int T60PRE = 100;
int T61PRE = 100;
int T27PRE = 100;
int T70PRE = 100;
int T71PRE = 100;
int T72PRE = 100;

bool T58TT;
bool T59TT;
bool T60TT;
bool T61TT;
bool T63TT;
bool T6TT;
bool T67TT;
bool T39TT;
bool T55TT;
bool T52TT;

//MegunoLink
//char FromTable[100];
//TimePlot MyPlot;
//InterfacePanel Panel("Test");
//CommandHandler<> SerialCommandHandler;
//Table MyTable;

//-----------------------------------------------------
//MegunoLink
//void Cmd_LED13(CommandParameter &Parameters)
//{
//  const char *State = Parameters.NextParameter();
//
//  if (strcmp(State, "on") == 0)
//  {
//    digitalWrite(LED13, HIGH);
//  }
//  else
//  {
//    digitalWrite(LED13, LOW);
//  }
//}
//
//void Cmd_Unknown()
//{
//  Serial.println(F("I don't know that command. Try another. "));
//}

//-----------------------------------------------------------
// Clock
void ClockGestion(){
// Activate RTC module


  setTime(SetHeure, SetMin, 0,  SetJour ,SetMois , SetAnnee); //set the system time to

  RTC.set(now());  
  
}

//-----------------------------------------------------------
// Start GSM shield
void GsmAutorisation() {


  switch (GSMPasAPas) {
    case 0:
      GSMPasAPas = 1;
      break;
    case 1:
      Serial.println("GSMAccess");
      if (gsmAccess.begin(PINNUMBER) == GSM_READY) {
        notConnected = false;
        ValnotConnected = 0;
        Serial.println("CONNECTER");
        //if (sms.available()) {
        //  sms.flush();
        Message = "";
        //}
        GSMPasAPas = 2;
      }
      else
      {
        Serial.println("SMS=Not connected");
        AutorisationSMS = false;
        notConnected = true;
        ValnotConnected = 1;
        delay(10);
        GSMPasAPas = 10;
      }
      break;
    case 2:
      if (!AutorisationSMS) {//-cond ons
        if (!Ons2) {  //---bit ons
          Ons2 = 1;
          //sms.flush();
        }
      }
      else {
        Ons2 = 0;//---bit ons
      }

      if (AutorisationSMS) {
        if (!B3[304] && SMSNbrCycle <= 0 && !B3[122]) {
          Message = SMS_Receive();
          SMSNbrCycle = 10;
          // Traitement SMS 1
          if (Message == "ANNULER ORDRE") {
            Message = "";
            MessSMS = 1;
            N7[80] = 0;
          }
          // Traitement SMS 2
          if (Message == "BOOST RESEAU") {
          //  Serial.println("BOOST RESEAU");
            Message = "";
            MessSMS = 2;
            N7[80] = 1;
          }

          // Traitement SMS 3
          if (Message == "DELESTAGE") {
          //  Serial.println("DELESTAGE");
            Message = "";
            MessSMS = 3;
            N7[80] = 2;
          }

          // Traitement SMS 4
          if (Message == "AUGMENTER CONSOMMATION") {
            Message = "";
            MessSMS = 4;
            N7[80] = 3;
          }

          // Traitement SMS 5
          if (Message == "CHARGER BATTERIE") {
            Message = "";
            MessSMS = 5;
            N7[80] = 4;
          }
        }
       // Serial.println(TDN[85]);
        if (!TDN[85]) {
          Message = "";
          MessSMS = 1;
        }
      }
      break;

    case 10:
      break;
  }
  //Fin du Pas a Pas
  if (GSMPasAPas != MemoGSMPas) {//-cond ons
    if (!Ons1) {  //---bit ons
      Ons1 = 1;
     // Serial.println(GSMPasAPas);
      MemoGSMPas = GSMPasAPas;
    }
  }
  else {
    Ons1 = 0;//---bit ons
  }
}

//----------------------------------------------------------------------------------SETUP
void setup() {

  Serial.begin(19200); // initialise la vitesse de la connexion série
  Serial3.begin(9600);

  //MegunoLink
  //  Serial.print(F("gemunolink"));
  //  SerialCommandHandler.AddCommand(F("LED13"), Cmd_LED13);
  //  SerialCommandHandler.SetDefaultHandler(Cmd_Unknown);
  //  MyTable.SetDescription("Cycle", "Temps de scanning");

  //-------------------------------------------------------------
  Timer1.initialize(100000);         // initialize timer1
  //Timer1.setPeriod(100);
  Timer1.attachInterrupt(callback);






  //------------------------------------------------------LCD
  //  lcd.begin(20, 4);
  //  lcd.backlight();
  //  lcd.setCurs||(0, 2);
  //  lcd.print("HELLO");

  

  //delay(5000);

  //  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(LED5, OUTPUT);
  pinMode(LED6, OUTPUT);
  pinMode(LED7, OUTPUT);
  pinMode(LED8, OUTPUT);
  pinMode(LED9, OUTPUT);
  pinMode(LED11, OUTPUT);
  pinMode(LED12, OUTPUT);
  pinMode(LED13, OUTPUT);
  //pinMode(LED41, OUTPUT);
  //pinMode(LED43, OUTPUT);
  // pinMode(LED45, OUTPUT);
  // pinMode(LED47, OUTPUT);
  pinMode(LED49, OUTPUT);
  pinMode(LED51, OUTPUT);
  pinMode(LED53, OUTPUT);

  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  digitalWrite(LED7, LOW);
  digitalWrite(LED8, LOW);
  digitalWrite(LED9, LOW);

  digitalWrite(LED11, LOW);
  digitalWrite(LED12, LOW);
  digitalWrite(LED13, LOW);
  digitalWrite(LED49, LOW);
  digitalWrite(LED51, LOW);
  digitalWrite(LED53, LOW);



  pinMode(buttonPin22, INPUT);
  pinMode(buttonPin23, INPUT);
  pinMode(buttonPin25, INPUT);
  pinMode(buttonPin27, INPUT);
  pinMode(buttonPin29, INPUT);
  pinMode(buttonPin31, INPUT);
  pinMode(buttonPin33, INPUT);
  pinMode(buttonPin35, INPUT);
  pinMode(buttonPin37, INPUT);
  pinMode(buttonPin38, INPUT);
  pinMode(buttonPin39, INPUT);
  pinMode(buttonPin40, INPUT);
  pinMode(buttonPin41, INPUT);
  pinMode(buttonPin42, INPUT);
  pinMode(buttonPin43, INPUT);
  pinMode(buttonPin44, INPUT);
  pinMode(buttonPin45, INPUT);
  pinMode(buttonPin46, INPUT);
  pinMode(buttonPin47, INPUT);
  pinMode(buttonPin48, INPUT);
  //pinMode(buttonPin49, INPUT);
  pinMode(buttonPin50, INPUT);

  // pinMode(buttonPin51, INPUT);
  pinMode(buttonPin52, INPUT);
  //pinMode(buttonPin53, INPUT);
  
  pinMode(DS1302_GND_PIN, OUTPUT);
  digitalWrite(DS1302_GND_PIN, LOW);


  pinMode(DS1302_VCC_PIN, OUTPUT);
  digitalWrite(DS1302_VCC_PIN, HIGH);
  F10[65] = -0.03;
  F10[66] = -0.03;
  F10[67] = -0.04;
  F10[68] = -0.04;
  F8[0] = 0.0;
  F8[1] = 10.0;
  F8[2] = 65.0;
  F8[3] = 99.0;
  F8[4] = 103.0;
  F8[5] = 46.0;
  F8[6] = 50.0;
  F8[7] = 52.6;
  F8[8] = 53.2;
  F8[9] = 56.0;

  F8[40] = 20.0;
  F8[41] = 20.0;
  F8[42] = 60.0;
  F8[43] = 60.0;

  F8[55] = 46.0;
  F8[56] = 48.0;
  F8[57] = 53.2;
  F8[58] = 59.0;
  F8[59] = 58.5;

  F8[67] = 0.02942;
  F8[70] = 0.039;
  F8[76] = 0.013;
  F8[85] = 0.03;
  F8[86] = 0.022;
  F8[87] = 0.020;
  F8[88] = 0.07;
  F8[90] = 0.03;
  F8[91] = 0.025;
  F8[92] = 0.018;
  F8[93] = 0.032;
  F8[95] = 55.6;
  F8[96] = 1.0;
  F8[99] = 55.6;
  F8[102] = -0.35;
  F8[103] = 0.01;
  F8[108] = -0.5;
  F8[109] = 0.5;
  F8[118] = 0.96;
  F8[119] = 0.97;
  F8[121] = 5.0;
  F8[123] = 1.98;
  F8[129] = 50.0;
  F8[144] = 0.0;
  F8[154] = 0.0;
  F8[164] = 0.0;
  F8[174] = 0.0;
  F8[184] = 0.0;
  F8[194] = 0.0;
  F8[204] = 0.0;
  F8[216] = 360.0;
  F8[244] = 0.0;
  F8[253] = 0.01;
  F8[254] = 0.5;
  F8[215] = 2.0;
  F8[213] = 100.0;


  F8[161] = 0.0;
  F8[132] = 0.0;
  F8[125] = 0.0;
  F8[105] = 0.0;



  N7[12] = 4;
  N7[13] = 2;
  N7[14] = 4;
  N7[16] = 3;
  N7[15] = 300;

  N7[17] = 15;
  N7[20] = 4;
  N7[21] = 8;
  N7[26] = -2;
  N7[23] = 10;
  N7[24] = -15;
  N7[25] = -100;
  N7[27] = 2;
  N7[28] = 6;
  N7[29] = 30;

  N7[37] = -100;
  N7[38] = -40;
  N7[39] = 40;
  N7[51] = 48;
  N7[52] = 100;

  N7[60] = 0;
  N7[63] = 20;
  N7[64] = 40;
  N7[65] = 70;
  N7[66] = 90;
  N7[67] = 98;
  N7[69] = 4;
  N7[89] = 95;
  N7[95] = 0;
  N7[100] = 0;
  N7[101] = 0;
  N7[102] = 0;
  N7[103] = 0;


  F10[11] = 0.02;
  F10[12] = 0.04;
  F10[14] = 0.03;
  F10[28] = 1.1;
  F10[54] = 57.4;

  F8[239] = 0.0;
  F8[236] = 0.0;
  F8[237] = 0.0;
  F8[238] = 0.0;
  F8[14] = 0.0;

  //  do
  //  {
  //    Ecriture ();
  //    delay(50);
  //  }
  //  while (SYNCHRO == 0);
  //********************************************************************************
  SetHeure=01;
  SetMin=00;
  SetJour=01;
  SetMois=01;
  SetAnnee=2016;
  
  ClockGestion();

  //Serial.println("fin setup");

  //EEPROM.put(0, 1.69);


 // GsmAutorisation();
  F8[44] = 60.0;
  T1SetUp = millis();
}

//----------------------------------------------------------------------------------LOOP
void loop() {
  //MegunoLink
  //SerialCommandHandler.Process();



  if (B3[31]) {
    T1Cent = 1;
  }
  /* if (T1CentSet) {
     //T1Cent = true;
     T1CentSet = false;
     //Serial.println("T1Cent");
    }
    if (T1SecSet) {
     T1Sec = true;
     T1SecSet = false;

    }
    if (T1DSecSet) {
     T1DSec = true;
     T1DSecSet = false;

    }*/
  //data=data+1;
  //EEPROM.put(0, data);
  //
  //EEPROM.get(0,data);
  //Serial.println(data);

  //-----------------------------------------------------------------------------------------------gsm
  //Ecriture ();// tablette vers arduino
  Scanin();
  Integration();

  if (N7[10] >= 370 && N7[10] <= 385) {
    F8[213] = 50.0;
  }
  else {
    F8[213] = 50.0;
  }
  //-LAD 6---ECHELLE----LAD 5--OFFSETS-----LAD 7--INTEGRATION---------------

  TonT1();

  if (buttonPin37Num < 4) {//-cond ons
    if (!B3[6]) {  //---bit ons
      B3[6] = 1;
      B3[32] = 0.0;
    }
  }
  else {
    B3[6] = 0;//---bit ons
  }
  TonT2();

  if (buttonPin45Num < 4) {
    F8[120] = 0.0;
  }
  F8[250] = F8[110] + F8[120];
  TonT3();

  if (N7[10] == 0 && !B3[214]) {
    F8[159] = 0;
  }
  if (N7[10] == 0 && B3[315] && (F8[159] > F10[66]) ) {
    F8[159] = F10[66];
  }
  if (N7[10] == 0 && !B3[213]) {
    F8[169] = 0;
  }
  if (N7[10] == 0 && B3[314] && (F8[169] > F10[65]) ) {
    F8[169] = F10[65];
  }
  if (N7[10] == 0 && !B3[216]) {
    F8[199] = 0;
  }
  if (N7[10] == 0 && !B3[216] && B3[312] && (F8[199] > F10[68]) ) {
    F8[199] = F10[68];
  }
  if (N7[10] == 0 && !B3[215]) {
    F8[209] = 0;
  }
  if (N7[10] == 0 && !B3[215] && B3[311] && (F8[209] > F10[67]) ) {
    F8[209] = F10[67];
  }

  TonT4();
  TonT5();
  TonT6();
  TonT7();
  TonT8();
  TonT9();

  TonT11();

  //-------------------------------------------------------------------------------------------------------------Lecture date et temps
  tmElements_t tm;
  if (! RTC.read(tm)) {
    //Serial.print("  Time = ");
    Heure = tm.Hour;
    //Serial.print(Heure);
    Minute = tm.Minute;
    //Serial.print(Minute);
    Seconde = tm.Second;
    //Serial.println(Seconde);
    //
    Jour = tm.Day;
    Mois = tm.Month;
    Annee = tmYearToCalendar(tm.Year);
    //Serial.print(Jour);
    //Serial.print(Mois);
    //Serial.println(Annee);

  } else {
    //Serial.println("DS1302 read err||!  Please check the circuitry.");
    //Serial.println(TempoT1);
    //delay(9000);
  }
  //-------------------------------------------------------------------------------------------------------------------------prg automate
  // PRG AUTOMATE

  //F8[100] = F8[110];
  F8[101] =  F8[159] + F8[169] + F8[179] + F8[189] + F8[199] + F8[209];

  //if (F8[101] > -0.1) {
  //  if (F8[101] < 0.1) {
  //    if (F8[100] > F8[102]) {
  //      F8[100] =  F8[102];
  //    }
  //  }
  //}
  
  F8[151] =  F8[159] * F8[45];
  F8[161] =  F8[169] * F8[45];
  F8[171] =  F8[179] * F8[45];
  F8[181] =  F8[189] * F8[45];
  F8[105] =  F8[171] + F8[181];
  F8[191] =  F8[199] * F8[45];
  F8[209] = F8[210] - F8[230];
  F8[201] =  F8[209] * F8[45];
  F8[211] =  F8[171] + F8[181] + F8[191] + F8[201];
  //------------------------------------------------------------------------------------------------------------ gestion des s||ties


  // ----------------------------U BATTERY-------------------------------------------------------- 27/4/2016

  F8[71] =  N7[31] * F8[70];// F8[69] ds void lecture  AI7

  F8[73] = F8[69] + F8[71];//AI7
  if ( B3[311] && !B3[312]) {  // b3:19/6 et /7   mw300316 deb
    F10[10] = F10[3];
  }

  if ( !B3[311] && B3[312]) {
    F10[10] = F10[8];
  }

  if ( B3[311] && B3[312]) {
    F10[10] = (F10[8] + F10[3]) / 2;
  }
  if ( !B3[311] && !B3[312]) {
    F10[10] = F10[14];
  }

  if (F10[10] > F10[12]) {
    B3[475] = 1;
  }

  if ( B3[475]) {
    F8[76] = F10[11];
  }

  if ( !B3[475]) {
    F8[76] = F10[10];
  }

  F8[77] =  F8[100] * F8[76];
  F8[78] =  F8[60] - F8[77];



  //-------------- -SENS COURANT BATTERIE  ----------------27/04/16
  if (F8[100] > 0) {
    B3[38] = 1;
  }
  else {
    B3[38] = 0;
  }

  if (F8[100] < 0) {
    B3[39] = 1;
  }
  else {
    B3[39] = 0;
  }
  //----CHARGEMENT DES PARAMETRES DE LA PARABOLE EN FONCTION DES ZONES DE TENSION DE LA BATTERIE
  //LE PARAMETRE ACTIF F8:39 DETERMINE LA VARIATION MAXIMALE DE TENSION INDUITE PAR L'EFFET MOUSSE EN VOLT PAR AMPERE CHARGE OU DECHARGE
  //LE PARAMETRE ACTIF F8:44 DETERMINE LA DUREE DE L'INFLUENCE DE LA MOUSSE
  //L'INFLUENCE DE LA COMPENSATION EST DIRECTEMENT PROP||TIONNELLE AU COURANT ET INVERSEMENT PROP||TIONNELLE AU TEMPS (PARABOLE)
  //L'EFFET DE MOUSSE DIFFERE SUIVANT LES ZONES DE TENSION DE LA BATTERIE (4 ZONES)
  //L'EFFET DE MOUSSE EST DIFFERENT EN CHARGE ET EN DECHARGE
  //LE SYSTEME PERMET DE "PREVOIR" A QUELLE TENSION LA BATTERIE SE STABILISERA A VIDE L||S DE LA DISPARITION DE LA CHARGE OU DE LA DECHARGE

  if (B3[33]) {
    F8[44] = F8[40];
    if (B3[38]) {
      F8[39] = F8[85];
    }

    if (B3[39]) {
      F8[39] = F8[90];
    }
  }

  if (B3[34]) {
    F8[44] = F8[41];
    if (B3[38]) {
      F8[39] = F8[86];
    }
    if (B3[39]) {
      F8[39] = F8[91];
    }
  }

  if (B3[35]) {
    F8[44] = F8[42];
    if (B3[38]) {
      F8[39] = F8[87];
    }
    if (B3[39]) {
      F8[39] = F8[92];
    }
  }

  if (B3[36]) {
    F8[44] = F8[43];
    if (B3[38]) {
      F8[39] = F8[88];
    }
    if (B3[39]) {
      F8[39] = F8[93];
    }
  }


  if (!TDN[7]) {
    CptT[7] = true;
  }
  if (TDN[7]) {
    T[7] =  0.0;
    TDN[7] =  0.0;
  }
  if (T[7] >= 10) {
    TDN[7] =  1.0;
    T[7] = 10;
    CptT[7] = false;
  }

  F8[49] =  F8[100] * F8[39];
  if (TDN[7]) {
    F8[82] = F8[49] - F8[81];
    F8[84] = F8[82] / F8[44];
    F8[81] = F8[84] + F8[81];
    F8[80] = F8[81] * (-1);
    F8[45] = F8[78] + F8[80];
  }
  if (!TDN[8]) {
    CptT[8] = true;
  }
  else {
    T[8] =  0.0;
    TDN[8] =  0.0;
    CptT[8] = false;
  }
  if (T[8] >= 600) {
    TDN[8] =  1.0;
    T[8] = 600;
  }

  if (TDN[8] && F8[34] > F8[53]) {
    F8[53] = F8[53] + 1;
  }
  if (TDN[8] && F8[34] < F8[53]) {
    F8[53] = F8[53] - 1;
  }
  // 27/04/16

  //---------------------------calcul charge batterie-zone batterie-----------------------------------27/4/2016
  F8[15] =  F8[6] - F8[5];
  F8[10] =  F8[1] - F8[0];
  F8[20] =  F8[10] / F8[15];
  if ((F8[45] > F8[5]) && (F8[45] < F8[6])) {
    B3[33] = 1;
    F8[25] =  F8[45] - F8[5];
    F8[30] =  F8[20] * F8[25];
    F8[34] =  F8[0] + F8[30];
  }
  else {
    B3[33] = 0;
  }


  F8[16] =  F8[7] - F8[6];
  F8[11] =  F8[2] - F8[1];
  F8[21] =  F8[11] / F8[16];
  if ((F8[45] > F8[6]) && (F8[45] < F8[7])) {
    B3[34] = 1;
    F8[26] =  F8[45] - F8[6];
    F8[31] =  F8[21] * F8[26];
    F8[34] =  F8[1] + F8[31];
  }

  else {
    B3[34] = 0;
  }



  F8[17] =  F8[8] - F8[7];
  F8[12] =  F8[3] - F8[2];
  F8[22] =  F8[12] / F8[17];
  if ((F8[45] >= F8[7]) && (F8[45] < F8[8])) {
    B3[35] = 1;
    F8[27] =  F8[45] - F8[7];
    F8[32] =  F8[22] * F8[27];
    F8[34] =  F8[2] + F8[32];
  }
  else {
    B3[35] = 0;
  }



  F8[18] =  F8[9] - F8[8];
  F8[13] =  F8[4] - F8[3];
  F8[23] =  F8[13] / F8[18];
  if (F8[45] >= F8[8]) {
    B3[36] = 1;
    F8[28] =  F8[45] - F8[8];
    F8[33] =  F8[23] * F8[28];
    F8[34] =  F8[3] + F8[33];
  }

  else {
    B3[36] = 0;
  }

  if (F8[34] > 101.0) {
    F8[34] = 101.0;
  }


  //10   -------------------------------calcul puissances-------------------------------------------MW

  F8[220] =  F8[100] * F8[60];
    
  TonT10();
  if (ST10 == 2) {
    F8[111] = F8[111] + F8[220];
    F8[112] = F8[112] + 1;
  }
  if (TDN[3]) {
    F8[113] = F8[111] / F8[112];  // moyenne puissance accumulée ds batterie pendant période échantillon
    F8[111] = 0.0;
    F8[112] = 0.0;
  }

  if (F8[113] >= 0) {
    F8[117] =  F8[113] * F8[118];
  }
  if (F8[113] < 0) {
    F8[117] =  F8[113] / F8[119];
  }

  if (TDN[3]) {
    F8[124] = N7[31] * F8[123];   //CONSOMMATION RELAIS
    F8[116] = F8[124] + F8[121];   //CONSOMMATION PARASITE TOTAL

  }
  F10[13] = F8[77] * F8[100];

  if (TDN[3]) {
    F8[125] = F8[117] - F8[116];   //PUISSANCE ACTUELLE CHARGE OU DECHARGE W/h
    F8[126] = F8[125] / 3600;  //CONSOMMATION PARASITE TOTAL   W/s
  }

  if (B3[305]) {
    N7[59] = N7[51] * N7[52];
  }

  if (B3[306]) {
    N7[59] = N7[51] * N7[52] * 2;
  }

  if (buttonPin37Num >= 4) {//-cond ons
    if (!B3[3]) {  //---bit ons
      B3[3] = 1;
      F8[114] = (F8[34] * N7[59]) / 100.0;
    }
  }
  else {
    B3[3] = 0;//---bit ons
  }

  if (TDN[3]) {
    F8[114] = F8[114] + F8[126];   //CAPACITE THE||IQUE COULOMB ACCUMULEE DS BATTERIE EN W/s
  }

  F8[128] =  (N7[59] * F8[53]) / 100;  // Capacite WH % actuelle METHODE TENSION
  F8[130] =   F8[128] - F8[114];
  F8[131] =   F8[130] / 3600.0;

  if (TDN[3]) {
    F8[114] = F8[114] + F8[131];
  }

  if ( F8[114] >   F8[128]) {
    F8[114] =   F8[128];
  }

  if ( F8[114] >   N7[59]) {
    F8[114] =   N7[59];
  }

  N7[60] =   (F8[114] * 100.0) / N7[59];
  F10[30] =   F8[52] + F8[247];

  F10[31] =   F10[30] * F8[60];


  B3[9] = B3[13];

  switch (B3[9]) {
    case 0:
      B3_8_MEMO = 0;
      break;
    case 1:
      if (B3_8_MEMO == 0) {
        F8[65] = F8[114];
      }
      B3_8_MEMO = 1;
      break;
  }

  if (TDN[0] && (B3[13] || F8[60] > F8[95])) { //-cond ons
    if (!B3[10]) {  //---bit ons
      B3[10] = 1;
      F8[114] = N7[59];
    }
  }
  else {
    B3[10] = 0;//---bit ons
  }

  if (F8[125] < 0) {
    F10[40] =  F8[114] / F8[125];
  }

  if (F8[125] >= 0) {
    F10[40] =  -9999.0;
  }

  if (F10[40] <= 0) {
    F10[43] =  F10[40] * -1;
  }

  if (F10[43] > 8) {
    B3[79] = 1;  // B3:4/14
    F10[44] = 8;
  }
  else {
    B3[79] = 0;  // B3:4/14
  }

  if (F10[43] <= 8) {
    F10[44] =  F10[43];
  }

  if (N7[60] < F8[129] && buttonPin31Num < 4 && !B3[304] && B3[1]) {
    B3[48] = 1;
  }
  else {
    B3[48] = 0;
  }


  if (B3[48]) {
    B3[245] = 1;
  }
  else {
    B3[245] = 0;
  }

  F10[41] = N7[59] - F8[114];

  if (F8[125] > 0) {
    F10[42] =  F10[41] / F8[125];
  }
  if (F8[125] <= 0) {
    F10[42] =  9999.0;
  }

  if (F10[42] > 8) {
    B3[80] = 1;  // B3:4/15
    F10[45] = 8;
  }
  else {
    B3[80] = 0;  // B3:4/15
  }

  if (F10[42] <= 8) {
    F10[45] =  F10[42];
  }


  //11------------------------------------------------ hysteresis--------------------27/4/16
  if ( B3[311] && !B3[312]) {
    F10[17] = F10[15];
  }

  if ( !B3[311] && B3[312]) {
    F10[17] = F10[16];
  }
  if ( B3[311] && B3[312]) {
    F10[17] = (F10[15] + F10[16]) / 2;
  }
  //------------------------------------------------------------hyst batt en decharge-----------
  if ( B3[314] && !B3[315]) {
    F10[22] = F10[20];
  }
  if ( !B3[314] && B3[315]) {
    F10[22] = F10[21];
  }
  if ( B3[315] && B3[314]) {
    F10[22] = (F10[20] + F10[21]) / 2;
  }
  //-------------------------------application hysteresis d'inversion sens du crt
  if (!TDN[34]) {                         //-----tempo increment inversion hyster
    CptT[34] = true;
  }
  if ( TDN[34]) {
    T[34] =  0.0;
    TDN[34] =  0.0;
    CptT[34] = false;
  }
  if (T[34] >= 10) {
    TDN[34] =  1.0;
    T[34] = 10;
  }

  if ( F8[100] > 0.0) {
    F8[102] = F10[17];
  }

  if ( F8[100] < 0.0) {
    F8[102] = F10[22];
  }

  if (TDN[34]) {
    if ( F8[255] < F8[102]) {
      F8[255] = F8[255] + F8[253];
    }
    if ( F8[255] > F8[102]) {
      F8[255] = F8[255] - F8[253];
    }
  }

  F8[104] = F8[250] - F8[255];  //---------------------crt batt avec hyst fct sens crt
  //--------------------------c||rec dynam crt batt fct U batt
  if (!TDN[33]) {                         //-----tempo compar  evolution U batt
    CptT[33] = true;
  }
  if (TDN[33]) {
    T[33] =  0.0;
    TDN[33] =  0.0;
    CptT[34] = false;
  }
  if (T[33] >= 300) {
    TDN[33] =  1.0;
    T[33] = 300;
  }

  if ( F8[69] >= F8[98]) {
    F8[74] = F8[74] + 1.0;
  }

  if ( F8[69] < F8[98]) {
    F8[74] = F8[74] - 1.0;
  }

  if ( F8[74] > 0.0) {
    B3[43] =  1.0;
  }
  else {
    B3[43] =  0.0;//------------------b3;2/10 U augmente
  }

  if ( F8[74] < 0.0) {
    B3[42] =  1.0;
  }
  else {
    B3[42] =  0.0;//------------------b3;2/9 U diminue
  }
  if ( F8[100] > 0.0) {
    B3[44] =  1.0;
  }
  else {
    B3[44] =  0.0;//------------------b3;2/11 crt sens charge
  }
  if ( F8[100] <= 0.0) {
    B3[45] =  1.0;
  }
  else {
    B3[45] =  0.0;//------------------b3;2/12 crt sens décharge
  }
  if ( F8[211] < 2.0) {
    B3[46] =  1.0;
  }
  else {
    B3[46] =  0.0;//------------------b3;2/13 pas de puissance de charge venant de device de charg
  }
  if ( !B3[215] && !B3[216] && !B3[224] && !B3[223]) {
    B3[47] =  1.0;
  }
  else {
    B3[47] =  0.0;//------------------b3;2/14 pas de device de charg en fonction
  }

  if (TDN[33]) {
    F8[98] =  F8[69];
    F8[74] = 0.0;
    if (B3[42] && B3[45]) {
      F8[99] =  F8[103] + F8[99];
    }
    if (B3[42] && B3[44]) {
      F8[99] = F8[99] - F8[103];            //      La U diminue et I positif donc c||rect de I
    }
    if (B3[44] && (B3[47]) || (B3[46] && !B3[47])) {
      F8[99] = F8[99] - F8[103];
    }
    if ((F8[137] > N7[24]) && B3[317]) {
      F8[99] = F8[99] - F8[103];
    }
  }

  if (F8[99] < F8[108]) {
    F8[99] = F8[108];
  }
  if (F8[99] > F8[109]) {
    F8[99] = F8[109];
  }
  F8[100] = F8[104] + F8[99];  //-----------------crt bat 1+2 mesures integre avec hyst

  //12-------------------------batterie-------------------------------------------------27/04/16
  if (N7[60] > N7[67]) {
    B3[81] =  1.0;
  }
  else {
    B3[81] =  0.0;//------------------b3;5/0 batterie aime pas charge intense
  }

  if (N7[60] >= N7[66] && N7[60] <= N7[67] ) {
    B3[82] =  1.0;
  }
  else {
    B3[82] =  0.0;//------------------b3;5/1 batterie préfere charge soft
  }

  if (N7[60] >= N7[65] && N7[60] <= N7[66] ) {
    B3[83] =  1.0;
  }
  else {
    B3[83] =  0.0;//------------------b3;5/2 batterie accepte tout
  }

  if (N7[60] >= N7[64] && N7[60] <= N7[65] ) {
    B3[84] =  1.0;
  }
  else {
    B3[84] =  0.0;//------------------b3;5/3 batterie dem&&e charge
  }

  if (N7[60] >= N7[63] && N7[60] <= N7[64] ) {
    B3[85] =  1.0;
  }
  else {
    B3[85] =  0.0;//------------------b3;5/4 batterie exige charge intense
  }

  if (N7[60] < N7[63] ) {
    B3[86] =  1.0;
  }
  else {
    B3[86] =  0.0;//------------------b3;5/5 batterie exige charge intense immediate
  }

  if (F8[60] < F8[57] ) {
    B3[95] =  1.0;
  }
  else {
    B3[95] =  0.0;//------------------b3;5/14 tension demarrage charge batterie par chargeur
  }

  F8[64] = F8[57] + 0.5;

  if (F8[60] < F8[64] ) {
    B3[318] =  0.0;;// MEMOIRE BALANCING EFFECTUE
  }
  F8[97] = F8[95] + F8[96];

  if ( !B3[87] && ( B3[122] || (F8[60] > F8[95])) ) {                         //-----tempo TENSION ATTEINTRE CHARGE BATTERIE
    CptT[9] = true;
  }
  else {
    T[9] =  0.0;
    TDN[9] =  0.0;
    CptT[9] = false;
  }
  if (T[9] >= 30) {
    TDN[9] =  1.0;
    T[9] = 30;
  }

  if (F8[60] > F8[59] ) {                         //-----tempo TENSION FIN DE CHARGE ATTEINTE TROP LONG
    CptT[25] = true;
  }
  else {
    T[25] =  0.0;
    TDN[25] =  0.0;
    CptT[25] = false;
  }
  if (T[25] >= 100) {
    TDN[25] =  1.0;
    T[25] = 100;
  }

  if (F8[60] > F8[58] ) {
    B3[89] =  1.0;
  }
  else {
    B3[89] =  0.0;//------------------ batterie TENSION SUPERIEURE ULTIME BATTERIE ATTEINTE
  }

  if (B3[89] || TDN[25] || TDN[9] ) {
    B3[90] =  1.0;
  }
  else {
    B3[90] =  0.0;//------------------ batterie STOP CHARGE BATTERIE U PREVUE ATTEINTE
  }

  if (F8[60] < F8[56] ) {
    B3[193] =  1.0;// ALARME TENSION BATTERIE TROP BASSE
  }
  if (F8[60] < F8[55] ) {
    B3[194] =  1.0;// DRAME TENSION BATTERIE M||TE
  }

  if (buttonPin37Num >= 4 && buttonPin35Num < 4 ) {
    B3[195] =  1.0;// COUPURE DE CHARGE PAR BMS
  }

  if (TDN[25] && !B3[122] ) {
    B3[196] =  1.0;// U FIN DE CHARGE DE CHARGE ATTEINTE SANS BCU ACTIF PROBLEME CIRCUIT BCU
  }
  if (B3[89] && B3[122] && !B3[123]) {
    B3[197] =  1.0;// U MAX ATTEINTE SANS TOUS LES BCU  PROBLEME BCU OU CELLULE
  }

  //13-------------------------grid----------------------------------------27/04/16-------------
  B3[97] = 1 ;  // gride refuse tout
  B3[98] = 1 ;// grid dem&&e delestage
  B3[99] = 1 ; // grid dem&&e boost
  B3[100] = 1 ;// grid accepte tout
  B3[101] = 1 ;// grid dem&&e soft charge
  B3[102] = 1 ;// grid dem&&e charge max
  B3[103] = 1 ;// grid exige charge à tout prix



  //14----------------------------voltaique--------------------------------------27/04/16-------
  if (B3[32] &&  F8[181] > N7[8] && ((B3[304] && F8[181] > N7[28]) || (!B3[304] && F8[181] > N7[29]))  ) {   // detection puissance pv 1
    N7[8] =  F8[181];
  }
  if  (!B3[304] && N7[8] > N7[29]) {
    B3[308] = 1 ;// autoconfig pv 1 detecte
  }
  if (B3[32] &&  F8[171] > N7[9] && ((B3[304] && F8[171] > N7[28]) || (!B3[304] && F8[171] > N7[29]))  ) {   // detection puissance pv 2
    N7[9] =  F8[171];
  }
  if (!B3[304] && N7[9] > N7[29]) {
    B3[309] = 1 ;// autoconfig pv 2 detecte
  }
  F8[134] = F8[189] + F8[179]; //I actuel tot pv 1 + 2
  if (buttonPin22Num >= 4 && buttonPin35Num >= 4 && B3[14] && !B3[89] && !B3[472] && !TDN[25]  ) {
    B3[130] = 1 ;// securite pv
  }
  else {
    B3[130] = 0 ;
  }
  if ( B3[124] && !TDN[2]  ) {
    B3[129] = 1 ;// cond ok balancing pv
  }
  else {
    B3[129] = 0 ;
  }
  if ( B3[129] && !TDN[27]  ) {
    B3[131] = 1 ;// dem&&e balancing pv
  }
  else {
    B3[131] = 0 ;
  }
  if (!TDN[9]  || (TDN[9] && !B3[122] && (F8[60] < F8[97]))  ) {
    B3[132] = 1 ;
  }
  else {
    B3[132] = 0 ;
  }
  if ((!B3[304]  || B3[269]) && B3[130] && (B3[131]  || B3[132]) && !T52TT ) { //    T52TT à déclarer et faire !! tof
    B3[224] = 1 ;
  }
  else {
    B3[224] = 0 ;
  }

  if (!B3[178] && B3[224]) {
    PWM49 = true;
    digitalWrite(LED49, HIGH);
    PWM51 = true;
    digitalWrite(LED51, HIGH);
  }
  else {
    PWM49 = false;
    digitalWrite(LED49, LOW);
    PWM51 = false;
    digitalWrite(LED51, LOW);
  }

  if (B3[224]) {
    T[52] =  0.0;
    TDN[52] =  1.0;
    CptT[52] = false;
  }
  else {
    CptT[52] = true;
  }

  if (T[52] >= 100) {
    TDN[52] =  0.0;
    T[52] = 100;
  }

  if (T[52] > 0 && TDN[52]) {
    T52TT = 1;
  }
  else {
    T52TT = 0;
  }

  if (B3[224] && T[9] > 0 && !B3[87] && (B3[308] && (F8[181] > N7[29])) || (B3[309] && F8[171] > N7[29])) {
    B3[135] = 1 ; // surplus production pv
  }
  if ( F8[60] < F8[64])  {
    B3[135] = 0 ; // surplus production pv
  }



  //15---------------------------ups----------------------------------------------------
  //------------------puissance UPS----------------------------------

  F8[136] = F8[151] + F8[161] + F8[171] + F8[181] + F8[191] + F8[201] + F10[13];
  F8[137] = F8[220] - F8[136];
  F8[132] = F8[137];//------------------------------------------------- affichage puissance consommée par UPD

  //-----------------------------------courant UPS-------------------------27/04/16-
  if ((F8[136] <= F10[33] && F8[136] >= F10[34]) || (F8[136] <= 10 && F8[136] >= -10)) {
    CptT[62] = true;
  }
  else {
    T[62] =  0.0;
    TDN[62] =  0.0;
    CptT[62] = false;
  }
  if (T[62] >= 20) {
    TDN[62] =  1.0;
    T[62] = 20;
  }

  F10[33] = F8[136] * F10[28];
  F10[34] = F8[136] / F10[28];
  if (!B3[317]) {
    F8[132] = 0.0;             //AFFICHAGE PUIS ACTUELLE UPS
  }

  if (B3[317] && F8[132] > N7[24]) {
    F8[132] = N7[24];             //AFFICHAGE PUIS ACTUELLE UPS
  }

  if (B3[215]) {//-cond ons
    if (!B3[49]) {  //---bit ons
      B3[49] = 1.0;
      B3[50] = 1.0;  //------------------------b3:3/1------- ONE SHOT START CHARGEUR 1
    }
    else {
      B3[50] = 0.0;
    }
  }
  else {
    B3[49] = 0.0;//---bit ons
    B3[50] = 0.0;
  }

  if (!B3[215]) {//-cond ons
    if (!B3[51]) {  //---bit ons
      B3[51] = 1.0;
      B3[52] = 1.0;  //------------------------b3:3/3------- ONE SHOT STOP CHARGEUR 1
    }
    else {
      B3[52] = 0.0;
    }
  }
  else {
    B3[51] = 0.0;//---bit ons
    B3[52] = 0.0;
  }

  T58PRE = (N7[19] + 2) * 10;
  if (B3[50] || B3[52]) {
    T[58] =  0.0;
    TDN[58] =  1.0;
    CptT[58] = false;
  }
  else {
    CptT[58] = true;
  }

  if (T[58] >= T58PRE) {
    TDN[58] =  0.0;
    T[58] = T58PRE;
  }

  if (T[58] > 0 && TDN[58]) {
    T58TT = 1;
  }
  else {
    T58TT = 0;
  }

  if (B3[216]) {//-cond ons
    if (!B3[53]) {  //---bit ons
      B3[53] = 1.0;
      B3[54] = 1.0;  //------------------------b3:3/5------- ONE SHOT START CHARGEUR 2
    }
    else {
      B3[54] = 0.0;
    }
  }
  else {
    B3[53] = 0.0;//---bit ons
    B3[54] = 0.0;
  }
  if (!B3[216]) {//-cond ons
    if (!B3[55]) {  //---bit ons
      B3[55] = 1.0;
      B3[56] = 1.0;  //------------------------b3:3/7------- ONE SHOT STOP CHARGEUR 2
    }
    else {
      B3[56] = 0.0;
    }
  }
  else {
    B3[55] = 0.0;//---bit ons
    B3[56] = 0.0;
  }

  //-----------------------------------------------------------rung 13 ups   27/04/16


  T59PRE = (N7[18] + 2) * 10;
  if (B3[54] || B3[56]) {
    T[59] =  0.0;
    TDN[59] =  1.0;
    CptT[59] = false;
  }
  else {
    CptT[59] = true;
  }

  if (T[59] >= T59PRE) {
    TDN[59] =  0.0;
    T[59] = T59PRE;
  }
  if (T[59] > 0 && TDN[59]) {
    T59TT = 1;
  }
  else {
    T59TT = 0;
  }


  if (B3[224]) {//-cond ons
    if (!B3[57]) {  //---bit ons
      B3[57] = 1.0;
      B3[58] = 1.0;  //------------------------b3:3/9------- ONE SHOT START PV 1
    }
    else {
      B3[58] = 0.0;
    }
  }
  else {
    B3[57] = 0.0;//---bit ons
    B3[58] = 0.0;
  }

  if (!B3[224]) {//-cond ons
    if (!B3[59]) {  //---bit ons
      B3[59] = 1.0;
      B3[60] = 1.0;  //------------------------b3:3/11------- ONE SHOT STOP PV 1
    }
    else {
      B3[60] = 0.0;
    }
  }
  else {
    B3[59] = 0.0;//---bit ons
    B3[60] = 0.0;
  }

  T60PRE = N7[16] * 10;
  if (B3[58] || B3[60]) {
    T[60] =  0.0;
    TDN[60] =  1.0;
    CptT[60] = false;
  }
  else {
    CptT[60] = true;
  }

  if (T[60] >= T60PRE) {
    TDN[60] =  0.0;
    T[60] = T60PRE;
  }
  if (T[60] > 0 && TDN[60]) {
    T60TT = 1;
  }
  else {
    T60TT = 0;
  }



  if (B3[223]) {//-cond ons
    if (!B3[61]) {  //---bit ons
      B3[61] = 1.0;
      B3[62] = 1.0;  //------------------------b3:3/13------- ONE SHOT START PV 2
    }
    else {
      B3[62] = 0.0;
    }
  }
  else {
    B3[61] = 0.0;//---bit ons
    B3[62] = 0.0;
  }
  if (!B3[223]) {//-cond ons
    if (!B3[63]) {  //---bit ons
      B3[63] = 1.0;
      B3[64] = 1.0;  //------------------------b3:3/15------- ONE SHOT STOP PV 2
    }
    else {
      B3[64] = 0.0;
    }
  }
  else {
    B3[63] = 0.0;//---bit ons
    B3[64] = 0.0;
  }
  T61PRE = N7[16] * 10;
  if (B3[62] || B3[64]) {
    T[61] =  0.0;
    TDN[61] =  1.0;
    CptT[61] = false;
  }
  else {
    CptT[61] = true;
  }

  if (T[61] >= T61PRE) {
    TDN[61] =  0.0;
    T[61] = T61PRE;
  }
  if (T[61] > 0 && TDN[61]) {
    T61TT = 1;
  }
  else {
    T61TT = 0;
  }

  if (!T58TT && !T59TT && !T60TT && !T61TT && TDN[62] && (F8[137] < F10[25]) ) {
    F10[25] = F8[137];
  }
  if ((F10[25] < N7[25] || buttonPin23Num >= 4) && !B3[304] && TDN[70]) {
    B3[317] = 1;
  }

  F10[27] = F10[31] - F8[136];

  if (B3[317] && !T58TT && !T59TT && !T60TT && !T61TT && TDN[62] && (F10[27] < F10[26]) ) {
    F10[26] = F10[27];
  }
  if (F10[26] > -200.0) {
    F10[26] = F10[25] * 2.0;
  }

  if (B3[304] || !B3[317]) {
    F10[25] = 0.0;
    F10[26] = 0.0;
  }

  if (B3[26]) {//-cond ons
    if (!B3[27]) {  //---bit ons
      B3[27] = 1.0;
      F10[25] = 0.0;
      F10[26] = 0.0;
    }
  }
  else {
    B3[27] = 0.0;//---bit ons
  }

  F8[149] = F8[137] / F8[60];
  if (F8[149] >= 0.0) {
    F8[133] = F8[149];
  }
  if (F8[149] < 0.0) {
    F8[133] = F8[149] * (-1);
  }
  //16-------------------------------onduleurs------------------------------------------

  if (!B3[150]) {
    F10[50] = F8[225] + F8[100];
  }
  if ((F10[50] > N7[38]) && ( B3[314] || B3[304])) {
    B3[146] = 1.0;
  }
  else {
    B3[146] = 0.0;
  }

  if (!B3[158]) {
    F10[51] = F8[227] + F8[100];
  }

  if ((F10[51] > N7[38]) && ( B3[315] || B3[304])) {
    B3[155] = 1.0;
  }
  else {
    B3[155] = 0.0;
  }

  if (!B3[150] && !B3[158]) {
    F10[52] = F8[227] + F10[50];
  }

  if ((F10[52] > N7[38]) && ( B3[314] || B3[315] || B3[304])) {
    B3[154] = 1.0;
  }
  else {
    B3[154] = 0.0;
  }

  if (B3[209] && buttonPin31Num >= 4) {
    CptT[29] = true;
  }

  else {
    T[29] =  0.0;
    TDN[29] =  0.0;
    CptT[29] = false;
  }
  if (T[29] >= 50) {
    TDN[29] =  1.0;
    T[29] = 50;
  }

  if ((!B3[85] && !B3[86] || B3[304]) && !B3[87] && !B3[215]  && !B3[216]) {
    B3[147] = 1.0;
  }
  else {
    B3[147] = 0.0;
  }

  if (buttonPin22Num >= 4 && buttonPin33Num < 4 && TDN[0] && !B3[468]) {
    B3[145] = 1.0;// SECURITE ONDULEUR
  }
  else {
    B3[145] = 0.0;
  }

  if (( B3[146] && !B3[213]) || ( B3[154] && !B3[213]) || ( ( F8[100] > N7[38]) && B3[213]) ) {
    B3[148] = 1.0;// CONDITION COURANT OK ONDULEUR
  }
  else {
    B3[148] = 0.0;
  }

  if ( B3[145] && TDN[29] && B3[147] && B3[148]) {
    B3[149] = 1.0;
  }
  else {
    B3[149] = 0.0;
  }

  if ( !B3[304]  && ( B3[135] || B3[161]) ) {
    B3[150] = 1.0;
  }
  else {
    B3[150] = 0.0;
  }

  if (B3[149] && (B3[150] || B3[249]) && !T39TT ) {
    B3[213] = 1.0;
  }
  else {
    B3[213] = 0.0;
  }
  if (B3[213] && !B3[178]) {
    PWM8 = 1;
    digitalWrite(LED8, HIGH);
  }
  else {
    PWM8 = 0;
    digitalWrite(LED8, LOW);
  }
  if (B3[213]) {
    T[39] =  0.0;
    TDN[39] =  1.0;
    CptT[39] = false;
  }
  else {
    CptT[39] = true;
  }
  if (T[39] >= 100) {
    TDN[39] =  0.0;
    T[39] = 100;
  }
  if (T[39] > 0 && TDN[39]) {
    T39TT = 1;
  }
  else {
    T39TT = 0;
  }


  if (B3[213]) {
    CptT[32] = true;
  }
  else {
    T[32] =  0.0;
    TDN[32] =  0.0;
    CptT[32] = false;
  }
  if (T[32] >= 900) {
    TDN[32] =  1.0;
    T[32] = 900;
  }


  if (buttonPin22Num >= 4 && buttonPin33Num < 4 && TDN[0] && !B3[469]) {
    B3[153] = 1.0;// SECURITE ONDULEUR
  }
  else {
    B3[153] = 0.0;
  }

  if ( ( B3[154] && !B3[214]) || ( ( F8[100] > N7[38]) && B3[214]) ) {
    B3[156] = 1.0;// CONDITION COURANT OK ONDULEUR
  }
  else {
    B3[156] = 0.0;
  }

  if ( B3[153] && TDN[29] && B3[147] && B3[156] && ( TDN[32] || B3[304])) {
    B3[157] = 1.0;
  }
  else {
    B3[157] = 0.0;
  }

  if ( !B3[304] && ( B3[135] || B3[161]) ) {
    B3[158] = 1.0;
  }
  else {
    B3[158] = 0.0;
  }

  if (B3[157] && (B3[158] || B3[255]) && !T55TT ) {
    B3[214] = 1.0;
  }
  else {
    B3[214] = 0.0;
  }
  if (B3[214] && !B3[178]) {
    PWM9 = 1;
    digitalWrite(LED9, HIGH);
  }
  else {
    PWM9 = 0;
    digitalWrite(LED9, LOW);
  }

  if (B3[214]) {
    T[55] =  0.0;
    TDN[55] =  1.0;
    CptT[55] = false;
  }
  else {
    CptT[55] = true;
  }

  if (T[55] >= 100) {
    TDN[55] =  0.0;
    T[55] = 100;
  }
  if (T[55] > 0 && TDN[55]) {
    T55TT = 1;
  }
  else {
    T55TT = 0;
  }
  if ( TDN[54] && (B3[213] || B3[214]) ) {
    B3[94] = 1.0;
  }
  else {
    B3[94] = 0.0;
  }

  if  ((B3[213] && (F8[161] < N7[37]) || (B3[214]) && (F8[151] < N7[37])) ) {
    B3[91] = 1.0;
  }
  else {
    B3[91] = 0.0;
  }

  F8[138] = F8[151] + F8[161];
  if (B3[213] && (F8[151] > N7[37]) ) {
    CptT[56] = true;
  }
  else {
    T[56] =  0.0;
    TDN[56] =  0.0;
    CptT[56] = false;
  }
  if (T[56] >= 1200) {
    TDN[56] =  1.0;
    T[56] = 1200;
  }
  if ( TDN[56] ) {
    B3[485] = 1.0;
  }


  if (B3[214] && (F8[151] > N7[37]) ) {
    CptT[57] = true;
  }
  else {
    T[57] =  0.0;
    TDN[57] =  0.0;
    CptT[57] = false;
  }
  if (T[57] >= 1200) {
    TDN[57] =  1.0;
    T[57] = 1200;
  }
  if ( TDN[57] ) {
    B3[486] = 1.0;
  }

  //17--------------------------------chargeurs-----------------------------------------

  if (F8[100] > N7[39] ) {
    T[63] =  0.0;
    TDN[63] =  1.0;
    CptT[63] = false;
  }
  else {
    CptT[63] = true;
  }

  if (T[63] >= 100) {
    TDN[63] =  0.0;
    T[63] = 100;
  }
  if (T[63] > 0 && TDN[63]) {
    T63TT = 1;
  }
  else {
    T63TT = 0;
  }

  F10[60] = F8[229] + F8[100];

  if (B3[311]  && (F10[60] < N7[39])) {
    CptT[64] = true;
  }
  else {
    T[64] =  0.0;
    TDN[64] =  0.0;
    CptT[64] = false;
  }
  if (T[64] >= 50) {
    TDN[64] =  1.0;
    T[64] = 50;
  }

  F10[61] = F8[100] + F8[231];

  if (B3[312]  && (F10[61] < N7[39])) {
    CptT[65] = true;
  }
  else {
    T[65] =  0.0;
    TDN[65] =  0.0;
    CptT[65] = false;
  }
  if (T[65] >= 50) {
    TDN[65] =  1.0;
    T[65] = 50;
  }



  if ( B3[311] && F8[229] >= F8[231] ) {
    B3[75] = 1 ;// dem&&e balancing pv
  }
  else {
    B3[75] = 0 ;
  }

  if ( B3[312] && !B3[75] && (F8[231] > F8[229])  ) {
    B3[76] = 1 ;
  }
  else {
    B3[76] = 0 ;
  }


  if ( buttonPin33Num < 4 && TDN[0] && (buttonPin41Num < 4 || buttonPin45Num < 4)) {
    B3[14] = 1 ; //cond secu charge batterie fuse et bms ok
  }
  else {
    B3[14] = 0 ;
  }

  if (buttonPin27Num >= 4 && (B3[209] && buttonPin31Num >= 4) || (B3[210] && buttonPin29Num >= 4 )) {
    B3[15] = 1 ; //cond reseau ok pour chargeur
  }
  else {
    B3[15] = 0 ;
  }

  if (buttonPin35Num < 4 || B3[90] || B3[123] || (buttonPin45Num >=4 && buttonPin43Num<4)) {
    B3[13] = 1 ;// fin charge batterie
  }
  else {
    B3[13] = 0 ;
  }

  if (B3[14] && B3[15] && !B3[13] && !B3[470] && !B3[475] ) {
    B3[16] = 1 ;// cond secu e reseau ok pour chargeur
  }
  else {
    B3[16] = 0 ;
  }

  if (B3[16] && !B3[161] && (!TDN[63] || B3[304]) && ( B3[95] || TDN[5] || B3[258] || B3[262])) {
    CptT[5] = true;
  }
  else {
    T[5] =  0.0;
    TDN[5] =  0.0;
    CptT[5] = false;
  }
  if (T[5] >= 30) {
    TDN[5] =  1.0;
    T[5] = 30;

  }

  if (TDN[5] && !TDN[27] && (TDN[2] || !B3[124] || B3[304])) {
    B3[2] = 1 ;// demande marche chargeur
  }
  else {
    B3[2] = 0 ;
  }

  if (B3[2] && !T6TT && ((!B3[304] && B3[75]) || (!B3[304] && TDN[68]) || B3[258]) && (TDN[64] || B3[215] || B3[258])) {
    B3[215] = 1 ;// bit interne pour avec dip switch faire la s||tie relais
  }
  else {
    B3[215] = 0 ;
  }
  if (B3[215] && !B3[178]) {
    PWM11 = 1;
    digitalWrite(LED11, HIGH);
  }
  else {
    PWM11 = 0;
    digitalWrite(LED11, LOW);
  }
  
  if (B3[215]) {
    T[6] =  0.0;
    TDN[6] =  1.0;
    CptT[6] = false;
  }
  else {
    CptT[6] = true;
  }

  if (T[6] >= 300) {
    TDN[6] =  0.0;
    T[6] = 300;
  }

  if (T[6] > 0 && TDN[6]) {
    T6TT = 1;
  }
  else {
    T6TT = 0;
  }

    if (N7[60]>80 && F8[100]>N7[69] && !B3[163] && !B3[164] && !B3[304]) {
    B3[252] = 1;
  }
  else {
   B3[252] = 0;
  }
   
  if ((B3[215] or !B3[311] or B3[304])&& !B3[252] && (!B3[162] or N7[60]<50)) {
    CptT[66] = true;
  }
  else {
    T[66] =  0.0;
    TDN[66] =  0.0;
    CptT[66] = false;
  }
  if (T[66] >= 200) {
    TDN[66] =  1.0;
    T[66] = 200;
  }

  
  if (B3[2] && !T67TT && !B3[210] && ((!B3[304] && B3[76]) || (!B3[304] && TDN[66]) || B3[262]) && (TDN[65] || B3[216] || B3[262])) {
    B3[216] = 1 ;// bit interne pour avec dip switch faire la s||tie relais
  }
  else {
    B3[216] = 0 ;
  }
  if (B3[216] && !B3[178]) {
    PWM12 = 1;
    digitalWrite(LED12, HIGH);
  }
  else {
    PWM12 = 0;
    digitalWrite(LED12, LOW);
  }

  if (B3[216]) {
    CptT[68] = true;
  }
  else {
    T[68] =  0.0;
    TDN[68] =  0.0;
    CptT[68] = false;
  }
  if (T[68] >= 200) {
    TDN[68] =  1.0;
    T[68] = 200;
  }


  if (B3[216]) {
    T[67] =  0.0;
    TDN[67] =  1.0;
    CptT[67] = false;
  }
  else {
    CptT[67] = true;
  }

  if (T[67] >= 200) {
    TDN[67] =  0.0;
    T[67] = 200;
  }

  if (T[67] > 0 && TDN[67]) {
    T67TT = 1;
  }
  else {
    T67TT = 0;
  }

  //18--------------------------------BALANCING-----------------------------------------

  if (((F8[60] > F10[54] || B3[122]) && (B3[132] || B3[2])) && !B3[304]) {
    B3[125] = 1;
  }
  else {
    B3[125] = 0;
  }

  if ((Heure == N7[23] || B3[125]) && !B3[304] && !B3[161]) {
    if (!B3[88]) {
      B3[88] = 1.0;
      B3[87] = 1.0;
      B3[66] = 0.0;
      B3[487] = 0.0;
      B3[488] = 0.0;
    }
  }
  else {
    B3[88] = 0.0;
  }


  if (B3[318] || B3[304] || B3[161] ) {
    B3[87] = 0.0;
  }

  if (B3[87]) {
    B3[92] = 1.0;
  }
  else {
    B3[92] = 0.0;
  }

  if (B3[318]) {
    B3[93] = 1.0;
  }
  else {
    B3[93] = 0.0;
  }

  if (B3[93]) {
    CptT[54] = true;
  }
  else {
    T[54] =  0.0;
    TDN[54] =  0.0;
    CptT[54] = false;
  }
  if (T[54] >= 50) {
    TDN[54] =  1.0;
    T[54] = 50;
  }
  
  if (!B3[306]){
N7[69]=N7[20];
  }
  
    if (B3[306]){
N7[69]=N7[21];
  }
  
  F8[147] = N7[69] + F8[133]; //----crt de charg mini pour balabcing


  if (B3[308] && (F8[189] > F8[147])) {    //---- crt pv1 suffisant pour faire balancing
    T[36] =  0.0;
    TDN[36] =  1.0;
    CptT[36] = false;
  }
  else {
    CptT[36] = true;
  }

  if (T[36] >= 100) {
    TDN[36] =  0.0;
    T[36] = 100;
  }

  if (B3[309] && (F8[189] > F8[147])) {          //---- crt pv2 suffisant pour faire balancing
    T[37] =  0.0;
    TDN[37] =  1.0;
    CptT[37] = false;
  }
  else {
    CptT[37] = true;
  }

  if (T[37] >= 100) {
    TDN[37] =  0.0;
    T[37] = 100;
  }

  if (B3[308] && B3[309] && (F8[134] > F8[147])) {    //---- crt pv1+2 suffisant pour faire balancing
    T[38] =  0.0;
    TDN[38] =  1.0;
    CptT[38] = false;
  }
  else {
    CptT[38] = true;
  }

  if (T[38] >= 100) {
    TDN[38] =  0.0;
    T[38] = 100;
  }

  if (TDN[36]) {
    B3[117] =  1.0;                   //-----------b3:7/4 ok balancing avec pv1
  }
  else {
    B3[117] = 0.0;
  }

  if (TDN[37] && !B3[117]) {
    B3[118] =  1.0;                   //-----------b3:7/5 ok balancing avec pv2
  }
  else {
    B3[118] = 0.0;
  }

  if (TDN[38] && !B3[117] && !B3[118]) {
    B3[119] =  1.0;                   //-----------b3:7/6 ok balancing avec pv1+2
  }
  else {
    B3[119] = 0.0;
  }

  if (!B3[117] && !B3[118] && !B3[119]) {          //---- crt pv insuffisant ok balancing avec chargeur
    CptT[2] = true;
  }
  else {
    T[2] =  0.0;
    TDN[2] =  0.0;
    CptT[2] = false;
  }
  if (T[2] >= 300) {
    TDN[2] =  1.0;
    T[2] = 300;
  }

  if (buttonPin50Num >= 4) {       //---- optocoupleur BCU actif batt1 au moins 1 cell balancing
    CptT[26] = true;
  }
  else {
    T[26] =  0.0;
    TDN[26] =  0.0;
    CptT[26] = false;
  }
  if (T[26] >= 10) {
    TDN[26] =  1.0;
    T[26] = 10;
  }


  if (buttonPin48Num >= 4) {         //---- optocoupleur BCU actif batt2 au moins 1 cell balancing
    CptT[31] = true;
  }
  else {
    T[31] =  0.0;
    TDN[31] =  0.0;
    CptT[31] = false;
  }
  if (T[31] >= 10) {
    TDN[31] =  1.0;
    T[31] = 10;
  }



  if (TDN[26] || TDN[31]) {
    B3[122] = 1.0;                 //------b3:7/9  au moins 1 BCU actif batt 1 ou 2
  }
  else {
    B3[122] = 0.0;
  }



  if ((buttonPin52Num >= 4) &&  ((buttonPin46Num >= 4) || buttonPin45Num < 4)) {
    B3[123] = 1.0;
    F8[14] = 69;
  }
  else {
    B3[123] = 0.0;
  }

  if (N7[10] == 0 && !B3[318] && (B3[122] || B3[124])) {
    B3[124] = 1.0;                 //------b3:7/11  tous les BCU actif batt 1+2
  }
  else {
    B3[124] = 0.0;
  }

  if (B3[123]) {
    B3[318] = 1.0;                 //------b3:19/13  memoire balancing done
  }
  F8[215] = F8[100];                 //-----I cgarge batt f||cage mini 2A

  if (F8[215] < 2.0) {
    F8[215] = 2.0;                 //-----I cgarge batt f||cage mini 2A
  }

  N7[22] = ((F8[216] / F8[215]) / (log( F8[215])) * 10 );
  T27PRE = N7[22];

  if (B3[122]) {
    CptT[27] = true;
  }
  else {
    T[27] =  0.0;
    TDN[27] =  0.0;
    CptT[27] = false;
  }
  if (T[27] >= T27PRE) {
    TDN[27] =  1.0;
    T[27] = T27PRE;
  }

  if (TDN[0] && !B3[2]) {
    if (!B3[17]) {
      B3[17] = 1.0;                 //------b3:1/0  ons coupure charg
      F8[46] = F8[60];
      N7[40] = Heure;
      N7[41] = Minute;
    }
  }
  else {
    B3[17] = 0.0;
  }

  if (TDN[0] && B3[123]) {//-cond ons
    if (!B3[65]) {  //---bit ons
      B3[65] = 1.0;  // a verifier
      B3[66] = 1.0;  //---------------------------b3:4/1---- memo coupure 16 BCU
    }
  }
  else {
    B3[65] = 0.0;//---bit ons
  }

  if (TDN[0] && buttonPin35Num < 4) {//-cond ons
    B3[487] = 1.0;  //------------------------b3:4/3------- memo coupure   BMS
  }

  if (TDN[0] && TDN[25]) {//-cond ons
    B3[488] = 1.0;
  }


  if (B3[122]) {//-cond ons
    if (!B3[73]) {  //---bit ons
      B3[73] = 1.0;
      if (!B3[72]) {
        F8[218] = F8[60];
      }
    }
  }
  else {
    B3[73] = 0.0;//---bit ons
  }

  if (B3[122]) {
    B3[72] = 1.0;
  }

  if (B3[87]) {//-cond ons
    if (!B3[77]) {  //---bit ons
      B3[77] = 1.0;
      B3[72] = 0.0;
    }
  }
  else {
    B3[77] = 0.0;//---bit ons
  }

  if (B3[123]) {//-cond ons
    if (!B3[71]) {  //---bit ons
      B3[71] = 1.0;
      F8[219] = F8[60];
    }
  }
  else {
    B3[71] = 0.0;//---bit ons
  }

  if (B3[123]) {//-cond ons
    if (!B3[74]) {  //---bit ons
      B3[74] = 1.0;
      F8[217] = F8[219] - F8[218];
    }
  }
  else {
    B3[74] = 0.0;//---bit ons
  }
  //FIN BALANCING

  //19--------------------------------------------------------------------------
  switch (B3[337]) {
    case 0:
      B3[513] = 0;
      break;
    case 1:
      if (B3[513] == 0) {
        N7[90] = 1;
      }
      B3[513] = 1;
      break;
  }

  if (B3[338]) {//-cond ons
    if (!B3[514]) {  //---bit ons
      B3[514] = 1.0;
      N7[90] = 2;
    }
  }
  else {
    B3[514] = 0.0;//---bit ons
  }

  if (B3[305]) {//-cond ons
    if (!B3[515]) {  //---bit ons
      B3[515] = 1.0;
      N7[90] = 3;
    }
  }
  else {
    B3[515] = 0.0;//---bit ons
  }
  if (B3[340]) {//-cond ons
    if (!B3[516]) {  //---bit ons
      B3[516] = 1.0;
      N7[90] = 4;
    }
  }
  else {
    B3[516] = 0.0;//---bit ons
  }
  if (B3[341]) {//-cond ons
    if (!B3[517]) {  //---bit ons
      B3[517] = 1.0;
      N7[90] = 5;
    }
  }
  else {
    B3[517] = 0.0;//---bit ons
  }
  if (B3[306]) {//-cond ons
    if (!B3[518]) {  //---bit ons
      B3[518] = 1.0;
      N7[90] = 6;
    }
  }
  else {
    B3[518] = 0.0;//---bit ons
  }
  if (B3[343]) {//-cond ons
    if (!B3[519]) {  //---bit ons
      B3[519] = 1.0;
      N7[90] = 7;
    }
  }
  else {
    B3[519] = 0.0;//---bit ons
  }
  if (B3[344]) {//-cond ons
    if (!B3[520]) {  //---bit ons
      B3[520] = 1.0;
      N7[90] = 8;
    }
  }
  else {
    B3[520] = 0.0;//---bit ons
  }


  if (B3[345]) {//-cond ons
    if (!B3[521]) {  //---bit ons
      B3[521] = 1.0;
      N7[90] = 9;
    }
  }
  else {
    B3[521] = 0.0;//---bit ons
  }

  if (B3[346]) {//cond ons
    if (!B3[522]) {  //---bit ons
      B3[522] = 1.0;
      N7[90] = 10;
      N7[95] = N7[95] + 1;
    }
  }
  else {
    B3[522] = 0.0;//---bit ons
  }


  if (B3[347]) {//-cond ons
    if (!B3[523]) {  //---bit ons
      B3[523] = 1.0;
      N7[90] = 11;
    }
  }
  else {
    B3[523] = 0.0;//---bit ons
  }

  if (B3[357]) {//-cond ons
    if (!B3[533]) {  //---bit ons
      B3[533] = 1.0;
      N7[90] = 21;
    }
  }
  else {
    B3[533] = 0.0;//---bit ons
  }
  if (B3[358]) {//-cond ons
    if (!B3[534]) {  //---bit ons
      B3[534] = 1.0;
      N7[90] = 22;
    }
  }
  else {
    B3[534] = 0.0;//---bit ons
  }
  if (B3[359]) {//-cond ons
    if (!B3[535]) {  //---bit ons
      B3[535] = 1.0;
      N7[90] = 23;
    }
  }
  else {
    B3[535] = 0.0;//---bit ons
  }
  if (B3[360]) {//-cond ons
    if (!B3[536]) {  //---bit ons
      B3[536] = 1.0;
      N7[90] = 24;
    }
  }
  else {
    B3[536] = 0.0;//---bit ons
  }
  if (B3[361]) {//-cond ons
    if (!B3[537]) {  //---bit ons
      B3[537] = 1.0;
      N7[90] = 25;
    }
  }
  else {
    B3[537] = 0.0;//---bit ons
  }
  if (B3[362]) {//-cond ons
    if (!B3[538]) {  //---bit ons
      B3[538] = 1.0;
      N7[90] = 26;
    }
  }
  else {
    B3[538] = 0.0;//---bit ons
  }
  if (B3[363]) {//-cond ons
    if (!B3[539]) {  //---bit ons
      B3[539] = 1.0;
      N7[90] = 27;

    }
  }
  else {
    B3[539] = 0.0;//---bit ons
  }
  if (B3[364]) {//-cond ons
    if (!B3[540]) {  //---bit ons
      B3[540] = 1.0;
      N7[90] = 28;
    }
  }
  else {
    B3[540] = 0.0;//---bit ons
  }
  if (B3[365]) {//-cond ons
    if (!B3[541]) {  //---bit ons
      B3[510] = 1.0;
      N7[90] = 29;
    }
  }
  else {
    B3[541] = 0.0;//---bit ons
  }
  if (B3[366]) {//-cond ons
    if (!B3[542]) {  //---bit ons
      B3[542] = 1.0;
      N7[90] = 30;
    }
  }
  else {
    B3[542] = 0.0;//---bit ons
  }
  if (B3[367]) {//-cond ons
    if (!B3[543]) {  //---bit ons
      B3[543] = 1.0;
      N7[90] = 31;
    }
  }
  else {
    B3[543] = 0.0;//---bit ons
  }
  if (B3[368]) {//-cond ons
    if (!B3[544]) {  //---bit ons
      B3[544] = 1.0;
      N7[90] = 32;
    }
  }
  else {
    B3[544] = 0.0;//---bit ons
  }
  if (B3[368]) {//-cond ons
    if (!B3[544]) {  //---bit ons
      B3[544] = 1.0;
      N7[90] = 32;
    }
  }
  else {
    B3[544] = 0.0;//---bit ons
  }
  if (B3[369]) {//-cond ons
    if (!B3[545]) {  //---bit ons
      B3[545] = 1.0;
      N7[90] = 33;
    }
  }
  else {
    B3[545] = 0.0;//---bit ons
  }


  if (B3[370]) {//-cond ons
    if (!B3[546]) {  //---bit ons
      B3[546] = 1.0;
      N7[90] = 34;
    }
  }
  else {
    B3[546] = 0.0;//---bit ons
  }
  if (B3[371]) {//-cond ons
    if (!B3[547]) {  //---bit ons
      B3[547] = 1.0;
      N7[90] = 35;
    }
  }
  else {
    B3[547] = 0.0;//---bit ons
  }
  if (B3[372]) {//-cond ons
    if (!B3[548]) {  //---bit ons
      B3[548] = 1.0;
      N7[90] = 36;
    }
  }
  else {
    B3[548] = 0.0;//---bit ons
  }
  if (B3[373]) {//-cond ons
    if (!B3[549]) {  //---bit ons
      B3[549] = 1.0;
      N7[90] = 37;
    }
  }
  else {
    B3[549] = 0.0;//---bit ons
  }
  if (B3[374]) {//-cond ons
    if (!B3[550]) {  //---bit ons
      B3[550] = 1.0;
      N7[90] = 38;
    }
  }
  else {
    B3[550] = 0.0;//---bit ons
  }
  if (B3[375]) {//-cond ons
    if (!B3[551]) {  //---bit ons
      B3[551] = 1.0;
      N7[90] = 39;
    }
  }
  else {
    B3[551] = 0.0;//---bit ons
  }
  if (B3[376]) {//-cond ons
    if (!B3[552]) {  //---bit ons
      B3[552] = 1.0;
      N7[90] = 40;
    }
  }
  else {
    B3[552] = 0.0;//---bit ons
  }

  if (B3[377]) {//-cond ons
    if (!B3[553]) {  //---bit ons
      B3[553] = 1.0;
      N7[90] = 41;
    }
  }
  else {
    B3[553] = 0.0;//---bit ons
  }
  if (B3[378]) {//-cond ons
    if (!B3[554]) {  //---bit ons
      B3[554] = 1.0;
      N7[90] = 42;
    }
  }
  else {
    B3[554] = 0.0;//---bit ons
  }
  if (B3[379]) {//-cond ons
    if (!B3[555]) {  //---bit ons
      B3[555] = 1.0;
      N7[90] = 43;
    }
  }
  else {
    B3[555] = 0.0;//---bit ons
  }
  if (B3[380]) {//-cond ons
    if (!B3[556]) {  //---bit ons
      B3[556] = 1.0;
      N7[90] = 44;
    }
  }
  else {
    B3[556] = 0.0;//---bit ons
  }
  if (B3[381]) {//-cond ons
    if (!B3[557]) {  //---bit ons
      B3[557] = 1.0;
      N7[90] = 45;
    }
  }
  else {
    B3[557] = 0.0;//---bit ons
  }
  if (B3[382]) {//-cond ons
    if (!B3[558]) {  //---bit ons
      B3[558] = 1.0;
      N7[90] = 46;
    }
  }
  else {
    B3[558] = 0.0;//---bit ons
  }
  if (B3[383]) {//-cond ons
    if (!B3[559]) {  //---bit ons
      B3[559] = 1.0;
      N7[90] = 47;
    }
  }
  else {
    B3[559] = 0.0;//---bit ons
  }
  if (B3[384]) {//-cond ons
    if (!B3[560]) {  //---bit ons
      B3[560] = 1.0;
      N7[90] = 48;
    }
  }
  else {
    B3[560] = 0.0;//---bit ons
  }
  if (B3[385]) {//-cond ons
    if (!B3[561]) {  //---bit ons
      B3[561] = 1.0;
      N7[90] = 49;
    }
  }
  else {
    B3[561] = 0.0;//---bit ons
  }
  if (B3[386]) {//-cond ons
    if (!B3[562]) {  //---bit ons
      B3[562] = 1.0;
      N7[90] = 50;
    }
  }
  else {
    B3[562] = 0.0;//---bit ons
  }
  if (B3[387]) {//-cond ons
    if (!B3[563]) {  //---bit ons
      B3[563] = 1.0;
      N7[90] = 51;
    }
  }
  else {
    B3[563] = 0.0;//---bit ons
  }
  if (B3[388]) {//-cond ons
    if (!B3[564]) {  //---bit ons
      B3[564] = 1.0;
      N7[90] = 52;
    }
  }
  else {
    B3[564] = 0.0;//---bit ons
  }
  if (B3[389]) {//-cond ons
    if (!B3[565]) {  //---bit ons
      B3[565] = 1.0;
      N7[90] = 53;
    }
  }
  else {
    B3[565] = 0.0;//---bit ons
  }
  if (B3[390]) {//-cond ons
    if (!B3[566]) {  //---bit ons
      B3[566] = 1.0;
      N7[90] = 54;
    }
  }
  else {
    B3[566] = 0.0;//---bit ons
  }
  if (B3[391]) {//-cond ons
    if (!B3[567]) {  //---bit ons
      B3[567] = 1.0;
      N7[90] = 55;
    }
  }
  else {
    B3[567] = 0.0;//---bit ons
  }
  if (B3[392]) {//-cond ons
    if (!B3[568]) {  //---bit ons
      B3[568] = 1.0;
      N7[90] = 56;
    }
  }
  else {
    B3[568] = 0.0;//---bit ons
  }
  if (B3[401]) {//-cond ons
    if (!B3[577]) {  //---bit ons
      B3[577] = 1.0;
      N7[90] = 57;
    }
  }
  else {
    B3[577] = 0.0;//---bit ons
  }
  if (B3[402]) {//-cond ons
    if (!B3[578]) {  //---bit ons
      B3[578] = 1.0;
      N7[90] = 58;
    }
  }
  else {
    B3[578] = 0.0;//---bit ons
  }
  if (B3[403]) {//-cond ons
    if (!B3[579]) {  //---bit ons
      B3[579] = 1.0;
      N7[90] = 59;
    }
  }
  else {
    B3[579] = 0.0;//---bit ons
  }
  if (B3[404]) {//-cond ons
    if (!B3[580]) {  //---bit ons
      B3[580] = 1.0;
      N7[90] = 60;
    }
  }
  else {
    B3[580] = 0.0;//---bit ons
  }
  if (B3[405]) {//-cond ons
    if (!B3[581]) {  //---bit ons
      B3[581] = 1.0;
      N7[90] = 61;
    }
  }
  else {
    B3[581] = 0.0;//---bit ons
  }
  if (B3[406]) {//-cond ons
    if (!B3[582]) {  //---bit ons
      B3[582] = 1.0;
      N7[90] = 62;
    }
  }
  else {
    B3[582] = 0.0;//---bit ons
  }
  if (B3[407]) {//-cond ons
    if (!B3[583]) {  //---bit ons
      B3[583] = 1.0;
      N7[90] = 63;
    }
  }
  else {
    B3[583] = 0.0;//---bit ons
  }

  if (B3[496]) {//-cond ons
    if (!B3[589]) {  //---bit ons
      B3[589] = 1.0;
      N7[90] = 69;
    }
  }
  else {
    B3[589] = 0.0;//---bit ons
  }
  if (B3[497]) {//-cond ons
    if (!B3[590]) {  //---bit ons
      B3[590] = 1.0;
      N7[90] = 70;
    }
  }
  else {
    B3[590] = 0.0;//---bit ons
  }
  if (B3[311]) {//-cond ons
    if (!B3[591]) {  //---bit ons
      B3[591] = 1.0;
      N7[90] = 71;
    }
  }
  else {
    B3[591] = 0.0;//---bit ons
  }
  if (B3[312]) {//-cond ons
    if (!B3[592]) {  //---bit ons
      B3[592] = 1.0;
      N7[90] = 72;
    }
  }
  else {
    B3[592] = 0.0;//---bit ons
  }

  if (B3[408]) {//-cond ons
    if (!B3[594]) {  //---bit ons
      B3[594] = 1.0;
      N7[90] = 74;
    }
  }
  else {
    B3[594] = 0.0;//---bit ons
  }
  if (B3[393]) {//-cond ons
    if (!B3[569]) {  //---bit ons
      B3[569] = 1.0;
      N7[90] = 75;
    }
  }
  else {
    B3[569] = 0.0;//---bit ons
  }
  if (B3[394]) {//-cond ons
    if (!B3[570]) {  //---bit ons
      B3[570] = 1.0;
      N7[90] = 76;
    }
  }
  else {
    B3[570] = 0.0;//---bit ons
  }
  if (B3[91]) {//-cond ons
    if (!B3[571]) {  //---bit ons
      B3[571] = 1.0;
      N7[90] = 77;
    }
  }
  else {
    B3[571] = 0.0;//---bit ons
  }
  if (B3[92]) {//-cond ons
    if (!B3[572]) {  //---bit ons
      B3[572] = 1.0;
      N7[90] = 78;
    }
  }
  else {
    B3[572] = 0.0;//---bit ons
  }
  if (B3[93]) {//-cond ons
    if (!B3[573]) {  //---bit ons
      B3[573] = 1.0;
      N7[90] = 79;
    }
  }
  else {
    B3[573] = 0.0;//---bit ons
  }

  if (B3[94]) {//-cond ons
    if (!B3[574]) {  //---bit ons
      B3[574] = 1.0;
      N7[90] = 80;
    }
  }
  else {
    B3[574] = 0.0;//---bit ons
  }



  if (B3[317]) {//-cond ons
    if (!B3[595]) {  //---bit ons
      B3[595] = 1.0;
      N7[90] = 83;
    }
  }
  else {
    B3[595] = 0.0;//---bit ons
  }

  if (B3[308]) {//-cond ons
    if (!B3[596]) {  //---bit ons
      B3[596] = 1.0;
      N7[90] = 84;
    }
  }
  else {
    B3[596] = 0.0;//---bit ons
  }

  if (B3[309]) {//-cond ons
    if (!B3[597]) {  //---bit ons
      B3[597] = 1.0;
      N7[90] = 85;
    }
  }
  else {
    B3[597] = 0.0;//---bit ons
  }

  if (B3[241]) {//-cond ons
    if (!B3[598]) {  //---bit ons
      B3[598] = 1.0;
      N7[90] = 86;
    }
  }
  else {
    B3[598] = 0.0;//---bit ons
  }

  if (B3[242]) {//-cond ons
    if (!B3[599]) {  //---bit ons
      B3[599] = 1.0;
      N7[90] = 87;
    }
  }
  else {
    B3[599] = 0.0;//---bit ons
  }

  if (B3[243]) {//-cond ons
    if (!B3[600]) {  //---bit ons
      B3[600] = 1.0;
      N7[90] = 88;
    }
  }
  else {
    B3[600] = 0.0;//---bit ons
  }

  if (B3[244]) {//-cond ons
    if (!B3[601]) {  //---bit ons
      B3[601] = 1.0;
      N7[90] = 89;
    }
  }
  else {
    B3[601] = 0.0;//---bit ons
  }

  if (B3[245]) {//-cond ons
    if (!B3[602]) {  //---bit ons
      B3[602] = 1.0;
      N7[90] = 90;
    }
  }
  else {
    B3[602] = 0.0;//---bit ons
  }

  if (B3[246]) {//-cond ons
    if (!B3[603]) {  //---bit ons
      B3[603] = 1.0;
      N7[90] = 91;
    }
  }
  else {
    B3[603] = 0.0;//---bit ons
  }

  if (B3[247]) {//-cond ons
    if (!B3[604]) {  //---bit ons
      B3[604] = 1.0;
      N7[90] = 92;
    }
  }
  else {
    B3[604] = 0.0;//---bit ons
  }

  if (B3[250]) {//-cond ons
    if (!B3[605]) {  //---bit ons
      B3[605] = 1.0;
      N7[90] = 93;
    }
  }
  else {
    B3[605] = 0.0;//---bit ons
  }

  if (B3[251]) {//-cond ons
    if (!B3[606]) {  //---bit ons
      B3[606] = 1.0;
      N7[90] = 94;
    }
  }
  else {
    B3[606] = 0.0;//---bit ons
  }

  if (B3[161]) {//-cond ons
    if (!B3[169]) {  //---bit ons
      B3[169] = 1.0;
      N7[90] = 95;
    }
  }
  else {
    B3[169] = 0.0;//---bit ons
  }

  if (B3[162]) {//-cond ons
    if (!B3[170]) {  //---bit ons
      B3[170] = 1.0;
      N7[90] = 96;
    }
  }
  else {
    B3[170] = 0.0;//---bit ons
  }
  if (B3[163]) {//-cond ons
    if (!B3[171]) {  //---bit ons
      B3[171] = 1.0;
      N7[90] = 97;
    }
  }
  else {
    B3[171] = 0.0;//---bit ons
  }

  if (B3[164]) {//-cond ons
    if (!B3[172]) {  //---bit ons
      B3[172] = 1.0;
      N7[90] = 98;
    }
  }
  else {
    B3[172] = 0.0;//---bit ons
  }
  if (B3[165]) {//-cond ons
    if (!B3[173]) {  //---bit ons
      B3[173] = 1.0;
      N7[90] = 99;
    }
  }
  else {
    B3[173] = 0.0;//---bit ons
  }

   if (B3[254]) {//-cond ons
    if (!B3[166]) {  //---bit ons
      B3[166] = 1.0;
      N7[90] = 100;
    }
  }
  else {
    B3[166] = 0.0;//---bit ons
  }
  
  
   if (B3[253]) {//-cond ons
    if (!B3[167]) {  //---bit ons
      B3[167] = 1.0;
      N7[90] = 101;
    }
  }
  else {
    B3[167] = 0.0;//---bit ons
  }
  

  if (B3[449]) {//-cond ons
    if (!B3[321]) {  //---bit ons
      B3[321] = 1.0;
      N7[91] = 1;
    }
  }
  else {
    B3[321] = 0.0;//---bit ons
  }


  if (B3[450]) {//-cond ons
    if (!B3[322]) {  //---bit ons
      B3[322] = 1.0;
      N7[91] = 2;
    }
  }
  else {
    B3[322] = 0.0;//---bit ons
  }


  if (B3[451]) {//-cond ons
    if (!B3[323]) {  //---bit ons
      B3[323] = 1.0;
      N7[91] = 3;
    }
  }
  else {
    B3[323] = 0.0;//---bit ons
  }


  if (B3[452]) {//-cond ons
    if (!B3[324]) {  //---bit ons
      B3[324] = 1.0;
      N7[91] = 4;
    }
  }
  else {
    B3[324] = 0.0;//---bit ons
  }




  if (B3[453]) {//-cond ons
    if (!B3[325]) {  //---bit ons
      B3[325] = 1.0;
      N7[91] = 5;
    }
  }
  else {
    B3[325] = 0.0;//---bit ons
  }


  if (B3[454]) {//-cond ons
    if (!B3[326]) {  //---bit ons
      B3[326] = 1.0;
      N7[91] = 6;
    }
  }
  else {
    B3[326] = 0.0;//---bit ons
  }


  if (B3[455]) {//-cond ons
    if (!B3[327]) {  //---bit ons
      B3[327] = 1.0;
      N7[91] = 7;
    }
  }
  else {
    B3[327] = 0.0;//---bit ons
  }


  if (B3[456]) {//-cond ons
    if (!B3[328]) {  //---bit ons
      B3[328] = 1.0;
      N7[91] = 8;
    }
  }
  else {
    B3[328] = 0.0;//---bit ons
  }

  if (B3[457]) {//-cond ons
    if (!B3[329]) {  //---bit ons
      B3[329] = 1.0;
      N7[91] = 9;
    }
  }
  else {
    B3[329] = 0.0;//---bit ons
  }

  if (B3[458]) {//-cond ons
    if (!B3[330]) {  //---bit ons
      B3[330] = 1.0;
      N7[91] = 10;
    }
  }
  else {
    B3[330] = 0.0;//---bit ons
  }

  if (B3[459]) {//-cond ons
    if (!B3[331]) {  //---bit ons
      B3[331] = 1.0;
      N7[91] = 11;
    }
  }
  else {
    B3[331] = 0.0;//---bit ons
  }

  if (B3[460]) {//-cond ons
    if (!B3[332]) {  //---bit ons
      B3[332] = 1.0;
      N7[91] = 12;
    }
  }
  else {
    B3[332] = 0.0;//---bit ons
  }

  if (B3[461]) {//-cond ons
    if (!B3[333]) {  //---bit ons
      B3[333] = 1.0;
      N7[91] = 13;
    }
  }
  else {
    B3[333] = 0.0;//---bit ons
  }

  if (B3[462]) {//-cond ons
    if (!B3[334]) {  //---bit ons
      B3[334] = 1.0;
      N7[91] = 14;
    }
  }
  else {
    B3[334] = 0.0;//---bit ons
  }

  if (B3[463]) {//-cond ons
    if (!B3[335]) {  //---bit ons
      B3[335] = 1.0;
      N7[91] = 15;
    }
  }
  else {
    B3[335] = 0.0;//---bit ons
  }

  if (B3[464]) {//-cond ons
    if (!B3[336]) {  //---bit ons
      B3[336] = 1.0;
      N7[91] = 16;
    }
  }
  else {
    B3[336] = 0.0;//---bit ons
  }

  if (B3[465]) {//-cond ons
    if (!B3[417]) {  //---bit ons
      B3[417] = 1.0;
      N7[91] = 17;
    }
  }
  else {
    B3[417] = 0.0;//---bit ons
  }

  if (B3[466]) {//-cond ons
    if (!B3[418]) {  //---bit ons
      B3[418] = 1.0;
      N7[91] = 18;
    }
  }
  else {
    B3[418] = 0.0;//---bit ons
  }
  if (B3[466]) {//-cond ons
    if (!B3[418]) {  //---bit ons
      B3[418] = 1.0;
      N7[91] = 18;
    }
  }
  else {
    B3[418] = 0.0;//---bit ons
  }
  if (B3[467]) {//-cond ons
    if (!B3[419]) {  //---bit ons
      B3[419] = 1.0;
      N7[91] = 19;
    }
  }
  else {
    B3[419] = 0.0;//---bit ons
  }
  if (B3[468]) {//-cond ons
    if (!B3[420]) {  //---bit ons
      B3[420] = 1.0;
      N7[91] = 20;
    }
  }
  else {
    B3[420] = 0.0;//---bit ons
  }
  if (B3[469]) {//-cond ons
    if (!B3[421]) {  //---bit ons
      B3[421] = 1.0;
      N7[91] = 21;
    }
  }
  else {
    B3[421] = 0.0;//---bit ons
  }
  if (B3[470]) {//-cond ons
    if (!B3[422]) {  //---bit ons
      B3[422] = 1.0;
      N7[91] = 22;
    }
  }
  else {
    B3[422] = 0.0;//---bit ons
  }
  if (B3[471]) {//-cond ons
    if (!B3[423]) {  //---bit ons
      B3[423] = 1.0;
      N7[91] = 23;
    }
  }
  else {
    B3[423] = 0.0;//---bit ons
  }
  if (B3[472]) {//-cond ons
    if (!B3[424]) {  //---bit ons
      B3[424] = 1.0;
      N7[91] = 24;
    }
  }
  else {
    B3[424] = 0.0;//---bit ons
  }
  if (B3[473]) {//-cond ons
    if (!B3[425]) {  //---bit ons
      B3[425] = 1.0;
      N7[91] = 25;
    }
  }
  else {
    B3[425] = 0.0;//---bit ons
  }
  if (B3[474]) {//-cond ons
    if (!B3[426]) {  //---bit ons
      B3[426] = 1.0;
      N7[91] = 26;
    }
  }
  else {
    B3[426] = 0.0;//---bit ons
  }
  if (B3[475]) {//-cond ons
    if (!B3[427]) {  //---bit ons
      B3[427] = 1.0;
      N7[91] = 27;
    }
  }
  else {
    B3[427] = 0.0;//---bit ons
  }
  if (B3[476]) {//-cond ons
    if (!B3[428]) {  //---bit ons
      B3[428] = 1.0;
      N7[91] = 28;
    }
  }
  else {
    B3[428] = 0.0;//---bit ons
  }

  if (B3[477]) {//-cond ons
    if (!B3[429]) {  //---bit ons
      B3[429] = 1.0;
      N7[91] = 29;
    }
  }
  else {
    B3[429] = 0.0;//---bit ons
  }
  if (B3[478]) {//-cond ons
    if (!B3[430]) {  //---bit ons
      B3[430] = 1.0;
      N7[91] = 30;
    }
  }
  else {
    B3[430] = 0.0;//---bit ons
  }
  if (B3[479]) {//-cond ons
    if (!B3[431]) {  //---bit ons
      B3[431] = 1.0;
      N7[91] = 31;
    }
  }
  else {
    B3[431] = 0.0;//---bit ons
  }
  if (B3[480]) {//-cond ons
    if (!B3[432]) {  //---bit ons
      B3[432] = 1.0;
      N7[91] = 32;
    }
  }
  else {
    B3[432] = 0.0;//---bit ons
  }
  if (B3[481]) {//-cond ons
    if (!B3[433]) {  //---bit ons
      B3[433] = 1.0;
      N7[91] = 33;
    }
  }
  else {
    B3[433] = 0.0;//---bit ons
  }
  if (B3[482]) {//-cond ons
    if (!B3[434]) {  //---bit ons
      B3[434] = 1.0;
      N7[91] = 34;
    }
  }
  else {
    B3[434] = 0.0;//---bit ons
  }
  if (B3[483]) {//-cond ons
    if (!B3[435]) {  //---bit ons
      B3[435] = 1.0;
      N7[91] = 35;
    }
  }
  else {
    B3[435] = 0.0;//---bit ons
  }
  if (B3[484]) {//-cond ons
    if (!B3[436]) {  //---bit ons
      B3[436] = 1.0;
      N7[91] = 36;
    }
  }
  else {
    B3[436] = 0.0;//---bit ons
  }
  if (B3[485]) {//-cond ons
    if (!B3[437]) {  //---bit ons
      B3[437] = 1.0;
      N7[91] = 37;
    }
  }
  else {
    B3[437] = 0.0;//---bit ons
  }
  if (B3[486]) {//-cond ons
    if (!B3[438]) {  //---bit ons
      B3[438] = 1.0;
      N7[91] = 38;
    }
  }
  else {
    B3[438] = 0.0;//---bit ons
  }
  if (B3[487]) {//-cond ons
    if (!B3[439]) {  //---bit ons
      B3[439] = 1.0;
      N7[91] = 39;
    }
  }
  else {
    B3[439] = 0.0;//---bit ons
  }
  if (B3[488]) {//-cond ons
    if (!B3[440]) {  //---bit ons
      B3[440] = 1.0;
      N7[91] = 40;
    }
  }
  else {
    B3[440] = 0.0;//---bit ons
  }
  if (B3[489]) {//-cond ons
    if (!B3[441]) {  //---bit ons
      B3[441] = 1.0;
      N7[91] = 41;
    }
  }
  else {
    B3[441] = 0.0;//---bit ons
  }
  if (B3[490]) {//-cond ons
    if (!B3[442]) {  //---bit ons
      B3[442] = 1.0;
      N7[91] = 42;
    }
  }
  else {
    B3[442] = 0.0;//---bit ons
  }
  if (B3[491]) {//-cond ons
    if (!B3[443]) {  //---bit ons
      B3[443] = 1.0;
      N7[91] = 43;
    }
  }
  else {
    B3[443] = 0.0;//---bit ons
  }
  if (B3[492]) {//-cond ons
    if (!B3[444]) {  //---bit ons
      B3[444] = 1.0;
      N7[91] = 44;
    }
  }
  else {
    B3[444] = 0.0;//---bit ons
  }
  if (B3[493]) {//-cond ons
    if (!B3[445]) {  //---bit ons
      B3[445] = 1.0;
      N7[91] = 45;
    }
  }
  else {
    B3[445] = 0.0;//---bit ons
  }
  if (B3[494]) {//-cond ons
    if (!B3[446]) {  //---bit ons
      B3[446] = 1.0;
      N7[91] = 46;
    }
  }
  else {
    B3[446] = 0.0;//---bit ons
  }
  if (B3[495]) {//-cond ons
    if (!B3[447]) {  //---bit ons
      B3[447] = 1.0;
      N7[91] = 47;
    }
  }
  else {
    B3[447] = 0.0;//---bit ons
  }
  if (B3[496]) {//-cond ons
    if (!B3[448]) {  //---bit ons
      B3[448] = 1.0;
      N7[91] = 48;
    }
  }
  else {
    B3[448] = 0.0;//---bit ons
  }

  PasMessage();

  //FIN MESSAGES

  //20---------------------AUTOCONFIG--------------------------------------------

  if (buttonPin22Num >= 4 && SYNCHRO == 1) { //-cond ons
    if (!B3[279]) {  //---bit ons
      B3[279] = 1.0;
      N7[10] = 5;
    }
  }
  else {
    B3[279] = 0.0;//---bit ons
  }

  if (buttonPin22Num < 4) {
    B3[1] = 0.0;
  }

  // tempo message au pas autoconfig

  if (N7[10] == N7[11]) {
    CptT[70] = true;

  }
  else {
    T[70] =  0.0;
    TDN[70] =  0.0;
    CptT[70] = false;
  }
  if (T[70] >= (N7[12] * 10.0)) {
    TDN[70] =  1.0;
    T[70] = N7[12] * 10.0;

  }
  //Serial.println(N7[10]);
  // tempo entre message au meme pas autoconfig

  if (TDN[70]) {
    CptT[71] = true;
  }
  else {
    T[71] =  0.0;
    TDN[71] =  0.0;
    CptT[71] = false;
  }
  if (T[71] >= (N7[13] * 10.0)) {
    TDN[71] =  1.0;
    T[71] = N7[13] * 10.0;

  }
  // tempo second message operateur decrit l'action autoconfig

  if (TDN[71]) {
    CptT[72] = true;
  }
  else {
    T[72] =  0.0;
    TDN[72] =  0.0;
    CptT[72] = false;
  }
  if (T[72] >= (N7[14] * 10.0)) {
    TDN[72] =  1.0;
    T[72] = N7[14] * 10.0;

  }

  N7[11] = N7[10];

  if (B3[275]) {                          // F||CER A LA CONSOLE ATTENTION MEMOIRE
    N7[10] = 5.0;  //--- pas 5
  }

  if (N7[10] ==  5.0) {                   //PAS 10
    B3[337] =  1.0;
    F8[14] = 1;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  10.0;
    }
  }
  else {
    B3[337] =  0.0;
  }

  if (N7[10] ==  10.0 && N7[11] == N7[10]) {                    //--- pas 10 detection 1 ou 2 batteries
    B3[338] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      if (buttonPin45Num >= 4) {                         // pas sur que I2/15 = pin 45 à vérifier et changer dans cette partie de prog (mw 13/3/2016)
        N7[10] =  30.0;
      }
      else {
        N7[10] =  20.0;
      }
    }
  }
  else {
    B3[338] =  0.0;
  }

  if (N7[10] ==  20.0 && N7[11] == N7[10]) { //--- pas 20 1 batterie seule
    B3[305] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  40.0;
    }
  }
  else {
  }

  if (N7[10] ==  30.0 && N7[11] == N7[10]) {//--- pas 30 offset en cours
    B3[306] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  60.0;
    }
  }
  else {
  }

  if (N7[10] ==  40.0 && N7[11] == N7[10]) { //--- pas 40
    B3[340] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      if (buttonPin37Num >= 4) {
        N7[10] =  50.0;
      }
      else {
        N7[10] =  100.0;
      }
    }
  }
  else {
    B3[340] =  0.0;
  }


  if (N7[10] ==  50.0 && N7[11] == N7[10]) { //--- pas 50 attente coupure disj
    B3[341] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin37Num < 4) {
      N7[10] =  100.0;
    }
    if (TDN[72]) {
      B3[401] =  1.0;
    }
    else {
      B3[401] =  0.0;
    }
  }
  else {
    B3[341] =  0.0;
    B3[401] =  0.0;
  }

  if (N7[10] ==  60.0 && N7[11] == N7[10]) {//--- pas 60 2 batt
    B3[343] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      if (buttonPin37Num >= 4) {
        N7[10] =  70.0;
      }
      else {
        N7[10] =  100.0;
      }
    }
  }
  else {
    B3[343] =  0.0;
  }


  if (N7[10] ==  70.0 && N7[11] == N7[10]) {//--- pas 70 attente disj
    B3[344] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin37Num < 4) {
      N7[10] =  100.0;
    }
    if (TDN[72]) {
      B3[402] =  1.0;
    }
    else {
      B3[402] =  0.0;
    }
  }
  else {
    B3[344] =  0.0;
    B3[402] =  0.0;
  }

  if (N7[10] ==  100.0 && N7[11] == N7[10]) {//--- pas 100 attent chutte UDC pour offset Hall
    B3[345] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[72] && B3[32] ) {
      N7[10] =  110.0;
    }
    if (TDN[70]) {
      B3[403] =  1.0;
    }
    else {
      B3[403] =  0.0;
    }
  }
  else {
    B3[345] =  0.0;
    B3[403] =  0.0;
  }



  if (N7[10] ==  110.0 && buttonPin45Num < 4 && N7[11] == N7[10]) { //--- pas 110 attent couper disj batt
    B3[346] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin37Num >= 4) {
      N7[10] =  120.0;
    }
    if (TDN[72]) {
      B3[404] =  1.0;
    }
    else {
      B3[404] =  0.0;
    }
  }
  else {
    B3[346] =  0.0;
    B3[404] =  0.0;
  }

  if (N7[10] ==  110.0 && buttonPin45Num >= 4 && N7[11] == N7[10]) { //--- pas 110
    B3[347] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin37Num >= 4 ) {
      N7[10] =  120.0;
    }
    if (TDN[72]) {
      B3[405] =  1.0;
    }
    else {
      B3[405] =  0.0;
    }
  }
  else {
    B3[347] =  0.0;
    B3[405] =  0.0;
  }


  if (N7[10] ==  120.0 && N7[11] == N7[10]) {  //--- pas 120
    B3[357] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  130.0;
    }
  }
  else {
    B3[357] =  0.0;
  }


  if (N7[10] ==  130.0 && N7[11] == N7[10] && buttonPin45Num < 4 && N7[11] == N7[10]) { //--- pas 130
    N7[10] =  200.0;
  }

  if (N7[10] ==  130.0 && N7[11] == N7[10]) {
    B3[358] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  200.0;
    }
  }
  else {
    B3[358] =  0.0;
  }


  if (N7[10] ==  200.0 && !B3[449] && N7[11] == N7[10]) {//--- vers 210 hall bat 1 ok
    N7[10] =  210.0;
  }
  if (N7[10] ==  200.0 && N7[11] == N7[10]) {
    B3[492] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  210.0;
    }
  }
  else {
  }
  if (N7[10] ==  210.0 && !B3[450] && N7[11] == N7[10]) {
    N7[10] =  230.0;
  }

  if (N7[10] ==  210.0 && N7[11] == N7[10]) {
    B3[493] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  230.0;
    }
  }
  else {
  }

  if (N7[10] ==  230.0 && !B3[452] && N7[11] == N7[10] ) {
    N7[10] =  240.0;
  }
  if (N7[10] ==  230.0 && N7[11] == N7[10]) {
    B3[495] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  240.0;
    }
  }
  else {
  }


  if (N7[10] ==  240.0 && !B3[453] && N7[11] == N7[10] ) {
    N7[10] =  250.0;
  }
  if (N7[10] ==  240.0 && N7[11] == N7[10]) {
    B3[496] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  250.0;
    }
  }
  else {
  }


  if (N7[10] ==  250.0 && !B3[454] && N7[11] == N7[10]) {
    N7[10] =  260.0;
  }
  if (N7[10] ==  250.0 && N7[11] == N7[10]) {
    B3[481] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  260.0;
    }
  }
  else {
  }


  if (N7[10] ==  260.0 && !B3[455] && N7[11] == N7[10] ) {
    N7[10] =  270.0;
  }
  if (N7[10] ==  260.0 && N7[11] == N7[10]) {
    B3[482] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  270.0;
    }
  }
  else {
  }


  if (N7[10] ==  270.0 && !B3[456] && N7[11] == N7[10]) {
    N7[10] =  280.0;
  }
  if (N7[10] ==  270.0 && N7[11] == N7[10]) {
    B3[483] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  280.0;
    }
  }
  else {
  }


  if (N7[10] ==  280.0 && !B3[457] && N7[11] == N7[10] ) {
    N7[10] =  290.0;
  }
  if (N7[10] ==  280.0 && N7[11] == N7[10]) {
    B3[484] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  290.0;
    }
  }
  else {
  }


  if (N7[10] ==  290.0 && N7[11] == N7[10]) {
    B3[359] =  1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  300.0;
    }
  }
  else {
    B3[359] =  0.0;  //-------------B3:22/6
  }


  if (N7[10] ==  300.0 && N7[11] == N7[10]) {
    B3[387] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin37Num >= 4) {
      N7[10] =  310.0;
    }
    if (TDN[72]) {
      B3[460] =  1.0;
    }
    else {
    }
  }
  else {
    B3[387] =  0.0; //-------------B3:24/3
  }

  if (N7[10] ==  310.0 && N7[11] == N7[10]) {
    B3[388] =  1.0;

    if (N7[11] == N7[10] && TDN[70] && buttonPin35Num >= 4) {
      N7[10] =  320.0;
    }
    if (TDN[72]) {
      B3[461] =  1.0;
    }
    else {
    }
  }
  else {
    B3[388] =  0.0; //-------------B3:24/3
  }

 if (N7[10] == 320.0 && N7[11] == N7[10] && buttonPin45Num >= 4 ) {
    N7[10] =  330.0;
  }
  
  
  if (N7[10] == 320.0 && N7[11] == N7[10] && buttonPin45Num < 4 ) {
    N7[10] =  340.0;
  }

  if (N7[10] ==  330.0 && N7[11] == N7[10]) {
    B3[389] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin43Num >= 4 ) {
      N7[10] =  340.0;
    }
    if (TDN[72]) {
      B3[462] =  1.0;
    }
    else {
    }
  }
  else {
    B3[389] =  0.0; //-------------B3:24/4
  }


  if (N7[10] ==  340.0 && N7[11] == N7[10]) {
    B3[360] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin31Num < 4 && buttonPin29Num < 4 ) {
      N7[10] =  350.0;
    }
    if (TDN[72]) {
      B3[406] =  1.0;
    }
    else {
      B3[406] =  0.0;
    }
  }
  else {
    B3[360] =  0.0; //-------------B3:22/7
    B3[406] =  0.0;
  }


  if (N7[10] ==  350.0 && N7[11] == N7[10]) {
    B3[361] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin31Num < 4 ) {
      N7[10] =  360.0;
    }
  }
  else {
    B3[361] =  0.0; //-------------B3:22/8
  }


  if (N7[10] ==  360.0 && N7[11] == N7[10]) {
    B3[362] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin31Num < 4 && buttonPin27Num < 4) {
      N7[10] =  365.0;
    }
    if (TDN[72]) {
      B3[467] =  1.0;
    }
    else {
    }
  }
  else {
    B3[362] =  0.0; //-------------B3:22/9
  }

  if (N7[10] == 365.0 && N7[11] == N7[10] && TDN[70] ) {
    N7[10] =  370.0;
  }

  if (N7[10] ==  370.0 && N7[11] == N7[10]) {
    B3[363] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin31Num >= 4) {
      N7[10] =  372.0;
    }
    if (TDN[72]) {
      B3[407] =  1.0;
    }
    else {
      B3[407] =  0.0;
    }
  }
  else {
    B3[363] =  0.0; //-------------B3:22/10
    B3[407] =  0.0;
  }


  if (N7[10] ==  372.0 && N7[11] == N7[10]) {
    B3[364] =  1.0;
    if  (TDN[70]) {
      N7[10] =  373.0;
    }
  }
  else {
    B3[364] =  0.0;
  }





  if (N7[10] == 373.0 && N7[11] == N7[10] && TDN[70] ) {
    N7[10] =  375.0;
  }

  if (N7[10] == 375.0 && N7[11] == N7[10] ) {
    if (((buttonPin27Num >= 4 && buttonPin25Num >= 4) || B3[232]))
    {
      B3[232] =  1.0;
    }
    else
    {
      B3[232] =  0.0;
    }
  }
  else
  {
    B3[232] =  0.0;
  }

  if (N7[10] ==  375.0 && N7[11] == N7[10] && B3[232] && TDN[70]) {
    N7[10] =  377.0;
  }

  if (N7[10] == 377.0 && N7[11] == N7[10] && TDN[70]  ) {
    N7[10] =  380.0;
  }

  if (N7[10] == 380.0 && N7[11] == N7[10] && buttonPin29Num >= 4 ) {
    B3[396] =  1.0;
    if ( TDN[70]) {
      N7[10] =  385.0;
    }
  }

  if (N7[10] ==  380.0 && N7[11] == N7[10] && buttonPin29Num < 4) {
    B3[397] =  1.0;
    if ( TDN[70] ) {
      N7[10] =  385.0;
    }
  }
  else {
    B3[397] =  0.0; //-------------B3:24/12
  }

  if (N7[10] == 385.0 && N7[11] == N7[10] ) {
    N7[10] =  390.0;
  }

  if (N7[10] == 390.0 && N7[11] == N7[10] && (buttonPin27Num < 4 || B3[227]) ) {
    B3[227] =  1;
  }
  else
  {
    B3[227] =  0;
  }

  if (N7[10] ==  390.0 && N7[11] == N7[10]) {
    B3[365] =  1.0;
    if (TDN[70] &&  B3[227]) {
      N7[10] =  400.0;
    }
    if (TDN[72]) {
      B3[463] =  1.0;
    }
    else {
    }
  }
  else {
    B3[365] =  0.0; //-------------B3:22/12
  }

  if (N7[10] == 400.0 && N7[11] == N7[10] && (buttonPin27Num >= 4 || B3[228]) ) {
    B3[228] =  1;
  }
  else
  {
    B3[228] =  0;
  }

  if (N7[10] ==  400.0 && N7[11] == N7[10]) {
    B3[366] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && B3[228]) {
      N7[10] =  410.0;
    }
    if (TDN[72]) {
      B3[464] =  1.0;
    }
    else {
    }
  }
  else {
    B3[366] =  0.0; //-------------B3:22/13
  }

  if (N7[10] == 410.0 && N7[11] == N7[10] && (buttonPin29Num < 4 || B3[229]) ) {
    B3[229] =  1;
  }
  else
  {
    B3[229] =  0;
  }

  if (N7[10] ==  410.0 && N7[11] == N7[10]) {
    B3[367] =  1.0;
    if (TDN[70] && B3[229]) {
      N7[10] =  420.0;
    }
    if (TDN[72]) {
      B3[465] =  1.0;
    }
    else {
    }
  }
  else {
    B3[367] =  0.0; //-------------B3:22/14
  }


  if (N7[10] == 420.0 && N7[11] == N7[10] && (buttonPin25Num < 4 || B3[230]) ) {
    B3[230] =  1;
  }
  else
  {
    B3[230] =  0;
  }

  if (N7[10] ==  420.0 && N7[11] == N7[10]) {
    B3[368] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && B3[230]) {
      N7[10] =  430.0;
    }
    if (TDN[72]) {
      B3[466] =  1.0;
    }
    else {
    }
  }
  else {
    B3[368] =  0.0; //-------------B3:22/15
  }

  if (N7[10] ==  430.0 && N7[11] == N7[10] && (buttonPin25Num >= 4 || B3[239])) {
    B3[239] = 1;
  }
  else {
    B3[239] = 0;
  }

  if (N7[10] == 430.0 && B3[239] && TDN[70]) {
    N7[10] =  500.0;
  }

  if (N7[10] ==  500.0 && N7[11] == N7[10]) {//--- pas 500 courant maxi onduleur 1
    B3[369] =  1.0;
    if (TDN[40] && TDN[70]) {
      N7[10] =  508.0;
    }
    if (TDN[72]) {
      B3[370] =  1.0;
    }
    else {
      B3[370] =  0.0; //-------------B3:23/1
    }
  }

  else {
    B3[369] =  0.0; //-------------B3:23/0
    B3[370] =  0.0; //-------------B3:23/1
  }




  if (N7[10] == 500.0 && N7[11] == N7[10] && B3[468] ) {
    N7[10] =  501.0;
  }
  if (N7[10] == 500.0 && N7[11] == N7[10] && TDN[70] && TDN[50] ) {
    N7[10] =  505.0;
  }
  if (N7[10] == 501.0 && N7[11] == N7[10] && TDN[70] ) {
    N7[10] =  0.0;
  }


  //  if (N7[10] ==  505.0 && N7[11] == N7[10]) {
  //    B3[391] =  1.0;
  //    if (N7[11] == N7[10] && TDN[70] ) {
  //      N7[10] =  540.0;
  //    }
  //  }
  //  else {
  //    B3[391] =  0.0; //-------------B3:24/6
  //  }

  if (N7[10] ==  505.0 && N7[11] == N7[10]) {
    B3[391] =  1.0;
    if (N7[11] == N7[10] && TDN[70] ) {
      N7[10] =  530.0;
    }
  }
  else {
    B3[391] =  0.0; //-------------B3:24/6
  }

  if (N7[10] ==  508.0 && N7[11] == N7[10]) {
    B3[393] =  1.0;
    if (N7[11] == N7[10] && TDN[70] ) {
      N7[10] =  510.0;
    }
  }
  else { //-------------B3:24/8
  }


  if (N7[10] ==  510.0 && N7[11] == N7[10]) {//--- pas 510 mesure hysteresis hall onduleur 1
    B3[371] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[41] ) {
      N7[10] =  520.0;
    }
  }
  else {
    B3[371] =  0.0; //-------------B3:23/2
  }

  if (N7[10] ==  520.0 && N7[11] == N7[10]) {   //--- pas 520 mesure histeresis en decharge batterie avec onduleur 1
    B3[372] =  1.0;
    if (N7[11] == N7[10] && TDN[70] ) {
      N7[10] =  530.0;
    }
  }
  else {
    B3[372] =  0.0; //-------------B3:23/3
  }

  if (N7[10] ==  530.0 &&  N7[11] == N7[10] && (buttonPin25Num >= 4 || B3[231])) {
    B3[231] = 1;
  }
  else {
    B3[231] = 0;
  }


  if (N7[10] ==  530.0 && B3[231] && TDN[70]) {  //--- pas 520 mesure histeresis en decharge batterie avec onduleur 1
    N7[10] =  540.0;
  }

  if (N7[10] ==  540.0 && N7[11] == N7[10]) {//--- pas 500 courant maxi onduleur 1
    B3[373] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[42]) {
      N7[10] =  548.0;
    }
    if (TDN[72]) {
      B3[374] =  1.0;
    }
    else {
      B3[374] =  0.0; //-------------B3:23/5
    }
  }
  else {
    B3[373] =  0.0; //-------------B3:23/4
    B3[374] =  0.0; //-------------B3:23/5
  }

  if (N7[10] == 540.0 && N7[11] == N7[10] && B3[469] ) {
    N7[10] =  541.0;
  }

  if (N7[10] == 540.0 && N7[11] == N7[10] && TDN[70] && TDN[51] ) {
    N7[10] =  545.0;
  }

  if (N7[10] == 541.0 && N7[11] == N7[10] && TDN[70] ) {
    N7[10] =  0.0;
  }

  if (N7[10] ==  545.0 && N7[11] == N7[10]) {
    B3[392] =  1.0;
    if (N7[11] == N7[10] && TDN[70] ) {
      N7[10] =  570.0;
    }
  }
  else {
    B3[392] =  0.0; //-------------B3:24/7
  }


  if (N7[10] ==  548.0 && N7[11] == N7[10]) {
    B3[394] =  1.0;
    if (N7[11] == N7[10] && TDN[70] ) {
      N7[10] =  550.0;
    }
  }
  else { //-------------B3:24/9
  }

  if (N7[10] ==  550.0 && N7[11] == N7[10]) {//--- pas 550 mesure hysteresis hall onduleur 2
    B3[375] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[43] ) {
      N7[10] =  560.0;
    }
  }
  else {
    B3[375] =  0.0; //-------------B3:23/6
  }

  if (N7[10] ==  560.0 && N7[11] == N7[10]) {   //--- pas 560 mesure histeresis en decharge batterie avec onduleur 2
    B3[376] =  1.0;
    if (N7[11] == N7[10] && TDN[70] ) {
      N7[10] =  570.0;
    }
  }
  else {
    B3[376] =  0.0; //-------------B3:23/7
  }

  if (N7[10] ==  570.0 && N7[11] == N7[10]) {//--- pas 570 conrol circuit bcu non actif bat 1
    B3[385] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && (buttonPin50Num < 4 || buttonPin50Num < 4)) {
      N7[10] =  580.0;
    }
    if (TDN[72]) {
      B3[458] =  1.0; //-------------B3:28/9
    }
    else {
    }
  }
  else {
    B3[385] =  0.0; //-------------B3:24/0
  }

    if (N7[11] == N7[10] && N7[10] == 580.0 && buttonPin45Num >= 4) {
    N7[10] =  590.0;
  }
  
  if (N7[11] == N7[10] && N7[10] == 580.0 && buttonPin45Num < 4) {
    N7[10] =  595.0;
  }

  if (N7[10] ==  590.0 && N7[11] == N7[10]) {//--- pas 570 conrol circuit bcu non actif bat 1
    B3[386] =  1.0;
    if (TDN[70] && (buttonPin48Num < 4 || buttonPin46Num < 4)) {
      N7[10] =  595.0;
    }
    if (TDN[72]) {
      B3[459] =  1.0; //-------------B3:28/9
    }
    else {
    }
  }
  else {
    B3[386] =  0.0; //-------------B3:24/0
  }

  if (N7[10] ==  595.0 && N7[11] == N7[10] && (buttonPin25Num >= 4 || B3[238])) {
    B3[238] = 1;
  }
  else {
    B3[238] = 0;
  }

  if (N7[10] == 595.0 && B3[238] && TDN[70] ) { //-----pas 580---- si pas bat 2
    N7[10] =  600.0;
  }


  if (N7[10] ==  600.0 && N7[11] == N7[10]) {//--- pas 600 message chargeur 1
    B3[377] =  1.0;
    if (TDN[70] && (TDN[44] || B3[234]) ) {
      N7[10] =  619.0;
    }
  }
  else {
    B3[377] =  0.0; //-------------B3:23/8
  }

  if (N7[10] == 600.0 && N7[11] == N7[10] && B3[470] ) { //-----pas 600---- ANOMALIE COURANT NEG CHARG 1
    N7[10] =  601.0;
  }
  if (N7[10] == 601.0 && N7[11] == N7[10] && TDN[70] ) { //-----pas 601---- MESSAGE ALARME COURANT NEG CHARG 1
    N7[10] =  0.0;
  }
  if (N7[10] == 619.0 && N7[11] == N7[10] && TDN[70] ) { //-----pas 619---- MESSAGE CHARG 1 DETECTE
    N7[10] =  620.0;
  }
  if (N7[10] ==  620.0 && N7[11] == N7[10]) {//--- pas 620 messURE HYSTERESIS HALL chargeur 1
    B3[378] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[45] ) {
      N7[10] =  630.0;
    }
  }
  else {
    B3[378] =  0.0; //-------------B3:23/9
  }


  if (N7[10] ==  630.0 && N7[11] == N7[10]) {//--- pas 630 messURE HYSTERESIS EN CHARGE BATT chargeur 1
    B3[379] =  1.0;
    if (TDN[70]) {
      N7[10] =  640.0;
    }
  }
  else {
    B3[379] =  0.0; //-------------B3:23/10
  }


  if (N7[10] == 640.0 && N7[11] == N7[10] && ( buttonPin25Num >= 4 || B3[237] )) {
    B3[237] =  1.0;
  }
  else {
    B3[237] =  0.0;                                           //-------------B3:23/14
  }


  if (N7[10] ==  640.0 && B3[237] && TDN[70]) {
    N7[10] =  650.0;
  }

  if (N7[10] ==  650.0 && N7[11] == N7[10]) {//--- pas 600 message chargeur 2
    B3[380] =  1.0;
    if (TDN[70] && (TDN[46] || B3[235]) ) {
      N7[10] =  659.0;
    }
  }
  else {
    B3[380] =  0.0; //-------------B3:23/11
  }
  if (N7[10] == 650.0 && N7[11] == N7[10] && B3[471] ) { //-----pas 650---- ANOMALIE COURANT NEG CHARG 2
    N7[10] =  651.0;
  }

  if (N7[10] == 651.0 && N7[11] == N7[10] && TDN[70] ) { //-----pas 651---- MESSAGE ALARME COURANT NEG CHARG 2
    N7[10] =  0.0;
  }

  if (N7[10] == 659.0 && N7[11] == N7[10] && TDN[70] ) { //-----pas 659---- MESSAGE CHARG 2 DETECTE
    N7[10] =  660.0;
  }

  if (N7[10] ==  660.0 && N7[11] == N7[10]) {                        //--- pas 660 messURE HYSTERESIS HALL chargeur 2
    B3[381] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[47] ) {
      N7[10] =  670.0;
    }
  }
  else {
    B3[381] =  0.0;                         //-------------B3:23/12
  }

  if (N7[10] ==  670.0 && N7[11] == N7[10]) {                        //--- pas 660 messURE HYSTERESIS HALL chargeur 2
    B3[382] =  1.0;
    if (TDN[70]) {
      N7[10] =  690.0;
    }
  }
  else {
    B3[382] =  0.0;                         //-------------B3:23/12
  }

  if ((buttonPin25Num >= 4 || B3[233]) && N7[10] ==  690.0 && N7[11] == N7[10]) {                      //--- pas 660 messURE HYSTERESIS HALL chargeur 2
    B3[233] =  1.0;
  }
  else {
    B3[233] =  0.0;
  }

  if (N7[10] ==  690.0 && B3[233] && TDN[70]) {                       //--- pas 670 messURE HYSTERESIS EN CHARGE BATT chargeur 2
    N7[10] =  700.0;
  }

  if (N7[10] ==  700.0 && N7[11] == N7[10]) {                         //--- pas 700 messURE PV 1
    B3[383] =  1.0;
    if (TDN[70] && TDN[48]  ) {
      N7[10] =  710.0;
    }
    if (TDN[70]  ) {
      B3[67] =  1.0;
    }
  }
  else {
    B3[383] =  0.0; 
    B3[67] =  0.0;    //-------------B3:23/14
  }

  if (N7[10] == 700.0 && N7[11] == N7[10] && B3[472] ) { //-----pas 700---- ANOMALIE COURANT NEG PV1
    N7[10] =  0.0;
  }

  if (N7[10] == 710.0 && N7[11] == N7[10] && ( buttonPin25Num >= 4 || B3[236] )) {
    B3[236] =  1.0;
  }
  else {
    B3[236] =  0.0;                                           //-------------B3:23/14
  }

  if (N7[10] ==  710.0 && B3[236] && TDN[70]) {                       //--- pas 670 messURE HYSTERESIS EN CHARGE BATT chargeur 2
    N7[10] =  720.0;
  }

  if (N7[10] ==  720.0 && N7[11] == N7[10]) {                             //--- pas 720 messURE PV 2
    B3[384] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && TDN[49] ) {
      N7[10] =  790.0;
    }
        if (TDN[70] ) {
      B3[68] =  1.0;
    }
  }
  else {
    B3[384] =  0.0; 
    B3[68] =  0.0;    //-------------B3:23/14
  }


  if (N7[10] == 720.0 && N7[11] == N7[10] && B3[473] ) { //-----pas 720---- ANOMALIE COURANT NEG PV2
    N7[10] =  0.0;
  }

  if (N7[10] ==  790.0 && B3[416] && N7[11] == N7[10]) {                             //--- pas 790 FIN CYCLE AUTOCONF
    B3[390] =  1.0;
    B3[1] = 1.0;
    if (N7[11] == N7[10] && TDN[70]) {
      N7[10] =  0.0;
    }
  }
  else {
    B3[390] =  0.0;
  }

  if (N7[10] ==  790.0 && !B3[416] && N7[11] == N7[10]) {                            //--- pas 790 FIN CYCLE AUTOCONF AVEC ALARM
    B3[408] =  1.0;
    if (N7[11] == N7[10] && TDN[70] && buttonPin31Num < 4) {
      N7[10] =  0.0;
    }
  }
  else {
    B3[408] =  0.0;                                    //-------------B3:25/7
  }


  if (N7[10] != 0.0  ) {                       //------AUTOCONFIG EN COURS
    B3[304] =  1.0;               //------------------------B3:18/15
  }
  else {
    B3[304] =  0.0;               //------------------------B3:18/15
  }

  if  (N7[10] == 10) {
    F8[53] = F8[34];
    B3[305] =  0.0;     //-------- B3:19/0
    B3[306] =  0.0;
    B3[307] =  0.0;
    B3[308] =  0.0;
    B3[309] =  0.0;
    B3[310] =  0.0;
    B3[311] =  0.0;
    B3[312] =  0.0;
    B3[313] =  0.0;
    B3[314] =  0.0;
    B3[315] =  0.0;
    B3[316] =  0.0;
    B3[317] =  0.0;
    B3[318] =  0.0;
    B3[319] =  0.0;
    B3[320] =  0.0;

    B3[385] =  0.0;     //  b3:24
    B3[386] =  0.0;
    B3[387] =  0.0;
    B3[388] =  0.0;
    B3[389] =  0.0;
    B3[390] =  0.0;
    B3[391] =  0.0;
    B3[392] =  0.0;
    B3[393] =  0.0;
    B3[394] =  0.0;
    B3[395] =  0.0;
    B3[396] =  0.0;
    B3[397] =  0.0;
    B3[398] =  0.0;
    B3[399] =  0.0;
    B3[400] =  0.0;

    B3[449] =  0.0;     //-------- B3:28/0
    B3[450] =  0.0;
    B3[451] =  0.0;
    B3[452] =  0.0;
    B3[453] =  0.0;
    B3[454] =  0.0;
    B3[455] =  0.0;
    B3[456] =  0.0;
    B3[457] =  0.0;
    B3[458] =  0.0;
    B3[459] =  0.0;
    B3[460] =  0.0;
    B3[461] =  0.0;
    B3[462] =  0.0;
    B3[463] =  0.0;
    B3[464] =  0.0;

    B3[465] =  0.0;     //-------- B3:29/0
    B3[466] =  0.0;
    B3[467] =  0.0;
    B3[468] =  0.0;
    B3[469] =  0.0;
    B3[470] =  0.0;
    B3[471] =  0.0;
    B3[472] =  0.0;
    B3[473] =  0.0;
    B3[474] =  0.0;
    B3[475] =  0.0;
    B3[476] =  0.0;
    B3[477] =  0.0;
    B3[478] =  0.0;
    B3[479] =  0.0;
    B3[480] =  0.0;

    B3[481] =  0.0;
    B3[482] =  0.0;
    B3[483] =  0.0;
    B3[484] =  0.0;
    B3[485] =  0.0;
    B3[486] =  0.0;
    B3[487] =  0.0;
    B3[488] =  0.0;
    B3[489] =  0.0;
    B3[490] =  0.0;
    B3[491] =  0.0;
    B3[492] =  0.0;
    B3[493] =  0.0;
    B3[494] =  0.0;
    B3[495] =  0.0;
    B3[496] =  0.0;

    F10[25] =  0.0;
    F10[26] =  0.0;

    N7[19] =  N7[17];
    N7[18] =  N7[17];

    N7[8] =  0;
    N7[9] =  0;

    F10[15] =  0.0;
    F10[16] =  0.0;

    F10[20] =  0.0;
    F10[21] =  0.0;
    F10[0] =  0.0;
    F10[1] =  0.0;
    F10[2] =  0.0;
    F10[3] =  0.0;
    F10[5] =  0.0;
    F10[6] =  0.0;
    F10[7] =  0.0;
    F10[8] =  0.0;

    F8[225] =  0.0;
    F8[226] =  0.0;
    F8[227] =  0.0;
    F8[228] =  0.0;
    F8[229] =  0.0;
    F8[230] =  0.0;
    F8[231] =  0.0;
    F8[232] =  0.0;

  }



  if (N7[10] == 372 && !TDN[70] ) {//    ------------------  MESURE CHUTE U PAR RELAIS 30A
    F10[85] = F8[69] + F10[85];
    F10[86] = F10[86] + 1;
    F10[87] = F10[85] / F10[86];
  }

  if (N7[10] == 10 ) {
    F10[85] = 0.0;
    F10[86] = 0.0;
  }

  if (N7[10] == 377 && TDN[73] ) {
    F10[90] = F10[90] + F8[69];
    F10[91] = F10[91] + 1;
    F10[92] = F10[90] / F10[91];
  }

  if (N7[10] == 10 ) {
    F10[90] = 0.0;
    F10[91] = 0.0;
  }

  if (N7[10] == 385 ) {
    F8[36] = F10[87] - F10[92];
  }

  if (N7[10] == 385 && B3[396] ) {
    F8[37] = F8[36] / 3.0;
  }
  if (N7[10] == 385 && !B3[396] ) {
    F8[37] = F8[36] / 2.0;
  }

  if (N7[10] == 385 ) {                    //    ------------------  MESURE AVEC RELAIS GRID - RELAIS GROUP
    if (F8[37] < 0.01 || F8[37] > 0.04  ) {
      B3[476] =   1.0;                                    //------B3:29/11
    }
  }

  if (N7[10] == 385 && (F8[37] >= 0.01 && F8[37] <= 0.04 ) ) {
    F8[70] =   F8[37];
  }

  if (N7[10] !=  790.0 && ((N7[10] >= 375 && N7[10] <= 380) || N7[10] >= 400))   {
    B3[266] =   1.0; //------B3:16/9
  }
  else {
    B3[266] =   0.0;                                    //------B3:16/9    DMDE RELAIS GRID
  }

  if (N7[10] >= 375 && N7[10] <= 380) {
    B3[267] =   1.0; //------B3:16/10
    B3[268] =   1.0;
  }
  else {
    B3[267] =   0.0;
    B3[268] =   0.0; //------B3:16/10    DMDE RELAIS GROUP
  }

  if (N7[10] >= 375 && N7[10] <= 380)   {
    CptT[73] = true;
  }
  else {
    T[73] =  0.0;
    TDN[73] =  0.0;
    CptT[73] = false;
  }

  if (T[73] >= 20) {
    TDN[73] =  1.0;
    T[73] = 20;
  }


  if ((N7[10] == 430 || N7[10] == 530 || N7[10] == 595 || N7[10] == 640 || N7[10] == 690 || N7[10] == 710 )) {
    CptT[80] = true;
  }
  //if (N7[10] != 430 && N7[10] != 530 && N7[10] != 595 && N7[10] != 640 && N7[10] != 690 && N7[10] != 710)
  else {
    T[80] =  0.0;
    TDN[80] =  0.0;
    CptT[80] = false;
  }

  if (T[80] >= 30) {
    TDN[80] =  1.0;
    T[80] = 30;
  }



  if (N7[10] == 500) {
    CptT[40] = true;
  }
  else {
    T[40] =  0.0;
    TDN[40] =  0.0;
    CptT[8] = false;
  }

  if (T[40] >= 900) {
    TDN[40] =  1.0;
    T[40] = 900;

  }

  if (N7[10] == 500 ) {
    B3[249] =   1.0;                                        //------B3:15/8
  }
  else {
    B3[249] =   0.0;                                    //------B3:15/8    DMDE MARCHE ONDULEUR
  }

  if (N7[10] == 500 && F8[169] > N7[26]) {
    CptT[50] = true;
  }
  else {
    T[50] =  0.0;
    TDN[50] =  0.0;
    CptT[50] = false;
  }

  if (T[50] >= 400) {
    TDN[50] =  1.0;
    T[50] = 400;
  }

  if (N7[10] == 500 && F8[169] > N7[27]) {
    B3[468] =   1.0;                                 //------B3:29/3 ALARM RACC||DEMENT ONDUL 1
  }

  if (N7[10] == 500 && F8[169] < F8[225]) {
    F8[225] =  F8[169];                                   //CRT MAXI ONDUL 1
  }

  if (N7[10] == 500 && F8[225] < N7[26]) {
    B3[314] =   1.0;                                  //------B3:19/9 ONDUL 1 DETECTE
  }

  if (N7[10] == 510) {
    B3[248] =   1.0;
    CptT[41] = true;
  }
  else {
    B3[248] =   0.0;                                    //------B3:15/7 ATTENT ARRET ONDUL 1
    T[41] =  0.0;
    TDN[41] =  0.0;
    CptT[41] = false;
  }

  if (T[41] >= 80) {
    TDN[41] =  1.0;
    T[41] = 80;
  }

  if (N7[10] == 510 && TDN[41]) {
    F8[226] =  F8[169];                       //------CRT HYSTERESIS ONDUL 1
  }

  if (N7[10] == 520) {
    F10[20] =  F8[250] * F8[254];                         //------CRT HYSTERESIS ONDUL 1
  }

  if (N7[10] == 540) {
    B3[255] =   1.0;                              //------B3:15/14 DMD MARCHE ONDUL 2
    CptT[42] = true;
  }
  else {
    B3[255] =   0.0; //------B3:15/14 DMD MARCHE ONDUL 2
    T[42] =  0.0;
    TDN[42] =  0.0;
    CptT[42] = false;
  }

  if (T[42] >= 900) {
    TDN[42] =  1.0;
    T[42] = 900;
  }


  if (N7[10] == 540 && F8[159] > N7[26]) {
    CptT[51] = true;
  }
  else {
    T[51] =  0.0;
    TDN[51] =  0.0;
    CptT[51] = false;
  }

  if (T[51] >= 400) {
    TDN[51] =  1.0;
    T[51] = 400;
  }

  if (N7[10] == 540 && F8[159] > N7[27]) {
    B3[469] =   1.0;                                 //------B3:29/4 ALARM RACC||DEMENT ONDUL 2
  }

  if (N7[10] == 540 && F8[159] < F8[227]) {
    F8[227] =  F8[159];                                   //CRT MAXI ONDUL 2
  }

  if (N7[10] == 540 && F8[227] < N7[26]) {
    B3[315] =   1.0;                                  //------B3:19/10 ONDUL 2 DETECTE
  }

  if (N7[10] == 550) {
    B3[256] =   1.0;
    CptT[43] = true;
  }
  else {
    B3[256] =   0.0;                                    //------B3:15/15 ATTENT ARRET ONDUL 2
    T[43] =  0.0;
    TDN[43] =  0.0;
    CptT[43] = false;
  }

  if (T[43] >= 80) {
    TDN[43] =  1.0;
    T[43] = 80;
  }

  if (N7[10] == 550 && TDN[43]) {
    F8[228] =  F8[159];                                 //------CRT HYSTERESIS ONDUL 2
  }

  if (N7[10] == 560) {
    F10[21] =  F8[250] * F8[254];                        //------CRT HYSTERESIS ONDUL 2
  }

  if (N7[10] == 600) {
    CptT[44] = true;
  }
  else {
    T[44] =  0.0;
    TDN[44] =  0.0;
    CptT[44] = false;
  }

  if (T[44] >= 200) {
    TDN[44] =  1.0;
    T[44] = 200;
  }

  if (N7[10] == 600 ) {
    B3[258] =   1.0;                                //------B3:16/1
  }
  else {
    B3[258] =   0.0;                                    //------B3:16/1    DMDE MARCHE CHARGEUR 1
  }

  if (N7[10] == 600 && F8[210] < N7[26]) {
    B3[470] =   1.0;                                 //------B3:29/5 ALARM RACC||DEMENT CHARGEUR 1
  }

  if (N7[10] == 600 && F8[73] > F10[0]) {
    F10[0] =  F8[73];                                   //CRT MAXI CHARGEUR 1
  }

  if (N7[10] == 600 && F8[210] > F8[229]) {
    F8[229] =  F8[210];                                   //CRT MAXI CHARGEUR 1
  }

  if (N7[10] == 600 && F8[229] > N7[27]) {
    B3[311] =   1.0;                                  //------B3:19/6 CHARG 1 DETECTE
  }

  if (N7[10] == 619 && !B3[311] ) {
    B3[241] =   1.0;                                  //------B3:19/6 CHARG 1 DETECTE
  }
  else {
    B3[241] =   0.0;
  }

  if (N7[10] == 600 && F8[210] < N7[27] && B3[311] ) {
    B3[234] =   1.0;                                  //------B3:19/6 CHARG 1 DETECTE
  }
  else {
    B3[234] =   0.0;
  }


  if (B3[311]) {//-cond ons
    if (!B3[24]) {  //---bit ons
      B3[24] = 1;
      N7[19] = T[44] / 10;
    }
  }
  else {
    B3[24] = 0;//---bit ons
  }



  if (N7[10] == 620) {
    B3[259] =   1.0;
    CptT[45] = true;
  }
  else {
    B3[259] =   0.0;                                    //------B3:16/2 ATTENT ARRET CHARG 1
    T[45] =  0.0;
    TDN[45] =  0.0;
    CptT[45] = false;
  }

  if (T[45] >= 30) {
    TDN[45] =  1.0;
    T[45] = 30;
  }

  if (N7[10] == 620 && F8[210] < 0.2 ) {
    F10[1] =  F8[73];                                //------MEM uU BATT A CRT MINI CHARGEUR 1
  }

  if (N7[10] == 620) {
    F10[2] =  F10[0] - F10[1];                                //------DIFF U BATT ENTRE CHARG MAX ET 0 CHARG
  }

  if (N7[10] == 620 && F8[229] > 0.0 && B3[311]) {
    F10[3] =  F10[2] / F8[229];                                //------RESIST OHM CIRCUIT PUISS AVEC CHAGEUR 1
  }

  if (N7[10] == 620 && TDN[45]) {
    F8[230] =  F8[209];                                            //------CRT HYSTERESIS CHARG 1
  }

  if (N7[10] == 630) {
    F10[15] =  F8[254] * F8[250];                                //------MEMO HYST CHARG 1 BATT CHARG
  }

  if (N7[10] == 650) {
    B3[262] =   1.0;                                         //------B3:16/5                       DMD MARCHE CHARG 2
    CptT[46] = true;
  }
  else {
    B3[262] =   0.0; //------B3:16/5                               DMD MARCHE CHARGL 2
    T[46] =  0.0;
    TDN[46] =  0.0;
    CptT[46] = false;
  }

  if (T[46] >= 200) {
    TDN[46] =  1.0;
    T[46] = 200;
  }

  if (N7[10] == 650 && F8[199] < N7[26]) {
    B3[471] =   1.0;                                 //------B3:29/6 ALARM RACC||DEMENT CHARG 2
  }

  if (N7[10] == 650 && F8[73] > F10[5]) {
    F10[5] =  F8[73];                                   //CRT MAXI CHARG 2
  }

  if (N7[10] == 650 && F8[199] > F8[231]) {
    F8[231] =  F8[199];                                   //CRT MAXI CHARG 2
  }

  if (N7[10] == 650 && F8[231] > N7[27]) {
    B3[312] =   1.0;                                  //------B3:19/7 CHARG 2 DETECTE
  }

  if (N7[10] == 659 && !B3[312] ) {
    B3[242] =   1.0;                                  //------B3:19/6 CHARG 2 DETECTE
  }
  else {
    B3[242] =   0.0;
  }


  if (N7[10] == 650 && F8[199] < N7[27] && B3[312] ) {
    B3[235] =   1.0;                                  //------B3:19/6 CHARG 2 DETECTE
  }
  else {
    B3[235] =   0.0;
  }

  if (B3[312]) {//-cond ons
    if (!B3[25]) {  //---bit ons
      B3[25] = 1;
      N7[18] = T[44] / 10;
    }
  }
  else {
    B3[25] = 0;//---bit ons
  }


  if (N7[10] == 660) {
    B3[263] =   1.0;
    CptT[47] = true;
  }
  else {
    B3[263] =   0.0;                                    //------B3:16/6 ATTENT ARRET CHARG 2
    T[47] =  0.0;
    TDN[47] =  0.0;
    CptT[47] = false;
  }

  if (T[47] >= 30) {
    TDN[47] =  1.0;
    T[47] = 30;
  }

  if (N7[10] == 660 && F8[199] < 0.2 ) {
    F10[6] =  F8[73];                                //------MEM uU BATT A CRT MINI CHARGEUR 2
  }

  if (N7[10] == 660) {
    F10[7] =  F10[5] - F10[6];                                //------DIFF U BATT ENTRE CHARG MAX ET 0 CHARG
  }

  if (N7[10] == 660 && F8[231] > 0.0 && B3[312]) {
    F10[8] =  F10[7] / F8[231];                                //------RESIST OHM CIRCUIT PUISS AVEC CHAGEUR 2
  }

  if (N7[10] == 660 && TDN[47]) {
    F8[232] =  F8[199];  //------CRT HYSTERESIS ONDUL 2
  }

  if (N7[10] == 670) {
    F10[16] =  F8[250] * F8[254];  //------MEMO HYST CHARG 2 BATT CHARG
  }


  if ((N7[10] >= 600 && N7[10] <= 620) || ( N7[10] >= 650 && N7[10] <= 660)) {
    B3[5] =   1.0; // bit fast execution arduino
  }
  else {
    B3[5] =   0.0;
  }



  if (N7[10] == 700) {
    B3[269] =   1.0;
    CptT[48] = true;

  }
  else {
    B3[269] =   0.0;                                    //------B3:16/12 ATTENT march pv 1
    T[48] =  0.0;
    TDN[48] =  0.0;
    CptT[48] = false;
  }
  if (T[48] >= 100) {
    TDN[48] =  1.0;
    T[48] = 100;
  }

  if (N7[10] == 700 && F8[189] < N7[26] ) {
    B3[472] =  1.0;                                //b3;29/7------anomalie cablage pv1
  }

  if (N7[10] == 700 && F8[181] > N7[8] ) {
    N7[8] =  F8[181];                                //------mesure puiss pv1
  }

  if (N7[10] == 700 && N7[8] > N7[28] && B3[67] ) {
    B3[308] =  1.0;                                //B3;19/3------anomalie cablage pv1
  }


  if (N7[10] == 700 && !B3[308] && B3[67]) {
    B3[243] =  1.0;                                //b3;29/8------anomalie cablage pv2
  }
  else {
    B3[243] =  0.0;
  }

  if (N7[10] == 720) {
    B3[270] =   1.0;
    CptT[49] = true;
  }
  else {
    B3[270] =   0.0;                                    //------B3:16/13 ATTENT march pv 2
    T[49] =  0.0;
    TDN[49] =  0.0;
    CptT[49] = false;
  }
  if (T[49] >= 100) {
    TDN[49] =  1.0;
    T[49] = 100;
  }


  if (N7[10] == 720 && F8[179] < N7[26] ) {
    B3[473] =  1.0;                                //b3;29/8------anomalie cablage pv2
  }

  if (N7[10] == 720 && F8[171] > N7[9] ) {
    N7[9] =  F8[171];                                //------mesure puiss pv1
  }

  if (N7[10] == 720 && N7[9] > N7[28] && B3[68] ) {
    B3[309] =  1.0;                                //B3;19/4------anomalie cablage pv1
  }

  if (N7[10] == 720 && !B3[309] && B3[68] ) {
    B3[244] =  1.0;                                //b3;29/8------anomalie cablage pv2
  }
  else {
    B3[244] =  0.0;
  }

  if (N7[10] == 790.0) {        //---- optocoupleur BCU batt1 tous actif
    B3[11] = 1;
  }

  if (F8[145] > 556 || F8[145] < 456) {
    B3[449] =   1.0;                                     //------B3:28/0
  }
  else {
    B3[449] =   0.0;                                    //------B3:28/0    alarm offset hall bat 1
  }

  if (buttonPin45Num >= 4 && (F8[245] > 556 || F8[245] < 456)) {
    B3[450] =   1.0;                                     //------B3:28/1
  }
  else {
    B3[450] =   0.0;                                    //------B3:28/1    alarm offset hall bat 2
  }


  if (F8[165] > 556 || F8[165] < 456) {
    B3[452] =   1.0;                                     //------B3:28/3
  }
  else {
    B3[452] =   0.0;                                    //------B3:28/3    alarm offset hall ondul 1
  }

  if (F8[155] > 556 || F8[155] < 456) {
    B3[453] =   1.0;                                     //------B3:28/4
  }
  else {
    B3[453] =   0.0;                                    //------B3:28/4    alarm offset hall ondul 2
  }

  if (F8[185] > 556 || F8[185] < 456) {
    B3[454] =   1.0;                                     //------B3:28/5
  }
  else {
    B3[454] =   0.0;                                    //------B3:28/5    alarm offset hall pv 1
  }

  if (F8[175] > 556 || F8[175] < 456) {
    B3[455] =   1.0;                                     //------B3:28/6
  }
  else {
    B3[455] =   0.0;                                    //------B3:28/6    alarm offset hall pv 2
  }

  if (F8[205] > 556 || F8[205] < 456) {
    B3[456] =   1.0;                                     //------B3:28/7
  }
  else {
    B3[456] =   0.0;                                    //------B3:28/7    alarm offset hall charg 1
  }
  if (F8[195] > 556 || F8[195] < 456) {
    B3[457] =   1.0;                                     //------B3:28/8
  }
  else {
    B3[457] =   0.0;                                    //------B3:28/8    alarm offset hall charg 2
  }

  // FIN PRG AUTOCONFIG

  //-----------------------------------------------------------MOUCHARD PUISSANCE FIN DE CHARGE
  if (F8[60] > F8[95]) {
    F8_60_LIM = 1;
  }
  else {
    F8_60_LIM = 0;
  }
  B3[9] = B3[12] + F8_60_LIM + ST2;
  switch (B3[9]) {
    case 5:
      if (B3_9_MEMO == 0) {
        F8[114] = N7[59];
      }
      B3_9_MEMO = 1;
      break;
    default:
      B3_9_MEMO = 0;
      break;
  }

  if (B3[31] && T1Cent ) {
    T1Cent = 0;
    B3[31] = 0;
  }

  //--------------------------------------------------------21 EXTERNE-------------------------------------------------
  if (N7[80] != N7[81] && ( N7[81] == 0.0 ||  N7[80] == 0.0)) {
    N7[81] = N7[80];
  }
  if (N7[81] > 0) {
    CptT[35] = true;
  }
  else {
    T_35=0.0;
    TDN[35] =  0.0;
    CptT[35] = false;
  }

  if ( T_35 >= (N7[15]*10)) {
    TDN[35] =  1.0;
     T_35 = (N7[15]*10);
  }
    F10[95]=T_35;
    
  if (TDN[35] || N7[80] == 0) {
    N7[81] = 0;
        N7[80] = 0;
  }      
  if (B3[1]) {
    if(!B3[7])  {
        B3[7]=1;
        N7[81] = 0;
        N7[80] = 0;
        }
  }
  else {
    B3[7]=0;
  }
  
  if (N7[81] == 1) {
    B3[161] = 1;
  }
  else {
    B3[161] = 0;
  }

  if (B3[161] &&  N7[60] < N7[89]) {
    N7[80] = 0;
  }
  if (N7[81] == 2) {
    B3[162] = 1;
  }
  else {
  B3[162] = 0;
  }
  if (N7[81] == 3) {
    B3[163] = 1;
  }
    else {
  B3[163] = 0;
  }
  if (N7[81] == 4) {
    B3[164] = 1;
  }
    else {
  B3[164] = 0;
  }
  if (B3[164] && N7[60] < N7[89]) {
    N7[80] = 0;
  }
  if (N7[81] == 0) {
    B3[165] = 1;
  }
  else {
    B3[165] = 0;
  }
  //------------------------------------------------------03-PRG SEQUENCE----------------------------------

  if (buttonPin39Num >= 4 || buttonPin38Num >= 4) {
    B3[178] = 1;
  }
  else {
    B3[178] = 0;
  }


  if (buttonPin31Num >= 4) {
    CptT[10] = true;
  }
  else {
    T[10] =  0.0;
    TDN[10] =  0.0;
    CptT[10] = false;
  }

  if (T[10] >= 1.0) {
    TDN[10] =  1.0;
    T[10] = 1.0;
  }

    if ((B3[304]|| B3[139])&& TDN[10]){
    B3[139]=1;
  }
  else {
      B3[139]=0;
  }
  
      if (!TDN[10] && B3[1]){
    B3[253]=1;
  }
  else {
    B3[253]=0;
  }
  
  
  if (TDN[10] && B3[1] && !B3[139]){
    B3[254]=1;
  }
  else {
    B3[254]=0;
  }
  

  
  if (buttonPin29Num >= 4) {
    CptT[11] = true;
  }
  else {
    T[11] =  0.0;
    TDN[11] =  0.0;
    CptT[11] = false;
  }

  if (T[11] >= 10) {
    TDN[11] =  1.0;
    T[11] = 10;
  }

  
      if (!TDN[11] && B3[1] && B3[251]){
    B3[250]=1;
  }
  else {
    B3[250]=0;
  }
  
    if (TDN[11] && B3[1]){
    B3[251]=1;
  }
  else {
    B3[251]=0;
  }
  
  
  if (buttonPin27Num >= 4) {
    CptT[12] = true;
  }
  else {
    T[12] =  0.0;
    TDN[12] =  0.0;
    CptT[12] = false;
  }

  if (T[12] >= 1.0) {
    TDN[12] =  1.0;
    T[12] = 1.0;
  }


  if (buttonPin27Num >= 4) {
    T[13] =  0.0;
    TDN[13] =  1.0;
    CptT[13] = false;
  }
  else {
    CptT[13] = true;
  }

  if (T[13] >= 1.0) {
    TDN[13] =  0.0;
    T[13] = 1.0;
  }

  if (buttonPin25Num >= 4) {
    CptT[14] = true;
  }
  else {
    T[14] =  0.0;
    TDN[14] =  0.0;
    CptT[14] = false;
  }

  if (T[14] >= 1.0) {
    TDN[14] =  1.0;
    T[14] = 1.0;
  }

  if (buttonPin25Num >= 4) {
    T[15] =  0.0;
    TDN[15] =  1.0;
    CptT[15] = false;
  }
  else {
    CptT[15] = true;
  }

  if (T[15] >= 1.0) {
    TDN[15] =  0.0;
    T[15] = 1.0;
  }

  if (buttonPin23Num >= 4) {
    CptT[16] = true;
  }
  else {
    T[16] =  0.0;
    TDN[16] =  0.0;
    CptT[16] = false;
  }

  if (T[16] >= 1.0) {
    TDN[16] =  1.0;
    T[16] = 1.0;
  }
  
  
  if (N7[100] == 0 && TDN[10] && !B3[210] && !TDN[13] && !B3[177]  ) {
    N7[100] = 10;
  }
  if ((N7[100] == 10) && (!TDN[10] || B3[177]))  {
    N7[100] = 0;
  }
  if (N7[101] == 0 && TDN[11] && !B3[209] && !TDN[13] ) {
    N7[101] = 10;
  }
  if (N7[101] == 10 && ((TDN[10] && !B3[177]) || !TDN[11])) {
    N7[101] = 0;
  }



  if (N7[102] == 0 && TDN[12] && !B3[212] && !TDN[15] ) {
    N7[102] = 10;
  }



  if (N7[102] == 10 && !TDN[12] && !TDN[10] && !(TDN[11]) ) {
    N7[102] = 0;
  }
  if (N7[103] == 0 && TDN[16] && !B3[211] && !TDN[15] && !TDN[13] && !TDN[10] && !TDN[11]) {
    N7[103] = 10;
  }
  if (N7[103] == 10 && TDN[12] ) {
    N7[103] = 0;
  }
  if (buttonPin31Num >= 4 ) {
    N7[103] = 0;
  }
  if (buttonPin29Num >= 4 ) {
    N7[103] = 0;
  }
  if (buttonPin27Num >= 4 ) {
    N7[103] = 0;
  }
  if (!B3[209] && buttonPin31Num >= 4 && buttonPin27Num >= 4 && !B3[304] && (!B3[11] || buttonPin39Num < 4)) {
    B3[108] = 1;
  }
  else {
    B3[108] = 0;
  }
  if ((((!B3[304] && N7[100] == 10 && !B3[210]) || B3[266]) && TDN[0]) && !B3[108])  {
    B3[209] = 1;
  }
  else {
    B3[209] = 0;
  }
  if (B3[209] && !B3[178])  {
    PWM4 = 1;
    digitalWrite(LED4, HIGH);
  }
  else {
    PWM4 = 0;
    digitalWrite(LED4, LOW);
  }

  if (!B3[210] && buttonPin29Num >= 4 && buttonPin27Num >= 4 ) {
    B3[107] = 1;
  }
  else {
    B3[107] = 0;
  }




  if ((((!B3[304] && N7[101] == 10 && !B3[209]) || B3[267]) && TDN[0]) && !B3[107])  {
    B3[210] = 1;
  }
  else {
    B3[210] = 0;
  }

  if (B3[210] && !B3[178])  {
    PWM5 = 1;
    digitalWrite(LED5, HIGH);
  }
  else {
    PWM5 = 0;
    digitalWrite(LED5, LOW);
  }

  if (!B3[211] && buttonPin25Num >= 4 && !B3[304] && (!B3[11] || buttonPin39Num < 4)  ) {
    B3[106] = 1;
  }
  else {
    B3[106] = 0;
  }

  if (((!B3[304] && N7[102] == 10) || (B3[304] && TDN[80]) || (B3[304] && B3[268])) && !B3[212] && TDN[0] && !B3[106] ) {
    B3[211] = 1;
  }
  else {
    B3[211] = 0;
  }

  if (B3[211] && !B3[178])  {
    PWM6 = 1;
    digitalWrite(LED6, HIGH);
  }
  else {
    PWM6 = 0;
    digitalWrite(LED6, LOW);
  }

  if ( B3[209] && B3[211] ) {
    B3[11] = 0.0;
  }

  if (!B3[212] && buttonPin25Num >= 4 ) {
    B3[105] = 1;
  }
  else {
    B3[105] = 0;
  }
  if (buttonPin31Num < 4 && buttonPin27Num < 4 && buttonPin29Num < 4 && !B3[209] && !B3[210] && !B3[211] && B3[1] ) {
    B3[109] = 1;
  }
  else {
    B3[109] = 0;
  }
  if (!B3[304] && N7[103] == 10 && !B3[211] && B3[109] && TDN[0] && !B3[105] ) {
    B3[212] = 1;
  }
  else {
    B3[212] = 0;
  }
  if (B3[212] && !B3[178] ) {
    PWM7 = 1;
    digitalWrite(LED7, HIGH);
  }
  else {
    PWM7 = 0;
    digitalWrite(LED7, LOW);
  }
  if (B3[209]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[210]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[211]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[212]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[213]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[214]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[215]) {

    N7[30] = N7[30] + 1;
  }
  if (B3[216]) {

    N7[30] = N7[30] + 1;
  }
  N7[31] = N7[30];
  N7[30] = 0;

  //scanning
  if (CptT[70]) {
    F10[130] = F10[130] + 1;
  }
  if (TDN[70]) {
    F10[131] = F10[130] / 4;
    F10[130] = 0;
  }

  //Calcul du temps de cycle
  cycleTime = millis() - T1SetUp;
  Cycle = cycleTime - CyclePrescedent;
  CyclePrescedent = cycleTime;
  //Serial.println(Cycle);

  if (FirstScan == 1) {
    //..............
  }
  else {
  }
  
  if (AutorisationSMS) {
  CptT[85] = true;
  }
  else {
  T[85] =  0.0;
  TDN[85] =  0.0;
  CptT[85] = false;
  }
  if (T[85] >= 50) {
  TDN[85] =  1.0;
  T[85] = 50;
  }
  // Serial.println(buttonPin40Num);
  if(buttonPin40Num>=4)
  {
    GsmAutorisation(); //gsm blocage
    }
    FirstScan = 0;
 
  //FIN SEQUENCE
  //-----------------------------------------------
  MessTraitement();
  Ecriture ();// tablette vers arduino
  Lecture (); // arduino vers tablette

  //MegunoLink
  //Gemunolink ();
  if (B3[29]) {
    AffProcessing();
  }

}

//==========================================================
String SMS_Receive()
{
  String Mess = "";
  char c;
  //Serial.println("SMS_Receive");
  if (sms.available())
  {
    //  // Get remote number
    //   sms.remoteNumber(senderNumber, 20);
    //   if (sms.peek() == '#')
    //   {
    //     sms.flush();
    //   }
    while (c = sms.read()) {
      //Serial.print(c);
      Mess = Mess + c;
    }
    sms.flush();
  }
  return Mess;
}
//==============================================================================================================================Send message
//void SMS_Send()
//{
//  // send the message
//  sms.beginSMS("+32497478075");//remoteNum);
//  sms.print("hello message de l arduino");//txtMsg);
//  sms.endSMS();
//}

//==============================================================================================================================AffLcd
//void  AffLcd(float ValAff, int PosLigne, int Ligne, int RetraintCar)
//{
//  //lcd.clear();
//  dtostrf(ValAff, 4, 1, charVal);  //4 is mininum width, 3 is precision; float value is copied onto buff
//  f|| (int i = 0; i < sizeof(charVal) - RetraintCar; i++)
//  {
//    lcd.setCurs||(PosLigne + i, Ligne);
//    lcd.write(charVal[i]);
//  }
//}

//====================================================TEMPO===========================================================================Tempo1 OFFSET AI9  T5 secondes
void TonT1() {   //          T4:30

  switch (ST1) {
    case 1:
      if  (buttonPin37Num < 4 && buttonPin35Num >= 4 ) { // element declencheur du calcul d'offset de l'input AI9 AI13
        T1 = 50;
        ST1 = 2;
      }
      else {
        F8[141] = 0.0;// AI9n
        F8[140] = 0.0;
        F8[181] = 0.0;//AI13
        F8[180] = 0.0;
        F8[171] = 0.0;//AI12
        F8[170] = 0.0;
        F8[201] = 0.0;//AI15
        F8[200] = 0.0;
        F8[191] = 0.0;//AI14
        F8[190] = 0.0;
        F8[161] = 0.0;//AI11
        F8[160] = 0.0;
        F8[151] = 0.0;//AI10
        F8[150] = 0.0;
        CptOffset = 0;
      }
      break;
    case 2:       //TT
      if (T1 <= 0) {
        ST1 = 3;
      }
      else {
        F8[141] = F8[141] + 1;
        F8[140] = F8[140] + analogRead(analogPin9);//AI9

        F8[180] = F8[180] + analogRead(analogPin13);//AI13

        F8[170] = F8[170] + analogRead(analogPin12);//AI12

        F8[200] = F8[200] + analogRead(analogPin15);//AI15

        F8[190] = F8[190] + analogRead(analogPin14);//AI14

        F8[160] = F8[160] + analogRead(analogPin11);//AI11

        F8[150] = F8[150] + analogRead(analogPin10);//AI10

        F8[240] = F8[240] + analogRead(analogPin8);//AI8
      }
      if (buttonPin37Num >= 4 || buttonPin35Num < 4 )  {
        ST1 = 1;
        T1 = 0;
      }
      break;
    case 3:
      //Serial.println("ST1 FAIT");
      //DN
      F8[142] = F8[140] / F8[141];//AI9
      F8[140] = 0.0;
      F8[182] = F8[180] / F8[141];//AI13
      F8[180] = 0.0;
      F8[172] = F8[170] / F8[141];//AI12
      F8[170] = 0.0;
      F8[202] = F8[200] / F8[141];//AI15
      F8[200] = 0.0;
      F8[192] = F8[190] / F8[141];//AI14
      F8[190] = 0.0;
      F8[162] = F8[160] / F8[141];//AI11
      F8[160] = 0.0;
      F8[152] = F8[150] / F8[141];//AI10
      F8[150] = 0.0;
      F8[242] = F8[240] / F8[141];//AI10
      F8[240] = 0.0;
      F8[141] = 0.0;
      ST1 = 1;
      CptOffset = CptOffset + 1;
      if (CptOffset == 2) {
        F8[143] = F8[142];          //AI9
        F8[145] = F8[143] + F8[144];
        F8[183] = F8[182];          //AI13
        F8[185] = F8[183] + F8[184];
        F8[173] = F8[172];          //AI12
        F8[175] = F8[173] + F8[174];
        F8[203] = F8[202];          //AI15
        F8[205] = F8[203] + F8[204];
        F8[193] = F8[192];          //AI14
        F8[195] = F8[193] + F8[194];
        F8[163] = F8[162];          //AI11
        F8[165] = F8[163] + F8[164];
        F8[153] = F8[152];          //AI10
        F8[155] = F8[153] + F8[154];
        F8[243] = F8[242];          //AI18
        F8[245] = F8[243] + F8[244];
        CptOffset = 0;
        B3[32] = 1;
      }
  }
}

//================================================================================================================================Tempo2 BC DC
void TonT2() {   // T4:0  BUS STABILISE

  switch (ST2) {
    case 1:
      if (buttonPin22Num >= 4 && buttonPin37Num >= 4) {
        T2 = 30;
        ST2 = 2;
      }
      else {
        TDN[0] = 0;
        ST2 = 1;
        T2 = 0;
      }
      break;
    case 2:      //TT
      if (T2 <= 0) {
        ST2 = 3;
        TDN[0] = 1;
      }
      break;
    case 3:
      //DN
      if (buttonPin37Num < 4) {  // element declencheur du calcul d'offset de l'input AI9
        ST2 = 1;
        T2 = 0;
      }
      break;
  }
}

//================================================================================================================================Tempo3  INTEGRATION
void TonT3() {   // T4:4

  switch (ST3) {
    case 1:
      if (ST2 == 3) {
        T3 = 5;
        ST3 = 2;
      }
      else {
        F8[148] = 0.0;//AI9
        F8[139] = 0.0;
        F8[188] = 0.0;//AI13
        F8[189] = 0.0;
        F8[178] = 0.0;//AI12
        F8[179] = 0.0;
        F8[208] = 0.0;//AI15
        F8[209] = 0.0;
        F8[198] = 0.0;//AI14
        F8[199] = 0.0;
        F8[168] = 0.0;//AI11
        F8[169] = 0.0;
        F8[158] = 0.0;//AI10
        F8[159] = 0.0;
        F8[248] = 0.0;
      }

      break;
    case 2:  //TT
      if (T3 <= 0) {
        ST3 = 3;
      }
      else {
        F8[139] = F8[139] + 1;//  AI9
        F8[148] = F8[52] + F8[148];

        F8[188] = F8[187] + F8[188];//AI13

        F8[178] = F8[177] + F8[178];//AI12

        F8[208] = F8[207] + F8[208];//AI15

        F8[198] = F8[197] + F8[198];//AI14

        F8[168] = F8[167] + F8[168];//AI11

        F8[158] = F8[157] + F8[158];//AI10

        F8[248] = F8[247] + F8[248];//AI10
      }

      break;
    case 3:
      //DN

      F8[110] = F8[148] / F8[139];  // valeur integree du courant batterie 1 AI9
      F8[148] = 0.0;


      F8[189] = F8[188] / F8[139];  // valeur integree du courant batterie AI13
      F8[188] = 0.0;

      F8[179] = F8[178] / F8[139];  // valeur integree du courant batterie AI12
      F8[178] = 0.0;

      F8[199] = F8[198] / F8[139];  // valeur integree du courant batterie AI14
      F8[198] = 0.0;

      F8[169] = F8[168] / F8[139];  // valeur integree du courant batterie AI11
      F8[168] = 0.0;

      F8[159] = F8[158] / F8[139];  // valeur integree du courant batterie AI10
      F8[158] = 0.0;

      F8[210] = F8[208] / F8[139];  // valeur integree du courant batterie AI10
      F8[208] = 0.0;

      F8[120] = F8[248] / F8[139];  // valeur integree du courant batterie AI8
      F8[248] = 0.0;
      F8[250] = F8[110] + F8[120];

      F8[139] = 0.0;

      ST3 = 1;
      T3 = 5;
      break;
  }

}



//==============================================================================
void TonT4() {  //T4:?

  switch (ST4) {
    case 1:
      if (buttonPin38Num >= 4) {
        T4 = 50;
        ST4 = 2;
      }
      else {
      }
      break;
    case 2:
      if (T4 <= 0) {
        ST4 = 3;
      }
      if (buttonPin38Num < 4) {
        ST4 = 1;
        T4 = 0;
      }
      //TT

      break;
    case 3:
      //DN
      ST4 = 1;
      T4 = 0;
      break;
  }
}




//====================================================================================
void TonT5() {  //          T4:1

  switch (ST5) {
    case 1:

      if (ST5 == 1) {
        T5 = 5;
        ST5 = 2;
      }
      else {
        F8[62] = 0.0;
        F8[61] = 0.0;
      }
      break;
    case 2:      //TT
      if (T5 <= 0) {
        ST5 = 3;
      }
      else {
        F8[61] = F8[61] + 1;
        F8[62] = F8[62] + F8[73] ;
      }
      break;
    case 3:
      //DN
      F8[60] = F8[62] / F8[61];//AI7
      F8[61] = 0.0;
      F8[62] = 0.0;
      ST5 = 1;
      T5 = 0;

      break;
  }
}


//==================================================================================
void TonT6() {  //          T4:7

  switch (ST6) {
    case 1:
      if (ST6 == 1) {
        T6 = 10;
        ST6 = 2;
      }
      else {

      }
      break;
    case 2:      //TT
      if (T6 <= 0) {
        ST6 = 3;
      }
      break;
    case 3:
      //DN
      //      F8[82] = F8[49] - F8[81];
      //      F8[84] = F8[82] / F8[44];
      //      F8[81] = F8[84] + F8[81];
      //      F8[80] = F8[81] * -1.0;
      //      F8[45] = F8[78] + F8[80];

      ST6 = 1;
      T6 = 0;
      break;
  }
}

////=================================================================================
void TonT7() {  //          T4:8

  switch (ST7) {
    case 1:
      if (ST7 == 1) {
        T7 = 600;
        ST7 = 2;
      }
      else {

      }
      break;
    case 2:      //TT
      if (T7 <= 0) {
        ST7 = 3;
      }
      break;
    case 3:
      //DN
      //      if (F8[34] > F8[53]) {
      //        F8[53] = F8[53] + 1.0;
      //      }
      //      if (F8[34] < F8[53]) {
      //        F8[53] = F8[53] - 1.0;
      //      }
      ST7 = 1;
      T7 = 0;
      break;
  }
}

////==========================================================================
void TonT8() {  //          T4:2

  switch (ST8) {
    case 1:
      if (ST8 == 1) {
        T8 = 300;
        ST8 = 2;
      }
      else {

      }
      break;
    case 2:      //TT
      if (T8 <= 0) {
        ST8 = 3;
      }
      break;
    case 3:
      //DN
      if (F8[60] > F8[95]) {
        F8[96] = F8[60];
        F8[98] = F8[96] - F8[97];
        F8[97] = F8[60];
      }
      ST8 = 1;
      T8 = 0;
      break;
  }
}


////=======================================================================
void TonT9() {  //          T4:??32

  switch (ST9) {
    case 1:
      if (((F8[60] > F8[95]) && (F8[98] > F8[64])) || (F8[60] > F8[212])) { // element declencheur du calcul d'offset de l'input AI9 AI13
        T9 = 300;
        ST9 = 2;
      }
      else {
      }
      break;
    case 2:      //TT
      if (T9 <= 0) {
        ST9 = 3;
      }
      /*if (!((F8[60] > F8[95]) && (F8[98] > F8[64])) || (F8[60] > F8[212])) {
        ST9 = 1;
        T9 = 0;
        }*/
      break;
    case 3:
      //DN
      ST9 = 1;
      T9 = 0;
      break;
  }
}

//===================================================================================================================
void TonT10() {  //          T4:3

  switch (ST10) {
    case 1:
      if (ST10 == 1 && ST2 == 3) { // element declencheur du calcul d'offset de l'input AI9 AI13
        T10 = 10;
        ST10 = 2;
        TDN[3] = 0;
      }
      else {
        F8[111] = 0.0;
        F8[112] = 0.0;
        T10 = 0;
        TDN[3] = 0;
      }
      break;
    case 2:      //TT
      if (T10 <= 0) {
        ST10 = 3;
      }
      //if (ST10 == 2) {
      //   ST10 = 1;
      // }
      break;
    case 3:
      //DN
      TDN[3] = 1;
      ST10 = 1;
      T10 = 0;
      break;
  }
}

//===================================================================================================================
void TonT11() {  //          T4:??

  switch (ST11) {
    case 1:
      if (ST11 == 1) { // element declencheur du calcul d'offset de l'input AI9 AI13
        T11 = 50;
        ST11 = 2;
      }
      else {

      }
      break;
    case 2:      //TT
      if (T11 <= 0) {
        ST11 = 3;
      }
      break;
    case 3:
      //DN
      //ST10 = 1;
      T11 = 0;
      break;
  }
}

//========================================================ECRITURE
void Ecriture() {

  while (Serial3.available() > 0) { // si un caractère en réception

    octetReception = Serial3.read(); // lit le 1er octet de la file d'attente

    if (octetReception == 10) { // si Octet reçu est le saut de ligne

      //SET TIME
      if (chaineReception.substring(0, 3) == "SET")  {
    //  Serial.println("SET");
        String InString;
        InString = chaineReception.substring(3,5);        
        SetHeure = InString.toInt();
    //    Serial.println(SetHeure);       
        InString = chaineReception.substring(5,7);       
        SetMin = InString.toInt();
     //   Serial.println(SetMin);
        InString = chaineReception.substring(7,9);       
        SetJour = InString.toInt();
     //   Serial.println(SetJour);
        InString = chaineReception.substring(9,11);       
        SetMois = InString.toInt();
     //   Serial.println(SetMois);
        InString = chaineReception.substring(11);
        SetAnnee = InString.toInt();
      //  Serial.println(SetAnnee);
        ClockGestion();
        }

      //PWM4
      if (chaineReception == "PWM4=ON") {


      }
      if (chaineReception == "PWM4=OFF")  {

      }

      //PWM5
      if (chaineReception == "PWM5=ON") {

      }
      if (chaineReception == "PWM5=OFF")  {

      }
      //PWM6
      if (chaineReception == "PWM6=ON") {

      }
      if (chaineReception == "PWM6=OFF")  {

      }
      //PWM7
      if (chaineReception == "PWM7=ON") {

      }
      if (chaineReception == "PWM7=OFF")  {

      }

      //PWM8
      if (chaineReception == "PWM8=ON") {

      }
      if (chaineReception == "PWM8=OFF")  {

      }

      //PWM9
      if (chaineReception == "PWM9=ON") {

      }
      if (chaineReception == "PWM9=OFF")  {

      }

      //PWM11
      if (chaineReception == "PWM11=ON") {

      }
      if (chaineReception == "PWM11=OFF")  {

      }

      //PWM12
      if (chaineReception == "PWM12=ON") {

      }
      if (chaineReception == "PWM12=OFF")  {

      }

      //OUT41
      if (chaineReception == "OUT41=ON") {

      }
      if (chaineReception == "OUT41=OFF")  {

      }

      //OUT43
      if (chaineReception == "OUT43=ON") {

      }
      if (chaineReception == "OUT43=OFF")  {

      }

      //OUT45
      if (chaineReception == "OUT45=ON") {

      }
      if (chaineReception == "OUT45=OFF")  {

      }

      //OUT47
      if (chaineReception == "OUT47=ON") {

      }
      if (chaineReception == "OUT47=OFF")  {

      }

      //OUT49
      if (chaineReception == "OUT49=ON") {

      }
      if (chaineReception == "OUT49=OFF")  {

      }

      //OUT51
      if (chaineReception == "OUT51=ON") {

      }
      if (chaineReception == "OUT51=OFF")  {

      }

      //OUT53
      if (chaineReception == "OUT53=ON") {

      }
      if (chaineReception == "OUT53=OFF")  {

      }

      //
      if (chaineReception == "BP1ON")  {
        BPOnOff = 1;
        //digitalWrite(LED7, HIGH);
      }
      //
      if (chaineReception == "BP1OFF")  {
        BPOnOff = 0;
        // digitalWrite(LED7, LOW);
      }

      if (chaineReception.substring(0, 3) == "CSG")  {
        String InString;
        InString = chaineReception.substring(3);
        CSG = InString.toFloat();
        if (CSG == 69) {
          digitalWrite(LED13, HIGH);

        }
        else {
          digitalWrite(LED13, LOW);
        }

      }

      if (chaineReception.substring(0, 3) == "VAL")  {
        String InString;
        InString = chaineReception.substring(3);
        Value = InString.toFloat();
        F8[IndexDebug] = Value;
      }
      if (chaineReception.substring(0, 4) == "VALN")  {
        String InString;
        InString = chaineReception.substring(4);
        Value = InString.toFloat();
        N7[IndexDebugN] = Value;
      }
      if (chaineReception.substring(0, 4) == "VALF")  {
        String InString;
        InString = chaineReception.substring(4);
        Value = InString.toFloat();
        F10[IndexDebugF] = Value;
      }
      if (chaineReception.substring(0, 4) == "VALB")  {
        String InString;
        InString = chaineReception.substring(4);
        Value = InString.toFloat();
        B3[IndexDebugB] = Value;
      }
      //
      if (chaineReception == "EVENON")  {
        BPOnOff = 1;
        digitalWrite(LED13, HIGH);
      }
      //
      if (chaineReception == "EVENOFF")  {
        BPOnOff = 0;
        digitalWrite(LED13, LOW);
      }


      //synchro
      if (chaineReception == "SYNCHRO")  {
        SYNCHRO = 1;
        LU = 1;
        //Serial.print("SYNCHRO");
        //Serial.println(SYNCHRO);
      }

      if (chaineReception == "GSMON")  {
        AutorisationSMS = true;
        //Serial.println("AutorisationSMS true");
      }
      if (chaineReception == "GSMOFF")  {
        AutorisationSMS = false;
       //Serial.println("AutorisationSMS false");
      }

      //synchro off
      if (chaineReception == "SYNCHROFF")  {
        SYNCHRO = 0;
      }


      if (chaineReception == "MESSDIT")  {
        MessOut = 0;
        /*if (MessPoint > 1) {
          MessTbl[1] = 0;
          MessTbl[1] = MessTbl[2];
          MessTbl[2] = MessTbl[3];
          MessTbl[3] = MessTbl[4];
          MessTbl[4] = MessTbl[5];
          MessTbl[5] = MessTbl[6];
          MessTbl[6] = MessTbl[7];
          MessTbl[7] = MessTbl[8];
          MessTbl[8] = MessTbl[9];
          MessTbl[9] = MessTbl[10];
          MessPoint = -1;
          }*/
        //Serial.print(MessTbl[1]);
        //Serial.println(" :MESSDIT");
      }

      //LU
      if (chaineReception == "LU")  {
        LU = 1;
      }
      //DEBUG
      if (chaineReception.substring(0, 5) == "DEBUG")  {
        IndexDebug = chaineReception.substring(5).toInt();

      }
      if (chaineReception.substring(0, 6) == "DEBUGN")  {
        IndexDebugN = chaineReception.substring(6).toInt();

      }
      if (chaineReception.substring(0, 6) == "DEBUGF")  {
        IndexDebugF = chaineReception.substring(6).toInt();

      }
      if (chaineReception.substring(0, 6) == "DEBUGB")  {
        IndexDebugB = chaineReception.substring(6).toInt();

      }
      //Serial.println (chaineReception);

      chaineReception = ""; //RAZ le String de réception
      //delay(100); // pause
      break; // s||t de la boucle while
    }
    else { // si le caractère reçu n'est pas un saut de ligne
      caractereReception = char(octetReception); // récupere le caractere à partir du code Ascii
      chaineReception = chaineReception + caractereReception; // ajoute la caractère au String
      //Serial.println (chaineReception);
      //delay(1); // laisse le temps au caractères d'arriver
    }
  }
}

//=================================================LECTURE
void Lecture() {

    if ((LU == 1) && (SYNCHRO == 1) && (((GSMPasAPas == 2 or GSMPasAPas == 10) && buttonPin40Num>=4) || buttonPin40Num<4))  {
    
    //if ((LU == 1) && (SYNCHRO == 1)) 
    
    //Serial.println (LU);
    //Serial.println (SYNCHRO);
    //Serial.println (Cycle);
    LU = 0;
    ST11 = 1;
    Tram = "";

    if (N7M60 == NAN) {
      N7M60 = 0;
    }
    dtostrf( N7M60, 5, 0, charVal);
    Tram = Tram + "D01" +  charVal;
    Tram = Tram + "F";


    if (F8[161] == NAN) {
      F8M161 = 0.0;
    }
    dtostrf( -F8M161, 5, 2, charVal);
    Tram = Tram + "D02" +  charVal;
    Tram = Tram + "F";

    if (F8[132] == NAN) {
      F8[132] = 0.0;
    }
    dtostrf( -F8[132], 5, 2, charVal);
    Tram = Tram + "D03" +  charVal;
    Tram = Tram + "F";

    if (F8[125] == NAN) {
      F8[125] = 0.0;  
    }
    dtostrf( F8[125], 5, 2, charVal);
    Tram = Tram + "D04" +  charVal;
    Tram = Tram + "F";

    dtostrf( Heure, 2, 0, charVal);
    Tram = Tram + "D20" +  charVal;
    Tram = Tram + "F";

    dtostrf( Minute, 2, 0, charVal);
    Tram = Tram + "D21" +  charVal;
    Tram = Tram + "F";

    dtostrf( Seconde, 2, 0, charVal);
    Tram = Tram + "D22" +  charVal;
    Tram = Tram + "F";

    dtostrf( Jour, 2, 0, charVal);
    Tram = Tram + "D23" +  charVal;
    Tram = Tram + "F";

    dtostrf( Mois, 2, 0, charVal);
    Tram = Tram + "D24" +  charVal;
    Tram = Tram + "F";

    dtostrf( Annee, 2, 0, charVal);
    Tram = Tram + "D25" +  charVal;
    Tram = Tram + "F";

    dtostrf( StatusOndul, 2, 0, charVal);
    Tram = Tram + "D26" +  charVal;
    Tram = Tram + "F";

    if (CSG == NAN) {
      CSG = 0.0;
    }
    dtostrf( CSG, 5, 2, charVal);
    Tram = Tram + "D27" +  charVal;
    Tram = Tram + "F";

    if (F8M105 == NAN) {
      F8M105 = 0.0;
    }
    dtostrf( F8M105, 5, 2, charVal);
    Tram = Tram + "D28" +  charVal;
    Tram = Tram + "F";

    dtostrf( MessOut, 3, 0, charVal);
    Tram = Tram + "D29" +  charVal;
    Tram = Tram + "F";

    if (F8[IndexDebug] == NAN) {
      F8[IndexDebug] = 0.0;
    }
    dtostrf( F8[IndexDebug], 5, 3, charVal);
    Tram = Tram + "D30" +  charVal;
    Tram = Tram + "F";

    if (N7[IndexDebugN] == NAN) {
      N7[IndexDebugN] = 0;
    }
    dtostrf( N7[IndexDebugN], 5, 0, charVal);
    Tram = Tram + "D31" +  charVal;
    Tram = Tram + "F";

    if (F10[IndexDebugF] == NAN) {
      F10[IndexDebugF] = 0.0;
    }
    dtostrf( F10[IndexDebugF], 5, 3, charVal);
    Tram = Tram + "D32" +  charVal;
    Tram = Tram + "F";


    dtostrf( B3[IndexDebugB], 5, 0, charVal);
    Tram = Tram + "D33" +  charVal;
    Tram = Tram + "F";

    dtostrf( PWM4, 1, 0, charVal);
    Tram = Tram + "D34" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM5, 1, 0, charVal);
    Tram = Tram + "D35" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM6, 1, 0, charVal);
    Tram = Tram + "D36" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM7, 1, 0, charVal);
    Tram = Tram + "D37" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM8, 1, 0, charVal);
    Tram = Tram + "D38" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM9, 1, 0, charVal);
    Tram = Tram + "D39" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM11, 1, 0, charVal);
    Tram = Tram + "D40" +  charVal;
    Tram = Tram + "F";
    dtostrf( PWM12, 1, 0, charVal);
    Tram = Tram + "D41" +  charVal;
    Tram = Tram + "F";

    if (Cycle == NAN) {
      Cycle = 0;
    }
    dtostrf( (Cycle), 3, 0, charVal);
    Tram = Tram + "D42" +  charVal;
    Tram = Tram + "F";

    dtostrf( GSMPasAPas, 1, 0, charVal);
    Tram = Tram + "D43" +  charVal;
    Tram = Tram + "F";

    dtostrf( MessSMS, 1, 0, charVal);
    Tram = Tram + "D44" +  charVal;
    Tram = Tram + "F";
    
    dtostrf( F10[44], 1, 0, charVal);
    Tram = Tram + "D45" +  charVal;
    Tram = Tram + "F";
    
    Serial3.print(Tram);
    //Serial.println(Tram);


  }
  //delay(1);
}
//============================================================Intégration
void Integration() {

  N7M60 = ((N7M60 * N7I) + N7[60] ) / (N7I + 1);
  if (N7M60 < 0) {
    N7M60 = 0;
  }

  if (F8[161] == NAN) {
    F8M161 = 0;
  }
  F8M161 = ((F8M161 * F8I) + F8[138]) / (F8I + 1);
  if (-F8M161 < 0) {
    F8M161 = 0;
  }
  if (F8M161 == NAN) {
    F8M161 = 0;
  }

  if (F8[132] == NAN) {
  }
  F8M132 = ((F8M132 * F8I) + F8[132]) / (F8I + 1);
  if (F8M132 < 0) {
    F8M132 = 0;
  }


  if (F8[125] == NAN) {
    F8M125 = 0;

  }
  F8M125 = ((F8M125 * F8I) + F8[125]) / (F8I + 1);
  if (F8M125 < -2000) {
    F8M125 = -2000;
  }


  if (F8[105] == NAN) {
    F8M105 = 0;
  }
  F8M105 = ((F8M105 * F8I) + F8[105]) / (F8I + 1);
  if ((F8M105 < 0)) {
    F8M105 = 0;
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------------------Scanin
void Scanin() {

  if (digitalRead(buttonPin22) == HIGH) {
    buttonPin22Num = buttonPin22Num + 1;
  }
  else {
    buttonPin22Num = 0;
  }
  if (buttonPin22Num > 4) {
    buttonPin22Num = 4;
  }
  if (buttonPin22Num < 0) {
    buttonPin22Num = 0;
  }


  if (digitalRead(buttonPin23) == HIGH) {
    buttonPin23Num = buttonPin23Num + 1;
  }
  else {
    buttonPin23Num = 0;
  }
  if (buttonPin23Num > 4) {
    buttonPin23Num = 4;
  }
  if (buttonPin23Num < 0) {
    buttonPin23Num = 0;
  }


  if (digitalRead(buttonPin25) == HIGH) {
    buttonPin25Num = buttonPin25Num + 1;
  }
  else {
    buttonPin25Num = 0;
  }
  if (buttonPin25Num > 4) {
    buttonPin25Num = 4;
  }
  if (buttonPin25Num < 0) {
    buttonPin25Num = 0;
  }


  if (digitalRead(buttonPin27) == HIGH) {
    buttonPin27Num = buttonPin27Num + 1;
  }
  else {
    buttonPin27Num = 0;
  }
  if (buttonPin27Num > 4) {
    buttonPin27Num = 4;
  }
  if (buttonPin27Num < 0) {
    buttonPin27Num = 0;
  }

  if (digitalRead(buttonPin29) == HIGH) {
    buttonPin29Num = buttonPin29Num + 1;
  }
  else {
    buttonPin29Num = 0;
  }
  if (buttonPin29Num > 4) {
    buttonPin29Num = 4;
  }
  if (buttonPin29Num < 0) {
    buttonPin29Num = 0;
  }


  if (digitalRead(buttonPin31) == HIGH) {
    buttonPin31Num = buttonPin31Num + 1;
  }
  else {
    buttonPin31Num = 0;
  }
  if (buttonPin31Num > 4) {
    buttonPin31Num = 4;
  }
  if (buttonPin31Num < 0) {
    buttonPin31Num = 0;
  }


  if (digitalRead(buttonPin33) == HIGH) {
    buttonPin33Num = buttonPin33Num + 1;
  }
  else {
    buttonPin33Num = 0;
  }
  if (buttonPin33Num > 4) {
    buttonPin33Num = 4;
  }
  if (buttonPin33Num < 0) {
    buttonPin33Num = 0;
  }

  if (digitalRead(buttonPin35) == HIGH) {
    buttonPin35Num = buttonPin35Num + 1;
  }
  else {
    buttonPin35Num = 0;
  }
  if (buttonPin35Num > 4) {
    buttonPin35Num = 4;
  }
  if (buttonPin35Num < 0) {
    buttonPin35Num = 0;
  }


  if (digitalRead(buttonPin37) == HIGH) {
    buttonPin37Num = buttonPin37Num + 1;
  }
  else {
    buttonPin37Num = 0;
  }
  if (buttonPin37Num > 4) {
    buttonPin37Num = 4;
  }
  if (buttonPin37Num < 0) {
    buttonPin37Num = 0;
  }

  if (digitalRead(buttonPin38) == HIGH) {
    buttonPin38Num = buttonPin38Num + 1;
  }
  else {
    buttonPin38Num = 0;
  }
  if (buttonPin38Num > 4) {
    buttonPin38Num = 4;
  }
  if (buttonPin38Num < -4) {
    buttonPin38Num = -4;
  }

  if (digitalRead(buttonPin39) == HIGH) {
    buttonPin39Num = buttonPin39Num + 1;
  }
  else {
    buttonPin39Num = 0;
  }
  if (buttonPin39Num > 4) {
    buttonPin39Num = 4;
  }
  if (buttonPin39Num < -4) {
    buttonPin39Num = -4;
  }

  if (digitalRead(buttonPin40) == HIGH) {
    buttonPin40Num = buttonPin40Num + 1;
  }
  else {
    buttonPin40Num = 0;
  }
  if (buttonPin40Num > 4) {
    buttonPin40Num = 4;
  }
  if (buttonPin40Num < -4) {
    buttonPin40Num = -4;
  }


  if (digitalRead(buttonPin41) == HIGH) {
    buttonPin41Num = buttonPin41Num + 1;
  }
  else {
    buttonPin41Num = 0;
  }
  if (buttonPin41Num > 4) {
    buttonPin41Num = 4;
  }
  if (buttonPin41Num < -4) {
    buttonPin41Num = -4;
  }

  if (digitalRead(buttonPin42) == HIGH) {
    buttonPin42Num = buttonPin42Num + 1;
  }
  else {
    buttonPin42Num = 0;
  }
  if (buttonPin42Num > 4) {
    buttonPin42Num = 4;
  }
  if (buttonPin42Num < -4) {
    buttonPin42Num = -4;
  }


  if (digitalRead(buttonPin43) == HIGH) {
    buttonPin43Num = buttonPin43Num + 1;
  }
  else {
    buttonPin43Num = 0;

  }
  if (buttonPin43Num > 4) {
    buttonPin43Num = 4;
  }
  if (buttonPin43Num < -4) {
    buttonPin43Num = -4;
  }

  if (digitalRead(buttonPin44) == HIGH) {
    buttonPin44Num = buttonPin44Num + 1;
  }
  else {
    buttonPin44Num = 0;
  }
  if (buttonPin44Num > 4) {
    buttonPin44Num = 4;
  }
  if (buttonPin44Num < -4) {
    buttonPin44Num = -4;
  }

  if (digitalRead(buttonPin45) == HIGH) {
    buttonPin45Num = buttonPin45Num + 1;
  }
  else {
    buttonPin45Num = 0;
  }
  if (buttonPin45Num > 4) {
    buttonPin45Num = 4;
  }
  if (buttonPin45Num < -4) {
    buttonPin45Num = -4;
  }

  if (digitalRead(buttonPin46) == HIGH) {
    buttonPin46Num = buttonPin46Num + 1;
  }
  else {
    buttonPin46Num = 0;
  }
  if (buttonPin46Num > 4) {
    buttonPin46Num = 4;
  }
  if (buttonPin46Num < -4) {
    buttonPin46Num = -4;
  }
  if (digitalRead(buttonPin47) == HIGH) {
    buttonPin47Num = buttonPin47Num + 1;
  }
  else {
    buttonPin47Num = 0;
  }
  if (buttonPin47Num > 4) {
    buttonPin47Num = 4;
  }
  if (buttonPin47Num < -4) {
    buttonPin47Num = -4;
  }
  if (digitalRead(buttonPin48) == HIGH) {
    buttonPin48Num = buttonPin48Num + 1;
  }
  else {
    buttonPin48Num = 0;
  }
  if (buttonPin48Num > 4) {
    buttonPin48Num = 4;
  }
  if (buttonPin48Num < -4) {
    buttonPin48Num = -4;
  }


  if (digitalRead(buttonPin50) == HIGH) {
    buttonPin50Num = buttonPin50Num + 1;
  }
  else {
    buttonPin50Num = 0;
  }
  if (buttonPin50Num > 4) {
    buttonPin50Num = 4;
  }
  if (buttonPin50Num < -4) {
    buttonPin50Num = -4;
  }
  if (digitalRead(buttonPin52) == HIGH) {
    buttonPin52Num = buttonPin52Num + 1;
  }
  else {
    buttonPin52Num = 0;
  }
  if (buttonPin52Num > 4) {
    buttonPin52Num = 4;
  }
  if (buttonPin52Num < -4) {
    buttonPin52Num = -4;
  }

  //mwf10214

  AI7AUX1 = analogRead(analogPin7);

  AI7AUX2 = (AI7Calculer * F8[213]);
  AI7AUX3 = (analogRead(analogPin7) + AI7AUX2);
  AI7Calculer = AI7AUX3 / (F8[213] + 1);
  F8[214] = AI7Calculer;
  //F8[214] = analogRead(analogPin7);

  if (N7[10] == 375 || N7[10] == 380 || N7[10] == 370)  {
    F10[214] = F10[214] + analogRead(analogPin7);
    F10[213] = F10[213] + 1;
  }
  else {
    F10[213] = 0;
  }
  if (F10[213] >= 70.0) {
    F10[215] = ((F10[214] / 70.0) * F8[67]) + 30.0;
    F10[214] = 0;
    F10[213] = 0;
  }

  F8[235] = ((F8[235] * 200.0) + F8[52]) / 201.0;
  F8[69] = (F8[67] * F8[214]) + 30.0; //(analogRead(analogPin7)) + 30.0------------------------------------------------------Tension batterie

  if (F8[145] == 0) F8[145] = 509;
  F8[146] = analogRead(analogPin9) - F8[145];       //---------------------------------------------------Courant batterie AI9  F8[52]
  F8[52] = (F8[146] * 0.122);

  if (F8[185] == 0) F8[185] = 509;
  F8[186] = analogRead(analogPin13) - F8[185];//-----------------------------------------------------------------------Courant MPPT1  AI13
  F8[187] = (F8[186] * (-0.122));

  if (F8[175] == 0) F8[175] = 509;
  F8[176] = analogRead(analogPin12) - F8[175];//-----------------------------------------------------------------------Courant MPPT1  AI12
  F8[177] = (F8[176] * (-0.122));


  if (F8[205] == 0) F8[205] = 509;
  F8[235] = analogRead(analogPin15);
  F8[206] = analogRead(analogPin15) - F8[205];//-----------------------------------------------------------------------Courant chargeur 1  AI15
  F8[207] = (F8[206] * (-0.122));


  if (F8[195] == 0) F8[195] = 509;
  F8[196] = analogRead(analogPin14) - F8[195];//-----------------------------------------------------------------------Courant MPPT1  AI14
  F8[197] = (F8[196] * (-0.122));

  if (F8[165] == 0) F8[165] = 509;
  F8[166] = analogRead(analogPin11) - F8[165];//-----------------------------------------------------------------------Courant MPPT1  AI11
  F8[167] = (F8[166]  * (-0.122));

  if (F8[155] == 0) F8[155] = 509;
  F8[156] = analogRead(analogPin10) - F8[155];//-----------------------------------------------------------------------Courant onduleur 2  AI10
  F8[157] = (F8[156]  * (-0.122));

  if (F8[245] == 0) F8[245] = 509;
  F8[246] = analogRead(analogPin8) - F8[245];       //---------------------------------------------------Courant batterie 2  F8[52]
  if (B3[306]) {
    F8[247] = (F8[246] * 0.122);
  }
  else {
    F8[247] = 0.0;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------Message
void MessTraitement() {

  byte MessInByte;
  if (MessOut == 0 ) {
    MessIn1 = N7[90];
    if (MessInMemo1 != MessIn1) {
      MessInMemo1 = MessIn1;
      MessOut = MessIn1;
      //Serial.println(MessOut);
    }
  }
  if (MessOut == 0 ) {
    MessIn2 = N7[91] + 200;
    if (MessInMemo2 != MessIn2) {
      MessInMemo2 = MessIn2;
      MessOut = MessIn2;
      //Serial.println(MessOut);
    }
  }
}
//-----------------------------------------------
//void Gemunolink() {
//
//
//  MyPlot.SendData("My Sensor", (Cycle));
//
//  Panel.SetProgress("Label1", (Cycle));
//  Panel.SetProgress("Cpt", (Cycle));
//  Panel.SetText("Status", FromTable);
//  MyTable.SendData("Cycle", Cycle);
//  //MyTable.GetData("FromTable");
//
//}
//------------------------------------------------
void callback() {

  SMSNbrCycle = SMSNbrCycle - 1;

  if (CptT[2]) {
    T[2] =  T[2] + 1.0;
  }
  if (CptT[3]) {
    T[3] =  T[3] + 1.0;
  }
  if (CptT[5]) {
    T[5] =  T[5] + 1.0;
  }
  if (CptT[6]) {
    T[6] =  T[6] + 1.0;
  }
  if (CptT[7]) {
    T[7] =  T[7] + 1.0;
  }
  if (CptT[7]) {
    T[7] =  T[7] + 1.0;
  }
  if (CptT[8]) {
    T[8] =  T[8] + 1.0;
  }
  if (CptT[9]) {
    T[9] =  T[9] + 1.0;
  }
  if (CptT[10]) {
    T[10] =  T[10] + 1.0;
  }
  if (CptT[11]) {
    T[11] =  T[11] + 1.0;
  }
  if (CptT[12]) {
    T[12] =  T[12] + 1.0;
  }
  if (CptT[13]) {
    T[13] =  T[13] + 1.0;
  }
  if (CptT[14]) {
    T[14] =  T[14] + 1.0;
  }
  if (CptT[15]) {
    T[15] =  T[15] + 1.0;
  }
  if (CptT[16]) {
    T[16] =  T[16] + 1.0;
  }
  if (CptT[25]) {
    T[25] =  T[25] + 1.0;
  }
  if (CptT[26]) {
    T[26] =  T[26] + 1.0;
  }
  if (CptT[27]) {
    T[27] =  T[27] + 1.0;
  }
  if (CptT[28]) {
    T[28] =  T[28] + 1.0;
  }
  if (CptT[29]) {
    T[29] =  T[29] + 1.0;
  }
  if (CptT[31]) {
    T[31] =  T[31] + 1.0;
  }
  if (CptT[32]) {
    T[32] =  T[32] + 1.0;
  }
  if (CptT[33]) {
    T[33] =  T[33] + 1.0;
  }
  if (CptT[34]) {
    T[34] =  T[34] + 1.0;
  }
  if (CptT[35]) {
     T_35 =  T_35 + 1.0;
  }
  if (CptT[36]) {
    T[36] =  T[36] + 1.0;
  }
  if (CptT[37]) {
    T[37] =  T[37] + 1.0;
  }
  if (CptT[38]) {
    T[38] =  T[38] + 1.0;
  }
  if (CptT[39]) {
    T[39] =  T[39] + 1.0;
  }

  if (CptT[55]) {
    T[55] =  T[55] + 1.0;
  }
  if (CptT[56]) {
    T[56] =  T[56] + 1.0;
  }
  if (CptT[57]) {
    T[57] =  T[57] + 1.0;
  }

  if (CptT[63]) {
    T[63] =  T[63] + 1.0;
  }
  if (CptT[64]) {
    T[64] =  T[64] + 1.0;
  }
  if (CptT[65]) {
    T[65] =  T[65] + 1.0;
  }
  if (CptT[66]) {
    T[66] =  T[66] + 1.0;
  }
  if (CptT[67]) {
    T[67] =  T[67] + 1.0;
  }
  if (CptT[68]) {
    T[68] =  T[68] + 1.0;
  }
  if (CptT[70]) {
    T[70] =  T[70] + 1.0;
  }
  if (CptT[71]) {
    T[71] =  T[71] + 1.0;
  }
  if (CptT[72]) {
    T[72] =  T[72] + 1.0;
  }
  if (CptT[40]) {
    T[40] =  T[40] + 1.0;
  }
  if (CptT[41]) {
    T[41] =  T[41] + 1.0;
  }
  if (CptT[42]) {
    T[42] =  T[42] + 1.0;
  }
  if (CptT[43]) {
    T[43] =  T[43] + 1.0;
  }
  if (CptT[44]) {
    T[44] =  T[44] + 1.0;
  }
  if (CptT[45]) {
    T[45] =  T[45] + 1.0;
  }
  if (CptT[46]) {
    T[46] =  T[46] + 1.0;
  }
  if (CptT[47]) {
    T[47] =  T[47] + 1.0;
  }
  if (CptT[48]) {
    T[48] =  T[48] + 1.0;
  }
  if (CptT[49]) {
    T[49] =  T[49] + 1.0;
  }
  if (CptT[50]) {
    T[50] =  T[50] + 1.0;
  }
  if (CptT[54]) {
    T[54] =  T[54] + 1.0;
  }
  if (CptT[51]) {
    T[51] =  T[51] + 1.0;
  }
  if (CptT[52]) {
    T[52] =  T[52] + 1.0;
  }
  if (CptT[58]) {
    T[58] =  T[58] + 1.0;
  }
  if (CptT[59]) {
    T[59] =  T[59] + 1.0;
  }
  if (CptT[60]) {
    T[60] =  T[60] + 1.0;
  }
  if (CptT[61]) {
    T[61] =  T[61] + 1.0;
  }
  if (CptT[62]) {
    T[62] =  T[62] + 1.0;
  }
  if (CptT[73]) {
    T[73] =  T[73] + 1.0;
  }
  if (CptT[74]) {
    T[74] =  T[74] + 1.0;
  }
  if (CptT[80]) {
    T[80] =  T[80] + 1.0;
  }
  if (CptT[85]) {
    T[85] =  T[85] + 1.0;
  }
  if (ST1 == 2) {
    T1 = T1 - 1;
  }
  if (ST2 == 2) {
    T2 = T2 - 1;
  }
  if (ST3 == 2) {
    T3 = T3 - 1;
  }
  if (ST4 == 2) {
    T4 = T4 - 1;
  }
  if (ST5 == 2) {
    T5 = T5 - 1;
  }
  if (ST6 == 2) {
    T6 = T6 - 1;
  }
  if (ST7 == 2) {
    T7 = T7 - 1;
  }
  if (ST8 == 2) {
    T8 = T8 - 1;
  }
  if (ST9 == 2) {
    T9 = T9 - 1;
  }
  if (ST10 == 2) {
    T10 = T10 - 1;
  }
  if (ST11 == 2) {
    T11 = T11 - 1;
  }

  B3[31] = 1;

  /* T1CentSet = true;
    T1SecCpt = T1SecCpt + 1;
    T1DSecCpt = T1DSecCpt + 1;


    if (T1DSecCpt >= 100) {
     T1DSecSet = true;
     T1DSecCpt = 0;
    }
    if (T1SecCpt >= 1000) {
     T1SecSet = true;
     T1SecCpt = 0;
    }*/
}
//-------------------------------------
void AffPointeur(String P ) {
  int  Default = 0;
  if (P == "ST1") {
    Serial.print("VARIABLE: ");
    Serial.print(ST1);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST2") {
    Serial.print("VARIABLE: ");
    Serial.print(ST2);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST3") {
    Serial.print("VARIABLE: ");
    Serial.print(ST3);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST4") {
    Serial.print("VARIABLE: ");
    Serial.print(ST4);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST5") {
    Serial.print("VARIABLE: ");
    Serial.print(ST5);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST6") {
    Serial.print("VARIABLE: ");
    Serial.print(ST6);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST7") {
    Serial.print("VARIABLE: ");
    Serial.print(ST7);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST8") {
    Serial.print("VARIABLE: ");
    Serial.print(ST8);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST9") {
    Serial.print("VARIABLE: ");
    Serial.print(ST9);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST10") {
    Serial.print("VARIABLE: ");
    Serial.print(ST10);
    Serial.print("/");
    Default = 1;
  }
  if (P == "ST11") {
    Serial.print("VARIABLE: ");
    Serial.print(ST11);
    Serial.print("/");
    Default = 1;
  }
  if (P == "Message") {
    Serial.print("VARIABLE: ");
    Serial.print(Message);
    Serial.print("/");
    Default = 1;
  }
  if (P == "SYNCHRO") {
    Serial.print("VARIABLE: ");
    Serial.print(SYNCHRO);
    Serial.print("/");
    Default = 1;
  }
  if (P == "LU") {
    Serial.print("VARIABLE: ");
    Serial.print(LU);
    Serial.print("/");
    Default = 1;
  }
  if (P == "MessIn") {
    Serial.print("VARIABLE: ");
    Serial.print(MessIn);
    Serial.print("/");
    Default = 1;
  }
  if (P == "MessOut") {
    Serial.print("VARIABLE: ");
    Serial.print(MessOut);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T1SecCpt") {
    Serial.print("VARIABLE: ");
    Serial.print(T1SecCpt);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T58TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T58TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T59TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T59TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T60TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T60TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T61TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T61TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T63TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T63TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T6TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T6TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T67TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T67TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T39TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T39TT);
    Serial.print("/");
    Default = 1;
  }
  if (P == "T55TT") {
    Serial.print("VARIABLE: ");
    Serial.print(T55TT);
    Serial.print("/");
    Default = 1;
  }
  if (Default == 0) {
    Serial.print("VARIABLE: ");
    Serial.print("nan");
    Serial.print("/");
  }
}

//------------------------------------------------
void PasMessage() {
  if (!B3[449] && !B3[450] && !B3[451] && !B3[452] && !B3[453] && !B3[454] && !B3[455] && !B3[456] && !B3[457] && !B3[458] && !B3[459]) {
    if (!B3[460] && !B3[461] && !B3[462] && !B3[463] && !B3[464]) {
      N7[77] = 1;
    }
    else {
      N7[77] = 0;
    }
  }
  if (!B3[465] && !B3[466] && !B3[467] && !B3[468] && !B3[469] && !B3[470] && !B3[471] && !B3[472] && !B3[473] && !B3[474] && !B3[475]) {
    if (!B3[476] && !B3[477] && !B3[478] && !B3[479] && !B3[480]) {
      N7[78] = 1;
    }
    else {
      N7[78] = 0;
    }
  }
  if (!B3[481] && !B3[482] && !B3[483] && !B3[484] && !B3[485] && !B3[486] && !B3[487] && !B3[488] && !B3[489] && !B3[490] && !B3[491]) {
    if (!B3[492] && !B3[493] && !B3[494] && !B3[495] && !B3[496]) {
      N7[79] = 1;
    }
    else {
      N7[79] = 0;
    }
  }
  if (N7[77] == 1 && N7[78] == 1 && N7[79] == 1) {
    B3[416] = 1;
  }
  else {
    B3[416] = 0;
  }
}




//--------------------------------------------------------------------AFFICHAGE PROCESSING
void AffProcessing() {
  while (Serial.available() > 0) { // si un caractère en réception

    octetReceptionProc = Serial.read(); // lit le 1er octet de la file d'attente

    if (octetReceptionProc == '/') {
      if (chaineReceptionProc.substring(0, 2) == "F8")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(2);
        IndexDebugProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 2) == "N7")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(2);
        IndexDebugNProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 3) == "F10")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(3);
        IndexDebugFProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 2) == "B3")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(2);
        IndexDebugBProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 1) == "T")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(1);
        IndexDebugTProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 3) == "TDN")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(3);
        IndexDebugTDProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 2) == "TT")  {
        String InStringProc;
        InStringProc = chaineReceptionProc.substring(2);
        IndexDebugTTProc = InStringProc.toFloat();
      }
      if (chaineReceptionProc.substring(0, 3) == "VAR")  {
        String InStringProc;
        InStringProc = (chaineReceptionProc.substring(3));
        Pointeur = InStringProc;
      }
      if (chaineReceptionProc.substring(0, 6) == "OKPROC")  {
        Processing = true;
      }
      chaineReceptionProc = "";
      //delay(1); // pause
    }
    else {
      caractereReceptionProc = char(octetReceptionProc);
      chaineReceptionProc = chaineReceptionProc + caractereReceptionProc;
      //delay(1);
    }
  }

  if (Processing) {
    dtostrf( F8[IndexDebugProc], 5, 3, charVal);
    TramProc = charVal;
    TramProc = TramProc + "/";

    dtostrf( N7[IndexDebugNProc], 5, 0, charVal);
    TramProc = TramProc + charVal;
    TramProc = TramProc + "/";

    dtostrf( F10[IndexDebugFProc], 5, 3, charVal);
    TramProc = TramProc + charVal;
    TramProc = TramProc + "/";

    dtostrf( B3[IndexDebugBProc], 5, 0, charVal);
    TramProc = TramProc + charVal;
    TramProc = TramProc + "/";

    dtostrf( T[IndexDebugTProc], 5, 0, charVal);
    TramProc = TramProc + charVal;
    TramProc = TramProc + "/";

    TramProc = TramProc + IndexDebugProc;
    TramProc = TramProc + "/";

    TramProc = TramProc + IndexDebugNProc;
    TramProc = TramProc + "/";

    TramProc = TramProc + IndexDebugFProc;
    TramProc = TramProc + "/";

    TramProc = TramProc + IndexDebugBProc;
    TramProc = TramProc + "/";

    TramProc = TramProc + IndexDebugTProc;
    TramProc = TramProc + "/";

    Serial.print(TramProc);

    Serial.print("T1= ");
    Serial.print(T1);
    Serial.print("/");

    Serial.print("T2= ");
    Serial.print(T2);
    Serial.print("/");

    Serial.print("T3= ");
    Serial.print(T3);
    Serial.print("/");

    Serial.print("T4= ");
    Serial.print(T4);
    Serial.print("/");

    Serial.print("T5= ");
    Serial.print(T5);
    Serial.print("/");

    Serial.print("T6= ");
    Serial.print(T6);
    Serial.print("/");

    Serial.print("T7= ");
    Serial.print(T7);
    Serial.print("/");

    Serial.print("T8= ");
    Serial.print(T8);
    Serial.print("/");

    Serial.print("T9= ");
    Serial.print(T9);
    Serial.print("/");

    Serial.print("T10= ");
    Serial.print(T10);
    Serial.print("/");

    Serial.print("ST1= ");
    Serial.print(ST1);
    Serial.print("/");

    Serial.print("ST2= ");
    Serial.print(ST2);
    Serial.print("/");

    Serial.print("ST3= ");
    Serial.print(ST3);
    Serial.print("/");

    Serial.print("ST4= ");
    Serial.print(ST4);
    Serial.print("/");

    Serial.print("ST5= ");
    Serial.print(ST5);
    Serial.print("/");

    Serial.print("ST6= ");
    Serial.print(ST6);
    Serial.print("/");

    Serial.print("ST7= ");
    Serial.print(ST7);
    Serial.print("/");

    Serial.print("DN70]= ");
    Serial.print(TDN[70]);
    Serial.print("/");

    Serial.print("DN72]= ");
    Serial.print(TDN[72]);
    Serial.print("/");

    Serial.print("DN71]= ");
    Serial.print(TDN[71]);
    Serial.print("/");


    for (int i = 23; i <= 35; i = i + 2) {
      Serial.print("IN");
      Serial.print(i);
      Serial.print("=");
      Serial.print(digitalRead(i));
      Serial.print("/");
    }
    for (int i = 37; i <= 52; i = i + 1) {
      Serial.print("IN");
      Serial.print(i);
      Serial.print("=");
      Serial.print(digitalRead(i));
      Serial.print("/");
    }
    for (int i = 209; i <= 224; i = i + 1) {
      Serial.print("OUT");
      Serial.print(i);
      Serial.print("=");
      Serial.print(B3[i]);
      Serial.print("/");
    }
    for (int i = 0; i <= 15; i = i + 1) {
      Serial.print("AI");
      Serial.print(i);
      Serial.print("=");
      Serial.print(analogRead(i));
      Serial.print("/");
    }

    Serial.print("T.BATT1= ");
    Serial.print(F8[69]);
    Serial.print("/");

    Serial.print("I.BATT1= ");
    Serial.print(F8[52]);
    Serial.print("/");

    Serial.print("I.MPPT1= ");
    Serial.print(F8[187]);
    Serial.print("/");

    Serial.print("I.MPPT2= ");
    Serial.print(F8[177]);
    Serial.print("/");

    Serial.print("I.CHARG1= ");
    Serial.print(F8[207]);
    Serial.print("/");

    Serial.print("I.CHARG2= ");
    Serial.print(F8[197]);
    Serial.print("/");

    Serial.print("I.ONDUL1= ");
    Serial.print(F8[167]);
    Serial.print("/");

    Serial.print("I.ONDUL2= ");
    Serial.print(F8[157]);
    Serial.print("/");


    Serial.print("I.BATT2= ");
    Serial.print(F8[247]);
    Serial.print("/");

    Serial.print("I N7:60= ");
    Serial.print(N7M60);
    Serial.print("/");

    Serial.print("I F8:161= ");
    Serial.print(F8M161);
    Serial.print("/");

    Serial.print("I F8:132= ");
    Serial.print(F8M132);
    Serial.print("/");

    Serial.print("I F8:125= ");
    Serial.print(F8M125);
    Serial.print("/");

    Serial.print("I F8:105= ");
    Serial.print(F8M105);
    Serial.print("/");

    dtostrf( TDN[IndexDebugTDProc], 1, 0, charVal);
    TramProc = charVal;
    TramProc = TramProc + "/";
    Serial.print(TramProc);

    dtostrf( TDN[IndexDebugTTProc], 1, 0, charVal);
    TramProc = charVal;
    TramProc = TramProc + "/";
    Serial.print(TramProc);

    Serial.print("IN22= ");
    Serial.print(digitalRead(22));
    Serial.print("/");

    Serial.print("TDN[70]= ");
    Serial.print(TDN[70]);
    Serial.print("/");

    dtostrf( N7[90], 1, 0, charVal);
    TramProc = charVal;
    TramProc = TramProc + "/";
    Serial.print(TramProc);

    AffPointeur(Pointeur);

    Serial.println();

  }
}
