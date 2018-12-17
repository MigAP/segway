
// Eviter des erreurs de compilation 
namespace std
{
  void __throw_bad_function_call(){
    Serial.println("STL ERROR - HALT NOW");
    //return;
    }
}

//IMU
#include <CurieIMU.h>
#include <BMI160.h>
#include <MadgwickAHRS.h>
Madgwick filter;
float accelScale, gyroScale;
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float pitch; //roll, pitch, heading;   //pitch=inclinaison
//unsigned long microsNow;

//GESTION TIMER

#define f_ech 800
#include "CurieTimerOne.h"
int microSecondsTimer; 

//SimpleTimer timer_asserv;

// Driver moteur
#define ena 5  //puissance moteur1
#define dir1 3 //sens moteur1
#define enb 6   //puissance moteur2
#define dir2 4   //sens moteur2
#define buz 9 
#define joystick 11 

//ASSERVISSMENT
// const int relayStatus = 7; // relay status qui permet d'envoyer ou pas la commande 
float kp = 35; //30; // coef proportionnel
float ki = 0.001;//0.01;  //coef intégrateur
float kd = 20;//5;   // coef dérivateur
float k_gy = 0; //0.4;  //coef de correction relatif à la vitesse angulaire d'inclinaison : à supprimer ?
float k_gz = 50;    //coef relatif à la différence de vitesse des moteurs --> pour tourner

#define P_MAX 255 //limitation de la puissance Max (max etant 255)
#define pitch_max 20//Angle max autorisé; Au dela, le robot est consideré comme en chute

float err = 0;
float consigne = -3.8;   
float consigne_roll = 0;   //pour pouvoir tourner

float somme_err = 0;
float delta_err = 0;
float err_precedente = 0;
float cmd_m1 = 0;
float cmd_m2 = 0;

void asservissement(); 
void PID_ANGLE();

//DRIVER CONTROL
void moteur1(unsigned int pwr, boolean sens);
void moteur2(unsigned int pwr, boolean sens);
void moteur_off();

//IMU CONTROL
float convertRawAcceleration(int aRaw); // conversion des données brutes de l'accéléromètre+gyro
float convertRawGyro(int gRaw) ;

//COMMUNICATION   
//BLE CONFIG 
#include <CurieBLE.h>

BLEPeripheral blePeripheral;
const int dataAmount = 5; // number of bytes we are sending through BLE for the IMU 

char asciiBuffer[9]; // buffer to convert float numbers to ASCII code

//Buffer to hold "dataAmount" bytes
unsigned char  bleData[ dataAmount ];
unsigned char pidData[3] {kp,ki,kd}; 

// create a custom bluetooth service that will have two characteristic 
BLEService customService("19B10010-E8F2-537E-4F6C-D104768A1216");
BLECharacteristic sensorCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify,dataAmount);
BLECharacteristic pidCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite,3);

//=============================================================================
//                        SETUP
//=============================================================================
void setup() {
  //Serial.begin(9600); 
  //BLE CONFIG 
  blePeripheral.setLocalName("Segway");
  blePeripheral.setDeviceName("Segway"); 
  blePeripheral.setAdvertisedServiceUuid(customService.uuid());
  blePeripheral.addAttribute(customService);
  blePeripheral.addAttribute(sensorCharacteristic);
  blePeripheral.addAttribute(pidCharacteristic); 
  

  sensorCharacteristic.setValue(bleData,dataAmount); // Initialisation des characteristiques 
  pidCharacteristic.setValue(pidData,3); // Initialisation des characteristiques 

  blePeripheral.begin();

  //CONFIG DRIVER
  pinMode(ena, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(enb, OUTPUT);
  pinMode(dir2, OUTPUT);
  pinMode(buz, OUTPUT); 

  moteur_off();
  pinMode(13, OUTPUT);

  //CONFIG IMU
  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(f_ech);   //détermine la fréquence d'échantillonnage du gyro. Le programme reçoit une donnée tous les 1/f_ech.
  CurieIMU.setAccelerometerRate(f_ech); //de même pour l'accéléromètre
  filter.begin(f_ech);   //lancement du filtrage à la même freq

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // GESTION DES TIMERS
  //timer_asserv.setInterval(1000 / f_ech, asservissement); //Timer d'asservissement
  microSecondsTimer = (1.0/f_ech)*1000000; 
  CurieTimerOne.start(microSecondsTimer, &asservissement);
  
}

//=============================================================================
//                        MAIN LOOP
//=============================================================================

void loop() {
 
  BLEDevice central = BLE.central(); // BLE central connected to peripheral 

  if(central){

    while (central.connected()){

      //Programme principale
      //timer_asserv.run();
      

      // Si l'utilisateur modifie la valeur du PID on met a jour les valeur des correcteurs 
      if(pidCharacteristic.written()){ 
        kp = pidCharacteristic.value()[0]; 
        ki = pidCharacteristic.value()[1]/100.0; 
        kd = pidCharacteristic.value()[2]; 
        //Serial.println("kp "+String(kp)+ "ki "+String(ki)+ "kd "+String(kd)); 
      }  
    }
  }
  joystickValue = analogRead(joystick);
  consigne_roll = (joystickValue - 512)/512; //[0,1023] -> [-1,1]
  delay(10); 
}



