//=============================================================================
//                        DEFINITION DES FONCTIONS
//=============================================================================

//Les 2 fonctions suivantes sont liées au fonctionnement du gyro et assure la conversion des données brutes gyro + accéléromètre

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}



//Mise en place du correcteur PID

void PID_ANGLE() {
  err = consigne - pitch; // calcul de l'erreur sur l'angle d'équilibre    pitch=angle d'inclinaison mesuré     
  delta_err = err - err_precedente; // calcul de la dérivée de l'erreur sur l'angle d'équilibre
  somme_err = somme_err + err;  //Calcul de l'integrale de l'erreur
  err_precedente = err;

  float cmd = kp * err + ki * somme_err + kd * delta_err - k_gy * gy; // calcul du corrceteur PID    //k-gy est un coef   gy est la vitesse angulaire d'inclinaison

  cmd_m1 = cmd + k_gz * gz + consigne_roll;     // Elaboration de la commande de la roue gauche       //consign-roll permet de tourner et permet une vitesse de roue différentielle. consigne_roll sera déterminé grâce au joystick.
  cmd_m2 = cmd - k_gz * gz - consigne_roll; // Elaboration de la commande de la roue droite      //k-gz est un coef      gz est la vitesse angulaire relative au changement de direction du seg

  cmd_m1 = constrain(cmd_m1, -P_MAX, P_MAX);    //Saturation de la commande
  cmd_m2 = constrain(cmd_m2, -P_MAX, P_MAX);  //Saturation de la commande

  

  //Contrôle des moteurs, envoi de la commande au driver
  
  if (abs(pitch) < pitch_max) { // Controle de securité : verification en cas de chute --> couper les moteurs

    if (cmd_m1 >= 0) {
      moteur1(abs(cmd_m1), 1);

    }
    else {
      moteur1(abs(cmd_m1), 0);

    }

    if (cmd_m2 >= 0) {
      moteur2(abs(cmd_m2), 1);
    }
    else {
      moteur2(abs(cmd_m2), 0);
    }
    
  }
  else {
    moteur_off(); //Si pitch > ptich_max -> arret des moteurs / securité
    somme_err = 0; //remise à zero de la somme des erreurs pour empecher une reprise erronnée de la commande
    delta_err = 0; //idem
  }
  
}


//Fonction d'asservissement
void asservissement() {

  // Lecture de la centrale Inertielle
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  // Conversion en metre/s2 des données brutes de l'acceleromètre
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);

  //Conversion en degrees/second des données brutes du gyroscope
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);

  filter.updateIMU(gx, gy, gz, ax, ay, az); //Mise à jour du filtre avec les nouvelle valeures
  pitch = filter.getPitch(); // Mise à jour de l'angle d'inclinaison
  //Serial.println(pitch); 
  // Mise a jour de la caracteristique BLE 

  // Convert the filter results to ASCII code for bluetooth comunication 
  snprintf(asciiBuffer, sizeof asciiBuffer, "%f", pitch);
  bleData[0] = asciiBuffer[0];
  bleData[1] = asciiBuffer[1] ;
  bleData[2] = asciiBuffer[2]; 
  bleData[3] = asciiBuffer[3]; 

  bleData[4] = '/'; 
  sensorCharacteristic.setValue(bleData,dataAmount); 

  if(digitalRead(relayStatus)){
    PID_ANGLE(); // Appel de la fonction d'asservissement
  }
  
}


