
//=============================================================================
//                        DEFINITION DES FONCTIONS CONTROLE MOTEUR
//=============================================================================


//moteur1 : moteur gauche ?
//moteur2: moteur droit ?

void moteur1(unsigned int pwr, boolean sens) {
  if (sens) {
    digitalWrite(dir1, HIGH);
  }
  else {
    digitalWrite(dir1, LOW);
  }
  //Serial.println("cmd_m1 " + String(pwr) + " Sens "+String(sens)); 
  analogWrite(ena, pwr);
}

void moteur2(unsigned int pwr, boolean sens) {
  if (!sens) {
    digitalWrite(dir2, HIGH);
  }
  else {
    digitalWrite(dir2, LOW);
  }
  //Serial.println("cmd_m2 " + String(pwr) + " Sens "+String(sens) ); 
  analogWrite(enb, pwr);
}

void moteur_off() {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
}
