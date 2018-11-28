
//=============================================================================
//                        DEFINITION DES FONCTIONS CONTROLE MOTEUR
//=============================================================================


//moteur1 : moteur gauche ?
//moteur2: moteur droit ?

void moteur1(unsigned int pwr, boolean sens) {
  if (sens) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  }
  else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  }

  analogWrite(ena, pwr);
}

void moteur2(unsigned int pwr, boolean sens) {
  if (!sens) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  }
  else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  }

  analogWrite(enb, pwr);
}

void moteur_off() {
  analogWrite(ena, 0);
  analogWrite(enb, 0);
}
