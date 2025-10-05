#include <Arduino.h>
#include <librobus.h>
int parcours[3][10];        //Déclaration du tableau pour le parcours
int position_actuelle_range;
int pinCapteurGauche = 39; // DESC: branchements de la del de gauche sur l'arduino
int pinCapteurDroit = 45; // DESC: branchement de la del de droite sur l'arduino
int position_actuelle_colonne;
bool objet_detecte;
int etape = 0;
int rotation_demander;    //1 = gauche, 2 = droite, 0 = aucune
int seuilProx = 500;      //false = pas d'object devant, true = objet devant
bool detectionObject = false; 
bool rotation_droite_fait;
bool rotation_gauche_fait;
bool avancer_demander;
bool deplacement_terminer;
bool deplacement_terminer_rotation;
int32_t total_encodeur_gauche;
int32_t total_encodeur_droit;
int debut_deplacement;
void ENCODER_Reset(uint8_t id);
float speed; 
const float diametre = 7.62; // en cm
const int pulses_par_tour = 3200;
const float distance_parcourue = 50; // en cm
int32_t ENCODER_Read(uint8_t id);
void MOTOR_SetSpeed(uint8_t id, float speed);
int pulses_necessaires;
const int rayon = 9.6; //en cm rayon du robot
const int angle = 90; // en degre
int32_t compteur_encodeur_droit;
int32_t compteur_encodeur_gauche;
bool test;
uint32_t StartDecelerationTime;
uint32_t LastDecelerationTime;
uint32_t last_sample_time = 0;
// Speed calculation variables
float motor_0_rpm = 0.0;
float motor_1_rpm = 0.0;
int nb_rotation;
// PID parameters for motor 0
float kp_0  = 0.008;
float ki_0  = 0.002250;//0.00288; //0.00825f;
float kd_0  = 0;//0.00001;//0.00007f;  //0.00001f

static float prev_error_0 = 0.0f;
static float integral_0   = 0.0f;

// PID parameters for motor 1
float kp_1 = 0.0099;  //0.007f
float ki_1 = 0.00225;//0.0075f; //0.0025f
float kd_1 = 0.0000013;//0.000013; //0.00001f
float P0;
float I0;
float D0;
float output0;
float P1;
float I1;
float D1;
float output1;
float error0;
float error1;
float target_rpm_motor0;
float target_rpm_motor1;
uint32_t elapsed;
int32_t pulses_0;
int32_t pulses_1;
float power_0;
float power_1;
static float prev_error_1 = 0.0f;
static float integral_1   = 0.0f;
int dernier_rotation;
float targetRPM_afterdown;
// moving
unsigned long moveStartTime = 0;
bool movingForward = false;


void position(int, int);
void ReglerPositionParcours(int, int);
bool fonctionDetectionObjet(int, int);


void setup() {
  BoardInit();

  ENCODER_Reset(0);
  ENCODER_Reset(1);

  last_sample_time = millis();
  pinMode(pinCapteurGauche, INPUT);
  pinMode(pinCapteurDroit, INPUT);
  ReglerPositionParcours(1,0);
}


float calculateRPM(int32_t pulse_count, float time_interval_seconds) {
  return (pulse_count * 60.0f) / (time_interval_seconds * 3200);
}

float pidControllerMotor0(float setpoint, float measured, float dt) {
  error0 = setpoint - measured;
  P0 = kp_0 * error0;
  integral_0 += error0 * dt;
  I0 = ki_0 * integral_0;
  D0 = kd_0 * ((error0 - prev_error_0) / dt);
  prev_error_0 = error0;
  output0 = P0 + I0 + D0;
  return output0;
}
float pidControllerMotor1(float setpoint, float measured, float dt) {
  error1 = setpoint - measured;
  P1 = kp_1 * error1;
  integral_1 += error1 * dt;
  I1 = ki_1 * integral_1;
  D1 = kd_1 * ((error1 - prev_error_1) / dt);
  prev_error_1 = error1;
  output1 = P1 + I1 + D1;
  return output1;
}
void FinMouvement(){
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  deplacement_terminer = 1;
  compteur_encodeur_droit = 0;
  compteur_encodeur_gauche = 0;
  target_rpm_motor0=0;
  target_rpm_motor1=0;
  power_0=0;
  power_1=0;
  debut_deplacement = 0;
  power_0=0;
  power_1=0;
  integral_0=0;
  error0=0;
  I0=0;
  prev_error_0=0;
  integral_1=0;
  error1=0;
  I1=0;
  prev_error_1=0;
  motor_1_rpm = 0;
  motor_0_rpm = 0;
  movingForward=false;
}

bool fonctionDetectionObjet(int pinCapteurGauche, int pinCapteurDroit) { 
  int valeurCapteurGauche = digitalRead(pinCapteurGauche); 
  int valeurCapteurDroit = digitalRead(pinCapteurDroit);
    if (valeurCapteurGauche != 1 || valeurCapteurDroit != 1) {
      return true;
    }
    else {
      return false;
    }
}
// minimize the oscillation at beginning
float rampTargetRPM(float targetRPM, uint32_t startTime, uint32_t currentTime) {
  elapsed = currentTime - startTime;
  float rampTime = 750; // 1 seconds ramp
  if (elapsed >= rampTime){  
    return targetRPM;
  } 
  return targetRPM * (elapsed / rampTime);
}

float downTargetRPM(float targetRPM, int32_t compteur_encodeur) {
  float deac = 2.5;
  targetRPM = targetRPM - deac;
  if(targetRPM<=5){
    return 5;
  }else{
    return (targetRPM = targetRPM - deac);
  }
}


void avancer(float vitesse_gauche, float vitesse_droite){
  if(debut_deplacement == 0){
    uint32_t current_time = millis();
    if (current_time - last_sample_time >= 30) {
      if (!movingForward) {
        movingForward = true;
        motor_0_rpm = 0;
        motor_1_rpm=0;
        moveStartTime = current_time;
      }
      if (movingForward) {

        float dt = (current_time - last_sample_time) / 1000.0f;

        // Update encoder values
        pulses_0  = ENCODER_Read(0);
        pulses_1 = ENCODER_Read(1);
        compteur_encodeur_gauche += pulses_0;
        compteur_encodeur_droit += pulses_1;
        total_encodeur_gauche += pulses_0;
        total_encodeur_droit += pulses_1;
        ENCODER_Reset(0);
        ENCODER_Reset(1);
        
        motor_0_rpm = calculateRPM(pulses_0, dt);
        motor_1_rpm = calculateRPM(pulses_1, dt);

        // Calculate PID outputs
        if(compteur_encodeur_gauche < 4500){
          target_rpm_motor0 = rampTargetRPM(vitesse_gauche, moveStartTime, millis());
          target_rpm_motor1 = rampTargetRPM(vitesse_droite, moveStartTime, millis());
        }else{
          target_rpm_motor0 = downTargetRPM(target_rpm_motor0, compteur_encodeur_gauche);
          target_rpm_motor1 = downTargetRPM(target_rpm_motor1, compteur_encodeur_droit);
        }

        power_0  = pidControllerMotor0(target_rpm_motor0, motor_0_rpm, dt);
        power_1 = pidControllerMotor1(target_rpm_motor1, motor_1_rpm, dt);

        MOTOR_SetSpeed(0, power_0);
        MOTOR_SetSpeed(1, power_1);
        last_sample_time = current_time;
      } 
    }
  }
}

void LigneDroite(float vitesse_gauche, float vitesse_droite){
  avancer(130, 128.5);
  pulses_necessaires = ((pulses_par_tour*distance_parcourue)/ (PI*diametre));  // pour parcourir notre distance de 50 cm
  if (compteur_encodeur_gauche >= pulses_necessaires){
    if (rotation_droite_fait){
      position_actuelle_colonne++;
    }
    if (rotation_gauche_fait){
      position_actuelle_colonne--;
    }
    if(!rotation_droite_fait & !rotation_gauche_fait){
      position_actuelle_range ++;
    }
    
    deplacement_terminer = 1;

    if(deplacement_terminer == 1){
      objet_detecte = fonctionDetectionObjet(pinCapteurGauche, pinCapteurDroit);
        FinMouvement();      
    }
  }
}

void TournerDroite(){
  pulses_necessaires = 1935;// Pulses de l'encodeur//(rayon * angle); // pour tourner a 90 degre
  if(debut_deplacement ==1){
    ENCODER_Reset(0); // encodeur gauche  //Vérifier si on peut l'enlever
    ENCODER_Reset(1); // encodeur droit   //Vérifier si on peut l'enlever
    debut_deplacement = 0;
  }
  avancer(55, -55);
  if (abs(compteur_encodeur_gauche) >= pulses_necessaires){
    FinMouvement();
  }
}

void TournerGauche(){
  pulses_necessaires = 1876;// Pulses de l'encodeur //(rayon * angle); // pour tourner a 90 degre
  if (debut_deplacement == 1){
    ENCODER_Reset(0); // encodeur gauche  //Vérifier si on peut l'enlever
    ENCODER_Reset(1); // encodeur droit   //Vérifier si on peut l'enlever
    debut_deplacement=0;
  }
  avancer(-55, 56.2);
  if (abs(compteur_encodeur_gauche) >= pulses_necessaires){
    FinMouvement();
  }

}
void loop() {
  
  //Étape initiale
  if(etape == 0){
    if (ROBUS_IsBumper(3)) {
      etape = 5;
      //float kp_1 = 0.0063f;  //0.007f
    }
  }

  if (etape == 1){
    if(deplacement_terminer == 0){
      if((rotation_droite_fait == 1) & (position_actuelle_colonne == 2)){

        etape=2;
      }else if((rotation_gauche_fait == 1) & (position_actuelle_colonne == 0)){
        etape=3;
      }
      else if(position_actuelle_range == 9){
        etape = 0;
      }
      else{
        nb_rotation = 0;
        LigneDroite(40, 40);

      }
    }
    if(deplacement_terminer == 1){
      etape = 5;
      deplacement_terminer = 0;    
    }
  }
  if(etape == 2){

      //Vérifier que nous ne somme pas au bout du labyrinthe
    //kd_1 = 0;
    //kp_1=0.0067;
    TournerGauche();
    if(deplacement_terminer == 1){
        
      nb_rotation++;
      etape = 5;

      deplacement_terminer = 0;
      if(rotation_droite_fait){
        rotation_droite_fait = 0;
        dernier_rotation = 2;
      }else{
        rotation_gauche_fait =1;
      }
    }
      
  }
  if(etape == 3){

    //Vérifier que nous ne somme pas au bout du labyrinthe
    //kd_1 = 0;
    //kp_1=0.0067;
    TournerDroite();
    if(deplacement_terminer == 1){
        
      nb_rotation++;
      etape = 5;
        deplacement_terminer = 0;
      if(rotation_gauche_fait){
        rotation_gauche_fait = 0;
        dernier_rotation = 1;
      }else{
        rotation_droite_fait =1;
      }

    }
    
  } 
  if (etape == 5){ //object détecté

    objet_detecte = fonctionDetectionObjet(pinCapteurGauche, pinCapteurDroit);
    if (objet_detecte == 1){
      etape=10;
    }else{
        etape = 1;
    }
  }
    //Un objet est détecté à l'avant
  if (etape ==10){
    if (parcours[position_actuelle_colonne][position_actuelle_range] == 2){
      //La cellule a déjà été vérifiée, alors passer à la prochaine cellule
      etape=50;
    }else{
      //Marqué la position actuelle comme vérifié 
      parcours[position_actuelle_colonne][position_actuelle_range] = 2;
    }
    //Si la colonne est la première
    if (position_actuelle_colonne == 0){
      etape=20;
    }
      //Si la colonne est la deuxième ou la troisième
    if(position_actuelle_colonne == 1 or position_actuelle_colonne ==2){
      etape=30;
    }

    //test
    if(position_actuelle_colonne == 2){
      etape = 2;
    }

    if(position_actuelle_colonne == 0){
      etape = 3;
    }
  }
    //La présente case est la première et qu'un objet est détecté en avant
  if(etape ==20){
    etape = 3;
  }
  if(etape ==30){
    if(position_actuelle_colonne == 1 & nb_rotation != 2){
      if (rotation_gauche_fait){
        etape = 3;       
      } else if(rotation_droite_fait){
        etape = 2;
      }
    }
    if(((position_actuelle_colonne == 2 or position_actuelle_colonne == 1) & (!rotation_gauche_fait & !rotation_droite_fait)) & (nb_rotation != 2)){
      etape =2;
    }
    if (nb_rotation == 2){
      if(dernier_rotation == 1){
        etape = 3;
      }
      if(dernier_rotation == 2){
        etape = 2;
      }
    }    
  }
}

void ReglerPositionParcours(int colonne, int range){
  position_actuelle_range = range;
  position_actuelle_colonne = colonne;
}