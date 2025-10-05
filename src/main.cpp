#include <Arduino.h>
#include <librobus.h>

/**************************************
 *  DÉFINIR LES FONCTIONS
 **************************************/
void etat_attente();                // Attend que le robot soit lancé
void etat_avancer();                // Avance dans le labyrinthe
void etat_tourner_gauche();         // Effectue un virage à gauche
void etat_tourner_droite();         // Effectue un virage à droite
void etat_verif_obstacle();         // Vérifie présence d'obstacle devant
void etat_obstacle_detecte();       // Gère la présence d'un obstacle
void etat_premiere_colonne();       // Spécifique à la case de gauche
void etat_choix_rotation();     // Gère les cases du centre et droite
void etat_cellule_verifiee();       // Si la cellule a déjà été traitée

/****************************************
 *       VARIABLES PRINCIPALES          
 ****************************************/
int parcours[3][10];           // Tableau pour mémoriser les cases déjà explorées (3 colonnes x 10 lignes)
int rang_actuel;               // Ligne actuelle du robot dans le labyrinthe
int colonne_actuelle;          // Colonne actuelle du robot dans le labyrinthe

int pin_capteur_gauche = 39;   // Numéro de broche Arduino pour le capteur gauche (de détection d'obstacle)
int pin_capteur_droit = 45;    // Numéro de broche Arduino pour le capteur droit

bool obstacle_detecte;         // Stocke le fait qu'un obstacle est devant le robot
int etat = 0;                  // L'état courant de la machine à états (voir enum plus bas)
int rotation_demande;          // Indique si une rotation est demandée (1=gauche, 2=droite, 0=aucune)
bool rotation_droite_fait;     // Vrai si une rotation à droite vient d'être effectuée
bool rotation_gauche_fait;     // Vrai si une rotation à gauche vient d'être effectuée
bool deplacement_termine;      // Indique si l'action/mouvement du robot est terminée
int32_t total_encodeur_gauche;
int32_t total_encodeur_droit; 
int debut_deplacement;         // Indicateur pour savoir si on commence un nouveau déplacement

// Paramètres physiques du robot
const float diametre_roue = 7.62;    // Diamètre d'une roue en cm
const int pulses_par_tour = 3200;
const float distance_a_parcourir = 50;  // Distance à parcourir sur chaque ligne en cm

// Variables techniques pour le pilotage
int pulses_necessaires;              // Nombre de pulses nécessaires pour parcourir une case
int32_t cpt_encodeur_droit, cpt_encodeur_gauche;
float vitesse_moteur_0 = 0.0, vitesse_moteur_1 = 0.0;
int nb_rotations;                    // Nombre de rotations effectuées (utile pour la logique du labyrinthe)

// Paramètres et états du PID pour le contrôle moteur
float kp_0  = 0.008, ki_0  = 0.00225, kd_0  = 0;
static float erreur_prec_0 = 0.0f, integrale_0 = 0.0f;

float kp_1 = 0.0099, ki_1 = 0.00225, kd_1 = 0.0000013;
static float erreur_prec_1 = 0.0f, integrale_1 = 0.0f;

float prop_0, inte_0, deri_0, commande_0, err_0;
float prop_1, inte_1, deri_1, commande_1, err_1;
float cible_rpm_moteur_0, cible_rpm_moteur_1;
float puissance_0, puissance_1;
uint32_t der_temps_ech = 0;
uint32_t temps_ecoule;
int32_t pulses_0, pulses_1;
int derniere_rotation;                 // Indique si la dernière rotation était à droite(2) ou à gauche(1)
unsigned long debut_mouvement = 0;
bool en_mouvement = false;          // Vrai si une commande d’avance est en cours
uint32_t dernier_echantillon = 0;
float cible_rpm_apres_desc = 0.0f;
unsigned long temps_debut_deplacement = 0;
bool avancer_demande;
bool detection_objet = false;

/*******************************************
 *     array(list en python) pour état   
 *******************************************/
enum Etats {
  E_ATTENTE = 0,
  E_AVANCER = 1,
  E_TOURNER_GAUCHE = 2,
  E_TOURNER_DROITE = 3,
  E_VERIF_OBSTACLE = 5,
  E_OBSTACLE_DETECTE = 10,
  E_CHOIX_ROTATION = 30,
  E_CELLULE_VERIFIEE = 50,
  E_NB_ETATS = 51           // Nombre total d’états connus
};

// function pointer en C++
typedef void (*GestionnaireEtat)();
GestionnaireEtat gestionnaires_etat[E_NB_ETATS] = {
  etat_attente,            // 0 : Attente du bouton
  etat_avancer,            // 1 : Avancer dans une case
  etat_tourner_gauche,     // 2 : Tourner gauche
  etat_tourner_droite,     // 3 : Tourner droite
  nullptr,                 // 4 : null
  etat_verif_obstacle,     // 5 : Vérifier obstacle
  nullptr, nullptr, nullptr, nullptr, // 6-9
  etat_obstacle_detecte,   // 10 : Gestion obstacle
  nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, // 11-19
  nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, // 20-29
  etat_choix_rotation,     // 30 : Logique choix rotation colonne
  nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, // 31-39
  nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, // 40-49
  etat_cellule_verifiee    // 50 : Cellule déjà vérifiée
};


/***************************************************
 *          FONCTIONS DE CALCUL DE BASE
 ***************************************************/

float calculerRPM(int32_t nb_pulses, float intervalle_sec) {
  // nb_pulses * 60s / deltaT * 3200 pulses
  return (nb_pulses * 60.0f) / (intervalle_sec * pulses_par_tour);
}

// PID pour le moteur gauche
float PID_0(float vitesse_souhaitee, float vitesse_reel, float dt) { 
  err_0 = vitesse_souhaitee - vitesse_reel;
  prop_0 = kp_0 * err_0;
  integrale_0 += err_0 * dt;
  inte_0 = ki_0 * integrale_0;
  deri_0 = kd_0 * ((err_0 - erreur_prec_0) / dt);
  erreur_prec_0 = err_0;
  commande_0 = prop_0 + inte_0 + deri_0;
  return commande_0;
}

// PID pour le moteur droit
float PID_1(float vitesse_souhaitee, float vitesse_reel, float dt) {
  err_1 = vitesse_souhaitee - vitesse_reel;
  prop_1 = kp_1 * err_1;
  integrale_1 += err_1 * dt;
  inte_1 = ki_1 * integrale_1;
  deri_1 = kd_1 * ((err_1 - erreur_prec_1) / dt);
  erreur_prec_1 = err_1;
  commande_1 = prop_1 + inte_1 + deri_1;
  return commande_1;
}

// Arrête le mouvement et réinitialise toutes les variables de déplacement
void FinDeplacement() {
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  deplacement_termine = true;
  cpt_encodeur_droit = 0;
  cpt_encodeur_gauche = 0;
  cible_rpm_moteur_0 = 0;
  cible_rpm_moteur_1 = 0;
  puissance_0 = puissance_1 = 0;
  debut_deplacement = 0;
  integrale_0 = err_0 = inte_0 = erreur_prec_0 = 0;
  integrale_1 = err_1 = inte_1 = erreur_prec_1 = 0;
  vitesse_moteur_1 = 0;
  vitesse_moteur_0 = 0;
  en_mouvement = false;
}

// Renvoie true si au moins un capteur voit un obstacle devant
bool fonctionDetectionObjet(int broche_g, int broche_d) {
  int valeur_captuer_g = digitalRead(broche_g);
  int valeur_capteur_d = digitalRead(broche_d);
  return (valeur_captuer_g != 1 || valeur_capteur_d != 1);
}

float accelerationRPM(float RPM_souhaitee, uint32_t debut, uint32_t actuel) {
  uint32_t ecoule = actuel - debut;
  float rampe_ms = 750;       // temps d'acceleration
  if (ecoule >= rampe_ms) {   // Si le temps écoulé est plus grand que la rampe, on retourn RPM_souhaitee
    return RPM_souhaitee;
  } else {                    // Sinon, on augmente progressivement la vitesse
    return RPM_souhaitee * (float(ecoule) / rampe_ms);
  }
}

float decelererConsigneRPM(float RPM_souhaitee) {
  float decelerer = 2.5;  // coefficient de décélération

  // On réduit la vitesse souhaitée
  RPM_souhaitee -= decelerer;
  
  if (RPM_souhaitee <= 5) {   // RPM minimum 5
    return 5;
  } else {
    return RPM_souhaitee;
  }
}


/*************************************************
 *                MOUVEMENTS DE BASE
 *************************************************/

// donnée la vitesse(+, -) pour bouger le robot
void bouger(float vitesse_gauche, float vitesse_droite) {
  if (debut_deplacement == 0) {
    uint32_t maintenant = millis();
    if (maintenant - dernier_echantillon >= 30) {
      if (!en_mouvement) {
        en_mouvement = true;
        vitesse_moteur_0 = vitesse_moteur_1 = 0;
        debut_mouvement = maintenant;
      }
      if (en_mouvement) {
        float dt = (maintenant - dernier_echantillon) / 1000.0f;
        pulses_0 = ENCODER_Read(0);
        pulses_1 = ENCODER_Read(1);

        cpt_encodeur_gauche += pulses_0;
        cpt_encodeur_droit += pulses_1;

        total_encodeur_gauche += pulses_0;
        total_encodeur_droit += pulses_1;

        ENCODER_Reset(0);
        ENCODER_Reset(1);

        vitesse_moteur_0 = calculerRPM(pulses_0, dt);
        vitesse_moteur_1 = calculerRPM(pulses_1, dt);

        if (cpt_encodeur_gauche < 4500) {
          cible_rpm_moteur_0 = accelerationRPM(vitesse_gauche, debut_mouvement, millis());
          cible_rpm_moteur_1 = accelerationRPM(vitesse_droite, debut_mouvement, millis());
        } else {
          cible_rpm_moteur_0 = decelererConsigneRPM(cible_rpm_moteur_0);
          cible_rpm_moteur_1 = decelererConsigneRPM(cible_rpm_moteur_1);
        }
        puissance_0 = PID_0(cible_rpm_moteur_0, vitesse_moteur_0, dt);
        puissance_1 = PID_1(cible_rpm_moteur_1, vitesse_moteur_1, dt);
        MOTOR_SetSpeed(0, puissance_0);
        MOTOR_SetSpeed(1, puissance_1);
        dernier_echantillon = maintenant;
      }
    }
  }
}

// Avance sur une ligne droite de 50 cm
void avanceEnLigne(float v_g, float v_d) {
  bouger(130, 128.5);    // PARAM: RPM_souhaitee
  
  // calcul pour parcourir notre distance de 50 cm
  pulses_necessaires = ((pulses_par_tour*distance_a_parcourir)/(PI*diametre_roue));
  
  if (cpt_encodeur_gauche >= pulses_necessaires) {
    // mis a jour sur sa position
    if (rotation_droite_fait) colonne_actuelle++;
    if (rotation_gauche_fait) colonne_actuelle--;
    if(!rotation_droite_fait && !rotation_gauche_fait) rang_actuel++;

    deplacement_termine = 1;
    if(deplacement_termine) {
      obstacle_detecte = fonctionDetectionObjet(pin_capteur_gauche, pin_capteur_droit);
      FinDeplacement();
    }
  }
}

// Effectue un virage sur place à droite
void tournerDroite(){
  pulses_necessaires = 1935;
  if(debut_deplacement == 1){
    ENCODER_Reset(0); ENCODER_Reset(1); debut_deplacement = 0;
  }
  bouger(55, -55);
  if (abs(cpt_encodeur_gauche) >= pulses_necessaires) FinDeplacement();
}

// Effectue un virage sur place à gauche
void tournerGauche(){
  pulses_necessaires = 1876;
  if (debut_deplacement == 1){
    ENCODER_Reset(0); ENCODER_Reset(1); debut_deplacement=0;
  }
  bouger(-55, 56.2);
  if (abs(cpt_encodeur_gauche) >= pulses_necessaires) FinDeplacement();
}

// Initialise la position du robot au démarrage (void setup)
void reglerPositionParcours(int colonne, int range){
  rang_actuel = range;
  colonne_actuelle = colonne;
}


/*****************************************
 *   HANDLERS POUR CHAQUE ÉTAT DU ROBOT
 *****************************************/
/* E_ATTENTE
Attente du bouton de démarrage pour lancer le parcours
*/
void etat_attente() {
  // doit changer pour ROBUS_IsBumper(3) pour le sifflet
  if (ROBUS_IsBumper(3)) etat = E_VERIF_OBSTACLE;
}

/* E_AVANCER
Avance dans le labyrinthe tant que pas de détection ou fin de ligne
*/
void etat_avancer() {
  if(!deplacement_termine){

    // les cas des deux colonnes aux extrémités
    if((rotation_droite_fait) && (colonne_actuelle == 2)) etat=E_TOURNER_GAUCHE;
    else if((rotation_gauche_fait) && (colonne_actuelle == 0)) etat=E_TOURNER_DROITE;

    // terminer la parcours
    else if(rang_actuel == 9) etat = E_ATTENTE;

    // cas normal
    else { nb_rotations = 0; avanceEnLigne(40, 40); }
  }

  if(deplacement_termine){
    etat = E_VERIF_OBSTACLE;
    deplacement_termine = 0;    
  }
}

/* E_TOURNER_GAUCHE
Tourne à gauche puis revient sur la logique obstacle 
*/
void etat_tourner_gauche() {
  tournerGauche();

  if(deplacement_termine){
    nb_rotations++;
    etat = E_VERIF_OBSTACLE;
    deplacement_termine = 0;

    if(rotation_droite_fait) {
      rotation_droite_fait = false;
      derniere_rotation = 2;
    }

    else rotation_gauche_fait = true;
  }
}

/* E_TOURNER_DROITE
Tourne à droite
*/
void etat_tourner_droite() {
  tournerDroite();

  if(deplacement_termine){
    nb_rotations++;
    etat = E_VERIF_OBSTACLE;
    deplacement_termine = 0;

    if(rotation_gauche_fait){
      rotation_gauche_fait = false;
      derniere_rotation = 1;
    }

    else rotation_droite_fait = true;
  }
}

/* E_VERIF_OBSTACLE
Vérifie s’il y a un obstacle devant
*/
void etat_verif_obstacle() {
  obstacle_detecte = fonctionDetectionObjet(pin_capteur_gauche, pin_capteur_droit);
  if (obstacle_detecte) etat = E_OBSTACLE_DETECTE;
  else etat = E_AVANCER;
}

/* E_OBSTACLE_DETECTE
Réagit à la détection obstacle en marquant la cellule et décidant le prochain état (rotation ou déplacement)
*/
void etat_obstacle_detecte() {
  // vérifi is les caisse déjà visitées
  if (parcours[colonne_actuelle][rang_actuel] == 2) etat=E_CELLULE_VERIFIEE;
  else parcours[colonne_actuelle][rang_actuel] = 2;

  etat=E_CHOIX_ROTATION;
}

/* E_CHOIX_ROTATION
Gère le choix de rotation selon les rotations précédentes
*/
void etat_choix_rotation() {
  // rendre le robot droit
  if((colonne_actuelle == 0 || colonne_actuelle == 1 || colonne_actuelle == 2) && nb_rotations != 2){
    if (rotation_gauche_fait) etat = E_TOURNER_DROITE;       
    else if(rotation_droite_fait) etat = E_TOURNER_GAUCHE;
  }

  // tourner gauche par défaut (colonnes 1 et 2)
  if(((colonne_actuelle == 2 || colonne_actuelle == 1) && (!rotation_gauche_fait && !rotation_droite_fait)) && (nb_rotations != 2)){
    etat = E_TOURNER_GAUCHE;
  }

  // tourner droit dans les colonnes 0
  if((colonne_actuelle == 0 && (!rotation_gauche_fait && !rotation_droite_fait)) && (nb_rotations != 2)){
    etat = E_TOURNER_DROITE;
  }

  // tourner et détecter une obstacle.
  if (nb_rotations == 2){
    if(derniere_rotation == 1) etat = E_TOURNER_DROITE;
    if(derniere_rotation == 2) etat = E_TOURNER_GAUCHE;
  }    
}

/* E_CELLULE_VERIFIEE
Si la cellule a déjà été vérifiée, reste sur place ou attend un nouvel événement
*/
void etat_cellule_verifiee() {
}

/*****************************************
 *   INITIALISATION ET BOUCLE
 *****************************************/
void setup() {
  BoardInit();
  ENCODER_Reset(0);
  ENCODER_Reset(1);
  dernier_echantillon = millis();
  pinMode(pin_capteur_gauche, INPUT);
  pinMode(pin_capteur_droit, INPUT);
  reglerPositionParcours(1,0);
}

void loop() {
  if (etat >= 0 && etat < E_NB_ETATS) {
    // utiliser function pointer qui est défini au début
    GestionnaireEtat h = gestionnaires_etat[etat];
    if (h) h();
  }
}
