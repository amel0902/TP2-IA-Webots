#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 32

// Fonctions d'appartenance pour les entrées (distance droite)
// Seuils bruts de l'e-puck (à ajuster selon vos tests) : [0, 80, 200] pour Proche, etc.
double membership_proche(double val) {
    if (val < 0) val = 0;
    if (val > 150) return 0.0;
    return (150.0 - val) / 150.0;
}

double membership_moyen(double val) {
    if (val < 50 || val > 300) return 0.0;
    if (val < 150) return (val - 50.0) / 100.0;
    return (300.0 - val) / 150.0;
}

double membership_loin(double val) {
    if (val < 200) return 0.0;
    if (val > 600) return 1.0;
    return (val - 200.0) / 400.0;
}

int main(int argc, char **argv) {
  wb_robot_init();

  // Initialisation des moteurs de l'e-puck
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  // Initialisation des capteurs infrarouges demandés : ps1, ps2, ps5, ps6
  WbDeviceTag ps1 = wb_robot_get_device("ps1");
  WbDeviceTag ps2 = wb_robot_get_device("ps2");
  WbDeviceTag ps5 = wb_robot_get_device("ps5");
  WbDeviceTag ps6 = wb_robot_get_device("ps6");
  
  wb_distance_sensor_enable(ps1, TIME_STEP);
  wb_distance_sensor_enable(ps2, TIME_STEP);
  wb_distance_sensor_enable(ps5, TIME_STEP);
  wb_distance_sensor_enable(ps6, TIME_STEP);

  // Boucle principale de simulation
  while (wb_robot_step(TIME_STEP) != -1) {
    double val_ps1 = wb_distance_sensor_get_value(ps1);
    double val_ps2 = wb_distance_sensor_get_value(ps2);
    
    // Entrée floue : distance à droite (max entre ps1 et ps2)[cite: 2]
    double d_droite = (val_ps1 > val_ps2) ? val_ps1 : val_ps2;
    
    // 1. Fuzzification (calcul des degrés d'appartenance)
    double mu_proche = membership_proche(d_droite);
    double mu_moyen  = membership_moyen(d_droite);
    double mu_loin   = membership_loin(d_droite);
    
    // 2. Base de règles et Inférence pour l'orientation (steering)
    // - Règle 1 : SI Proche ALORS Tourner à gauche (steering négatif)
    // - Règle 2 : SI Moyen ALORS Aller tout droit (steering nul)
    // - Règle 3 : SI Loin ALORS Tourner à droite (steering positif)
    double out_tourner_gauche = mu_proche * (-3.0);
    double out_tout_droit     = mu_moyen * 0.0;
    double out_tourner_droite = mu_loin * 3.0;
    
    // 3. Défuzzification (Moyenne pondérée)
    double somme_poids = mu_proche + mu_moyen + mu_loin;
    double steering = 0.0;
    
    if (somme_poids > 0.0) {
        steering = (out_tourner_gauche + out_tout_droit + out_tourner_droite) / somme_poids;
    }
    
    // Vitesse de base modulée par la logique floue (plus c'est loin, plus on avance vite, plus c'est proche, plus on ralentit)
    double base_speed = 2.0 + (mu_moyen * 2.0);

    // 4. Conversion en cinématique différentielle (roues gauche et droite)[cite: 2]
    double left_speed = base_speed + steering;
    double right_speed = base_speed - steering;

    wb_motor_set_velocity(left_motor, left_speed);
    wb_motor_set_velocity(right_motor, right_speed);
    
    // Affichage console pour observer le comportement flou en direct
    printf("Dist Droite: %.1f | P: %.2f M: %.2f L: %.2f | Steering: %.2f\n", 
           d_droite, mu_proche, mu_moyen, mu_loin, steering);
  }

  wb_robot_cleanup();
  return 0;
}