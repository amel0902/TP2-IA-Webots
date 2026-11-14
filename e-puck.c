#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <stdio.h>

#define TIME_STEP 32

// Fonctions d'appartenance pour la distance latérale (droite)
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

// Fonctions d'appartenance pour l'avant et l'arrière
double membership_frontal_proche(double val) {
    if (val < 80) return 0.0;
    if (val > 350) return 1.0;
    return (val - 80.0) / (350.0 - 80.0);
}

double membership_arriere_proche(double val) {
    if (val < 80) return 0.0;
    if (val > 350) return 1.0;
    return (val - 80.0) / (350.0 - 80.0);
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

    // Initialisation de tous les capteurs (ps1, ps2, ps3, ps4, ps5, ps6)
    WbDeviceTag ps1 = wb_robot_get_device("ps1");
    WbDeviceTag ps2 = wb_robot_get_device("ps2");
    WbDeviceTag ps3 = wb_robot_get_device("ps3");
    WbDeviceTag ps4 = wb_robot_get_device("ps4");
    WbDeviceTag ps5 = wb_robot_get_device("ps5");
    WbDeviceTag ps6 = wb_robot_get_device("ps6");
    
    wb_distance_sensor_enable(ps1, TIME_STEP);
    wb_distance_sensor_enable(ps2, TIME_STEP);
    wb_distance_sensor_enable(ps3, TIME_STEP);
    wb_distance_sensor_enable(ps4, TIME_STEP);
    wb_distance_sensor_enable(ps5, TIME_STEP);
    wb_distance_sensor_enable(ps6, TIME_STEP);

    // Boucle principale de simulation
    while (wb_robot_step(TIME_STEP) != -1) {
        double val_ps1 = wb_distance_sensor_get_value(ps1);
        double val_ps2 = wb_distance_sensor_get_value(ps2);
        double val_ps3 = wb_distance_sensor_get_value(ps3);
        double val_ps4 = wb_distance_sensor_get_value(ps4);
        double val_ps5 = wb_distance_sensor_get_value(ps5);
        double val_ps6 = wb_distance_sensor_get_value(ps6);
        
        // 1. Entrées floues
        double d_droite = (val_ps1 > val_ps2) ? val_ps1 : val_ps2;
        double d_avant  = (val_ps5 > val_ps6) ? val_ps5 : val_ps6;
        double d_arriere = (val_ps3 > val_ps4) ? val_ps3 : val_ps4; // Max entre ps3 et ps4
        
        // Degrés d'appartenance
        double mu_proche = membership_proche(d_droite);
        double mu_moyen  = membership_moyen(d_droite);
        double mu_loin   = membership_loin(d_droite);

        double mu_avant_proche   = membership_frontal_proche(d_avant);
        double mu_arriere_proche = membership_arriere_proche(d_arriere);
        
        // 2. Base de règles et Inférence pour la direction latérale
        double out_tourner_gauche = mu_proche * (-3.0);
        double out_tout_droit     = mu_moyen * 0.0;
        double out_tourner_droite = mu_loin * 3.0;
        
        double somme_poids = mu_proche + mu_moyen + mu_loin;
        double steering = 0.0;
        if (somme_poids > 0.0) {
            steering = (out_tourner_gauche + out_tout_droit + out_tourner_droite) / somme_poids;
        }
        
        // 3. Gestion des priorités (Avant, Arrière, Côtés)
        double base_speed = 2.0 + (mu_moyen * 2.0);
        
        if (mu_avant_proche > 0.3) {
            // Obstacle devant : ralentir et tourner à gauche
            base_speed = 0.5;
            steering = -5.0;
        } 
        else if (mu_arriere_proche > 0.3) {
            // Obstacle derrière : accélérer vers l'avant pour s'éloigner
            base_speed = 4.0;
        }

        // 4. Cinématique différentielle
        double left_speed = base_speed + steering;
        double right_speed = base_speed - steering;

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        
        // Affichage console pour vérifier les lectures
        printf("Dr: %.1f | Av: %.1f | Ar: %.1f | Steering: %.2f\n", 
               d_droite, d_avant, d_arriere, steering);
    }

    wb_robot_cleanup();
    return 0;
}