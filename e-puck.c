
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28

#define CRUISE_SPEED 2.5
#define FRONT_WARNING 180.0
#define FRONT_DANGER 350.0
#define SIDE_DANGER 650.0
#define CLEAR_THRESHOLD 160.0

// 1. FONCTIONS UTILITAIRES

double clamp(double x, double min, double max) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

// Fonction d'appartenance trapezoidale
double trapeze(double x, double a, double b,
               double c, double d) {

  if (x < a || x > d)
    return 0.0;

  if (x >= b && x <= c)
    return 1.0;

  if (x < b)
    return (b == a) ? 1.0 : (x - a) / (b - a);

  return (d == c) ? 1.0 : (d - x) / (d - c);
}


// 2. FONCTIONS D'APPARTENANCE FLOUES

// Une valeur elevee signifie un obstacle proche

double proche(double x) {
  return trapeze(x, 300, 650, 4095, 4095);
}

double moyen(double x) {
  return trapeze(x, 100, 250, 400, 750);
}

double loin(double x) {
  return trapeze(x, 0, 0, 80, 250);
}

// 3. PROGRAMME PRINCIPAL

int main(int argc, char **argv) {

  wb_robot_init();

  // Initialisation des moteurs
  WbDeviceTag left =
      wb_robot_get_device("left wheel motor");

  WbDeviceTag right =
      wb_robot_get_device("right wheel motor");

  if (!left || !right) {
    fprintf(stderr, "Moteurs introuvables\n");
    wb_robot_cleanup();
    return 1;
  }

  wb_motor_set_position(left, INFINITY);
  wb_motor_set_position(right, INFINITY);

  wb_motor_set_velocity(left, 0.0);
  wb_motor_set_velocity(right, 0.0);

  // 4. INITIALISATION DES CAPTEURS

  WbDeviceTag ps[7] = {0};
  char name[8];

  for (int i = 1; i <= 6; i++) {

    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name);

    if (!ps[i]) {
      fprintf(stderr,
              "Capteur %s introuvable\n", name);
      wb_robot_cleanup();
      return 1;
    }

    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  // Variables pour l'evitement
  int avoiding = 0;
  int turn_direction = 1;
  int clear_steps = 0;
  int turn_steps = 0;

  int iteration = 0;

  printf("Controleur flou initialise\n");

  // 5. BOUCLE PRINCIPALE

  while (wb_robot_step(TIME_STEP) != -1) {

    double v[7] = {0};

    // Lecture des six capteurs
    for (int i = 1; i <= 6; i++) {
      v[i] =
          wb_distance_sensor_get_value(ps[i]);
    }

    // Estimation des obstacles
    double front = fmax(v[1], v[6]);

    double right_obstacle =
        fmax(v[1], v[2]);

    double left_obstacle =
        fmax(v[5], v[6]);

    double left_speed = 0.0;
    double right_speed = 0.0;

    const char *state = "AVANCE";

    // 6. DETECTION D'UN DANGER

    if (!avoiding &&
        (front > FRONT_DANGER ||
         v[2] > SIDE_DANGER ||
         v[5] > SIDE_DANGER)) {

      avoiding = 1;
      clear_steps = 0;
      turn_steps = 0;

      // Tourner vers le cote le moins encombre
      if (left_obstacle < right_obstacle)
        turn_direction = 1;
      else
        turn_direction = -1;
    }

    // 7. EVITEMENT PRIORITAIRE

    if (avoiding) {

      state = "EVITEMENT";

      double turn_speed = 2.0;

      // Rotation sur place
      left_speed =
          -turn_direction * turn_speed;

      right_speed =
          turn_direction * turn_speed;

      turn_steps++;

      // Verification du degagement
      if (front < CLEAR_THRESHOLD &&
          v[2] < SIDE_DANGER &&
          v[5] < SIDE_DANGER) {

        clear_steps++;

      } else {
        clear_steps = 0;
      }

      if (clear_steps >= 5 &&
          turn_steps >= 8) {

        avoiding = 0;
        clear_steps = 0;
      }

      // Eviter une rotation infinie
      if (turn_steps > 100) {
        turn_direction = -turn_direction;
        turn_steps = 0;
      }

    } else {

      // 8. FUZZIFICATION

      double mu_right_close =
          proche(right_obstacle);

      double mu_left_close =
          proche(left_obstacle);

      double mu_right_medium =
          moyen(right_obstacle);

      double mu_left_medium =
          moyen(left_obstacle);

      double mu_right_far =
          loin(right_obstacle);

      double mu_left_far =
          loin(left_obstacle);

      // 9. CALCUL DE LA DIRECTION
    

      // Obstacle a droite : tourner a gauche
      // Obstacle a gauche : tourner a droite
      // Aucun obstacle : avancer

      double sum =
          mu_right_close +
          mu_left_close +
          mu_right_medium +
          mu_left_medium +
          mu_right_far * mu_left_far;

      double steering = 0.0;

      if (sum > 0.0001) {

        steering =
            (2.2 * mu_right_close
           - 2.2 * mu_left_close
           + 0.6 * mu_right_medium
           - 0.6 * mu_left_medium)
            / sum;
      }

      // 10. RALENTISSEMENT PROGRESSIF

      double speed = CRUISE_SPEED;

      if (front > FRONT_WARNING) {

        double danger = clamp(
            (front - FRONT_WARNING) /
            (FRONT_DANGER - FRONT_WARNING),
            0.0, 1.0
        );

        speed =
            CRUISE_SPEED * (1.0 - 0.7 * danger);

        state = "RALENTISSEMENT";
      }

      // Commande des moteurs
      left_speed = speed - steering;
      right_speed = speed + steering;
    }

    // 11. LIMITATION DES VITESSES

    left_speed =
        clamp(left_speed, -MAX_SPEED, MAX_SPEED);

    right_speed =
        clamp(right_speed, -MAX_SPEED, MAX_SPEED);

    wb_motor_set_velocity(left, left_speed);
    wb_motor_set_velocity(right, right_speed);

    // 12. AFFICHAGE DE DIAGNOSTIC

    iteration++;

    if (iteration % 10 == 0) {

      printf(
          "%s | ps1: %.0f | ps2: %.0f | "
          "ps5: %.0f | ps6: %.0f | "
          "Av: %.0f | Dr: %.0f | Ga: %.0f | "
          "VG: %.2f | VD: %.2f\n",
          state,
          v[1], v[2], v[5], v[6],
          front,
          right_obstacle,
          left_obstacle,
          left_speed,
          right_speed
      );
    }
  }

  wb_robot_cleanup();
  return 0;
}
