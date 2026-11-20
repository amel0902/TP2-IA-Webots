#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28

/* Choix du cote du mur a suivre */
#define SUIVI_MUR_DROITE 1

/* Correction de l'orientation mesuree en simulation */
#define ALPHA_BIAS 35.0

#define DEBUG_PERIODE 10
#define N_ECHANTILLONS 121

/* Ensembles flous */

typedef struct {
  double a, b, c, d; /* bornes du trapeze */
} Trapeze;

typedef enum { D_LOIN, D_BONNE, D_PROCHE, NB_D } Distance;
typedef enum { F_LIBRE, F_MOYEN, F_PROCHE, F_CONTACT, NB_F } Frontal;
typedef enum { O_ELOIGNE, O_PARALLELE, O_VERS_MUR, O_ARRIERE, NB_O } Orientation;
typedef enum {
  R_FORT_VERS_MUR,
  R_LEGER_VERS_MUR,
  R_NUL,
  R_LEGER_LOIN,
  R_FORT_LOIN,
  NB_R
} Rotation;
typedef enum { V_RECUL, V_LENTE, V_MOYENNE, V_RAPIDE, NB_V } Vitesse;

/* Distance au mur */
static const Trapeze SET_DIST[NB_D] = {
    {0, 0, 80, 200},
    {100, 220, 320, 450},
    {350, 500, 4095, 4095},
};

/* Detection des obstacles devant le robot */
static const Trapeze SET_FRONT[NB_F] = {
    {0, 0, 100, 140},
    {110, 150, 220, 300},
    {220, 300, 600, 900},
    {600, 900, 4095, 4095},
};

static const Trapeze SET_ORIENT[NB_O] = {
    {-700, -400, -250, -60},
    {-150, -40, 40, 150},
    {60, 250, 4095, 4095},
    {-4095, -4095, -900, -500},
};

/* Rotation : negative vers le mur, positive pour s'en eloigner */
#define ROT_MIN (-3.0)
#define ROT_MAX 3.0
static const Trapeze SET_ROT[NB_R] = {
    {-3.0, -3.0, -2.0, -1.0},
    {-1.6, -0.8, -0.4, 0.0},
    {-0.5, -0.1, 0.1, 0.5},
    {0.0, 0.5, 1.0, 2.0},
    {1.0, 2.0, 3.0, 3.0},
};

/* Vitesse du robot */
#define VIT_MIN (-1.0)
#define VIT_MAX 2.5
static const Trapeze SET_VIT[NB_V] = {
    {-1.0, -1.0, -0.6, -0.1},
    {0.0, 0.0, 0.4, 0.9},
    {0.6, 1.1, 1.5, 2.0},
    {1.6, 2.0, 2.5, 2.5},
};

/* Fonctions utiles */

static double clamp(double x, double lo, double hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static double trapeze(double x, const Trapeze *t) {
  if (x < t->a || x > t->d) return 0.0;
  if (x >= t->b && x <= t->c) return 1.0;
  if (x < t->b) return (x - t->a) / (t->b - t->a);
  return (t->d - x) / (t->d - t->c);
}

/* Passage des mesures aux ensembles flous */

typedef struct {
  double dist[NB_D];
  double front[NB_F];
  double orient[NB_O];
} Entrees;

static void fuzzification(Entrees *e, double dist, double front,
                          double orient) {
  for (int i = 0; i < NB_D; i++) e->dist[i] = trapeze(dist, &SET_DIST[i]);
  for (int i = 0; i < NB_F; i++) e->front[i] = trapeze(front, &SET_FRONT[i]);
  for (int i = 0; i < NB_O; i++)
    e->orient[i] = trapeze(orient, &SET_ORIENT[i]);
}

/* Application des regles floues */

typedef struct {
  double rot[NB_R];
  double vit[NB_V];
} Sorties;

static double min3(double a, double b, double c) {
  return fmin(a, fmin(b, c));
}

static void regle(Sorties *s, double force, int rot, int vit) {
  if (force > s->rot[rot]) s->rot[rot] = force;
  if (force > s->vit[vit]) s->vit[vit] = force;
}

static void inference(const Entrees *e, Sorties *s) {
  for (int i = 0; i < NB_R; i++) s->rot[i] = 0.0;
  for (int i = 0; i < NB_V; i++) s->vit[i] = 0.0;

  const double libre = e->front[F_LIBRE];
  const double moyen = e->front[F_MOYEN];
  const double proche = e->front[F_PROCHE];
  const double contact = e->front[F_CONTACT];

  /* On evite d'abord les obstacles devant le robot */
  regle(s, contact, R_FORT_LOIN, V_RECUL); /* Recul si obstacle tres proche */
  regle(s, proche, R_FORT_LOIN, V_LENTE);
  regle(s, moyen, R_FORT_LOIN, V_LENTE);

  /* On se rapproche du mur */
  regle(s, min3(e->dist[D_LOIN], e->orient[O_PARALLELE], libre),
        R_LEGER_VERS_MUR, V_MOYENNE);
  regle(s, min3(e->dist[D_LOIN], e->orient[O_ELOIGNE], libre),
        R_FORT_VERS_MUR, V_LENTE);
  regle(s, min3(e->dist[D_LOIN], e->orient[O_VERS_MUR], libre),
        R_NUL, V_MOYENNE);

  /* On garde la bonne distance */
  regle(s, min3(e->dist[D_BONNE], e->orient[O_PARALLELE], libre),
        R_NUL, V_RAPIDE);
  regle(s, min3(e->dist[D_BONNE], e->orient[O_VERS_MUR], libre),
        R_LEGER_LOIN, V_MOYENNE);
  regle(s, min3(e->dist[D_BONNE], e->orient[O_ELOIGNE], libre),
        R_LEGER_VERS_MUR, V_MOYENNE);

  /* On s'ecarte du mur */
  regle(s, min3(e->dist[D_PROCHE], e->orient[O_PARALLELE], libre),
        R_LEGER_LOIN, V_MOYENNE);
  regle(s, min3(e->dist[D_PROCHE], e->orient[O_VERS_MUR], libre),
        R_FORT_LOIN, V_LENTE);
  regle(s, min3(e->dist[D_PROCHE], e->orient[O_ELOIGNE], libre),
        R_NUL, V_MOYENNE);

  /* Si l'arriere touche le mur, on avance pour se degager */
  regle(s, fmin(e->orient[O_ARRIERE], libre), R_NUL, V_MOYENNE);
}

/* Calcul des commandes finales par centre de gravite */

static double centroide(const double *force, const Trapeze *sets, int n,
                        double xmin, double xmax, double defaut) {
  double num = 0.0, den = 0.0;

  for (int k = 0; k < N_ECHANTILLONS; k++) {
    double x = xmin + (xmax - xmin) * k / (N_ECHANTILLONS - 1);
    double m = 0.0;
    for (int i = 0; i < n; i++) {

      double mi = fmin(force[i], trapeze(x, &sets[i]));
      if (mi > m) m = mi;
    }
    num += x * m;
    den += m;
  }

  if (den < 1e-6) return defaut;
  return num / den;
}

/* Programme principal */

int main(void) {
  wb_robot_init();

  WbDeviceTag left = wb_robot_get_device("left wheel motor");
  WbDeviceTag right = wb_robot_get_device("right wheel motor");

  if (!left || !right) {
    fprintf(stderr, "Moteurs introuvables\n");
    wb_robot_cleanup();
    return 1;
  }

  wb_motor_set_position(left, INFINITY);
  wb_motor_set_position(right, INFINITY);
  wb_motor_set_velocity(left, 0.0);
  wb_motor_set_velocity(right, 0.0);

  WbDeviceTag ps[8] = {0};
  char name[8];

  for (int i = 0; i <= 7; i++) {
    sprintf(name, "ps%d", i);
    ps[i] = wb_robot_get_device(name);
    if (!ps[i]) {
      fprintf(stderr, "Capteur %s introuvable\n", name);
      wb_robot_cleanup();
      return 1;
    }
    wb_distance_sensor_enable(ps[i], TIME_STEP);
  }

  int iteration = 0;
  printf("Controleur flou (suivi de mur, trapezes) initialise\n");

  while (wb_robot_step(TIME_STEP) != -1) {
    double v[8] = {0};
    for (int i = 0; i <= 7; i++) v[i] = wb_distance_sensor_get_value(ps[i]);

    /* Lecture des capteurs utiles */
#if SUIVI_MUR_DROITE
    const double lateral = v[2];
    const double diag = v[1];
    const double arriere = v[3];
    const double cote = 1.0;
    /* On ne compte pas le mur suivi comme obstacle frontal */
    const double front = fmax(fmax(v[0], v[7]), v[6]);
#else
    const double lateral = v[5];
    const double diag = v[6];
    const double arriere = v[4];
    const double cote = -1.0;
    const double front = fmax(fmax(v[0], v[7]), v[1]);
#endif
    const double orient = diag - arriere - ALPHA_BIAS;

    Entrees e;
    Sorties s;
    fuzzification(&e, lateral, front, orient);
    inference(&e, &s);

    double rot_mur = centroide(s.rot, SET_ROT, NB_R, ROT_MIN, ROT_MAX, 0.0);
    double vitesse = centroide(s.vit, SET_VIT, NB_V, VIT_MIN, VIT_MAX, 1.0);

    /* Conversion en vitesses des deux roues */
    double steering = cote * rot_mur;
    double left_speed = clamp(vitesse - steering, -MAX_SPEED, MAX_SPEED);
    double right_speed = clamp(vitesse + steering, -MAX_SPEED, MAX_SPEED);

    wb_motor_set_velocity(left, left_speed);
    wb_motor_set_velocity(right, right_speed);

    iteration++;
    if (iteration % DEBUG_PERIODE == 0) {
      printf("lat: %4.0f | av: %4.0f | alpha: %5.0f | rot: %5.2f | "
             "v: %4.2f | VG: %5.2f | VD: %5.2f\n",
             lateral, front, orient, rot_mur, vitesse, left_speed,
             right_speed);
    }
  }

  wb_robot_cleanup();
  return 0;
}
 