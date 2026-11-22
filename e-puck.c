/*
 * TP2 - Controle flou d'un robot e-puck sous Webots
 * Detection des obstacles avec les capteurs ps1, ps2, ps5 et ps6.
 * Le controleur ajuste la vitesse et la direction du robot.
 * Code écrit par Amel Benaissa
 */

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>

#include <stdio.h>
#include <math.h>

#define TIME_STEP 32
#define DEBUG_PERIOD 5
#define NB_SAMPLES 101

#define MAX_SPEED 6.28

#define ROT_MIN -2.5
#define ROT_MAX  2.5
#define VIT_MIN  0.0
#define VIT_MAX  3.5

#define VITESSE_LIBRE 1.9
#define ROTATION_MAX  1.45

#define SEUIL_DETECTION 95.0
#define SEUIL_PROCHE    180.0
#define SEUIL_CRITIQUE  300.0   /* seuil de degagement */
#define SEUIL_SORTIE    160.0   /* seuil de sortie */
#define SEUIL_RECUL     1500.0  /* seuil de recul */

#define MAINTIEN_VIRAGE 7
#define DEGAGEMENT_MAX  45

#define BALAYAGE_AMPLITUDE 0.18
#define BALAYAGE_PERIODE   110.0

typedef struct {
    double a, b, c, d;
} Trapeze;

enum { LOIN, MOYEN, PROCHE, NB_PROX };

enum {
    FORTE_DROITE,
    LEGERE_DROITE,
    TOUT_DROIT,
    LEGERE_GAUCHE,
    FORTE_GAUCHE,
    NB_ROT
};

enum { TRES_LENTE, LENTE, MOYENNE, RAPIDE, NB_VIT };

enum { MODE_NORMAL, MODE_VIRAGE, MODE_DEGAGEMENT };

/* Ensembles flous */

/* Proximite des obstacles selon la valeur des capteurs */
static const Trapeze SET_PROX[NB_PROX] = {
    {0,   0,   85,  125},
    {85,  125, 170, 230},
    {170, 240, 4095, 4095}
};

/* Sens de rotation du robot */
static const Trapeze SET_ROT[NB_ROT] = {
    {-2.5, -2.5, -2.1, -1.5},
    {-1.7, -1.2, -0.7,  0.0},
    {-0.3, -0.1,  0.1,  0.3},
    { 0.0,  0.7,  1.2,  1.7},
    { 1.5,  2.1,  2.5,  2.5}
};

/* Niveaux de vitesse */
static const Trapeze SET_VIT[NB_VIT] = {
    {0.0, 0.0, 0.2, 0.5},
    {0.3, 0.6, 1.0, 1.4},
    {1.1, 1.6, 2.0, 2.6},
    {2.4, 3.0, 3.5, 3.5}
};

/* Regles floues selon les obstacles a droite et a gauche */

static const int REGLE_ROT[NB_PROX][NB_PROX] = {
    /* G=LOIN         G=MOYEN         G=PROCHE   */
    {TOUT_DROIT,    LEGERE_DROITE, FORTE_DROITE},  /* D=LOIN   */
    {LEGERE_GAUCHE, TOUT_DROIT,    FORTE_DROITE},  /* D=MOYEN  */
    {FORTE_GAUCHE,  FORTE_GAUCHE,  TOUT_DROIT}     /* D=PROCHE */
};

static const int REGLE_VIT[NB_PROX][NB_PROX] = {
    {RAPIDE,  MOYENNE,    LENTE},
    {MOYENNE, LENTE,      TRES_LENTE},
    {LENTE,   TRES_LENTE, TRES_LENTE}
};

/* Fonctions utilitaires */

static double minimum(double a, double b) { return a < b ? a : b; }
static double maximum(double a, double b) { return a > b ? a : b; }

static double limiter(double x, double min, double max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

/* Calcul du degre d'appartenance */
static double appartenance(double x, Trapeze t) {
    if (x < t.a || x > t.d)
        return 0.0;
    if (x >= t.b && x <= t.c)
        return 1.0;
    if (x < t.b) {
        if (t.a == t.b)
            return 1.0;
        return (x - t.a) / (t.b - t.a);
    }
    if (t.c == t.d)
        return 1.0;
    return (t.d - x) / (t.d - t.c);
}

/* Conversion d'une mesure en degres d'appartenance */
static void fuzzification(double mesure, double mu[NB_PROX]) {
    for (int i = 0; i < NB_PROX; i++)
        mu[i] = appartenance(mesure, SET_PROX[i]);
}

/* Application des regles flous (min et max) */
static void inference(
    const double muD[NB_PROX],
    const double muG[NB_PROX],
    double activationRot[NB_ROT],
    double activationVit[NB_VIT]
) {
    for (int i = 0; i < NB_ROT; i++) activationRot[i] = 0.0;
    for (int i = 0; i < NB_VIT; i++) activationVit[i] = 0.0;

    for (int d = 0; d < NB_PROX; d++) {
        for (int g = 0; g < NB_PROX; g++) {
            double force = minimum(muD[d], muG[g]);
            int r = REGLE_ROT[d][g];
            int v = REGLE_VIT[d][g];
            activationRot[r] = maximum(activationRot[r], force);
            activationVit[v] = maximum(activationVit[v], force);
        }
    }
}

/* Calcul de la sortie par centre de gravite */
static double centre_gravite(
    const double activation[],
    const Trapeze ensembles[],
    int nb,
    double min,
    double max,
    double defaut
) {
    double sommeX = 0.0;
    double sommeMu = 0.0;

    for (int i = 0; i < NB_SAMPLES; i++) {
        double x = min + (max - min) * i / (NB_SAMPLES - 1);
        double mu = 0.0;

        for (int j = 0; j < nb; j++) {
            double valeur = minimum(activation[j],
                                    appartenance(x, ensembles[j]));
            mu = maximum(mu, valeur);
        }
        sommeX += x * mu;
        sommeMu += mu;
    }

    if (sommeMu < 1e-9)
        return defaut;
    return sommeX / sommeMu;
}

/* Programme principal */

int main(void) {

    wb_robot_init();

    WbDeviceTag moteurG = wb_robot_get_device("left wheel motor");
    WbDeviceTag moteurD = wb_robot_get_device("right wheel motor");

    if (!moteurG || !moteurD) {
        fprintf(stderr, "Moteurs introuvables\n");
        wb_robot_cleanup();
        return 1;
    }

    wb_motor_set_position(moteurG, INFINITY);
    wb_motor_set_position(moteurD, INFINITY);
    wb_motor_set_velocity(moteurG, 0.0);
    wb_motor_set_velocity(moteurD, 0.0);

    /* Initialisation des capteurs */
    WbDeviceTag ps[8] = {0};
    int indices[4] = {1, 2, 5, 6};
    char nom[8];

    for (int i = 0; i < 4; i++) {
        int n = indices[i];
        sprintf(nom, "ps%d", n);
        ps[n] = wb_robot_get_device(nom);

        if (!ps[n]) {
            fprintf(stderr, "Capteur %s introuvable\n", nom);
            wb_robot_cleanup();
            return 1;
        }
        wb_distance_sensor_enable(ps[n], TIME_STEP);
    }

    int iteration = 0;
    int mode = MODE_NORMAL;
    int dernierSens = 1;     /* +1 : gauche, -1 : droite */
    int maintien = 0;
    int degagement = 0;

    double VGprecedente = 0.0;
    double VDprecedente = 0.0;

    while (wb_robot_step(TIME_STEP) != -1) {

        /* Lecture des capteurs */
        double p1 = wb_distance_sensor_get_value(ps[1]);
        double p2 = wb_distance_sensor_get_value(ps[2]);
        double p5 = wb_distance_sensor_get_value(ps[5]);
        double p6 = wb_distance_sensor_get_value(ps[6]);

        /* Valeur maximale detectee de chaque cote */
        double D = limiter(maximum(p1, p2), 0.0, 4095.0);
        double G = limiter(maximum(p5, p6), 0.0, 4095.0);

        double obstacleMax = maximum(D, G);

        /* Detection d'un obstacle devant le robot */
        double avant = maximum(p1, p6);

        /* Fuzzification */
        double muD[NB_PROX];
        double muG[NB_PROX];
        fuzzification(D, muD);
        fuzzification(G, muG);

        /* Evaluation des regles */
        double activationRot[NB_ROT];
        double activationVit[NB_VIT];
        inference(muD, muG, activationRot, activationVit);

        /* Defuzzification */
        double rotation = centre_gravite(activationRot, SET_ROT, NB_ROT,
                                         ROT_MIN, ROT_MAX, 0.0);
        double vitesse  = centre_gravite(activationVit, SET_VIT, NB_VIT,
                                         VIT_MIN, VIT_MAX, 2.0);

        /* Choix du sens pour eviter l'obstacle */
        if (mode != MODE_DEGAGEMENT) {
            if (D > G + 25.0)
                dernierSens = 1;
            else if (G > D + 25.0)
                dernierSens = -1;
        }

        /* Activation du degagement si l'obstacle est trop proche */
        if (mode != MODE_DEGAGEMENT && avant > SEUIL_CRITIQUE) {
            mode = MODE_DEGAGEMENT;
            degagement = 0;
            maintien = 0;

            /* On tourne du cote oppose a l'obstacle le plus proche. */
            if (p1 > p6 + 25.0)
                dernierSens = 1;
            else if (p6 > p1 + 25.0)
                dernierSens = -1;
        }

        /* Gestion des modes de deplacement */
        if (mode == MODE_DEGAGEMENT) {

            /* Pivot sur place, avec recul si necessaire */
            vitesse = (avant > SEUIL_RECUL) ? -0.5 : 0.0;
            rotation = dernierSens * 1.15;

            degagement++;

            /* Fin du degagement lorsque l'avant est suffisamment libre */
            if (avant < SEUIL_SORTIE || degagement >= DEGAGEMENT_MAX) {
                mode = MODE_VIRAGE;
                maintien = MAINTIEN_VIRAGE;
                degagement = 0;
            }

        } else {

            /* Ralentissement a l'approche d'un obstacle */
            vitesse = minimum(vitesse, VITESSE_LIBRE);

            if (obstacleMax > 95.0)
                vitesse = minimum(vitesse, 1.8);
            if (obstacleMax > 140.0)
                vitesse = minimum(vitesse, 1.15);
            if (obstacleMax > SEUIL_PROCHE)
                vitesse = minimum(vitesse, 0.75);
            if (obstacleMax > 300.0)
                vitesse = minimum(vitesse, 0.45);

            if (obstacleMax > SEUIL_DETECTION) {

                mode = MODE_VIRAGE;
                maintien = MAINTIEN_VIRAGE;

                if (obstacleMax > SEUIL_PROCHE) {
                    rotation = dernierSens *
                        maximum(fabs(rotation), 1.05);
                }

            } else if (maintien > 0) {

                /* Maintien du virage pendant quelques iterations */
                rotation = dernierSens * maximum(fabs(rotation), 0.55);
                vitesse = minimum(vitesse, 1.5);
                maintien--;
                mode = MODE_VIRAGE;

            } else {

                mode = MODE_NORMAL;

                /* Legere variation de direction en espace libre */
                rotation += BALAYAGE_AMPLITUDE *
                    sin(2.0 * M_PI * iteration / BALAYAGE_PERIODE);
            }

            rotation = limiter(rotation, -ROTATION_MAX, ROTATION_MAX);
        }

        /* Calcul de la vitesse de chaque roue */
        double VGcible = vitesse - rotation;
        double VDcible = vitesse + rotation;

        if (mode == MODE_DEGAGEMENT) {
            /* Limitation des vitesses pendant le degagement */
            VGcible = limiter(VGcible, -1.15, 1.15);
            VDcible = limiter(VDcible, -1.15, 1.15);
        } else {
            /* Recul autorise pres d'un mur pour faciliter le virage */
            double mini = (obstacleMax > SEUIL_PROCHE) ? -1.0 : 0.0;
            VGcible = limiter(VGcible, mini, MAX_SPEED);
            VDcible = limiter(VDcible, mini, MAX_SPEED);
        }

        /* Lissage des vitesses pour eviter les changements brusques */
        double alpha;
        if (mode == MODE_DEGAGEMENT)
            alpha = 0.75;
        else if (obstacleMax > 180.0)
            alpha = 0.75;
        else if (obstacleMax > 95.0)
            alpha = 0.60;
        else
            alpha = 0.25;

        double VG = VGprecedente + alpha * (VGcible - VGprecedente);
        double VD = VDprecedente + alpha * (VDcible - VDprecedente);

        VG = limiter(VG, -MAX_SPEED, MAX_SPEED);
        VD = limiter(VD, -MAX_SPEED, MAX_SPEED);

        VGprecedente = VG;
        VDprecedente = VD;

        wb_motor_set_velocity(moteurG, VG);
        wb_motor_set_velocity(moteurD, VD);

        /* Affichage des valeurs pour suivre le comportement */
        if (++iteration % DEBUG_PERIOD == 0) {
            const char *nomMode =
                mode == MODE_NORMAL ? "NORMAL" :
                mode == MODE_VIRAGE ? "VIRAGE" :
                "DEGAGEMENT";

            printf("D:%6.1f | G:%6.1f | Rot:%5.2f | Vit:%4.2f | "
                   "VG:%5.2f | VD:%5.2f | %s\n",
                   D, G, rotation, vitesse, VG, VD, nomMode);
        }
    }

    wb_robot_cleanup();
    return 0;
}