#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>

#include "../include/kinematics.h"

using namespace std;

#define shoulder_length 0.055
#define arm_length 0.105
#define forearm_length 0.136

#define precision 0.003 // Works: 3, 5
#define leng sqrt(pow(arm_length + forearm_length, 2) + shoulder_length * shoulder_length)
#define edge ((int) (leng / precision))

double lerp1D(double Z1, double Z2, double Zx) {
    return (Z1 * (1.0 - Zx)) + (Z2 * Zx);
}

double lerp2D(double Z1, double Z2, double Z3, double Z4, double Zx, double Zy) {
    return lerp1D(lerp1D(Z1, Z2, Zx), lerp1D(Z3, Z4, Zx), Zy);
}

double lerp3D(double Z1, double Z2, double Z3, double Z4, double Z5, double Z6, double Z7, double Z8, double Zx, double Zy, double Zw) {
    return lerp1D(lerp2D(Z1, Z2, Z3, Z4, Zx, Zy), lerp2D(Z5, Z6, Z7, Z8, Zx, Zy), Zw);
}

double readLUT(double* arrayIn, double prec, double len, double edg, double x, double y, double z) {
    int index = (((int) ((x + len)/prec)) + (2 * edg + 1) * ((int) ((y + len)/prec)) + (2 * edg + 1) * (2 * edg + 1) * ((int) ((z + len)/(prec))));
    return arrayIn[index];
}

double readLUT(double* arrayIn, double prec, double len, double edg, int x, int y, int z, bool indexable) {
    int index;
    if (indexable) {
        index = (x + (2 * edg + 1) * y + (2 * edg + 1) * (2 * edg + 1) * z);
    }
    else {
        index = (((int) ((x + len)/prec)) + (2 * edg + 1) * ((int) ((y + len)/prec)) + (2 * edg + 1) * (2 * edg + 1) * ((int) ((z + len)/(prec))));
    }
    return arrayIn[index];
}

double lerpLUT(double* arrayIn, double prec, double len, double edg, double x, double y, double z) {
    int xindex = (int) ((x + len)/prec);
    int yindex = (int) ((y + len)/prec);
    int zindex = (int) ((z + len)/prec);

    double Z1 = readLUT(arrayIn, prec, len, edg, xindex, yindex, zindex, true);
    double Z2 = readLUT(arrayIn, prec, len, edg, xindex+1, yindex, zindex, true);
    double Z3 = readLUT(arrayIn, prec, len, edg, xindex, yindex+1, zindex, true);
    double Z4 = readLUT(arrayIn, prec, len, edg, xindex+1, yindex+1, zindex, true);

    double Z5 = readLUT(arrayIn, prec, len, edg, xindex, yindex, zindex+1, true);
    double Z6 = readLUT(arrayIn, prec, len, edg, xindex+1, yindex, zindex+1, true);
    double Z7 = readLUT(arrayIn, prec, len, edg, xindex, yindex+1, zindex+1, true);
    double Z8 = readLUT(arrayIn, prec, len, edg, xindex+1, yindex+1, zindex+1, true);

    double Zx = ((x + len)/prec) - xindex;
    double Zy = ((y + len)/prec) - yindex;
    double Zw = ((z + len)/prec) - zindex;


    return lerp3D(Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8, Zx, Zy, Zw);

}

double ik_shoulder(double superior_right_x, double superior_right_y, double superior_right_z) {
    double superior_right_dyz = sqrt(superior_right_y * superior_right_y + superior_right_z * superior_right_z);
    double superior_right_lyz = sqrt(superior_right_dyz * superior_right_dyz - shoulder_length * shoulder_length);
    double superior_right_g1 = atan2(superior_right_y, superior_right_z);
    double superior_right_g2 = atan2(shoulder_length, superior_right_lyz);
    double superior_right_lxz = sqrt(superior_right_lyz * superior_right_lyz - superior_right_x * superior_right_x);
    double nmult = 1 / (2 * arm_length);
    double superior_right_n = (superior_right_lxz * superior_right_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;
    double superior_right_a1 = atan2(superior_right_x, superior_right_lyz);
    double superior_right_a2 = -acos((arm_length + superior_right_n) / superior_right_lxz);
    return (-(superior_right_g1 + superior_right_g2)) + M_PI;
}

double ik_arm(double superior_right_x, double superior_right_y, double superior_right_z) {
    double superior_right_dyz = sqrt(superior_right_y * superior_right_y + superior_right_z * superior_right_z);
    double superior_right_lyz = sqrt(superior_right_dyz * superior_right_dyz - shoulder_length * shoulder_length);
    double superior_right_g1 = atan2(superior_right_y, superior_right_z);
    double superior_right_g2 = atan2(shoulder_length, superior_right_lyz);
    double superior_right_lxz = sqrt(superior_right_lyz * superior_right_lyz - superior_right_x * superior_right_x);
    double nmult = 1 / (2 * arm_length);
    double superior_right_n = (superior_right_lxz * superior_right_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;
    double superior_right_a1 = atan2(superior_right_x, superior_right_lyz);
    double superior_right_a2 = -acos((arm_length + superior_right_n) / superior_right_lxz);
    return -(superior_right_a1 + superior_right_a2);
}

double ik_forearm(double superior_right_x, double superior_right_y, double superior_right_z) {
    double superior_right_dyz = sqrt(superior_right_y * superior_right_y + superior_right_z * superior_right_z);
    double superior_right_lyz = sqrt(superior_right_dyz * superior_right_dyz - shoulder_length * shoulder_length);
    double superior_right_g1 = atan2(superior_right_y, superior_right_z);
    double superior_right_g2 = atan2(shoulder_length, superior_right_lyz);
    double superior_right_lxz = sqrt(superior_right_lyz * superior_right_lyz - superior_right_x * superior_right_x);
    double nmult = 1 / (2 * arm_length);
    double superior_right_n = (superior_right_lxz * superior_right_lxz - forearm_length * forearm_length - arm_length * arm_length) * nmult;
    double superior_right_a1 = atan2(superior_right_x, superior_right_lyz);
    double superior_right_a2 = -acos((arm_length + superior_right_n) / superior_right_lxz);
    return acos(superior_right_n / forearm_length);

}