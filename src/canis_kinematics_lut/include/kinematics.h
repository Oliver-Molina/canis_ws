#ifndef KINEMATICS_H
#define KINEMATICS_H

double lerp1D(double Z1, double Z2, double Zx);

double lerp2D(double Z1, double Z2, double Z3, double Z4, double Zx, double Zy);

double lerp3D(double Z1, double Z2, double Z3, double Z4, double Z5, double Z6, double Z7, double Z8, double Zx, double Zy, double Zw);

double readLUT(double* arrayIn, double prec, double len, double edg, double x, double y, double z);

double readLUT(double* arrayIn, double prec, double len, double edg, int x, int y, int z, bool indexable);

double lerpLUT(double* arrayIn, double prec, double len, double edg, double x, double y, double z);

double ik_shoulder(double superior_right_x, double superior_right_y, double superior_right_z);

double ik_arm(double superior_right_x, double superior_right_y, double superior_right_z);

double ik_forearm(double superior_right_x, double superior_right_y, double superior_right_z);

#endif