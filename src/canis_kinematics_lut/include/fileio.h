#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>

using namespace std;

#ifndef FILEIO_H
#define FILEIO_H

void writeArrayToFile(double* array, int len, string fileName);

double* readFileToArray(string fileName, int arrayLen);

int getFileSize(string fileName);

#endif