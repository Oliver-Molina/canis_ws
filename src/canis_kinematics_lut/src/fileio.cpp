#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <chrono>
#include <thread>

#include "../include/fileio.h"

using namespace std;

#define shoulder_length 0.055
#define arm_length 0.105
#define forearm_length 0.136

#define precision 0.003 // Works: 3, 5
#define leng sqrt(pow(arm_length + forearm_length, 2) + shoulder_length * shoulder_length)
#define edge ((int) (leng / precision))

#define PERIOD 5

void writeArrayToFile(double* array, int len, string fileName) {
    ofstream myfile;
    myfile.open (fileName);
    myfile << "shoulder_length: " << shoulder_length << endl;
    myfile << "arm_length: "      << arm_length      << endl;
    myfile << "forearm_length: "  << forearm_length  << endl;
    myfile << "precision: "       << precision       << endl;
    myfile << "leng: "            << leng            << endl;
    myfile << "edge: "            << edge            << endl << endl;
    for (int i = 0; i < len; i++) {
        myfile << array[i] << endl;
    }
    myfile.close();
} 

double* readFileToArray(string fileName, int arrayLen) {
    string line;
    ifstream myfile (fileName);
    double* arrayNew = new double[arrayLen];
    if (myfile.is_open())
    {
        int index = 0;
        int skipIndex = 0;
        while (skipIndex <= 6 && getline(myfile,line))
        {
            skipIndex++;
        }
        while ( getline (myfile,line) )
        {
            arrayNew[index] = stod(line);
            index++;
        }
        myfile.close();
        return arrayNew;
    }

    else cout << "Unable to open file: " << fileName << endl; 
    return new double[1];
}

int getFileSize(string fileName) {
    string line;
    ifstream myfile (fileName);
    int number_of_lines = 0;
    if (myfile.is_open())
    {
        while ( getline (myfile,line) )
        {
        ++number_of_lines;
        }
        myfile.close();
        std::cout << "Length of array: " << number_of_lines - 7 << endl;
        return number_of_lines - 7;
    }

    else cout << "Unable to open file"; 
    return -1;
}