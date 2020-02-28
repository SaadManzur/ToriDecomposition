#ifndef READ_FILE_H
#define READ_FILE_H

#include <iostream>
#include <string>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
using namespace std;

void readFile(const string filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F);

#endif