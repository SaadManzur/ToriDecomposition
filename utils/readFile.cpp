#include "readFile.h"

void readFile(const string filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F) {

    int indexOfExtension = filename.find_last_of('.');
    string extension = filename.substr(indexOfExtension+1, 3);

    if(extension == "obj") {

        igl::readOBJ(filename, V, F);
    }
    else if (extension == "off") {

        igl::readOFF(filename, V, F);
    }
    else {

        cout << "Unknown extension. Only supports obj, off for now" << endl;
    }
}