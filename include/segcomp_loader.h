#ifndef SEGCOMP_LOADER_H_
#define SEGCOMP_LOADER_H_

#include <string>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>

/* The number of pixel-rows in an input image */
int ROWS = 512;
/* The number of pixel-columns in an input image */
int COLS = 512;

void loadSegCompPerceptron3(std::string file_name, int rows, int cols, float* P[3]) {
    for (int i=0; i<3; i++)
      P[i]=(float *)calloc(rows*cols,sizeof(float)); 

    // fprintf(stderr, "output resized ....\n");
    fprintf(stderr, "file name: %s \n", file_name.c_str());

    std::ifstream input(file_name.c_str(), std::ios::in);
    if(!input.good()){
        std::cerr << "Could not read file: " << file_name << std::endl;
    }

    int i=0;
    std::string line;
    while(getline(input, line)) {
        std::stringstream ss(line);
        // float px, py, pz;
        // ss >> px >> py >> pz;
        // if(i==0) {
        //     std::cout << px << ", " << py << ", " << pz << std::endl;
        // }
        std::string spx, spy, spz;
        std::getline(ss, spx, ' ');
        std::getline(ss, spy, ' ');
        std::getline(ss, spz, ' ');
        float px, py, pz;
        if(spx=="NaN" || spy=="NaN" || spz=="NaN") {
            px = py = pz = std::numeric_limits<float>::quiet_NaN();
        } else {
            px = std::stof(spx);
            py = std::stof(spy);
            pz = std::stof(spz);
        }
        P[0][i] = px;
        P[1][i] = py;        
        P[2][i] = pz;            
        i++;
    }
    input.close();

    fprintf(stderr, "loaded point num: %d vs: %d (%d x %d) \n", i, rows*cols, rows, cols);
}


#endif
