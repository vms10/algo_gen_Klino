
//#include <sstream>
#include <iostream>
#include <fstream>
//#include <cstdlib>
//#include <cstdarg>
//#include <stdlib.h>
//#include <math.h>


using namespace std; 

int main (int argc, const char* argv[]){
    ofstream BestIndividualFile;
    for (int iterations = 0; iterations <= 4; iterations++){
        BestIndividualFile.open("archivo_de_prueba.txt", fstream::app);
        BestIndividualFile << iterations << endl;
	    BestIndividualFile.close();
    }
    return 0;
}