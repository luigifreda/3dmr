#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <iostream>
#include <cstring>

using namespace std;

#define RESOLUTION 1.0 // seconds

#define MAXIDLENESS 500.0 // seconds

int main (int argc, char **argv)
{   
    if (argc<2) {
        cout << "Use: " << argv[0] << " <filename> " << endl;
        exit(-1);
    }
    
    int n = (int)(MAXIDLENESS/RESOLUTION)+1;
    int v[n]; for (int k=0; k<n; k++) v[k]=0;
    
    ifstream f;
    f.open(argv[1]);
    
    double time,idleness; int robot,node,interferences; char ch;
    int sum=0;
    while (f.good()) {
        char line[80];
        f.getline(line,80);
        stringstream ss(line);
        // current_time, id_robot, goal, current_idleness[goal], interferences
        ss >> time; ss>>ch;
        ss >> robot; ss>>ch;
        ss >> node; ss>>ch;
        ss >> idleness; ss>>ch;
        ss >> interferences;
        
        // cout << idleness << endl;
        
        if (idleness>MAXIDLENESS) {
           cout << "WARNING: Idleness " << idleness << " out of range! Discarded!" << endl; 
        }
        else {
            int b = (int)(idleness/RESOLUTION);
            v[b]++; sum++;
        }
    }
    f.close();
    
    char nf[strlen(argv[1])+8], cnf[strlen(argv[1])+8];
    strcpy(nf,argv[1]);
    nf[strlen(nf)-4]='\0';
    strcpy(cnf,nf);
    strcat(nf,".hist");
    strcat(cnf,".chist");
    cout << "Histogram output file: " << nf << endl;
    ofstream of1; of1.open(nf);
    ofstream of2; of2.open(cnf);
    double c=0;
    for (int k=0; k<n; k++) {
        of1 << k*RESOLUTION << " " << (double)v[k]/sum << endl;
        c += (double)v[k]/sum;
        of2 << k*RESOLUTION << " " << c << endl;
    }
    
    of1.close();   of2.close();
    
}

/*
 * 
 * 
 * 
    gnuplot> plot ("results/idl_grid_4_RAND.csv.hyst") w li
    gnuplot> replot ("results/idl_grid_4_DTAG.csv_OLD.hyst") w li

 * 
 * 
 * 
 * 
 */


