//
//  DStar.hpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#ifndef DStar_hpp
#define DStar_hpp

#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "State.hpp"
#include "List.hpp"
#include "Map.hpp"

using namespace std;
using namespace Eigen;

class Dstar{
public:
    Dstar(int rows, int cols);
    double ProcessState();
    double MinKValue();
    void InsertNode(State * state, double h_new);
    void remove(State * state);
    void ModifyCost(State * state);
    void Modify(State * state);
    void run(Vector2i robotLocation, Vector2i destinationLocation, int robotViewRange);
    void ResetopenList();
    State * MinState();
    List * openList = NULL;
    
private:
    State * robotState;
    State * destinationState;
};

#endif /* DStar_hpp */
