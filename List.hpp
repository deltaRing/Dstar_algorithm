//
//  List.hpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#ifndef List_hpp
#define List_hpp

#include <cmath>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "State.hpp"
#include "List.hpp"
#include "Map.hpp"

using namespace std;
using namespace Eigen;

struct List{
    State * state = NULL;
    List * next = NULL;
};

List * InsertList(List * openList, State * state);
List * RemoveList(List * openList, State * state);
long GetListSize(List * openList);

#endif /* List_hpp */
