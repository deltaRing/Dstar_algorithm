//
//  Map.hpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#ifndef Map_hpp
#define Map_hpp

#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>
#include "State.hpp"

#define _Symbol_PATH_ '.'
#define _Symbol_BARRIER_ '#'
#define _Symbol_ROBOT_ '+'
#define _Symbol_DEST_ 'X'
#define _Symbol_ROUTE_ 'o'
#define _Symbol_UNKNOWN_ '_'

using namespace std;
using namespace Eigen;

struct neighbors{
    State * neighbor; // point to map
    int length;
};

struct MAP{
    int row;
    int col;
    State ** map;
    State ** realmap;
    State * execute_action(State * robotLocation);
    void init_map(int row, int col);
    neighbors * get_neighbors(State * state);
    void print_map();
    void update();
    void update_view(Vector2i location, int Range=5);
    void set_obstacle(ArrayXXi pointList);
    void set_robot(Vector2i location);
    void set_destination(Vector2i location);
    bool isOver(Vector2i robot, Vector2i destination);
};

#endif /* Map_hpp */
