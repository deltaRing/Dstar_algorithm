//
//  State.hpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#ifndef State_hpp
#define State_hpp

#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <iostream>

using namespace std;
using namespace Eigen;

// 定义地图的状态中
#define _PATH_ 0
#define _BARR_ 1
#define _ROBT_ 2
#define _DEST_ 3
#define _ROUT_ 4
#define _UKWN_ 5

#define _new_ 0
#define _open_ 1
#define _close_ 2

#define _Value_PATH_ 1.0
#define _Value_PATH_CROSS_ACTION_ sqrt(2)
#define _Value_BARRIER_ INFINITY
#define _Value_ROBOT_ 0.0
#define _Value_DESTINATION_ 0.0
#define _Value_ROUTE_ 1.0
#define _Value_ROUTE_CROSS_ACTION_ sqrt(2)
#define _Value_UNKNOWN_ 1.5

struct State{
    int x = 0;
    int y = 0;
    int state = _PATH_;
    int t = _new_;
    double h = 0.0;
    double k = 0.0;
    State * parent = NULL;
    void reset();
    void set_xy(int x, int y);
    double cost(State * state);
    void set_state(int state);
};

#endif /* State_hpp */
