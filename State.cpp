//
//  State.cpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#include "State.hpp"

void State::set_xy(int x, int y){
    this->x = x;
    this->y = y;
}

void State::reset(){
    this->state = _PATH_;
    this->t = _new_;
    this->h = 0.0;
    this->k = 0.0;
    this->parent = NULL;
}

double State::cost(State * state){
    if (this->state == _BARR_ || state->state == _BARR_)
        return INFINITY;
    
    int deltaX = abs(this->x - state->x);
    int deltaY = abs(this->y - state->y);
    double costs = deltaX + deltaY > 1?_Value_PATH_CROSS_ACTION_:1.0;
    double value;
    if (this->state == _PATH_)
        value = _Value_PATH_;
    else if (this->state == _ROBT_)
        value = _Value_ROBOT_;
    else if (this->state == _ROUT_)
        value = _Value_PATH_;
    else if (this->state == _BARR_)
        value = _Value_BARRIER_; // 不该发生
    else if (this->state == _DEST_)
        value = _Value_DESTINATION_;
    else if (this->state == _UKWN_)
        value = _Value_UNKNOWN_;
    else{
        value = _Value_PATH_;
    }
    return costs * value;
}

void State::set_state(int state){
    this->state = state;
}
