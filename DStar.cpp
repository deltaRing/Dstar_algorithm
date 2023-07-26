//
//  DStar.cpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#include "DStar.hpp"

MAP maps;

Dstar::Dstar(int rows, int cols){
    maps.init_map(rows, cols);
}

double Dstar::ProcessState(){
    State * x = MinState();
    if (x == NULL)
        return -1.0;
    double old_k = MinKValue();
    remove(x);
    long size = GetListSize(openList);
    neighbors * neighbor = maps.get_neighbors(x);
    if (old_k < x->h){
        if (neighbor != NULL){
            for (int ii = 0; ii < neighbor[0].length; ii++){
                neighbors y = neighbor[ii];
                bool f1 = old_k >= y.neighbor->h;
                bool f2 = x->h > y.neighbor->h + x->cost(y.neighbor);
                if (f1 && f2){
                    x->parent = y.neighbor;
                    x->h = y.neighbor->h + x->cost(y.neighbor);
                }
            }
        }
    }
    else if (old_k == x->h){
        if (neighbor != NULL){
            for (int ii = 0; ii < neighbor[0].length; ii++){
                neighbors y = neighbor[ii];
                bool f1 = y.neighbor->t == _new_;
                bool f2 = y.neighbor->parent == x;
                bool f3 = y.neighbor->h != x->h + x->cost(y.neighbor);
                bool f4 = y.neighbor->parent != x;
                bool f5 = y.neighbor->h > x->h + x->cost(y.neighbor);
                bool f6 = y.neighbor != this->destinationState;
                if ((f1 || (f2 && f3) || (f4 && f5)) && f6){
                    y.neighbor->parent = x;
                    InsertNode(y.neighbor, x->h + x->cost(y.neighbor));
                }
            }
        }
    }
    else{
        if (neighbor != NULL){
            for (int ii = 0; ii < neighbor[0].length; ii++){
                neighbors y = neighbor[ii];
                bool f1 = y.neighbor->t == _new_;
                bool f2 = y.neighbor->parent == x;
                bool f3 = y.neighbor->h != x->h + x->cost(y.neighbor);
                if (f1 || (f2 && f3)){
                    y.neighbor->parent = x;
                    InsertNode(y.neighbor, x->h + x->cost(y.neighbor));
                }
                else {
                    bool f4 = y.neighbor->parent != x;
                    bool f5 = y.neighbor->h > x->h + x->cost(y.neighbor);
                    if (f4 && f5){
                        InsertNode(x, x->h);
                    }
                    else{
                        bool f7 = y.neighbor->parent != x;
                        bool f8 = x->h > y.neighbor->h + x->cost(y.neighbor);
                        bool f9 = y.neighbor->t == _close_;
                        bool f10 = y.neighbor->h > old_k;
                        if (f7 && f8 && f9 && f10){
                            InsertNode(y.neighbor, y.neighbor->h);
                        }
                    }
                }
            }
        }
    }
    delete neighbor;
    size = GetListSize(openList);
    return MinKValue();
}

State * Dstar::MinState(){
    if (GetListSize(openList) == 0)
        return NULL;
    List * tempList = openList;
    double tmp;
    tmp = tempList->state->k;
    State * _result_ = tempList->state;
    for (; tempList != NULL; tempList = tempList->next){
        if (tmp > tempList->state->k){
            tmp = tempList->state->k;
            _result_ = tempList->state;
        }
    }
    return _result_;
}

double Dstar::MinKValue(){
    if (GetListSize(openList) == 0)
        return -1.0;
    List * tempList = openList;
    double tmp;
    tmp = tempList->state->k;
    for (; tempList != NULL; tempList = tempList->next){
        if (tmp > tempList->state->k){
            tmp = tempList->state->k;
        }
    }
    return tmp;
}

// 记得更新
void Dstar::InsertNode(State * state, double h_new){
    if (state->t == _new_)
        state->k = h_new;
    else if (state->t == _open_)
        state->k = min(h_new, state->k);
    else if (state->t == _close_)
        state->k = min(h_new, state->h);
    state->h = h_new;
    state->t = _open_;
    openList = InsertList(openList, state);
}

void Dstar::remove(State * state){
    if (state->t == _open_){
        state->t = _close_;
    }
    List * tempList = openList;
    for (; tempList != NULL; tempList = tempList->next){
        if (tempList->state == state){
            openList = RemoveList(openList, state);
        }
    }
}
void Dstar::ModifyCost(State * state){
    if (state->parent == NULL)
        return;
    if (state->t == _close_){
        InsertNode(state, state->parent->h + state->cost(state->parent));
    }
}

void Dstar::Modify(State * state){
    ModifyCost(state);
    double k_min;
    while (true){
        k_min = ProcessState();
        if (state->h <= k_min)
            break;
    }
}

void Dstar::run(Vector2i robotLocation, Vector2i destinationLocation, int robotViewRange = 3){
    int robotLx = robotLocation(0);
    int robotLy = robotLocation(1);
    int destiLx = destinationLocation(0);
    int destiLy = destinationLocation(1);
    maps.set_robot(robotLocation);
    maps.set_destination(destinationLocation);
    this->robotState = &maps.map[robotLx][robotLy];
    this->destinationState = &maps.map[destiLx][destiLy];
    InsertNode(destinationState, 0.0);
    
    ArrayXXi barriers = ArrayXXi::Zero(2, 14);
    barriers << 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5,
                2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;
    maps.set_obstacle(barriers);
    
    ArrayXXi barriers2 = ArrayXXi::Zero(2, 20);
    barriers2 << 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 17, 17, 16, 16,
                2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 20, 21, 22;
    maps.set_obstacle(barriers2);
    maps.update_view(robotLocation, robotViewRange);
    
    while (true){
        ProcessState();
        if (this->robotState->t == _close_)
            break;
    }
    this->robotState->set_state(_ROBT_);
    State * s = this->robotState;
    while (s->parent != this->destinationState){
        s = s->parent;
        s->set_state(_ROUT_);
    }
    
    while (!maps.isOver(robotLocation, destinationLocation)){
        State * currentLocation = maps.execute_action(this->robotState);
        robotLocation = Vector2i(currentLocation->x, currentLocation->y);
        Modify(currentLocation);
    }
    
    s = this->robotState;
    while (s->parent != this->destinationState){
        s = s->parent;
        s->set_state(_ROUT_);
    }
    
    maps.print_map();
}

