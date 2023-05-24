//
//  Map.cpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#include "Map.hpp"

void MAP::init_map(int row, int col){
    this->map = new State * [row];
    this->realmap = new State * [row];
    this->row = row;
    this->col = col;
    for (int ii = 0; ii < row; ii++){
        this->map[ii] = new State[col];
        this->realmap[ii] = new State[col];
        for (int jj = 0; jj < col; jj++){
            this->map[ii][jj].set_xy(ii, jj);
            this->realmap[ii][jj].set_xy(ii, jj);
            this->map[ii][jj].state = _UKWN_;
            this->realmap[ii][jj].state = _PATH_;
        }
    }
}

neighbors * MAP::get_neighbors(State * state){
    neighbors * state_list = NULL;
    int length = 0;
    for (int ii = state->x - 1; ii <= state->x + 1; ii++){
        for (int jj = state->y - 1; jj <= state->y + 1; jj++){
            if (ii == state->x && jj == state->y)
                continue;
            if (ii < 0 || ii >= row)
                continue;
            if (jj < 0 || jj >= col)
                continue;
            length += 1;
        }
    }
    state_list = new neighbors[length];
    int index = 0;
    for (int ii = state->x - 1; ii <= state->x + 1; ii++){
        for (int jj = state->y - 1; jj <= state->y + 1; jj++){
            if (ii == state->x && jj == state->y)
                continue;
            if (ii < 0 || ii >= row)
                continue;
            if (jj < 0 || jj >= col)
                continue;
            state_list[index].neighbor = &this->map[ii][jj];
            state_list[index].length = length;
            index++;
        }
    }
    return state_list;
}

void MAP::print_map(){
    for (int ii = 0; ii < row; ii++){
        for (int jj = 0; jj < col; jj++){
            if (this->map[ii][jj].state == _PATH_)
                cout << _Symbol_PATH_;
            if (this->map[ii][jj].state == _BARR_)
                cout << _Symbol_BARRIER_;
            if (this->map[ii][jj].state == _ROBT_)
                cout << _Symbol_ROBOT_;
            if (this->map[ii][jj].state == _DEST_)
                cout << _Symbol_DEST_;
            if (this->map[ii][jj].state == _ROUT_)
                cout << _Symbol_ROUTE_;
            if (this->map[ii][jj].state == _UKWN_)
                cout << _Symbol_UNKNOWN_;
        }
        cout << endl;
    }
}

void MAP::set_obstacle(ArrayXXi pointList){
    if (pointList.rows() != 2 && pointList.cols() != 2)
        return; // invalid
    if (pointList.rows() == 2){
        for (int ii = 0; ii < pointList.cols(); ii++){
            int x = pointList(0, ii);
            int y = pointList(1, ii);
            this->realmap[x][y].set_state(_BARR_);
        }
    }else{
        for (int ii = 0; ii < pointList.rows(); ii++){
            int x = pointList(ii, 0);
            int y = pointList(ii, 1);
            this->realmap[x][y].set_state(_BARR_);
        }
    }
}

void MAP::set_robot(Vector2i location){
    int x = location(0);
    int y = location(1);
    this->map[x][y].state = _ROBT_;
    this->realmap[x][y].state = _ROBT_;
}

void MAP::set_destination(Vector2i location){
    int x = location(0);
    int y = location(1);
    this->map[x][y].state = _DEST_;
    this->realmap[x][y].state = _DEST_;
}

bool MAP::isOver(Vector2i robot, Vector2i destination){
    if (robot == destination)
        return true;
    return false;
}

void MAP::update(){
    for (int ii = 0; ii < row; ii++){
        for (int jj = 0; jj < col; jj++){
            this->map[ii][jj].reset();
        }
    }
}

// where the
State * MAP::execute_action(State * robotLocation){
    State * temp = robotLocation;
    while (temp->parent->state == _PATH_ || temp->parent->state == _DEST_ || temp->parent->state == _UKWN_ || temp->parent->state == _ROUT_){
        temp->parent->state = _ROBT_;
        temp->state = _PATH_;
        temp = temp->parent;
        Vector2i _result_ = Vector2i(temp->x, temp->y);
        update_view(_result_);
        if (temp->parent == NULL){
            break;
        }
    }
    return temp;
}

void MAP::update_view(Vector2i location, int Range){
    int iterStartX = location(0) - Range;
    int iterStartY = location(1) - Range;
    int iterEndX = location(0) + Range;
    int iterEndY = location(1) + Range;
    
    if (iterStartX < 0) iterStartX = 0;
    if (iterStartY < 0) iterStartY = 0;
    if (iterEndX > row) iterEndX = row;
    if (iterEndY > col) iterEndY = col;
    
    for (int ii = iterStartX; ii < iterEndX; ii++){
        for (int jj = iterStartY; jj < iterEndY; jj++){
            if (ii == location(0) && jj == location(1))
                continue;;
            this->map[ii][jj].state = this->realmap[ii][jj].state;
        }
    }
}
