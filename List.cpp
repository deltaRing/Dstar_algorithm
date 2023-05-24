//
//  List.cpp
//  GaoBuLaiShiBuKeNengDa
//
//  Created by 向宇涛 on 2021/8/3.
//

#include "List.hpp"

List * InsertList(List * openList, State * state){
    if (openList == NULL){
        openList = new List;
        openList->state = state;
        openList->next = NULL;
        return openList;
    }
    List * tempList = openList;
    while (tempList->next != NULL){
        // Set 要求结构体地址保持不一致
        if (tempList->state == state){
            return openList; // 傻逼设定
        }
        tempList = tempList->next;
    }
    // 遍历下去
    List * _resultList_ = new List;
    tempList->next = _resultList_;
    _resultList_->state = state;
    return openList;
}

List * RemoveList(List * openList, State * state){
    List * tempList = openList;
    List * tempListbef = NULL;
    if (tempList == NULL){
        return NULL;
    }
    if (GetListSize(tempList) == 0){
        return NULL;
    }
    while (tempList != NULL){
        if (tempList->state == state){
            break;
        }
        tempListbef = tempList;
        tempList = tempList->next;
    }
    if (tempListbef == NULL){
        // 首节点
        if (tempList->next == NULL)
            return NULL;
        openList = tempList->next;
    }
    else if (tempList->next == NULL){
        // 尾节点
        if (tempListbef == openList)
            return NULL;
        tempListbef->next = NULL;
    }
    else{
        tempListbef->next = tempList->next;
    }
    return openList;
}

long GetListSize(List * openList){
    long _result_ = 0;
    List * tempList = openList;
    while (tempList != NULL){
        _result_++;
        tempList = tempList->next;
    }
    return _result_;
}
