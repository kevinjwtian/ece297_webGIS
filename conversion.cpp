/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   conversion.cpp
 * Author: zhongh22
 * 
 * Created on January 26, 2020, 6:27 PM
 */

#include "conversion.h"

double degree_to_radian(double degree){
    return degree * 0.017453292519943295769236907684886;
}

double radian_to_degree(double radian){
    return radian / 0.017453292519943295769236907684886;
}

/*intersection returnIntersectionWRef(int index, std::vector<intersection>& intersectionVecRef){
    return intersectionVecRef[index];
}

streetSeg returnStreetSegWRef(int index, std::vector<streetSeg>& streetSegVecRef){
    return streetSegVecRef[index];
}

street returnStreetWRef(int index, std::vector<street>& streetVecRef){
    return streetVecRef[index];
}

poi returnPoiWRef(int index, std::vector<poi>& poiVecRef){
    return poiVecRef[index];
}

bool intersection::operator==(intersection& RHS){
        if( (position.lat() == (RHS.position).lat()) && 
            (position.lon() == (RHS.position).lon()) ) return true;
        return false;
}
 * */