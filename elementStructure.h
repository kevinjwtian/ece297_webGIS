/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   elementStructure.h
 * Author: zhongh22
 *
 * Created on January 27, 2020, 8:39 PM
 */

#ifndef ELEMENTSTRUCTURE_H
#define ELEMENTSTRUCTURE_H

#include "LatLon.h"
#include "StreetsDatabaseAPI.h"
#include <vector>
class intersection {
public:
    intersection(int index);
    virtual ~intersection();
    LatLon getIntersectionPos();
    bool operator==(intersection& RHS);
private:
    LatLon position;
    int intersectionIndex;
};

class streetSeg {
public:
    streetSeg(int index);
    ~streetSeg();
    OSMID getOSMID();
    int getStartIntersectionIndex();
    int getEndIntersectionIndex();
    bool getIsOneWay();
    int getCurvePoint();
    int getSpeedLimit();
    int getStreetID();
    
private:
    InfoStreetSegment info;
    int streetSegIndex;
};

class street {
public:
    street(int index);
    ~street();
private:
    std::string name;
    int streetIndex;
};

class poi {
public:
    poi(int index);
    ~poi();
private:
    std::string type;
    std::string name;
    LatLon position;
    OSMID osmid;
    int poiIndex;
};

class intersectionPairStruct{
public:
    intersectionPairStruct(int index);
    ~intersectionPairStruct();
    bool getIfOneway();
    void setIfOneway(bool change);
private:
    int intersectionIndex;
    bool fromHereToThereOnly;
};

#endif /* ELEMENTSTRUCTURE_H */

