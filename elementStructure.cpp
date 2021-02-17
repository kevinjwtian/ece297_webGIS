/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   elementStructure.cpp
 * Author: zhongh22
 * 
 * Created on January 27, 2020, 8:39 PM
 */

#include "elementStructure.h"

intersection::intersection(int index) {
  position = getIntersectionPosition(index);
  intersectionIndex = index;
}

intersection::~intersection() {
  
}

LatLon intersection::getIntersectionPos(){
  return position;
}


streetSeg::streetSeg(int index) {
  info = getInfoStreetSegment(index);
  streetSegIndex = index;
}

streetSeg::~streetSeg() {
  
}

OSMID streetSeg::getOSMID(){
  return info.wayOSMID;
}

int streetSeg::getStartIntersectionIndex(){
  return info.from;
}

int streetSeg::getEndIntersectionIndex(){
  return info.to;
}

bool streetSeg::getIsOneWay(){
  return info.oneWay;
}

int streetSeg::getCurvePoint(){
  return info.curvePointCount;
}

int streetSeg::getSpeedLimit(){
  return info.speedLimit;
}

int streetSeg::getStreetID(){
  return info.streetID;
}

street::street(int index) {
    name = getStreetName(index);
}

street::~street() {
}

poi::poi(int index) {
    type = getPointOfInterestType(index);
    name = getPointOfInterestName(index);
    position = getPointOfInterestPosition(index);
    osmid = getPointOfInterestOSMNodeID(index);
    poiIndex = index;
}

poi::~poi() {
}

intersectionPairStruct::intersectionPairStruct(int index){
    intersectionIndex = index;
}

bool intersectionPairStruct::getIfOneway(){
    return fromHereToThereOnly;
}

void intersectionPairStruct::setIfOneway(bool change){
    fromHereToThereOnly = change;
}

intersectionPairStruct::~intersectionPairStruct(){
}