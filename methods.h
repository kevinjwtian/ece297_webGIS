/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   methods.h
 * Author: tianji21
 *
 * Created on February 4, 2020, 9:26 PM
 */
#pragma once
#ifndef METHODS_H
#define METHODS_H
#include"point.hpp"
#include"LatLon.h"
#include <vector>
#include <algorithm>


//turn upper letters into lower
void gstringU2L(std::string& str);
//erase spaces in the given string
void gtrim(std::string& str);
//to erase duplicate elements in the given list
void gunique(std::vector<int>& list);

//bool cmp_Inter_F(const Inter_F& elm_1, const Inter_F& elm_2);//used for comparing in m3

// int try_segment(Inter_Info &from_inter_info, int segment_idx, double turn_penalty); 
#endif /* METHODS_H */

