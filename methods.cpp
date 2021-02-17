/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */


#include "methods.h"

//to erase duplicate elements in the given list
void gunique(std::vector<int>& list)
{
	std::sort(list.begin(), list.end());
	std::vector<int>::iterator iter = std::unique(list.begin(), list.end());
	list.erase(iter, list.end());
}
