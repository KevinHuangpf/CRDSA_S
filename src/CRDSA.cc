/*
 * CRDSA.cpp
 *
 *  Created on: Dec 19, 2016
 *      Author: etienne
 */

#include <CRDSA.h>

CRDSA::CRDSA(unsigned int nOfInstances, unsigned int nOfSlots)
{
    mNOfInstances = nOfInstances;
    mNOfSlots = nOfSlots;
}

std::vector<int> CRDSA::generateLocArray(unsigned int numRand, unsigned int ID)
{
    std::vector<int> locArray;
    unsigned int Q = 16807;
    unsigned int M = 2147483647;
    int nOfInitStages = 9;
    unsigned int x = (numRand << 20) | (mNOfInstances << 16) | ID;
    for(int i = 0; i < nOfInitStages; i++)
    {
        x = (Q * x) % M;
    }
    for(int i = 0; i < mNOfInstances; i++)
    {
        x = (Q * x) % M;
        locArray.push_back((x >> 16) % mNOfSlots);
    }
    return locArray;
}
