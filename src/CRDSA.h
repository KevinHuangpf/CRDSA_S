/*
 * CRDSA.h
 *
 *  Created on: Dec 19, 2016
 *      Author: etienne
 */

#ifndef CRDSA_H_
#define CRDSA_H_

#include <vector>

class CRDSA
{
public:

    /// nOfInstances is an integer between 0 and 15
    /// nOfSlots is an integer between 0 and 16777215
    CRDSA(unsigned int nOfInstances, unsigned int nOfSlots);

    /// numRand is a pseudo genrated integer beween 0 and 4095
    /// ID is the mac address of the given terminal, between 0 and 65535
    std::vector<int> generateLocArray(unsigned int numRand, unsigned int ID);

private:

    // The number of instances of a given packet send within a frame
    int mNOfInstances;
    // The number of slots in a frame
    int mNOfSlots;
};

#endif /* CRDSA_H_ */
