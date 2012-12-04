/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *  

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                   Echo State Network Header File                          *
 *                                                                           *
 *****************************************************************************/
#ifndef __ESN_H
#define __ESN_H


#include <stdio.h>
#include "abstractmodel.h"

/**
 * class for robot control with sine, sawtooth and impuls  
 * 
 * period is the length of the period in steps and 
 * phaseshift is the phase difference between channels given in Pi/2
 */
class ESN : public AbstractModel {
public:
  enum function {Sine, SawTooth, Impulse};

  /**     
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  ESN();

  

protected:

  //
};

#endif 

