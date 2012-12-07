/***************************************************************************
 *   Copyright (C) 2005-2011 LpzRobots development team                    *
 *    Georg Martius  <georg dot martius at web dot de>                     *
 *    Frank Guettler <guettler at informatik dot uni-leipzig dot de        *
 *    Frank Hesse    <frank at nld dot ds dot mpg dot de>                  *
 *    Ralf Der       <ralfder at mis dot mpg dot de>                       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                         *
 ***************************************************************************/

#include <stdio.h>
#include <cmath>
#include <assert.h>
#include "FernandoController.h"
#include <selforg/controller_misc.h>
#include "DN.h"

using namespace std;
using namespace matrix;

int t = 0; 

FernandoController::FernandoController(unsigned long int controlmask, function func)
  : AbstractController("FernandoController", "$Id$"),
    controlmask(controlmask) {
  phase=0;
  addParameterDef("period", &period,50);
  addParameterDef("phaseshift", &phaseShift, 1);
  if(func==Impulse)
    addParameterDef("impulswidth", &impulsWidth, 0.5);
  addParameterDef("amplitude", &amplitude, 1);
  switch(func){
  case Sine:
    osci=sine; break;
  case SawTooth:
    osci=sawtooth; break;
  case Impulse:
    osci=impuls; break;
  default:
    assert("Unknown function type");
  }

  number_sensors=0;
  number_motors=0;  


};

/** initialisation of the controller with the given sensor/ motornumber 
    Must be called before use.
*/
void FernandoController::init(int sensornumber, int motornumber, RandGen* randGen){
  number_sensors=sensornumber;
  number_motors=motornumber;
  M.set(motornumber,sensornumber);
  smMem.set(motornumber + sensornumber +1, 1); //Create a matrix that stores the previous motor action. 
  dn = new DN(50); //Construct the Darwinian Neurodynamic controller. 
  dn->init(50, sensornumber + motornumber);//Initialize it with 10 random curiosity loops.  
  addInspectable(dn);
  //addConfigurable(dn);
	
};
  
void FernandoController::step(const sensor* sensors, int sensornumber, 
			  motor* motors, int motornumber) {
  t = t +1; 
  Matrix s(sensornumber,1,sensors);
  Matrix a = M*s;
  a = a.map(tanh) + a.map(random_minusone_to_one)*1.0; 
  a.convertToBuffer(motors,motornumber); 

  //motorMem.set(motors);  
  Matrix m(motornumber,1,motors);  
  
  //Modify the predictors using self-supervised learning which requires remembering the previous input vector. 
  dn->updatePredictions(smMem, s,m); 

  //Make prediction from this s(t), m(t) state of s(t+1) and m(t+1). 
  dn->makePredictions(s, m);  


	
  //Store the sm(t) so it can be used in the next ts to update predictors using self-supervised leanring.      
     matrix::Matrix sm = s.above(m);
     matrix::Matrix f; 
     f.set(1,1); 
     f.val(0,0) = 1; 
     sm = sm.above(f); 
     smMem = sm; 
	

//  stepNoLearning(sensors, sensornumber, motors, motornumber);
};

void FernandoController::stepNoLearning(const sensor* sensors, int number_sensors, 
				    motor* motors, int number_motors) {
  
  for (int i=0; i<min(number_motors,sizeof(controlmask)*8); i++){
    if(controlmask & (1<<i)){      
      motors[i]=amplitude*osci(phase + i*phaseShift*M_PI/4, impulsWidth);
    }else {
      motors[i]=0;
    }
  }  
  if(period!=0){
    phase += 2*M_PI/period;
    if(phase > 2*M_PI) phase -= 2*M_PI;
  }
};
 
 
double FernandoController::sine(double x, double _unused){
  return sin(x);
}
 
double FernandoController::sawtooth(double x, double _unused){
  while(x>M_PI) x-=2*M_PI;
  while(x<-M_PI) x+=2*M_PI;
  // x is centered around -PI and PI.
  if(x>-M_PI/2 && x <= M_PI/2)
    return x/M_PI;
  else{
    if(x<0)
      return -(x+M_PI)/M_PI;
    else
      return -(x-M_PI)/M_PI;
  }
}


double FernandoController::impuls(double x, double impulsWidth){
  while(x>M_PI) x-=2*M_PI;
  while(x<-M_PI) x+=2*M_PI;
  // x is centered around -PI and PI.
  // +-1 for |x| in ((0.5-impulswidth/2)*M_PI,(0.5+impulswidth/2)*M_PI]
  if(fabs(x) > (0.5-impulsWidth/2)*M_PI && fabs(x) < (0.5+impulsWidth/2)*M_PI)
    return sign(x);
  else
    return 0;
}




