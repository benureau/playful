/***************************************************************************
 *   Copyright (C) 2005 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    fhesse@informatik.uni-leipzig.de                                     *
 *    der@informatik.uni-leipzig.de                                        *
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

#include "normalsim.h"

#include <ode_robots/playground.h>

#include <selforg/sinecontroller.h>
#include <selforg/ffnncontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>

#include <ode_robots/barrel2masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

#include "ctrloutline.h"

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;
using namespace std;

class ClosedLoopController : public AbstractController {
public:
  ClosedLoopController()
    : AbstractController("ClosedLoopController", "0.6"){
    addParameterDef("coupling1",   &coupling1, 1, -1, 2, "diagonals of the matrix C");
    addParameterDef("coupling2",   &coupling2, .1, -1, 2, "non-diagonals of the matrix C");

    addInspectableMatrix("C", &C, false, "controller/coupling matrix");   
    addInspectableMatrix("h", &h, false, "bias term");   
  }
  virtual void init(int sensornumber, int motornumber, RandGen* randGen = 0){
      assert(sensornumber >=2 && motornumber >=2);
      C.set(motornumber, sensornumber);
      h.set(motornumber, 1);
      notifyOnChange("dummy");
  }

  virtual ~ClosedLoopController(){}
  virtual int getSensorNumber() const { return C.getM(); }
  virtual int getMotorNumber() const  { return C.getN(); }
  virtual void step(const sensor* s, int number_sensors, motor* m, int number_motors){
    stepNoLearning(s,number_sensors, m, number_motors);
  }
  virtual void stepNoLearning(const sensor* s, int number_sensors, motor* m, int number_motors){
    const Matrix x(number_sensors,1,s);    
    const Matrix& y = (C*x+h).map(tanh);
    y.convertToBuffer(m, number_motors);
  }
  virtual void setC(const matrix::Matrix& _C){
    assert(C.getM() == _C.getM() && C.getN() == _C.getN());
    C=_C;
  }
  virtual void seth(const matrix::Matrix& _h){
    assert(h.getM() == _h.getM() && h.getN() == _h.getN());
    h=_h;
  }
  virtual const matrix::Matrix& getC(){
    return C;
  }
  virtual const matrix::Matrix& geth(){
    return h;
  }

  // is called when a parameter is changed
  virtual void notifyOnChange(const paramkey& key){
    C.val(0,0) = coupling1;
    C.val(1,1) = coupling1;
    C.val(0,1) = coupling2;
    C.val(1,0) = -coupling2;
    h.val(0,0) = 0;
    h.val(1,0) = 0;
    cout << "new C matrix\n" << C << endl;
    cout << "new h vector:\n" << (h^T) << endl;
  }
  virtual bool store(FILE*) const {return false; }
  virtual bool restore(FILE*) {return false; } 
protected:
  matrix::Matrix C; // Controller Matrix
  matrix::Matrix h; // bias vector
  paramval coupling1;
  paramval coupling2;
};


// starting function (executed once at the beginning of the simulation loop)
void NormalSim::start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                      GlobalData& global) {
  int num_barrels=1;
  
  setCameraHomePos(Pos(-1.29913, 6.06201, 1.36947),  Pos(-165.217, -9.32904, 0));
  setCameraMode(Follow);
  // initialization
  global.odeConfig.setParam("noise",0.01);
  //  global.odeConfig.setParam("gravity",-10);
  global.odeConfig.setParam("controlinterval",1);
  global.odeConfig.setParam("cameraspeed",200);
  
  // add a new parameter to be configured on the console
  global.odeConfig.addParameterDef("friction", &friction, 0.05, "rolling friction coefficient");
    
  /* * * * BARRELS * * * */
  for(int i=0; i< num_barrels; i++){
    //****************
    Sphererobot3MassesConf conf = Barrel2Masses::getDefaultConf();  
    conf.pendularrange  = 0.3;//0.15; 
    conf.motorpowerfactor  = 200;//150; 
    conf.motorsensor=false;
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection, Sensor::X | Sensor::Y));
    conf.spheremass   = 1;
    robot = new Barrel2Masses ( odeHandle, osgHandle.changeColor(Color(0.0,0.0,1.0)), 
                                conf, "Barrel", 0.4); 
    
    robot->place (osg::Matrix::rotate(M_PI/2, 1,0,0) /* rotate to laying postion */ 
		  * osg::Matrix::rotate(M_PI/4, 0,1,0) /* both axis at 45Deg*/ 
		  * osg::Matrix::translate(0,0,.15+2*i));
    
  
    if(mode==NoLearn){
      controller = new ClosedLoopController();        
    } else {
      controller = new CtrlOutline();        
    }
    
    AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise());
    
    //      OdeAgent* agent = new OdeAgent ( PlotOption(File, Robot, 1) );
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , robot , wiring );

    // controller is now initialized and we can modify the matrices
    Matrix C(2,2);
    switch(mode){
    case NoLearn: break;
    case LearnContHS:
      controller->setParam("TLE",0);
      controller->setParam("epsA",0.1);
      controller->setParam("epsC",0);
      controller->setParam("creativity",0);      
      C.val(0,0)= .7;
      C.val(0,1)= .7;
      C.val(1,0)= -.4;
      C.val(1,1)= .4;
      if(dynamic_cast<CtrlOutline*>(controller)){
        dynamic_cast<CtrlOutline*>(controller)->setC(C);
      }      
      break;
    case LearnHS:
      controller->setParam("TLE",0);      
      controller->setParam("epsA",0.1);
      controller->setParam("epsC",0.1);
      controller->setParam("creativity",0);      
      global.odeConfig.setParam("friction", 0.1);
      break;
    case LearnHK:
      setParam("friction", 0.1);
      controller->setParam("TLE",1);      
      break;
      // // controller is now initialized and we can modify the matrices
      // Matrix C(2,2);
      // C.val(0,0)= -5;
      // C.val(0,1)= -5;
      // C.val(1,0)=  5;
      // C.val(1,1)= -5;
      // if(dynamic_cast<CtrlOutline*>(controller)){
      //   dynamic_cast<CtrlOutline*>(controller)->setC(C);
      // }
    }

    //  agent->setTrackOptions(TrackRobot(true, false, false, "", 50));
    global.agents.push_back ( agent );
    global.configs.push_back ( controller );
  }

}

void NormalSim::addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
  if(!pause){
    if(friction>0){
      Pos vel = robot->getMainPrimitive()->getAngularVel();
      robot->getMainPrimitive()->applyTorque(-vel*friction);
    }
  }
}

void NormalSim::bindingDescription(osg::ApplicationUsage & au) const {
  au.addKeyboardMouseBinding("Barrel: r","Randomize matrix C"); 
  au.addKeyboardMouseBinding("Barrel: R","Randomize matrix C and h");
  au.addKeyboardMouseBinding("Barrel: s","Scatter matrix C (0.2)");
  au.addKeyboardMouseBinding("Barrel: S","Scatter matrix C (0.5)");
  au.addKeyboardMouseBinding("Barrel: L","Initialize lolloping mode");
  
  au.addKeyboardMouseBinding("Barrel: x","add torque counter-clockwise");
  au.addKeyboardMouseBinding("Barrel: X","add torque clockwise");
};
  
// add own key handling stuff here, just insert some case values
bool NormalSim::command(const OdeHandle&, const OsgHandle&, GlobalData& globalData, 
                        int key, bool down){
  Matrix h(2,1); // zero filled vector
  Matrix C(2,2);
  bool changedCorH=false;
  double scatter=0.2;
  if (down) { // only when key is pressed, not when released
    switch ( (char) key ) {
    case 'R': 
      h = h.map(random_minusone_to_one)*3; // random matrix from +-3
      // deliberately go though to 'r'
    case 'r': 
      C = C.map(random_minusone_to_one)*5; // random matrix from +-3
      changedCorH = true;
      break;
    case 'S': 
      scatter = 0.5;
      // deliberately go though to 's'
    case 's': 
      h = dynamic_cast<ClosedLoopController*>(controller) ? 
        dynamic_cast<ClosedLoopController*>(controller)->geth() : 
        dynamic_cast<CtrlOutline*>(controller)->geth();
      C = dynamic_cast<ClosedLoopController*>(controller) ? 
        dynamic_cast<ClosedLoopController*>(controller)->getC() :  
        dynamic_cast<CtrlOutline*>(controller)->getC();
      C += C.map(random_minusone_to_one)*scatter; // add random matrix from +-scatter
      changedCorH = true;      
      break;      
    case 'L': 
      C.val(0,0)=2;
      C.val(0,1)=2;
      C.val(1,0)=-1;
      C.val(1,1)=-1;
      changedCorH = true;      
      break;      
    case 'x' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , 100 , 0 ); break;
    case 'X' : dBodyAddTorque ( robot->getMainPrimitive()->getBody() , 0 , -100 , 0 ); break;
    }
    if(changedCorH){
      cout << "new C matrix\n" << C << endl;
      cout << "new h vector:\n" << (h^T) << endl;        
      if(mode==NoLearn){
        if(dynamic_cast<ClosedLoopController*>(controller)){
          dynamic_cast<ClosedLoopController*>(controller)->setC(C);
          dynamic_cast<ClosedLoopController*>(controller)->seth(h);  
        }
      }else{
        if(dynamic_cast<CtrlOutline*>(controller)){
          dynamic_cast<CtrlOutline*>(controller)->setC(C);
          dynamic_cast<CtrlOutline*>(controller)->seth(h);  
        }
      }
      return true;
    }
  }
  return false;
}
  

