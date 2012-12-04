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
 *
 ***************************************************************************/

#include <selforg/noisegenerator.h>

#include <ode_robots/simulation.h>
#include <ode_robots/odeagent.h>
#include <selforg/one2onewiring.h>
#include <selforg/derivativewiring.h>
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>
#include <ode_robots/octaplayground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/passivebox.h>

//#include <selforg/deprivation.h>
#include <selforg/invertnchannelcontroller.h>
//#include <selforg/invertmotorspace.h>
#include <selforg/invertmotornstep.h>
#include <selforg/sinecontroller.h>
#include <selforg/sos.h>

#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/sliderwheelie.h>
#include <ode_robots/schlangeservo.h>
#include <ode_robots/schlangeservo2.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/plattfussschlange.h>
#include <ode_robots/hurlingsnake.h>
//#include <ode_robots/forcedsphere.h>
#include <ode_robots/nimm2.h>
#include <ode_robots/schlangeservo.h>

#include <selforg/onecontrollerperchannel.h>

using namespace lpzrobots;

double eps = 1;
bool track = false;

struct ControlGen : public ControllerGenerator {
  virtual ~ControlGen(){}
  virtual AbstractController* operator()( int index) { 
    AbstractController* c;
    c= new Sos(0.01);    
    c->setParam("epsC",eps);
    c->setParam("epsA",.1);
    return c; 
  }
};



class ThisSim : public Simulation {
public:

  AbstractController* controller;

  ThisSim(){
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");
  }  


  /// start() is called at the start and should create all the object (obstacles, agents...).
  virtual void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global){
    
    setCameraHomePos(Pos(1.86452, 21.1998, 8.7964),  Pos(174.993, -10.688, 0));
    setCameraMode(Follow);

    global.odeConfig.setParam("controlinterval",1);
    global.odeConfig.setParam("gravity", -9.81); 

    global.odeConfig.setParam("noise", 0);
    global.odeConfig.setParam("realtimefactor", 1.);

    int sliderwheelies = 1;
    int snakes = 0;

    Playground* playground = new Playground(odeHandle, osgHandle, 
			    osg::Vec3(120, 0.2, 2.5),.5,true);
    //    playground->setColor(Color(1,0.2,0,0.1));
    playground->setPosition(osg::Vec3(0,0,0)); // playground positionieren und generieren
    global.obstacles.push_back(playground);
    

    // Creation of Obstacle
    PassiveBox* b = 
      new  PassiveBox(odeHandle, 
                      osgHandle.changeColor(Color(180.0 / 255.0, 180.0 / 255.0, 130.0 / 255.0)), 
                      osg::Vec3(4.0, 20.0, .8), 0);
    b->setTexture(0,TextureDescr("Images/playfulmachines.rgb",1,1));
    b->setPosition(Pos(-32, 0, 0)); 
    global.obstacles.push_back(b);    
    


      /******* S L I D E R - W H E E L I E *********/
    for(int i=0; i<sliderwheelies; i++){
      SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
      mySliderWheelieConf.segmNumber=12;
      mySliderWheelieConf.jointLimitIn=M_PI/3;
      mySliderWheelieConf.frictionGround=0.5;
      mySliderWheelieConf.motorPower=8;
      mySliderWheelieConf.motorDamp=0.05;
      mySliderWheelieConf.sliderLength=0.5;
      mySliderWheelieConf.segmLength=1.4;
      OdeRobot* robot = new SliderWheelie(odeHandle, osgHandle, mySliderWheelieConf, "Slider Armband");
      robot->place(Pos(0,0,2.0)); 

      //controller = new Sos(1.0);
      controller = new OneControllerPerChannel(new ControlGen(),"OnePerJoint");
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); 
      OdeAgent* agent = new OdeAgent(global);
      // only the first controller is exported to guilogger and Co
      agent->addInspectable(((OneControllerPerChannel*)controller)->getControllers()[0]);
      agent->init(controller, robot, wiring);
      if(track) 
        agent->setTrackOptions(TrackRobot(true,true,false,false,"split_control",1));
      global.agents.push_back(agent);
      global.configs.push_back(agent);
    }


    //****** SNAKES **********/
    for(int i=0; i<snakes; i++){

      //****************/
      SchlangeConf conf = Schlange::getDefaultConf();
      // conf.segmMass   = .2;
      // conf.segmLength= .5// 0.8;
      //   conf.segmDia=.6;
      // conf.motorPower=.5;
      conf.segmNumber = 12+2*i;//-i/2; 
      conf.jointLimit=conf.jointLimit* 1.6;
      conf.frictionJoint=0.02;
      conf.useServoVel=true;

      SchlangeServo2* schlange1;
      if (i==0) {
	schlange1 = 

	  new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.1, 0.3, 0.8)),
			       conf, "S1");
      } else {
	schlange1 = 
	  new SchlangeServo2 ( odeHandle, osgHandle.changeColor(Color(0.1, 0.3, 0.8)),
			       conf, "S2");
      }
      //Positionieren und rotieren 
      schlange1->place(osg::Matrix::rotate(M_PI/2,0, 1, 0)*
		        osg::Matrix::translate(-.7+0.7*i,0,(i+1)*(.2+conf.segmNumber)/2.0/*+2*/));
                     // osg::Matrix::translate(5-i,2 + i*2,height+2));
      schlange1->setTexture("Images/whitemetal_farbig_small.rgb");
      if (i==0) {
	schlange1->setHeadColor(Color(1.0,0,0));
      } else {
	schlange1->setHeadColor(Color(0,1.0,0));
      }      
      OneControllerPerChannel *controller = new OneControllerPerChannel(new ControlGen(),"OnePerJoint");
      AbstractWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.05)); 

      OdeAgent* agent = new OdeAgent(global);
      agent->addInspectable(controller->getControllers()[0]);
      agent->init(controller, schlange1, wiring);
      global.agents.push_back(agent);
      global.configs.push_back(agent);
 
    }//creation of snakes End

  }

  // virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
  //   if(globalData.sim_step==1000){
  //     controller->setParam("epsC",1);
  //     controller->setParam("epsA",.1);
  //   }
  // };


};

int main (int argc, char **argv)
{  
  ThisSim sim;
  track = sim.contains(argv,argc,"-track") != 0;
  return sim.run(argc, argv) ? 0 : 1;
}


 
  
