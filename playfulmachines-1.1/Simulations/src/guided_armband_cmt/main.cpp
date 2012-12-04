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
 *   $Log: main.cpp,v $
 *   Revision 1.1  2010/01/25 13:05:32  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2009/08/07 14:39:52  martius
 *   guidance of SO with Cross Motor Couplings
 *
 *   Revision 1.7  2009/03/30 18:51:43  martius
 *   corrections and stuff
 *
 *   Revision 1.6  2009/03/28 18:08:12  martius
 *   *** empty log message ***
 *
 *   Revision 1.5  2009/03/27 22:31:00  martius
 *   *** empty log message ***
 *
 *   Revision 1.4  2009/03/27 18:38:33  martius
 *   *** empty log message ***
 *
 *   Revision 1.3  2009/03/27 13:37:09  martius
 *   parameters
 *
 *   Revision 1.2  2009/03/26 20:24:56  martius
 *   changed color and ground
 *
 *   Revision 1.1  2009/03/26 18:35:12  martius
 *   working version
 *
 *   Revision 1.2  2009/03/19 18:51:40  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2009/03/18 17:46:14  martius
 *   Teaching stuff
 *
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/complexplayground.h>
#include <ode_robots/passivebox.h>

#include <selforg/statistictools.h>
#include <selforg/statisticmeasure.h>

#include <selforg/invertmotornstep.h>
#include <selforg/semox.h>
#include <selforg/crossmotorcoupling.h>
#include <selforg/sinecontroller.h>
#include <selforg/noisegenerator.h>
#include <selforg/one2onewiring.h>
#include <selforg/feedbackwiring.h>
#include <selforg/stl_adds.h>

#include <ode_robots/sliderwheelie.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
const int segmnum = 13;
bool useSym = true;
double teacher = 0.00;
bool track = false;

class ThisConfig : public Configurable {
public:
  ThisConfig(): Configurable("Simulation of guided Armband","1.0") {
    controller=0;
    addParameterDef("D",&D,1,-1,1,"coupling direction {-1,1}");
    D_display=D;
    addParameterDef("gamma",&teacher,0,0,0.5,
                    "teaching strength (use this instead of gamma_teach of the controller)");
  }

  virtual void notifyOnChange(const paramkey& key){
    if(controller){
      teacher = clip(teacher,0.0,0.5);
      controller->setParam("gamma_teach", teacher);	  
      if(D>0) D=1;
      else D=-1;
      D_display=D;
      int k= (D+1)/2;
      std::list<int> perm;
      int len  = controller->getMotorNumber();
      for(int i=0; i<len; i++){
        perm.push_back((i+k+(len)/2)%len);
      }
      CMC cmc = controller->getPermutationCMC(perm);
      controller->setCMC(cmc);
    }
  }
  CrossMotorCoupling* controller;  
  double D_display; // used to display
  int D; // direction value set from the console
};

class ThisSim : public Simulation {
public:
  StatisticTools stats;

  CrossMotorCoupling* controller;
  //  SeMoX* controller;
  //InvertMotorNStep* controller;
  OdeRobot* vehicle;
  ThisConfig thisConfig;

  ThisSim(){
    controller=0;
    vehicle=0;
    setGroundTexture("Images/red_velour_wb.rgb");
    // setCaption("LpzRobots         www.playfulmachines.com");
    setTitle("Guided Armband");
    setCaption("Simulator by Martius et al");

  }


  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {

    setCameraHomePos(Pos(-0.777389, 6.34573, 1.83396),  Pos(-170.594, -5.10046, 0));
    setCameraMode(Follow);
    // initialization
    // - set noise to 0.1
    // - register file chess.ppm as a texture called chessTexture (used for the wheels)
    global.odeConfig.setParam("noise", 0.05);
    global.odeConfig.setParam("controlinterval", 1);
    global.odeConfig.setParam("realtimefactor", 1); 
    //    global.odeConfig.setParam("gravity", 0);

    for(int i=0; i< 4; i++){
      PassiveBox* b = new PassiveBox(odeHandle, osgHandle.changeColor(Color(.6, .6, .4)), 
				     osg::Vec3(1,10,0.3+i*0.02),0);
      b->setTexture(0,TextureDescr("Images/playfulmachines.rgb",1,1));
      b->setPosition(osg::Vec3(-75+i*30,0,0));
      global.obstacles.push_back(b);    
    }

    controller=0;
  

    /******* S L I D E R - w H E E L I E *********/
    SliderWheelieConf mySliderWheelieConf = SliderWheelie::getDefaultConf();
    mySliderWheelieConf.segmNumber   = segmnum;
    mySliderWheelieConf.motorPower   = 5;
    mySliderWheelieConf.jointLimitIn = M_PI/3;
//     mySliderWheelieConf.frictionGround=0.5;
//    mySliderWheelieConf.segmLength=1.4;
    mySliderWheelieConf.sliderLength = 0;
    mySliderWheelieConf.motorType    = SliderWheelieConf::CenteredServo;
    //mySliderWheelieConf.drawCenter   = false;
    vehicle = new SliderWheelie(odeHandle, osgHandle.changeColor(Color(1,222/255.0,0)), 
				mySliderWheelieConf, "Armband");

    vehicle->place(Pos(0,0,.1));    

    SeMoXConf cc = SeMoX::getDefaultConf();    
    cc.cInit=1.1; 
    cc.modelExt=false;
    cc.someInternalParams=false;
    SeMoX* semox = new SeMoX(cc);  

    if(useSym){
      semox->setParam("epsC", 0.1);
      semox->setParam("epsA", 0.1);
    }else{
      semox->setParam("epsC", 0.1);
      semox->setParam("epsA", 0.1);
    }
    semox->setParam("rootE", 3);
    semox->setParam("s4avg", 1);
    semox->setParam("gamma_cont", 0.005);

    semox->setParam("gamma_teach", teacher);

    //controller=semox;
    controller = new CrossMotorCoupling( semox, semox, 0.4);

    //    One2OneWiring* wiring = new One2OneWiring(new ColorUniformNoise(0.1));
    AbstractWiring* wiring = new FeedbackWiring(new ColorUniformNoise(0.1),
						FeedbackWiring::Motor, 0.75);
    //global.plotoptions.push_back(PlotOption(GuiLogger,Robot,5));
    OdeAgent* agent = new OdeAgent(global);
    agent->addCallbackable(&stats);	  
    agent->init(controller, vehicle, wiring);
    if(track) agent->setTrackOptions(TrackRobot(true,false,false, false, "uni", 20));
    global.agents.push_back(agent);
    global.configs.push_back(agent);

    thisConfig.controller=controller;    
    global.configs.push_back(&thisConfig);


    this->getHUDSM()->setColor(osgHandle.getColor("hud"));
    this->getHUDSM()->setFontsize(16);    
    //    this->getHUDSM()->setColor(Color(1.0,1.0,0));
    //    this->getHUDSM()->setFontsize(18);    
    this->getHUDSM()->addMeasure(teacher,"Gamma",ID,1);
    this->getHUDSM()->addMeasure(thisConfig.D_display,"D",ID,1);

//     if(useSym){
//       int k= 0;
//       std::list<int> perm;
//       int len  = controller->getMotorNumber();
//       for(int i=0; i<len; i++){
// 	perm.push_back((i+k+(len)/2)%len);
//       }
//       CMC cmc = controller->getPermutationCMC(perm);
//       controller->setCMC(cmc);
//     }

  }

};


int main (int argc, char **argv)
{ 

  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}

 
 

