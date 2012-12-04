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
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>


#include <selforg/one2onewiring.h>
#include <selforg/forceboostwiring.h>

#include <selforg/sos.h>
#include <selforg/sox.h>
#include <selforg/soml.h>
#include <selforg/sinecontroller.h>

#include <ode_robots/axisorientationsensor.h>

#include <ode_robots/schlangeservo2.h>

#include "environment.h"
#include <ode_robots/operators.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

// predicate that matches agents that have the same name prefix
struct agent_match_prefix : public unary_function<const OdeAgent*, bool> {
    agent_match_prefix(string nameprefix)
      : nameprefix(nameprefix) {
      len=nameprefix.length();
    }
    bool operator()(const OdeAgent* a) {
      if(!a || !a->getRobot()) return false;
      return nameprefix.compare(0,len, a->getRobot()->getName(),0,len) == 0;
    }

    string nameprefix;
    int len;
  };

class ThisSim : public Simulation {
public:

  Env env;
  double lastRobotCreation;
  bool useMultilayer;

  ThisSim(){
    addPaletteFile("colors/UrbanExtraColors.gpl");
    addColorAliasFile("colors/UrbanColorSchema.txt");
    setGroundTexture("Images/whiteground.jpg");
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    
    
    int numSnakes        = 1;
    int numSnakesOutside = 0;

    setCameraHomePos(Pos(-0.589171, 19.0292, 7.79985),  Pos(-178.94, -18.1136, 0));
    //(Pos(-0.282677, 28.654, 8.41382),  Pos(-178.667, -18.1136, 0));
    setCameraMode(Static);
    // initialization

    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",2);
    //    global.odeConfig.setParam("realtimefactor",0 );
    global.odeConfig.setParam("gravity",-4);
    global.odeConfig.addParameterDef("multilayer",&useMultilayer, 
                                     false, "use multilayer controller (SoML)");
    // environemnt type
    env.type=Env::Octa;
    //env.type=Env::Stacked;
    env.pitsize     = 1.7;
    env.heightInner = 4;

    env.create(odeHandle, osgHandle, global);

    env.numSpheres  = 0; 
    env.numBoxes    = 0;   
    env.numCapsules = 0;
    env.numSeeSaws  = 0;
    
    env.placeObstacles(odeHandle, osgHandle, global);

    global.configs.push_back(&env);
                
    // So wird das mit allen Robotern aussehen, createXXX, siehe unten
    for(int r=0; r < numSnakes; r++) {
      createSnake(odeHandle, osgHandle, global, 
                  ROTM(M_PI/2,0,1,0)*TRANSM(0,0,r*10),"PitSnake");
    }
    for(int r=0; r < numSnakesOutside; r++) {
      createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(0,10-r,0.2));
    }
       

    lastRobotCreation=-2;    
  }

  bool removeRobot(GlobalData& global, const string& nameprefix){
    OdeAgentList::reverse_iterator i = 
      find_if(global.agents.rbegin(), global.agents.rend(), agent_match_prefix(nameprefix));
    if(i!=global.agents.rend()){
      printf("Removed robot %s\n", (*i)->getRobot()->getName().c_str());
      OdeAgent* a = *i;
      global.agents.erase(i.base()-1);
      removeElement(global.configs, a);
      delete a;
      return true; 
    }
    return false;
  }
  

  OdeAgent* createSnake(const OdeHandle& odeHandle, const OsgHandle& osgHandle, 
                        GlobalData& global, osg::Matrix pose, string name = "Snake"){    
    // find robot and do naming
    int num = count_if(global.agents.begin(), global.agents.end(), 
                       agent_match_prefix(name));
    bool outside = (name == "Snake");
    name += "_" + itos(num+1);        
    SchlangeConf snakeConf = SchlangeServo2::getDefaultConf();
    snakeConf.useServoVel  = true;
    snakeConf.segmNumber   = 14+3*num;
    snakeConf.segmLength   = 1.15;
    snakeConf.jointLimit   = M_PI/3;
    snakeConf.motorPower   = 1;
    snakeConf.frictionJoint = 0;
    snakeConf.frictionRatio = 0.1;
    snakeConf.useSpaces = true;
    OdeHandle snakeHandle(odeHandle);
    if(outside)
      snakeHandle.substance.toPlastic(2);
    else {
      snakeHandle.substance.toPlastic(0.5);
      snakeHandle.substance.slip= 0.05;
    }

    OsgHandle snakeOsgH = osgHandle.changeColorSet((num+outside)%2);

    OdeRobot*    robot     = new SchlangeServo2 ( snakeHandle, snakeOsgH, snakeConf, name);
    robot->place(pose*TRANSM(0,0,outside ? 0 : 0.8*snakeConf.segmNumber)); 

     ForceBoostWiring* wiring = new ForceBoostWiring(new ColorUniformNoise(0.01), 0.0);
    AbstractController* controller;
    if(useMultilayer){
      SoMLConf sc = SoML::getDefaultConf();
      sc.useHiddenContr        = true;
      sc.useHiddenModel        = true;
      sc.hiddenContrUnitsRatio = 1.0;
      sc.hiddenModelUnitsRatio = 1.0;
      sc.useS = false;
      controller = new SoML(sc);
    }else{      
      controller = new Sox(.9, true);
      //      controller = new SineController();
    }


    controller->setParam("Logarithmic",1);
    controller->setParam("epsC",0.05);
    controller->setParam("epsA",0.01);
    controller->setParam("s4avg",1);
    controller->setParam("s4delay",1);
    controller->setParam("sense",2);
    //    AbstractWiring*     wiring     = new One2OneWiring(new ColorUniformNoise(0.1));
    //  AbstractWiring*     wiring     = new ForceBoostWiring(new ColorUniformNoise(0.1),0.01);
    OdeAgent*           agent      = new OdeAgent( global, PlotOption(NoPlot) );
    
    wiring->setParam("booster",.01);
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);
    global.configs.push_back(agent);
    return agent;
  }
  


  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global, int key, bool down)
  {
    if (down) { // only when key is pressed, not when released
      // lower case (create robot) but last creation less than a second ago
      if(key >= 'a' && key <='z' && lastRobotCreation > (global.time-2)) { 
        printf("You have to wait two seconds between robot creations.\n");
        return false;
      }

      switch ( (char) key )
	{
        case 'k': // test
          global.agents[0]->getRobot()->getMainPrimitive()->applyForce(20,0,0);
          break;
        case 's':
          createSnake(odeHandle, osgHandle, global, osg::Matrix::translate(4,4,2)); break;
        case 'S':
          removeRobot(global, "Snake"); break;          
        case 'i': // put Snake in center (useful in pit-mode
          createSnake(odeHandle, osgHandle, global, 
                      ROTM(M_PI/2,0,1,0)*TRANSM(0,0,0),"PitSnake"); break;
        case 'I':
          removeRobot(global, "PitSnake"); break;          
 	default:
	  return false;
	  break;
	}
      if(key >= 'a' && key <='z' && key != 'k') { // lower case -> created robot 
        lastRobotCreation = global.time;        
      }
      return true;
    }
    return false;
  }

  virtual void bindingDescription(osg::ApplicationUsage & au) const {
    au.addKeyboardMouseBinding("Simulation: s/S","add/remove Snake");
    au.addKeyboardMouseBinding("Simulation: i/I","add/remove Snake in Pit");
  }


  
};



int main (int argc, char **argv)
{ 

  ThisSim sim;
  // run simulation
  return sim.run(argc, argv) ? 0 : 1;
}
 
