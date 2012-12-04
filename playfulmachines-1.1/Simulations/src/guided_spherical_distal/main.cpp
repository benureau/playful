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
 *   Revision 1.2  2009/03/27 14:35:27  martius
 *   *** empty log message ***
 *
 *   Revision 1.1  2009/02/02 16:10:07  martius
 *   *** empty log message ***
 *
 *
 ***************************************************************************/

#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/playground.h>
#include <ode_robots/passivesphere.h>
#include <ode_robots/tmpprimitive.h>

#include <selforg/semox.h>
#include <selforg/sox.h>
#include <selforg/noisegenerator.h>
#include <selforg/sinecontroller.h>
#include <selforg/one2onewiring.h>
#include <selforg/selectiveone2onewiring.h>
#include <selforg/wiringsequence.h>
#include <selforg/statistictools.h>
#include <selforg/statisticmeasure.h>


#include <ode_robots/nimm2.h>
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/vierbeiner.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;
using namespace matrix;

int useTeaching = 1;
int axis        = 0;
double axis_display = axis + 1;


class ThisSim : public Simulation {
public:

  Teachable* teachable;
  AbstractController* controller;
  OdeRobot* robot;
  Sensor* sensor;
  double teacher;

  ThisSim(){
    robot=0;
    teachable=0;
    setTitle("Guided Spherical");
    //    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");

    //setGroundTexture("Images/red_velour_wb.rgb");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(-5.44372, 7.37141, 3.31768),  Pos(-142.211, -21.1623, 0));
    setCameraMode(Follow);
    
    // addParameterDef("gamma",&teacher,0,0,0.5,
    //                 "teaching strength (use this instead of gamma_teach of the controller)");
    // global.configs.push_back ( this );
    
    global.odeConfig.setParam("noise",0.05);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",2);

    
      
    // Spherical Robot with axis orientation sensors:
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();  
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    // regular behaviour
    conf.motorsensor=false;
    conf.diameter=1.0;
    conf.pendularrange= 0.25;
    conf.motorpowerfactor = 150;     
   
//     conf.diameter=1.0;
//     conf.pendularrange= 0.35;
    OdeRobot* robot;
    robot = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0.0)), 
                                     conf, "Sphere_Guided", 0.2);     
    robot->place ( ROTM(M_PI/2, 1,0,1) * TRANSM( 0 , 0 , 0.1 ));
    global.configs.push_back ( robot );
    
    controller = new Sox(0.8, true, true);
    controller->setParam("epsA",0.3); // model learning rate
    controller->setParam("epsC",1); // controller learning rate
    controller->setParam("causeaware",0.4); 
    controller->setParam("pseudo",2); 
    controller->setParam("gamma",0); 
    teachable = dynamic_cast<Sox*>(controller);

    // SeMoXConf sc = SeMoX::getDefaultConf();
    // sc.modelExt=false;
    // sc.someInternalParams=false;
    // sc.cInit=0.5;
    // sc.aInit=1.0;
    // controller = new SeMoX(sc);
    // controller->setParam("epsA",0.01); // model learning rate
    // controller->setParam("epsC",0.01); // controller learning rate
    // controller->setParam("rootE",3); // controller learning rate
    // controller->setParam("gamma_cont",0.001); 
    // controller->setParam("gamma_teach",teacher); 

    global.configs.push_back ( controller );

  
    AbstractWiring* wiring;
    //wiring = new One2OneWiring(new WhiteUniformNoise());
    wiring = new One2OneWiring(new ColorUniformNoise());
    
    //plotoptions.push_back(PlotOption(GuiLogger, 5));
    // std::list<const Configurable*> l;
    // l.push_back(&global.odeConfig);
    // plotoptions.push_back(PlotOption(File, 100, l));      
    
    // OdeAgent* agent = new OdeAgent ( l );

    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, robot, wiring);
    global.agents.push_back(agent);

    this->getHUDSM()->setColor(osgHandle.getColor("hud"));
    this->getHUDSM()->setFontsize(16);
    this->getHUDSM()->addMeasure(teacher,"Gamma",ID,1);
    this->getHUDSM()->addMeasure(axis_display,"Axis",ID,1);


  }

  // virtual void notifyOnChange(const paramkey& key){
  //   if(teachable){
  //     teacher = clip(teacher,0.0,0.5);
  //     controller->setParam("gamma_teach", teacher);	  
  //   }
  // }

  virtual void addCallback(GlobalData& globalData, bool draw, bool pause, bool control) {
    if(control){
      if(controller){
        teacher = controller->getParam("gamma");
      }
      if(useTeaching && teachable){
	Matrix last = teachable->getLastSensorValues();        
        //	if(fabs(last.val(axis,0))>0.2){
          Matrix teaching =  last;
          teaching.val(axis,0)=0;
	  teachable->setSensorTeaching(teaching);
          //	}
      }
    }
  }

  // add own key handling stuff here, just insert some case values
  virtual bool command(const OdeHandle&, const OsgHandle&, GlobalData& global, 
                       int key, bool down)
  {    
    if (down) { // only when key is pressed, not when released
      char txt[256];	  
      switch ( (char) key )
	{
	case 'g' :           
          // useTeaching = !useTeaching;
          // printf("Guidance %s\n", useTeaching ? "enabled" : "disabled");
          if(controller->getParam("gamma") <= 0 ) {
            controller->setParam("gamma", 0.005);
            sprintf(txt,"Guidance enabled  (gamma = %f)\n", controller->getParam("gamma"));
          } else {
            controller->setParam("gamma", 0.0);
            sprintf(txt,"Guidance disabled (gamma = %f)\n", controller->getParam("gamma"));
          }
          printf("%s\n",txt);
          global.removeExpiredObjects(global.time+15);
          global.addTmpObject(new TmpDisplayItem(new OSGText(std::string(txt),14), 
                                                 TRANSM(20,30,0), "hud", 0.8),15);
	  return true;
          break;
	case 'a' : {
          const char* axisnames[] = {"red", "green", "blue"};
	  axis = (axis + 1) % 3;
          axis_display = axis + 1;
          if(controller->getParam("gamma") <= 0 ) {            
            sprintf(txt,"Axis changes but Guidance disabled (type 'g')");          
          }else{
            sprintf(txt,"Guidance for rotation around %s axis", axisnames[axis]);          
          }
          printf("%s\n",txt);
          global.removeExpiredObjects(global.time+15);
          global.addTmpObject(new TmpDisplayItem(new OSGText(std::string(txt),14), 
                                                 TRANSM(20,30,0), "hud", 0.8),15);

	  return true;
	  break;
        }
	default:
	  return false;
	  break;
	}
    }
    return false;
  }
};


int main (int argc, char **argv)
{ 
  ThisSim sim;
  return sim.run(argc, argv) ? 0 :  1;
}

 
 
