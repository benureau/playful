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

// include simulation environment stuff
#include <ode_robots/simulation.h>

#include <ode_robots/odeagent.h>
#include <ode_robots/octaplayground.h> // arena
#include <ode_robots/passivesphere.h>  // passive balls

// controller
#include "cloud.h"
#include "clustercontroller.h"

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// robots
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;

bool track=false;

class ThisSim : public Simulation {
public:
  AbstractController* controller;
  Sphererobot3Masses* sphere1;
  Cloud               cloud;
  
  ThisSim(){
    setTitle("The Playful Machine (Der/Martius)");
    setCaption("Simulator by Martius et al");
  }  

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global) 
  {
    setCameraHomePos(Pos(5.2728, 7.2112, 3.31768), Pos(140.539, -13.1456, 0));
    setCameraMode(Follow);
    global.odeConfig.setParam("noise",0.1);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",4);
    
    //  global.odeConfig.setParam("gravity", 0); // no gravity

    for(int i=0; i<0; i++){
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
      s->setPosition(osg::Vec3(5,0,i*3)); 
      global.obstacles.push_back(s);    
    }

    
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
    sphere1 = new Sphererobot3Masses ( odeHandle, osgHandle.changeColor(Color(1.0,0.0,0)), 
				       conf, "Sphere1", 0.2);     
    ((OdeRobot*)sphere1)->place ( Pos( 0 , 0 , 0.1 ));
    global.configs.push_back ( sphere1 );
    
    //controller = new Sox(.8,true);
    controller = new ClusterController(cloud, .8, true);
    cloud.configure(((ClusterController*)controller)->getMotorNumber(), ((ClusterController*)controller)->getSensorNumber(), 20);
    
    controller->setParam("epsA",0.3); // model learning rate
    controller->setParam("epsC",1); // controller learning rate
    controller->setParam("causeaware",0.4); 
    controller->setParam("pseudo",2); 
    global.configs.push_back ( controller );

    One2OneWiring* wiring = new One2OneWiring ( new ColorUniformNoise() );
    OdeAgent* agent = new OdeAgent ( global );
    agent->init ( controller , sphere1 , wiring );
    if(track) agent->setTrackOptions(TrackRobot(true, true, true, false, "zaxis", 20)); 
    global.agents.push_back ( agent );

  }

  virtual void usage() const {
    printf("\t-corridor\tPlace robot in a circular corridor\n");
    printf("\t-track\tenable tracking (trajectory is written to file)\n");
  };


};

int main (int argc, char **argv)
{ 
  track = (Simulation::contains(argv,argc,"-track")>0);
  ThisSim sim;
  return sim.run(argc, argv) ? 0 : 1;
}
 
 
