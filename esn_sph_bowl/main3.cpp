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
#include <selforg/sox.h>
#include "ESN.h"
#include "groupController.h"

#include <selforg/noisegenerator.h> // include noisegenerator (used for adding noise to sensorvalues)
#include <selforg/one2onewiring.h>  // simple wiring

// used arena
#include <ode_robots/playground.h>
#include <ode_robots/terrainground.h>

// robots
#include <ode_robots/sphererobot3masses.h>
#include <ode_robots/axisorientationsensor.h>
#include <ode_robots/speedsensor.h>

// fetch all the stuff of lpzrobots into scope
using namespace lpzrobots;


const double height = 2;

enum Env {ThreeBump, SingleBasin, ElipticBasin};
Env env = SingleBasin;
bool track=false;
const char* envnames[3] = {"ThreePot", "SingleBasin","ElipticBasin"};

class ThisSim : public Simulation {
public:
  AbstractController* controller;
  Sphererobot3Masses* sphere1;

  ThisSim(){
    setTitle("The ESN Playful Machine");
    setCaption("Simulator by Martius et al");
  }

  // starting function (executed once at the beginning of the simulation loop)
  void start(const OdeHandle& odeHandle, const OsgHandle& osgHandle, GlobalData& global)
  {
    setCameraHomePos(Pos(-16.4509, 15.6927, 12.5683),  Pos(-133.688, -26.4496, 0));
    //setCameraMode(Follow);
    setCameraMode(Static);
    global.odeConfig.setParam("noise",0.1);
    global.odeConfig.setParam("controlinterval",4);
    global.odeConfig.setParam("realtimefactor",4);

    //Playground
    
 /*   Playground* playground;

     playground = new Playground(odeHandle, osgHandle, 
				  osg::Vec3(20, 0.2, 2+1.f), 2);

    playground->setColor(Color(.1,0.7,.1));
    playground->setTexture("");
    playground->setPosition(osg::Vec3(0,0,0.01f)); // playground positionieren und generieren
    global.obstacles.push_back(playground); */


     TerrainGround* terrainground = 
       new TerrainGround(odeHandle, osgHandle.changeColor(Color(1.0f,1.0f,1.0f)),
			    //                        "terrains/dip128_flat.ppm","terrains/dip128_flat_texture.ppm", 
                            "terrains/dip128.ppm","terrains/dip128_texture.ppm", 
                            20, env == SingleBasin ? 20 : 40, height, OSGHeightField::Red);
        terrainground->setPose(osg::Matrix::translate(0, 0, 0.1));
        global.obstacles.push_back(terrainground); 


    //  global.odeConfig.setParam("gravity", 0); // no gravity

    for(int i=0; i<0; i++){
      PassiveSphere* s = new PassiveSphere(odeHandle, osgHandle.changeColor(Color(0.0,1.0,0.0)), 0.5);
      s->setPosition(osg::Vec3(5,0,i*3));
      global.obstacles.push_back(s);
    }


    // Spherical Robot with axis orientation sensors:
    Sphererobot3MassesConf conf = Sphererobot3Masses::getDefaultConf();
    conf.addSensor(new AxisOrientationSensor(AxisOrientationSensor::ZProjection));
    // add context sensors (rotation speed around the internal axes,
    //  with 5 as maximum speed)
    conf.addSensor(new SpeedSensor(5, SpeedSensor::RotationalRel));

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

    Sox* sox = new Sox(1.2,true);
    sox->setParam("epsA",0.3); // model learning rate
    sox->setParam("epsC",0.3); // controller learning rate
    sox->setParam("Logarithmic", 0);
    sox->setParam("sense", 0.5);
   // sox->setParam("causeaware",0.4);
   // sox->setParam("pseudo",2);
    controller = new GroupController(sox, 3); // 3 context sensors

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


