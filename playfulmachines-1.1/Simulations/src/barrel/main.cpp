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
 *
 ***************************************************************************/

/* This is the main file of the Demonstration Simulator
   of Chapter "Outline" of the Book "Self-Organizing Robots"
*/

#include "openloop.h"
#include "normalsim.h"
#include "precession.h"

using namespace std;
using namespace lpzrobots;


int main (int argc, char **argv)
{ 
  cout << "\nWelcome to the demonstration software for the book The Playful Machine\n";
  cout << " by Georg Martius and Ralf Der (http://playfulmachines.com)\n";

  
  enum Sims { OpenLoop, ClosedLoop, ContFromMod, HS, HK, Precession, Last};
  vector<string> simName;
  vector<string> simPar;
  simName.resize(Last); 
  simPar.resize(Last); 
  simName[OpenLoop]   = "Open Loop";                          simPar[OpenLoop]   = "-ol";
  simName[ClosedLoop] = "Closed Loop";                        simPar[ClosedLoop] = "-cl";
  simName[ContFromMod]= "Learning Controller from the Model"; simPar[ContFromMod]= "-cfm";
  simName[HS]         = "Homeostatic Learning";               simPar[HS]         = "-hs";
  simName[HK]         = "Homeokinetic Learning";              simPar[HK]         = "-hk";
  simName[Precession] = "Homeokinetic Learning (upright)";    simPar[Precession] = "-prec";

  Sims simtype=OpenLoop;
  for(int i=OpenLoop; i<Last; i++){
    if(Simulation::contains(argv,argc,simPar[i].c_str()) != 0)
      simtype = (Sims)i;
  }

  if (OpenLoopSim::contains(argv, argc, "-h")) {
    OpenLoopSim sim;
    sim.run(argc, argv);    
    for(int i=OpenLoop; i<Last; i++){
      printf("\t%s\t%s\n", simPar[i].c_str(),simName[i].c_str());      
    }
    exit(0);
  }

  Simulation* sim;
  switch(simtype) {
    case OpenLoop: 
      {
	sim = new OpenLoopSim;
	break;
      }    
    case ClosedLoop: 
      {
	sim = new NormalSim(NormalSim::NoLearn);
	break;
      }
    case ContFromMod: 
      {
	sim = new NormalSim(NormalSim::LearnContHS);
	break;
      }
    case HS: 
      {
	sim = new NormalSim(NormalSim::LearnHS);
	break;
      }
    case HK: 
      {
	sim = new NormalSim(NormalSim::LearnHK);
	break;
      }
    case Precession: 
      {
	sim = new PrecessionSim;
	break;
      }
    default:   
      cerr << "unknown simulation type" << endl;
      return 0;
  }
  sim->setCaption("playfulmachines.com");
  sim->setTitle("Barrel Bot " + simName[simtype]);
  // run simulation
  sim->run(argc, argv);
  delete sim;
  
  return 0;
}
 
