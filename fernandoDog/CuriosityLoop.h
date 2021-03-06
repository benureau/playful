/*
 *
 *  Created by Chrisantha Fernando on 03/03/2009.
 *  Copyright 2009 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef __CURIOSITYLOOP_H
#define __CURIOSITYLOOP_H

#import <iostream>
#import <fstream>
#import <math.h>
#import <stddef.h>
#import  <time.h>
#import <string.h>
#import <stdio.h>
#import <sstream>
#include <stdlib.h>
#include <selforg/matrix.h>
#import <vector>
#include <selforg/inspectable.h>

using namespace std; 

class CuriosityLoop: public Inspectable {
		
	public: 
			
	CuriosityLoop(int); 
	virtual void makePrediction(const matrix::Matrix& s, const matrix::Matrix& m);  
	virtual double updatePrediction(const matrix::Matrix& smHist, const matrix::Matrix& s, const matrix::Matrix& m);  

// 	virtual void makePrediction(const sensor* s, const motor* m);  

	~CuriosityLoop();  
		
	//Each loop has a linear predictor trained by delta-rule. 
	//std::vector < std::vector <double> > predictorWeights;  //Predictor weight matrix
	matrix::Matrix predictorWeights;  
	matrix::Matrix predictorMask; 
        matrix::Matrix prediction;   
	//vector < double > prediction; //Stores the prediction from the previous timestep to compare with the current value.

	//Each loop has an actor evolved by CMAES 
	//std::vector < std::vector <double> > actorWeights;  //Predictor weight matrix 
	matrix::Matrix actorWeights;  
	
	//Each loop has a fitness  
	double fitness; //Predictor fitness is to be defined, e.g. prediction error, prediction progress, etc..  
	double prediction_error; 

	private: 
	
	
		
}; 

#endif 

