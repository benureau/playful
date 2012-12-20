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
			
	CuriosityLoop(int, int, int, int); 
	virtual void makePrediction(const matrix::Matrix& s, const matrix::Matrix& m);  
	virtual double updatePrediction(const matrix::Matrix& smHist, const matrix::Matrix& s, const matrix::Matrix& m, int phase);  
	virtual matrix::Matrix generateAction(const matrix::Matrix& sensors, int parentActing); 
	virtual void replicateMutateActor(); 
	virtual void replicateUnrestrictPredictor(int, int, int); 
// 	virtual void makePrediction(const sensor* s, const motor* m);  
	virtual void overwriteParentActor(); 
        virtual void savePredictorWeights(); 

	~CuriosityLoop();  
		
	//Each loop has a linear predictor trained by delta-rule. 
	//std::vector < std::vector <double> > predictorWeights;  //Predictor weight matrix
	matrix::Matrix predictorWeights;  
	matrix::Matrix predictorMask; 
        matrix::Matrix prediction;   
	matrix::Matrix pInput; //Binary vector showing which inputs the predictor gets. [Replaces the predictor mask]
	matrix::Matrix pOutput; //Binary vector showing which outputs the predictor predicts. [Replaces the predictor mask]
	//vector < double > prediction; //Stores the prediction from the previous timestep to compare with the current value.
	matrix::Matrix uPredictorWeights;  
	matrix::Matrix uPredictorMask; 
        matrix::Matrix uPrediction; 
	matrix::Matrix uPInput; //Binary vector showing which inputs the Unrestricted predictor gets. [Replaces the predictor mask]
	matrix::Matrix uPOutput; //Binary vector showing which outputs the Unrestricted predictor predicts. [Replaces the predictor mask]  

	//Each loop has an actor evolved by CMAES 
	//std::vector < std::vector <double> > actorWeights;  //Predictor weight matrix 
	matrix::Matrix actorWeights; 
	matrix::Matrix offspringActorWeights;   
	
	//Each loop has a fitness  
	double fitness; //Predictor fitness is to be defined, e.g. prediction error, prediction progress, etc..  
	double prediction_error; 
	double uPrediction_error; 
	double prediction_error_time_average; 
        double old_prediction_error_time_average; 

	double parentActorFitness; 
	double offspringActorFitness; 

	//Stores the errors over time for the restricted and unrestricted predictors in this loop. 
	matrix::Matrix parent_error; 	//This is NOT the parent error but the RESTRICTED PREDICTOR ERRRO for both parent and offspring. (parent < 250, offspring > 250). 
	matrix::Matrix offspring_error; //This is NOT the offspring actor error but the UNRESTRICTED PREDICTOR ERROR for both parent and offspring. 
	 

	private: 
	
	
		
}; 

#endif 

