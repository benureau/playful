/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *  

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                   Darwinian Neurodynamics  Header File                    *
 *                                                                           *
 *****************************************************************************/
#ifndef __DN_H
#define __DN_H


#include <stdio.h>
#include <selforg/abstractmodel.h>
#include <selforg/matrix.h>
#include "CuriosityLoop.h"
#include <vector>
#include <selforg/inspectable.h> 

class DN : public AbstractModel {
public:

  DN(int pop_size = 50);

  /** initialisation of the network with the given number of input and output units
      @param inputDim length of input vector
      @param outputDim length of output vector
      @param unit_map if 0 the parametes are choosen randomly. 
             Otherwise the model is initialised to represent a unit_map 
	     with the given response strength.
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int pop_size, unsigned  int inputSize, 
		    double unit_map = 0.0, RandGen* randGen = 0);

  /** passive processing of the input
     (this function is not constant since a recurrent network 
     for example might change internal states
  */

  virtual void updateLoops (const matrix::Matrix& s, const matrix::Matrix& m); 
//  virtual void updateLoops (const sensor*, const motor*); 

  virtual void updatePredictions (const matrix::Matrix& smMem, const matrix::Matrix& sensors, const matrix::Matrix& motors, int phase); 

  virtual void makePredictions (const matrix::Matrix& s, const matrix::Matrix& m);

  virtual const int actionSelection (const matrix::Matrix& s);

  virtual const matrix::Matrix  executeAction (int chosen, const matrix::Matrix& sensors, int parentActing);

  virtual void replicateMutate(int chosen_actor);  
  virtual void replicateUnrestrictPredictor(int chosen_actor, int i, int s); 
  virtual void determineActorFitness(int chosen_actor); 
  virtual void getErrorsFromChosenActor(int chosen_actor); 	


 //******************OLD FUNCTIONS FROM ESN**************************


  virtual const matrix::Matrix process (const matrix::Matrix& input);

  /* performs learning and returns the network output before learning.
     Neural networks process the input before. (no need to call process before)
     \param learnRateFactor can be given to modify eps for this learning step.
  */
  virtual const matrix::Matrix learn (const matrix::Matrix& input, 
				      const matrix::Matrix& nom_output, 
				      double learnRateFactor = 1);

  /// damps the weights and the biases by multiplying (1-damping)
  virtual void damp(double damping);

  /// returns the number of input neurons
  virtual unsigned int getInputDim() const;
  /// returns the number of output neurons
  virtual unsigned int getOutputDim() const;

  virtual bool store(FILE* f) const;

  virtual bool restore(FILE* f);
  
  //DN variables 
  vector <CuriosityLoop*> pop; //This initializes a vector to store a population of loops.  	
   

protected:

  int nbNeurons;
  int nbInputs;
  int nbOutputs;
  matrix::Matrix inputWeights;
  matrix::Matrix outputWeights;
  matrix::Matrix ESNNeurons;
  matrix::Matrix ESNWeights; 
  double error;
  double eps;
  matrix::Matrix predictionErrors; 

  matrix::Matrix parent_errors; 
  matrix::Matrix offspring_errors; 

  double chosen;

  //
};

#endif 

