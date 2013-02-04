/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *  

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                   Echo State Network Header File                          *
 *                                                                           *
 *****************************************************************************/
#ifndef __ESN_H
#define __ESN_H


#include <stdio.h>
#include <selforg/abstractmodel.h>
#include <selforg/matrix.h>


/**
 * class for robot control with sine, sawtooth and impuls  
 * 
 * period is the length of the period in steps and 
 * phaseshift is the phase difference between channels given in Pi/2
 */
class ESN : public AbstractModel {
public:

  /**     
     @param controlmask bitmask to select channels to control (default all)
     @param function controller function to use
   */
  ESN(int nbNeurons = 30);

  /** initialisation of the network with the given number of input and output units
      @param inputDim length of input vector
      @param outputDim length of output vector
      @param unit_map if 0 the parametes are choosen randomly. 
             Otherwise the model is initialised to represent a unit_map 
	     with the given response strength.
      @param randGen pointer to random generator, if 0 an new one is used
   */
  virtual void init(unsigned int inputDim, unsigned  int outputDim, 
		    double unit_map = 0.0, RandGen* randGen = 0);

  /** passive processing of the input
     (this function is not constant since a recurrent network 
     for example might change internal states
  */
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

  //
};

#endif 

