/*****************************************************************************
 *             FIAS winter school, playful machine group                     *
 *                Supervisors: Ralf Der, Georg Martius                       *  

 * Members: Fabien Benureau, Chrisantha Fernando, Quan Wang, Jimmy Baraglia  *
 *                    Echo State Network c++ File                            *
 *                                                                           *
 *****************************************************************************/

#include "ESN.h"
#include <selforg/controller_misc.h>

#define TIMESCALE 0.1
#define CONNECTION_RATIO 0.1
#define INRATIO 0.2
#define OUTRATIO 0.2

using namespace std;
using namespace matrix;

  /**
   * ESN class constructor
  */
  ESN::ESN(int nbNeurons)
	:AbstractModel("ESN","0.1"), nbNeurons(nbNeurons)
  {
        eps=0.01;
	addParameter("eps",&eps,0,1,"learning rate");  
	//nothing
	addInspectableMatrix("OutputWeights",&outputWeights,false,"output weights");
  	addInspectableMatrix("ESNNeurons",&ESNNeurons,false,"internal state");
  	addInspectableMatrix("ESNWeights",&ESNWeights,false,"internal weights");
	addInspectableValue("error",&error,"Learning error");
        error = 0;
  }

  void ESN::init(unsigned int inputDim, unsigned  int outputDim, double unit_map, RandGen* randGen)
  {
	int nbInternalConnection;
	int nbInputConnectionPN;
	int nbOutputConnectionPN;

	nbInputs = inputDim;
	nbOutputs = outputDim;

	nbInternalConnection = nbNeurons*nbNeurons*CONNECTION_RATIO;
	nbInputConnectionPN = nbNeurons*INRATIO;
	nbOutputConnectionPN = nbNeurons*OUTRATIO;

	inputWeights.set(nbNeurons,inputDim);
	outputWeights.set(outputDim,nbNeurons);
	ESNNeurons.set(nbNeurons,1);
	ESNWeights.set(nbNeurons,nbNeurons);

	//inputWeights=inputWeights.map(random_minusone_to_one)*TIMESCALE;
	//outputWeights=outputWeights.map(random_minusone_to_one)*TIMESCALE;
	
	for(int count1 = 0; count1 < nbInputs; count1++)
	{
		for(int count2 = 0; count2 < nbInputConnectionPN; count2++)
		{
			int i = rand()%nbNeurons;
			inputWeights.val(i,count1) = random_minusone_to_one(0)*TIMESCALE;
		}
	}

	for(int count1 = 0; count1 < nbOutputs; count1++)
	{
		for(int count = 0; count < nbOutputConnectionPN; count++)
		{
			int i = rand()%nbNeurons;
			outputWeights.val(count1,i) = random_minusone_to_one(0)*TIMESCALE;
		}
	}

	for(int count = 0; count < nbInternalConnection; count++)
	{
		int i = rand()%nbNeurons;
		int j = rand()%nbNeurons;
		ESNWeights.val(i,j) = random_minusone_to_one(0)*TIMESCALE;
	}
	 
  }


  const Matrix ESN::process (const Matrix& input)
  { 
	ESNNeurons = (inputWeights*input+ESNWeights*ESNNeurons).map(tanh); 
	return outputWeights* ESNNeurons;
  }

  
  const Matrix ESN::learn (const Matrix& input, const Matrix& nom_output, double learnRateFactor)
  {
	const Matrix& output = process(input);
	const Matrix& delta = nom_output - output;	
	error = delta.norm_sqr();
	outputWeights += (delta * (ESNNeurons^T)) * (learnRateFactor * eps);
	return output;
	
  }

  void ESN::damp(double damping)//Damp is Dumb
  {

  }

  unsigned int ESN::getInputDim() const
  {
	return inputWeights.getM();
  }

  unsigned int ESN::getOutputDim() const
  {
	return outputWeights.getN();
  }

  bool ESN::store(FILE* f) const{
  // save matrix values

    inputWeights.store(f);
    outputWeights.store(f);
    outputWeights.store(f);
    ESNWeights.store(f);
    Configurable::print(f,0);
    return true;
  }

  /* loads the ESN values from a given file. */
  bool ESN::restore(FILE* f){
    // save matrix values
    inputWeights.restore(f);
    outputWeights.restore(f);
    outputWeights.restore(f);
    ESNWeights.restore(f);
    Configurable::parse(f);
    return true;
  }

