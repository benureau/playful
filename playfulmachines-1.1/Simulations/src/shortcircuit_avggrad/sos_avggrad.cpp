/***************************************************************************
 *   Copyright (C) 2010 by Robot Group Leipzig                             *
 *    martius@informatik.uni-leipzig.de                                    *
 *    der@informatik.uni-leipzig.de                                        *
 *                                                                         *
 *   LICENSE:                                                              *
 *   This work is licensed under the Creative Commons                      *
 *   Attribution-NonCommercial-ShareAlike 2.5 License. To view a copy of   *
 *   this license, visit http://creativecommons.org/licenses/by-nc-sa/2.5/ *
 *   or send a letter to Creative Commons, 543 Howard Street, 5th Floor,   *
 *   San Francisco, California, 94105, USA.                                *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                  *
 *                                                                         *
 *                                                                         *
 ***************************************************************************/

#include "sos_avggrad.h"

#include <selforg/matrixutils.h>
using namespace matrix;
using namespace std;

SosAvgGrad::SosAvgGrad()
  : AbstractController("SosAvgGrad", "0.1"){
  t=0;

  addParameterDef("epsC", &epsC, 0.1,  "learning rate of the controller");
  addParameterDef("epsA", &epsA, 0.0,  "learning rate of the model");
  addParameterDef("s4avg", &s4avg, 1,  "smoothing (number of steps)");
  addParameterDef("s4delay", &s4delay, 1, "delay  (number of steps)");
  addParameterDef("sense", &sense, 1,  "sense");

  //  addInspectableMatrix("A", &A, false, "model matrix");
  //  addInspectableMatrix("b", &b, false, "model bias");
  addInspectableMatrix("C", &C, false, "controller matrix");
  addInspectableMatrix("S", &S, false, "model matrix 2");
  addInspectableMatrix("L", &L, false, "Jacobi matrix");
  addInspectableMatrix("R", &R, false, "AC+S");
  // addInspectableMatrix("CU1", &CU1, false, "C update driving");
  // addInspectableMatrix("CU2", &CU2, false, "C update confining");
  // addInspectableMatrix("CU3", &CU3, false, "C update (total)");
  addInspectableValue("Det", &det, "determinant of C+S");
  addInspectableValue("Trace", &trace, "trace of C+S");
  addInspectableValue("E",  &E, "Error");
 
};

SosAvgGrad::~SosAvgGrad(){
}


void SosAvgGrad::init(int sensornumber, int motornumber, RandGen* randGen){
  if(!randGen) randGen = new RandGen(); // this gives a small memory leak
 
  number_sensors= sensornumber;
  number_motors = motornumber;
  A.set(number_sensors, number_motors);
  C.set(number_motors, number_sensors);
  b.set(number_sensors, 1);
  S.set(number_sensors, number_sensors);

  A.toId(); // set a to identity matrix;
  C = (C^0)*0.5; // set a to identity matrix;

  L.set(number_sensors, number_sensors);

  R.set(number_sensors, number_sensors);
  // CU1.set(number_motors,number_motors);
  // CU2.set(number_motors,number_motors);
  // CU3.set(number_motors,number_motors);
  
  x.set(number_sensors,1);
  x_smooth.set(number_sensors,1);
  for (unsigned int k = 0; k < buffersize; k++) {
    x_buffer[k].set(number_sensors,1);   
    y_buffer[k].set(number_motors,1);   
  }
}

matrix::Matrix SosAvgGrad::getA(){
  return A;
}

void SosAvgGrad::setA(const matrix::Matrix& _A){
  assert(A.getM() == _A.getM() && A.getN() == _A.getN());
  A=_A;
}

matrix::Matrix SosAvgGrad::getC(){
  return C;
}

void SosAvgGrad::setC(const matrix::Matrix& _C){
  assert(C.getM() == _C.getM() && C.getN() == _C.getN());
  C=_C;
}

matrix::Matrix SosAvgGrad::getS(){
  return S;
}

void SosAvgGrad::setS(const matrix::Matrix& _S){
  assert(S.getM() == _S.getM() && S.getN() == _S.getN());
  S=_S;
}


// performs one step (includes learning). Calculates motor commands from sensor inputs.
void SosAvgGrad::step(const sensor* x_, int number_sensors, 
                      motor* y_, int number_motors){
  stepNoLearning(x_, number_sensors, y_, number_motors);
  if(t<=buffersize) return;
  t--; // stepNoLearning increases the time by one - undo here

  // learn controller and model
  learn();

  // update step counter
  t++;
};


// performs one step without learning. Calulates motor commands from sensor inputs.
void SosAvgGrad::stepNoLearning(const sensor* x_, int number_sensors, 
                                motor* y_, int number_motors){
  assert((unsigned)number_sensors <= this->number_sensors 
         && (unsigned)number_motors <= this->number_motors);

  x.set(number_sensors,1,x_); // store sensor values
  
  // averaging over the last s4avg values of x_buffer
  s4avg = ::clip(s4avg,1,buffersize-1);
  if(s4avg > 1)
    x_smooth += (x - x_smooth)*(1.0/s4avg);
  else
    x_smooth = x;
  
  x_buffer[t%buffersize] = x_smooth; // we store the smoothed sensor value
  
  // calculate controller values based on current input values (smoothed)  
  Matrix y =   (C*(x_smooth)).map(g);
  
  // Put new output vector in ring buffer y_buffer
  y_buffer[t%buffersize] = y;

  // convert y to motor* 
  y.convertToBuffer(y_, number_motors);

  // update step counter
  t++;
};
  

Matrix SosAvgGrad::diagonals(const Matrix& matrix){
  int l = min(matrix.getM(), matrix.getN());
  Matrix dia(l,1);
  for(int i=0; i<l; i++){
    dia.val(i,0) = matrix.val(i,i);
  }
  return dia;
}
  
// learn values h,C,A
void SosAvgGrad::learn(){

  Matrix C_update(number_motors,number_sensors);
  Matrix h_update(number_motors,1);

  // the effective x/y is (actual-steps4delay) element of buffer  
  s4delay = ::clip(s4delay,1,buffersize-1);
  const Matrix& x = x_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& y = y_buffer[(t - max(s4delay,1) + buffersize) % buffersize];
  const Matrix& x_fut = x_buffer[t% buffersize]; // future sensor (with respect to x,y)
 
  const Matrix& xsi = x_fut  - (A* y + b);
  A += (xsi * (y^T) * epsA + (A *  -0.0001) * ( epsA > 0 ? 1 : 0)).mapP(0.1,clip);
  b += (xsi *  epsA /*       + (b *  -0.001) * ( epsA > 0 ? 1 : 0)*/).mapP(0.1,clip);
 

  //************ TLE (homeokinetic) learning using averaged gradient **********
  const Matrix& z    = (C * (x));
  const Matrix& g_prime = z.map(g_s);
  L       = A * (C & g_prime) + S;
  const Matrix& Q       = (L.multMT()*L).pseudoInverse(); // 1/(L L^T L)
  const Matrix& epsrel  = diagonals(C*Q*A)  & g_prime * 2 * sense;
  
  
  C_update =  (((A^T) & g_prime) * (Q^T) -  (epsrel & y) * (x^T)) * (epsC);   
  
  // apply updates to h,C (clipped to [-.01,0.01])
  C += C_update.mapP(1, clip);

  // some statistics
  const Matrix& lltm1 = L.multMT()^(-1);
  E = lltm1.val(0,0)+lltm1.val(1,1);
   
  // CU1 += (((A^T) & g_prime) * (Q^T) -CU1)*0.01;
  // //  CU2 =  (epsrel & y) * (x^T) * -1;
  // CU2 +=  (((epsrel & y) * (x^T) * -1) -CU2)*0.01 ;
  // CU3 +=  (CU1+CU2 -CU3)*0.01;
  
  R = A*C+S;
  const Matrix& M = R;
  det=M.val(0,0)*M.val(1,1) - M.val(1,0)*M.val(0,1);
  trace=M.val(0,0)+M.val(1,1);
};

  
/* stores the controller values to a given file. */
bool SosAvgGrad::store(FILE* f) const{  
  // save matrix values
  C.store(f);
  A.store(f);
  b.store(f);
  S.store(f);
  Configurable::print(f,0);
  return true;
}

/* loads the controller values from a given file. */
bool SosAvgGrad::restore(FILE* f){
  // save matrix values
  C.restore(f);
  A.restore(f);
  b.restore(f);
  S.restore(f);
  Configurable::parse(f);
  t=0; // set time to zero to ensure proper filling of buffers
  return true;
}

