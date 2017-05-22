package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;

public abstract class ReceiverPosition extends Coordinates {

  // Fields related to receiver-satellite geometry
  
  /** receiver-satellite approximate range */
  double[] satAppRange; 
  
  /** receiver-satellite troposphere correction */
  double[] satTropoCorr; 

  /** receiver-satellite ionosphere correction */
  double[] satIonoCorr; 

  /** receiver-satellite vector */
  SimpleMatrix[] diffSat; 

  // Fields for satellite selection
  TopocentricCoordinates[] topo;

  // Fields for storing values from previous epoch
  /** rover L Carrier Phase predicted from previous epoch (based on Doppler) [cycle] */
  double[] dopplerPredPhase; 

  /** @return the rover Doppler predicted phase */
  public double getDopplerPredictedPhase(int satID) {
    return dopplerPredPhase[satID - 1];
  }
  
  /** @param roverDopplerPredictedPhase the Doppler predicted phase to set */
  public void setDopplerPredictedPhase(int satID, double roverDopplerPredictedPhase) {
    dopplerPredPhase[satID - 1] = roverDopplerPredictedPhase;
  }

}
