package org.gogpsproject.positioning;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public abstract class KalmanFilter extends LS_DD_code {

  /** The st dev init. */
  private static final double stDevInit = 1;

  /** The st dev e. */
  private static final double stDevE = 0.5;

  /** The st dev n. */
  private static final double stDevN = 0.5;

  /** The st dev u. */
  private static final double stDevU = 0.1;

  /** The st dev code c. */
  private static final double stDevCodeC = 0.3;

  /** The st dev code p. */
  private static final double[] stDevCodeP = { 0.6, 0.4 };

  /** The st dev phase. */
  static final double stDevPhase = 0.003;

  /** The st dev ambiguity. */
  static final double stDevAmbiguity = 10;

  int o1, o2, o3;
  int i1, i2, i3;
  int nN;

  SimpleMatrix T;
  SimpleMatrix H;
  SimpleMatrix y0;
  SimpleMatrix Cvv;
  SimpleMatrix Cee;
  SimpleMatrix Cnn;
  SimpleMatrix KFstate;
  SimpleMatrix KFprediction;

  // Fields for keeping track of satellite configuration changes
  ArrayList<Integer> satOld;
  ArrayList<Character> satTypeOld;
  int oldPivotId;
  char oldPivotType;
  
  public KalmanFilter( GoGPS goGPS ){
    super( goGPS );
  }

  abstract void setup(Observations roverObs, Observations masterObs, Coordinates masterPos);
  abstract void estimateAmbiguities( Observations roverObs, Observations masterObs, Coordinates masterPos, ArrayList<Integer> satAmb, int pivotIndex, boolean init);
  abstract void checkSatelliteConfiguration( Observations roverObs, Observations masterObs, Coordinates masterPos );


  /**
   * Gets the st dev code.
   *
   * @param roverObsSet the rover observation set
   * @param masterObsSet the master observation set
   * @param i the selected GPS frequency
   * @return the stDevCode
   */
  public double getStDevCode( ObservationSet obsSet, int i) {
    return obsSet.isPseudorangeP(i)?stDevCodeP[i]:stDevCodeC;
  }

  
  /**
   * @param roverObs
   * @param masterObs
   */
  void computeDopplerPredictedPhase( Observations roverObs, Observations masterObs ) {

    rover.dopplerPredPhase = new double[32];
    
    if (masterObs != null)
      master.dopplerPredPhase = new double[32];

    for (int i = 0; i < sats.availPhase.size(); i++) {

      int satID = sats.availPhase.get(i);
      char satType = sats.typeAvailPhase.get(i);
      
      double roverPhase = roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
      float roverDoppler = roverObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
      if (!Double.isNaN(roverPhase) && !Float.isNaN(roverDoppler))
        rover.setDopplerPredictedPhase(sats.availPhase.get(i), roverPhase - roverDoppler);
      
      if (masterObs != null) {
        double masterPhase = masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
        float masterDoppler = masterObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
        if (!Double.isNaN(masterPhase) && !Float.isNaN(masterDoppler))
          master.setDopplerPredictedPhase(sats.availPhase.get(i), masterPhase - masterDoppler);
      }
    }
  }

  public void init( Observations roverObs, Observations masterObs, Coordinates masterPos) {
  
    // Order-related quantities
    o1 = goGPS.getDynamicModel().getOrder();
    o2 = goGPS.getDynamicModel().getOrder() * 2;
    o3 = goGPS.getDynamicModel().getOrder() * 3;
  
    // Order-related indices
    i1 = o1 - 1;
    i2 = o2 - 1;
    i3 = o3 - 1;
  
    // Set number of ambiguities
    if (goGPS.isDualFreq())
      nN = 64;
    else
      nN = 32;
  
    // Allocate matrices
    T = SimpleMatrix.identity( o3 + nN);
    KFstate = new SimpleMatrix( o3 + nN, 1);
    KFprediction = new SimpleMatrix(o3 + nN, 1);
    Cvv = new SimpleMatrix(o3 + nN, o3 + nN);
    Cee = new SimpleMatrix(o3 + nN, o3 + nN);
  
    // System dynamics
    int j = 0;
    for (int i = 0; i < o3; i++) {
      if (j < (o1 - 1)) {
        T.set(i, i + 1, 1);
        j++;
      } else {
        j = 0;
      }
    }
  
    // Model error covariance matrix
    Cvv.zero();
    Cvv.set(i1, i1, Math.pow( stDevE, 2));
    Cvv.set(i2, i2, Math.pow( stDevN, 2));
    Cvv.set(i3, i3, Math.pow(stDevU, 2));
  
    // Improve approximate position accuracy by applying twice code double differences
    for (int i = 0; i < 2; i++) {
      // Select satellites available for double differences
      if (masterObs != null)
        sats.selectDoubleDiff(roverObs, masterObs, masterPos);
      else
        sats.selectStandalone(roverObs);
  
      if (sats.avail.size() >= 4) {
        if (masterObs != null)
          codeDoubleDifferences( roverObs, masterObs, masterPos);
        else
          codeStandalone( roverObs, false, false);
      } else {
        rover.setXYZ(0, 0, 0);
        return;
      }
    }
  
    // Estimate phase ambiguities
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    newSatellites.addAll(sats.availPhase);
    
    estimateAmbiguities( roverObs, masterObs, masterPos, newSatellites, sats.pivot, true);
  
    // Compute predicted phase ranges based on Doppler observations
    computeDopplerPredictedPhase(roverObs, masterObs);
  
    // Initial state
    KFstate.set(0, 0, rover.getX());
    KFstate.set(i1 + 1, 0, rover.getY());
    KFstate.set(i2 + 1, 0, rover.getZ());
  
    // Prediction
    KFprediction = T.mult(KFstate);
  
    // Covariance matrix of the initial state
    if(positionCovariance != null) {
      Cee.set(0, 0, positionCovariance.get(0, 0));
      Cee.set(i1 + 1, i1 + 1, positionCovariance.get(1, 1));
      Cee.set(i2 + 1, i2 + 1, positionCovariance.get(2, 2));
    } else {
      positionCovariance = new SimpleMatrix(3, 3);
      Cee.set(0, 0, Math.pow(stDevInit, 2));
      Cee.set(i1 + 1, i1 + 1, Math.pow(stDevInit, 2));
      Cee.set(i2 + 1, i2 + 1, Math.pow(stDevInit, 2));
    }
    for (int i = 1; i < o1; i++) {
      Cee.set(i, i, Math.pow(stDevInit, 2));
      Cee.set(i + i1 + 1, i + i1 + 1, Math.pow(stDevInit, 2));
      Cee.set(i + i2 + 1, i + i2 + 1, Math.pow(stDevInit, 2));
    }
  }

  SimpleMatrix compute_residuals( SimpleMatrix X ) {
	  SimpleMatrix residuals = y0.minus(H.mult(X));
	  return residuals;
  }
  
  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   *
   */
  public void loop( Observations roverObs, Observations masterObs, Coordinates masterPos) {

    final int minNumSat = 2;

    // Set linearization point (approximate coordinates by KF prediction at previous step)
    rover.setXYZ(KFprediction.get(0), KFprediction.get(i1 + 1), KFprediction.get(i2 + 1));

    // Save previous list of available satellites with phase
    satOld = sats.availPhase;
    satTypeOld = sats.typeAvailPhase;

    // Save the ID and index of the previous sats.pivot satellite
    try {
      oldPivotId   = sats.pos[sats.pivot].getSatID();
      oldPivotType = sats.pos[sats.pivot].getSatType();
    } catch(ArrayIndexOutOfBoundsException e) {
      oldPivotId = 0;
    }

    // Select satellites for standalone
    sats.selectStandalone(roverObs);

    if( sats.avail.size() >= 4)
      // Estimate receiver clock error by code stand-alone
      codeStandalone( roverObs, true, false);

    int obsReduction = 0;
    
    if (masterObs != null) {
      // Select satellites for double differences
      sats.selectDoubleDiff(roverObs, masterObs, masterPos);
      obsReduction = 1;
    }

    // Number of observations (code and phase)
    int nObs = sats.avail.size();

    // Double differences with respect to pivot satellite reduce number of observations by 1
    nObs = nObs - obsReduction;

    if( sats.availPhase.size() != 0) {
      // Add number of satellites with phase (minus 1 for double diff)
      nObs = nObs + sats.availPhase.size() - obsReduction;
    }

    if( sats.avail.size() >= minNumSat ) {
      // Allocate transformation matrix
      H = new SimpleMatrix(nObs, o3 + nN);

      // Allocate observation vector
      y0 = new SimpleMatrix(nObs, 1);

      // Allocate observation error covariance matrix
      Cnn = new SimpleMatrix(nObs, nObs);

      // Allocate K and G matrices
      SimpleMatrix K = new SimpleMatrix(o3 + nN, o3 + nN);
      SimpleMatrix G = new SimpleMatrix(o3 + nN, nObs);

      // Re-initialization of the model error covariance matrix
      Cvv.zero();

      // Set variances only if dynamic model is not static
      if (o1 != 1) {
        // Allocate and build rotation matrix
        SimpleMatrix R = Coordinates.rotationMatrix(rover);

        // Build 3x3 diagonal matrix with variances
        SimpleMatrix diagonal = new SimpleMatrix(3, 3);
        diagonal.set(0, 0, Math.pow(stDevE, 2));
        diagonal.set(1, 1, Math.pow(stDevN, 2));
        diagonal.set(2, 2, Math.pow(stDevU, 2));

        // Propagate local variances to global variances
        diagonal = R.transpose().mult(diagonal).mult(R);

        // Set global variances in the model error covariance matrix
        Cvv.set(i1, i1, diagonal.get(0, 0));
        Cvv.set(i1, i2, diagonal.get(0, 1));
        Cvv.set(i1, i3, diagonal.get(0, 2));
        Cvv.set(i2, i1, diagonal.get(1, 0));
        Cvv.set(i2, i2, diagonal.get(1, 1));
        Cvv.set(i2, i3, diagonal.get(1, 2));
        Cvv.set(i3, i1, diagonal.get(2, 0));
        Cvv.set(i3, i2, diagonal.get(2, 1));
        Cvv.set(i3, i3, diagonal.get(2, 2));
      }
      
      // Fill in Kalman filter transformation matrix, observation vector and observation error covariance matrix
      setup(roverObs, masterObs, masterPos);
      
      // Check if satellite configuration changed since the previous epoch
      checkSatelliteConfiguration(roverObs, masterObs, masterPos);

      // Identity matrix
      SimpleMatrix I = SimpleMatrix.identity(o3 + nN);

      // Kalman filter equations
      K = T.mult(Cee).mult(T.transpose()).plus(Cvv);
      G = K.mult(H.transpose()).mult(H.mult(K).mult(H.transpose()).plus(Cnn).invert());
      
      // look for outliers
      if( goGPS.searchForOutliers() ) {
    	  
	      SimpleMatrix Xhat_t_t = I.minus(G.mult(H)).mult(KFprediction).plus(G.mult(y0));
	      SimpleMatrix residuals = compute_residuals(Xhat_t_t);
	      
	//      remove observations with residuals exceeding thresholds
	      int r = 0;
	      for( ; r<residuals.getNumElements()/2; r++ ) {
	    	  	if( Math.abs( residuals.get(r) )> goGPS.getCodeResidThreshold()) {
	    	  		H.setRow( r, 0, new double[H.numCols()]);
	    	  		y0.setRow( r, 0, new double[y0.numCols()]);
	    	  		Cnn.setRow( r, 0, new double[Cnn.numCols()]);
	    	  	}
	      }
	      
	      for( ; r<residuals.getNumElements(); r++ ) {
	  	  	if( Math.abs( residuals.get(r) )> goGPS.getPhaseResidThreshold() ) {
		  		H.setRow( r, 0, new double[H.numCols()]);
		  		y0.setRow( r, 0, new double[y0.numCols()]);
		  		Cnn.setRow( r, 0, new double[Cnn.numCols()]);
	  	  	}
	      }
      }
      
      KFstate = I.minus(G.mult(H)).mult(KFprediction).plus(G.mult(y0));
      KFprediction = T.mult(KFstate);
      Cee = I.minus(G.mult(H)).mult(K);

    } else {

      // Positioning only by system dynamics
      KFstate = KFprediction;
      KFprediction = T.mult(KFstate);
      Cee = T.mult(Cee).mult(T.transpose());
    }

    // Compute predicted phase ranges based on Doppler observations
    computeDopplerPredictedPhase(roverObs, masterObs);

    // Set receiver position
    rover.setXYZ(KFstate.get(0), KFstate.get(i1 + 1), KFstate.get(i2 + 1));

    positionCovariance.set(0, 0, Cee.get(0, 0));
    positionCovariance.set(1, 1, Cee.get(i1 + 1, i1 + 1));
    positionCovariance.set(2, 2, Cee.get(i2 + 1, i2 + 1));
    positionCovariance.set(0, 1, Cee.get(0, i1 + 1));
    positionCovariance.set(0, 2, Cee.get(0, i2 + 1));
    positionCovariance.set(1, 0, Cee.get(i1 + 1, 0));
    positionCovariance.set(1, 2, Cee.get(i1 + 1, i2 + 1));
    positionCovariance.set(2, 0, Cee.get(i2 + 1, 0));
    positionCovariance.set(2, 1, Cee.get(i2 + 1, i1 + 1));

    // Allocate and build rotation matrix
    SimpleMatrix R = Coordinates.rotationMatrix(rover);

    // Propagate covariance from global system to local system
    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU = R.mult(positionCovariance).mult(R.transpose());

    // Kalman filter DOP computation
    rover.kpDop = Math.sqrt(positionCovariance.get(0, 0) + positionCovariance.get(1, 1) + positionCovariance.get(2, 2));
    rover.khDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    rover.kvDop = Math.sqrt(covENU.get(2, 2));

    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();
  }
}
