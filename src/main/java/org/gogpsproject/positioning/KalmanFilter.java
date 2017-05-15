package org.gogpsproject.positioning;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.Coordinates;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Observations;

public class KalmanFilter extends Core {

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
  private ArrayList<Integer> satOld;
  private ArrayList<Character> satTypeOld;
  private int oldPivotId;
  private char oldPivotType;
  
  public KalmanFilter( GoGPS goGPS ){
    super( goGPS );
  }
  
  public void init( Observations roverObs, Observations masterObs, Coordinates masterPos) {
  
    // Order-related quantities
    o1 = goGPS.getOrder();
    o2 = goGPS.getOrder() * 2;
    o3 = goGPS.getOrder() * 3;
  
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
    Cvv.set(i1, i1, Math.pow(goGPS.getStDevE(), 2));
    Cvv.set(i2, i2, Math.pow(goGPS.getStDevN(), 2));
    Cvv.set(i3, i3, Math.pow(goGPS.getStDevU(), 2));
  
    // Improve approximate position accuracy by applying twice code double differences
    for (int i = 0; i < 2; i++) {
      // Select satellites available for double differences
      if (masterObs != null)
        selectSatellitesDoubleDiff(roverObs, masterObs, masterPos);
      else
        selectSatellitesStandalone(roverObs);
  
      if (satAvail.size() >= 4) {
        if (masterObs != null)
          new LS_DD_code(goGPS).codeDoubleDifferences( roverObs, masterObs, masterPos);
        else
          new LS_SA_code(goGPS).codeStandalone( roverObs, false, false);
      } else {
        roverPos.setXYZ(0, 0, 0);
        return;
      }
    }
  
    // Estimate phase ambiguities
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    newSatellites.addAll(satAvailPhase);
    if (masterObs != null)
      estimateAmbiguitiesDoubleDiff( roverObs, masterObs, masterPos, newSatellites, pivot, true);
    else
      estimateAmbiguitiesStandalone( roverObs, newSatellites, true);
  
    // Compute predicted phase ranges based on Doppler observations
    computeDopplerPredictedPhase(roverObs, masterObs);
  
    // Initial state
    KFstate.set(0, 0, roverPos.getX());
    KFstate.set(i1 + 1, 0, roverPos.getY());
    KFstate.set(i2 + 1, 0, roverPos.getZ());
  
    // Prediction
    KFprediction = T.mult(KFstate);
  
    // Covariance matrix of the initial state
    if (positionCovariance != null) {
      Cee.set(0, 0, positionCovariance.get(0, 0));
      Cee.set(i1 + 1, i1 + 1, positionCovariance.get(1, 1));
      Cee.set(i2 + 1, i2 + 1, positionCovariance.get(2, 2));
    } else {
      positionCovariance = new SimpleMatrix(3, 3);
      Cee.set(0, 0, Math.pow(goGPS.getStDevInit(), 2));
      Cee.set(i1 + 1, i1 + 1, Math.pow(goGPS.getStDevInit(), 2));
      Cee.set(i2 + 1, i2 + 1, Math.pow(goGPS.getStDevInit(), 2));
    }
    for (int i = 1; i < o1; i++) {
      Cee.set(i, i, Math.pow(goGPS.getStDevInit(), 2));
      Cee.set(i + i1 + 1, i + i1 + 1, Math.pow(goGPS.getStDevInit(), 2));
      Cee.set(i + i2 + 1, i + i2 + 1, Math.pow(goGPS.getStDevInit(), 2));
    }
  }

  /**
   * @param roverObs
   */
  private void estimateAmbiguitiesStandalone( Observations roverObs, ArrayList<Integer> satAmb, boolean init) {

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();

    // Number of available satellites (i.e. observations) with phase
    int nObsAvailPhase = satAvailPhase.size();

    // Number of unknown parameters
    int nUnknowns = 4 + satAmb.size();

    // Estimated ambiguities
    double[] estimatedAmbiguities;
    estimatedAmbiguities = new double[satAmb.size()];

    // Covariance of estimated ambiguity combinations
    double[] estimatedAmbiguitiesCovariance;
    estimatedAmbiguitiesCovariance = new double[satAmb.size()];

    // Satellite ID
    int id = 0;

    // Define least squares matrices
    SimpleMatrix A;
    SimpleMatrix b;
    SimpleMatrix y0;
    SimpleMatrix Qcode;
    SimpleMatrix Qphase;
    SimpleMatrix Q;
    SimpleMatrix x;
    SimpleMatrix vEstim;
    SimpleMatrix covariance;
    SimpleMatrix tropoCorr;
    SimpleMatrix ionoCorr;

    // Least squares design matrix
    A = new SimpleMatrix(nObsAvail+nObsAvailPhase, nUnknowns);
    A.zero();

    // Vector for approximate pseudoranges
    b = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

    // Vector for observed pseudoranges
    y0 = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

    // Cofactor matrices
    Qcode = new SimpleMatrix(nObsAvail, nObsAvail);
    Qphase = new SimpleMatrix(nObsAvailPhase, nObsAvailPhase);
    Q = new SimpleMatrix(nObsAvail+nObsAvailPhase, nObsAvail+nObsAvailPhase);
    Q.zero();

    // Solution vector
    x = new SimpleMatrix(nUnknowns, 1);

    // Vector for observation error
    vEstim = new SimpleMatrix(nObsAvail, 1);

    // Error covariance matrix
    covariance = new SimpleMatrix(nUnknowns, nUnknowns);

    // Vectors for troposphere and ionosphere corrections
    tropoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);
    ionoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

    // Counters for available satellites
    int k = 0;
    int p = 0;

    // Set up the least squares matrices...
    // ... for code ...
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)) {

        // Fill in one row in the design matrix
        A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i]); /* X */
        A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i]); /* Y */
        A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i]); /* Z */

        A.set(k, 3, 1); /* clock error */

        // Add the approximate pseudorange value to b
        b.set(k, 0, roverSatAppRange[i] - pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT);

        // Add the observed pseudorange value to y0
        y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));

        // Fill in troposphere and ionosphere double corrections
        tropoCorr.set(k, 0, roverSatTropoCorr[i]);
        ionoCorr.set(k, 0, roverSatIonoCorr[i]);

        // Fill in the cofactor matrix
        double roverSatWeight = computeWeight(roverTopo[i].getElevation(), roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        Qcode.set(k, k, Qcode.get(k, k) + goGPS.getStDevCode(roverObs.getSatByID(id), goGPS.getFreq()) * roverSatWeight);

        // Increment available satellites counter
        k++;
      }
    }

    // ... and phase
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)) {

        // Fill in one row in the design matrix
        A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i]); /* X */

        A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i]); /* Y */

        A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i]); /* Z */

        A.set(k, 3, 1); /* clock error */

        if (satAmb.contains(id)) {
          A.set(k, 4 + satAmb.indexOf(id), -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq())); /* N */

          // Add the observed phase range value to y0
          y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()));
        } else {
          // Add the observed phase range value + known N to y0
          y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) + KFprediction.get(i3 + id) * roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()));
        }

        // Add the approximate pseudorange value to b
        b.set(k, 0, roverSatAppRange[i] - pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT);

        // Fill in troposphere and ionosphere corrections
        tropoCorr.set(k, 0, roverSatTropoCorr[i]);
        ionoCorr.set(k, 0, -roverSatIonoCorr[i]);

        // Fill in the cofactor matrix
        double roverSatWeight = computeWeight(roverTopo[i].getElevation(), roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        Qphase.set(p, p, Qphase.get(p, p) + Math.pow(goGPS.getStDevPhase(), 2) * roverSatWeight);
        int r = 1;
        for (int m = i+1; m < nObs; m++) {
          if (pos[m] !=null && satAvailPhase.contains(pos[m].getSatID())) {
            Qphase.set(p, p+r, 0);
            Qphase.set(p+r, p, 0);
            r++;
          }
        }

        // Increment available satellite counters
        k++;
        p++;
      }
    }

    // Apply troposphere and ionosphere correction
    b = b.plus(tropoCorr);
    b = b.plus(ionoCorr);

    //Build complete cofactor matrix (code and phase)
    Q.insertIntoThis(0, 0, Qcode);
    Q.insertIntoThis(nObsAvail, nObsAvail, Qphase);

    // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
    x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(y0.minus(b));

    // Receiver clock error
    roverPos.receiverClockError = x.get(3) / Constants.SPEED_OF_LIGHT;

    // Estimation of the variance of the observation error
    vEstim = y0.minus(A.mult(x).plus(b));
    double varianceEstim = (vEstim.transpose().mult(Q.invert()).mult(vEstim)).get(0) / (nObsAvail + nObsAvailPhase - nUnknowns);

    // Covariance matrix of the estimation error
    covariance = A.transpose().mult(Q.invert()).mult(A).invert().scale(varianceEstim);

    // Store estimated ambiguity combinations and their covariance
    for (int m = 0; m < satAmb.size(); m++) {
      estimatedAmbiguities[m] = x.get(4 + m);
      estimatedAmbiguitiesCovariance[m] = covariance.get(4 + m, 4 + m);
    }

    if (init) {
      for (int i = 0; i < satAmb.size(); i++) {
        // Estimated ambiguity
        KFstate.set(i3 + satAmb.get(i), 0, estimatedAmbiguities[i]);

        // Store the variance of the estimated ambiguity
        Cee.set(i3 + satAmb.get(i), i3 + satAmb.get(i), estimatedAmbiguitiesCovariance[i]);
      }
    } else {
      for (int i = 0; i < satAmb.size(); i++) {
        // Estimated ambiguity
        KFprediction.set(i3 + satAmb.get(i), 0, estimatedAmbiguities[i]);

        // Store the variance of the estimated ambiguity
        Cvv.set(i3 + satAmb.get(i), i3 + satAmb.get(i), Math.pow(goGPS.getStDevAmbiguity(), 2));
      }
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  private void estimateAmbiguitiesDoubleDiff( Observations roverObs,
      Observations masterObs, Coordinates masterPos, ArrayList<Integer> satAmb, int pivotIndex, boolean init) {

    // Check if pivot is in satAmb, in case remove it
    if (satAmb.contains(pos[pivotIndex].getSatID()))
      satAmb.remove(satAmb.indexOf(pos[pivotIndex].getSatID()));

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();

    // Number of available satellites (i.e. observations) with phase
    int nObsAvailPhase = satAvailPhase.size();

    // Double differences with respect to pivot satellite reduce
    // observations by 1
    nObsAvail--;
    nObsAvailPhase--;

    // Number of unknown parameters
    int nUnknowns = 3 + satAmb.size();

    // Pivot satellite ID
    int pivotId = roverObs.getSatID(pivotIndex);
    char satType = roverObs.getGnssType(pivotIndex);

    // Rover-pivot and master-pivot observed pseudorange
    double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Rover-pivot and master-pivot observed phase
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = diffRoverSat[pivotIndex];
    double roverPivotAppRange = roverSatAppRange[pivotIndex];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = masterSatAppRange[pivotIndex];

    // Estimated ambiguity combinations (double differences)
    double[] estimatedAmbiguityComb;
    estimatedAmbiguityComb = new double[satAmb.size()];

    // Covariance of estimated ambiguity combinations
    double[] estimatedAmbiguityCombCovariance;
    estimatedAmbiguityCombCovariance = new double[satAmb.size()];

    // Variables to store rover-satellite and master-satellite observed code
    double roverSatCodeObs;
    double masterSatCodeObs;

    // Variables to store rover-satellite and master-satellite observed phase
    double roverSatPhaseObs;
    double masterSatPhaseObs;

    // Variables to store rover-satellite and master-satellite approximate code
        double roverSatCodeAppRange;
        double masterSatCodeAppRange;

    // Variables to store code and phase double differences
    double codeDoubleDiffObserv;
    double codeDoubleDiffApprox;
    double phaseDoubleDiffObserv;

    // Satellite ID
    int id = 0;

    if (goGPS.getAmbiguityStrategy() == GoGPS.AMBIGUITY_OBSERV) {

      for (int i = 0; i < nObs; i++) {

        id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);

        if (pos[i]!=null && satAmb.contains(id) && id != pivotId) {

          // Rover-satellite and master-satellite observed code
          roverSatCodeObs = roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
          masterSatCodeObs = masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());

          // Rover-satellite and master-satellite observed phase
          roverSatPhaseObs = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());
          masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

          // Observed code double difference
          codeDoubleDiffObserv = (roverSatCodeObs - masterSatCodeObs) - (roverPivotCodeObs - masterPivotCodeObs);

          // Observed phase double difference
          phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs) - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          estimatedAmbiguityComb[satAmb.indexOf(id)] = (codeDoubleDiffObserv - phaseDoubleDiffObserv) / roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());
          estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
          * goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
          * goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
        }
      }
    } else if(goGPS.getAmbiguityStrategy() == GoGPS.AMBIGUITY_APPROX | (nObsAvail + nObsAvailPhase <= nUnknowns)) {

      for (int i = 0; i < nObs; i++) {

        id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);

        if( pos[i]!=null && satAmb.contains(id) && id != pivotId) {

          // Rover-satellite and master-satellite approximate pseudorange
                  roverSatCodeAppRange  = roverSatAppRange[i];
                  masterSatCodeAppRange = masterSatAppRange[i];

                  // Rover-satellite and master-satellite observed phase
          roverSatPhaseObs = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());
          masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

                  // Estimated code pseudorange double differences
                  codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange)
                                       - (roverPivotAppRange - masterPivotAppRange);

                  // Observed phase double differences
                  phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs)
                                  - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          estimatedAmbiguityComb[satAmb.indexOf(id)] = (codeDoubleDiffApprox - phaseDoubleDiffObserv) / roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());
          estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
          * goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
          * goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
        }
      }
    } 
    else if (goGPS.getAmbiguityStrategy() == GoGPS.AMBIGUITY_LS) {

      // Define least squares matrices
      SimpleMatrix A;
      SimpleMatrix b;
      SimpleMatrix y0;
      SimpleMatrix Qcode;
      SimpleMatrix Qphase;
      SimpleMatrix Q;
      SimpleMatrix x;
      SimpleMatrix vEstim;
      SimpleMatrix covariance;
      SimpleMatrix tropoCorr;
      SimpleMatrix ionoCorr;

      // Least squares design matrix
      A = new SimpleMatrix(nObsAvail+nObsAvailPhase, nUnknowns);
      A.zero();

      // Vector for approximate pseudoranges
      b = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Vector for observed pseudoranges
      y0 = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Cofactor matrices
      Qcode = new SimpleMatrix(nObsAvail, nObsAvail);
      Qphase = new SimpleMatrix(nObsAvailPhase, nObsAvailPhase);
      Q = new SimpleMatrix(nObsAvail+nObsAvailPhase, nObsAvail+nObsAvailPhase);
      Q.zero();

      // Solution vector
      x = new SimpleMatrix(nUnknowns, 1);

      // Vector for observation error
      vEstim = new SimpleMatrix(nObsAvail, 1);

      // Error covariance matrix
      covariance = new SimpleMatrix(nUnknowns, nUnknowns);

      // Vectors for troposphere and ionosphere corrections
      tropoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);
      ionoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

      // Counters for available satellites
      int k = 0;
      int p = 0;

      // Rover-pivot and master-pivot troposphere correction
      double roverPivotTropoCorr  = roverSatTropoCorr[pivotIndex];
      double masterPivotTropoCorr = masterSatTropoCorr[pivotIndex];;

      // Rover-pivot and master-pivot ionosphere correction
      double roverPivotIonoCorr  = roverSatIonoCorr[pivotIndex];
      double masterPivotIonoCorr = masterSatIonoCorr[pivotIndex];

      // Compute rover-pivot and master-pivot weights
      double roverPivotWeight = computeWeight(roverTopo[pivotIndex].getElevation(),
          roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
      double masterPivotWeight = computeWeight(masterTopo[pivotIndex].getElevation(),
          masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
      Qcode.set(goGPS.getStDevCode(roverObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
          * goGPS.getStDevCode(masterObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
          * (roverPivotWeight + masterPivotWeight));
      Qphase.set(Math.pow(goGPS.getStDevPhase(), 2) * (roverPivotWeight + masterPivotWeight));

      // Set up the least squares matrices...
      // ... for code ...
      for (int i = 0; i < nObs; i++) {

        id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
        
        if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)
            && i != pivotIndex) {

          // Fill in one row in the design matrix
          A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

          A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

          A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

          // Add the differenced approximate pseudorange value to b
          b.set(k, 0, (roverSatAppRange[i] - masterSatAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange));

          // Add the differenced observed pseudorange value to y0
          y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
              - (roverPivotCodeObs - masterPivotCodeObs));

          // Fill in troposphere and ionosphere double differenced
          // corrections
          tropoCorr.set(k, 0, (roverSatTropoCorr[i] - masterSatTropoCorr[i])
              - (roverPivotTropoCorr - masterPivotTropoCorr));
          ionoCorr.set(k, 0, (roverSatIonoCorr[i] - masterSatIonoCorr[i])
              - (roverPivotIonoCorr - masterPivotIonoCorr));

          // Fill in the cofactor matrix
          double roverSatWeight = computeWeight(roverTopo[i].getElevation(),
              roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
          double masterSatWeight = computeWeight(masterTopo[i].getElevation(),
              masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
          Qcode.set(k, k, Qcode.get(k, k) + goGPS.getStDevCode(roverObs.getSatByID(id), goGPS.getFreq())
              * goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq())
              * (roverSatWeight + masterSatWeight));

          // Increment available satellites counter
          k++;
        }
      }

      // ... and phase
      for (int i = 0; i < nObs; i++) {

        id = roverObs.getSatID(i);
        satType = roverObs.getGnssType(i);
        String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

        if( pos[i] !=null && gnssAvail.contains(checkAvailGnss)
            && i != pivotIndex) {

          // Fill in one row in the design matrix
          A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

          A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

          A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

          if (satAmb.contains(id)) {
            A.set(k, 3 + satAmb.indexOf(id), -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq())); /* N */

            // Add the differenced observed pseudorange value to y0
            y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
                - (roverPivotPhaseObs - masterPivotPhaseObs));
          } else {
            // Add the differenced observed pseudorange value + known N to y0
            y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
                - (roverPivotPhaseObs - masterPivotPhaseObs) + KFprediction.get(i3 + id));
          }

          // Add the differenced approximate pseudorange value to b
          b.set(k, 0, (roverSatAppRange[i] - masterSatAppRange[i])
              - (roverPivotAppRange - masterPivotAppRange));

          // Fill in troposphere and ionosphere double differenced corrections
          tropoCorr.set(k, 0, (roverSatTropoCorr[i] - masterSatTropoCorr[i]) - (roverPivotTropoCorr - masterPivotTropoCorr));
          ionoCorr.set(k, 0, -((roverSatIonoCorr[i] - masterSatIonoCorr[i]) - (roverPivotIonoCorr - masterPivotIonoCorr)));

          // Fill in the cofactor matrix
          double roverSatWeight = computeWeight(
              roverTopo[i].getElevation(), roverObs.getSatByIDType(id, satType)
              .getSignalStrength(goGPS.getFreq()));
          double masterSatWeight = computeWeight(
              masterTopo[i].getElevation(),
              masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
          Qphase.set(p, p, Qphase.get(p, p)
              + (Math.pow(goGPS.getStDevPhase(), 2) + Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2) * Cee.get(i3 + id, i3 + id))
              * (roverPivotWeight + masterPivotWeight)
              + (Math.pow(goGPS.getStDevPhase(), 2) + Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2) * Cee.get(i3 + id, i3 + id))
              * (roverSatWeight + masterSatWeight));
          int r = 1;
          for (int m = i+1; m < nObs; m++) {
            if (pos[m] !=null && satAvailPhase.contains(pos[m].getSatID()) && m != pivotIndex) {
              Qphase.set(p, p+r, 0);
              Qphase.set(p+r, p, 0);
              r++;
            }
          }
          //          int r = 1;
          //          for (int j = i+1; j < nObs; j++) {
          //            if (pos[j] !=null && satAvailPhase.contains(pos[j].getSatID()) && j != pivotIndex) {
          //              Qphase.set(p, p+r, Qphase.get(p, p+r)
          //                  + (Math.pow(lambda, 2) * Cee.get(i3 + pos[i].getSatID(), i3 + pos[j].getSatID()))
          //                  * (roverPivotWeight + masterPivotWeight));
          //              Qphase.set(p+r, p, Qphase.get(p, p+r));
          //              r++;
          //            }
          //          }

          // Increment available satellite counters
          k++;
          p++;
        }
      }

      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);

      //Build complete cofactor matrix (code and phase)
      Q.insertIntoThis(0, 0, Qcode);
      Q.insertIntoThis(nObsAvail, nObsAvail, Qphase);

      // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
      x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose())
      .mult(Q.invert()).mult(y0.minus(b));

      // Estimation of the variance of the observation error
      vEstim = y0.minus(A.mult(x).plus(b));
      double varianceEstim = (vEstim.transpose().mult(Q.invert())
          .mult(vEstim)).get(0)
          / (nObsAvail + nObsAvailPhase - nUnknowns);

      // Covariance matrix of the estimation error
      covariance = A.transpose().mult(Q.invert()).mult(A).invert()
      .scale(varianceEstim);

      // Store estimated ambiguity combinations and their covariance
      for (int m = 0; m < satAmb.size(); m++) {
        estimatedAmbiguityComb[m] = x.get(3 + m);
        estimatedAmbiguityCombCovariance[m] = covariance.get(3 + m, 3 + m);
      }
    }

    if (init) {
      for (int i = 0; i < satAmb.size(); i++) {
        // Estimated ambiguity
        KFstate.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

        // Store the variance of the estimated ambiguity
        Cee.set(i3 + satAmb.get(i), i3 + satAmb.get(i),
            estimatedAmbiguityCombCovariance[i]);
      }
    } else {
      for (int i = 0; i < satAmb.size(); i++) {
        // Estimated ambiguity
        KFprediction.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

        // Store the variance of the estimated ambiguity
        Cvv.set(i3 + satAmb.get(i), i3 + satAmb.get(i),
            Math.pow(goGPS.getStDevAmbiguity(), 2));
      }
    }
  }

  /**
   * @param roverObs
   */
  private void setupKalmanFilterInputStandalone( Observations roverObs) {

    // Definition of matrices
    SimpleMatrix A;
    SimpleMatrix covXYZ;
    SimpleMatrix covENU;

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();

    // Matrix containing parameters obtained from the linearization of the observation equations
    A = new SimpleMatrix(nObsAvail, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    covENU = new SimpleMatrix(3, 3);

    // Counter for available satellites
    int k = 0;

    // Counter for satellites with phase available
    int p = 0;

    // Satellite ID
    int id = 0;
    
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (pos[i]!=null && gnssAvail.contains(checkAvailGnss)) {

        // Compute parameters obtained from linearization of observation equations
        double alphaX = diffRoverSat[i].get(0) / roverSatAppRange[i];
        double alphaY = diffRoverSat[i].get(1) / roverSatAppRange[i];
        double alphaZ = diffRoverSat[i].get(2) / roverSatAppRange[i];

        // Fill in the A matrix
        A.set(k, 0, alphaX); /* X */
        A.set(k, 1, alphaY); /* Y */
        A.set(k, 2, alphaZ); /* Z */

        // Observed code
        double obsRangeCode = roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());

        // Observed phase
        double obsRangePhase = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

        // Compute approximate ranges
        double appRangeCode;
        double appRangePhase;
        if( goGPS.getFreq() == 0) {
          appRangeCode = roverSatAppRange[i] + Constants.SPEED_OF_LIGHT*(roverPos.receiverClockError - pos[i].getSatelliteClockError()) + roverSatTropoCorr[i] + roverSatIonoCorr[i];
          appRangePhase = roverSatAppRange[i] + Constants.SPEED_OF_LIGHT*(roverPos.receiverClockError - pos[i].getSatelliteClockError()) + roverSatTropoCorr[i] - roverSatIonoCorr[i];
        } else {
          appRangeCode = roverSatAppRange[i] + Constants.SPEED_OF_LIGHT*(roverPos.receiverClockError - pos[i].getSatelliteClockError()) + roverSatTropoCorr[i] + roverSatIonoCorr[i] * Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(1)/roverObs.getSatByIDType(id, satType).getWavelength(0), 2);
          appRangePhase = roverSatAppRange[i] + Constants.SPEED_OF_LIGHT*(roverPos.receiverClockError - pos[i].getSatelliteClockError()) + roverSatTropoCorr[i] - roverSatIonoCorr[i] * Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(1)/roverObs.getSatByIDType(id, satType).getWavelength(0), 2);
        }

        // Fill in one row in the design matrix (for code)
        H.set(k, 0, alphaX);
        H.set(k, i1 + 1, alphaY);
        H.set(k, i2 + 1, alphaZ);

        // Fill in one element of the observation vector (for code)
        y0.set(k, 0, obsRangeCode - appRangeCode + alphaX * roverPos.getX() + alphaY * roverPos.getY() + alphaZ * roverPos.getZ());

        // Fill in the observation error covariance matrix (for code)
        double roverSatWeight = computeWeight(roverTopo[i].getElevation(), roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        double CnnBase = Cnn.get(k, k);
        Cnn.set(k, k, CnnBase + Math.pow(goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq()), 2) * roverSatWeight);

        if (gnssAvail.contains(checkAvailGnss)){
//        if (satAvailPhase.contains(id) && satTypeAvailPhase.contains(satType)) {

          // Fill in one row in the design matrix (for phase)
          H.set(nObsAvail + p, 0, alphaX);
          H.set(nObsAvail + p, i1 + 1, alphaY);
          H.set(nObsAvail + p, i2 + 1, alphaZ);
          H.set(nObsAvail + p, i3 + id, -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()));

          // Fill in one element of the observation vector (for phase)
          y0.set(nObsAvail + p, 0, obsRangePhase - appRangePhase + alphaX * roverPos.getX() + alphaY * roverPos.getY() + alphaZ * roverPos.getZ());

          // Fill in the observation error covariance matrix (for phase)
          CnnBase = Cnn.get(nObsAvail + p, nObsAvail + p);
          Cnn.set(nObsAvail + p, nObsAvail + p, CnnBase + Math.pow(goGPS.getStDevPhase(), 2) * roverSatWeight);

          // Increment satellites with phase counter
          p++;
        }

        // Increment available satellites counter
        k++;
      }
    }

    // Compute covariance matrix from A matrix [ECEF reference system]
    covXYZ = A.transpose().mult(A).invert();

    // Allocate and build rotation matrix
    SimpleMatrix R = new SimpleMatrix(3, 3);
    R = Coordinates.rotationMatrix(roverPos);

    // Propagate covariance from global system to local system
    covENU = R.mult(covXYZ).mult(R.transpose());

    //Compute DOP values
    roverPos.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    roverPos.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    roverPos.vDop = Math.sqrt(covENU.get(2, 2));
  }

  /**
   * @param roverObs
   */
  private void checkSatelliteConfigurationStandalone( Observations roverObs) {

    // Lists for keeping track of satellites that need ambiguity (re-)estimation
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    ArrayList<Integer> slippedSatellites = new ArrayList<Integer>(0);

    // Check if satellites were lost since the previous epoch
    for (int i = 0; i < satOld.size(); i++) {

      // Set ambiguity of lost satellites to zero
//      if (!gnssAvailPhase.contains(satOld.get(i))) {
      if (!satAvailPhase.contains(satOld.get(i)) && satTypeAvailPhase.contains(satOld.get(i))) {

        if(goGPS.isDebug()) System.out.println("Lost satellite "+satOld.get(i));

        KFprediction.set(i3 + satOld.get(i), 0, 0);
      }
    }

    // Check if new satellites are available since the previous epoch
    for (int i = 0; i < pos.length; i++) {

      if (pos[i] != null && satAvailPhase.contains(pos[i].getSatID()) && satTypeAvailPhase.contains(pos[i].getSatType())
          && !satOld.contains(pos[i].getSatID()) && satTypeOld.contains(pos[i].getSatType())) {

        newSatellites.add(pos[i].getSatID());
        if(goGPS.isDebug()) System.out.println("New satellite "+pos[i].getSatID());
      }
    }

    // Cycle-slip detection
    boolean lossOfLockCycleSlipRover;
    boolean dopplerCycleSlipRover;
    boolean obsCodeCycleSlip;
    boolean cycleSlip;
    
    for (int i = 0; i < roverObs.getNumSat(); i++) {

      int satID = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(satID);

      if (gnssAvailPhase.contains(checkAvailGnss)) {

        // cycle slip detected by loss of lock indicator (disabled)
        lossOfLockCycleSlipRover = roverObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
        lossOfLockCycleSlipRover = false;

        // cycle slip detected by Doppler predicted phase range
        if (goGPS.getCycleSlipDetectionStrategy() == GoGPS.DOPPLER_PREDICTED_PHASE_RANGE) {
          dopplerCycleSlipRover = getRoverDopplerPredictedPhase(satID) != 0.0 && (Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - getRoverDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
        } else {
          dopplerCycleSlipRover = false;
        }

        // Rover-satellite observed pseudorange
        double roverSatCodeObs = roverObs.getSatByIDType(satID, satType).getCodeC(goGPS.getFreq());

        // Rover-satellite observed phase
        double roverSatPhaseObs = roverObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());

        // Store estimated ambiguity combinations and their covariance
        double estimatedAmbiguity = (roverSatCodeObs - roverSatPhaseObs - 2*roverSatIonoCorr[i]) / roverObs.getSatByIDType(satID, satType).getWavelength(goGPS.getFreq());

        obsCodeCycleSlip = Math.abs(KFprediction.get(i3+satID) - estimatedAmbiguity) > goGPS.getCycleSlipThreshold();

        cycleSlip = (lossOfLockCycleSlipRover || dopplerCycleSlipRover || obsCodeCycleSlip);

        if (!newSatellites.contains(satID) && cycleSlip) {

          slippedSatellites.add(satID);

          if (dopplerCycleSlipRover)
            if(goGPS.isDebug()) System.out.println("[ROVER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - getRoverDopplerPredictedPhase(satID))+")");
        }
      }
    }

    // Ambiguity estimation
    if (newSatellites.size() != 0 || slippedSatellites.size() != 0) {
      // List of satellites that need ambiguity estimation
      ArrayList<Integer> satAmb = newSatellites;
      satAmb.addAll(slippedSatellites);
      estimateAmbiguitiesStandalone(roverObs, satAmb, false);
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  private void setupKalmanFilterInputDoubleDiff(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    // Definition of matrices
    SimpleMatrix A;
    SimpleMatrix covXYZ;
    SimpleMatrix covENU;

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();

    // Double differences with respect to pivot satellite reduce observations by 1
    nObsAvail--;

    // Matrix containing parameters obtained from the linearization of the observation equations
    A = new SimpleMatrix(nObsAvail, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    covENU = new SimpleMatrix(3, 3);

    // Counter for available satellites
    int k = 0;

    // Counter for satellites with phase available
    int p = 0;

    // Pivot satellite ID
    int pivotId = roverObs.getSatID(pivot);
    char satType = roverObs.getGnssType(pivot);

    // Store rover-pivot and master-pivot observed pseudoranges
    double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Compute and store rover-pivot and master-pivot observed phase ranges
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = diffRoverSat[pivot];
    double roverPivotAppRange = roverSatAppRange[pivot];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = masterSatAppRange[pivot];

    // Rover-pivot and master-pivot troposphere correction
    double roverPivotTropoCorr = roverSatTropoCorr[pivot];
    double masterPivotTropoCorr = masterSatTropoCorr[pivot];;

    // Rover-pivot and master-pivot ionosphere correction
    double roverPivotIonoCorr = roverSatIonoCorr[pivot];
    double masterPivotIonoCorr = masterSatIonoCorr[pivot];

    // Compute rover-pivot and master-pivot weights
    double roverElevation = roverTopo[pivot].getElevation();
    double masterElevation = masterTopo[pivot].getElevation();
    double roverPivotWeight = computeWeight(roverElevation,
        roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
    double masterPivotWeight = computeWeight(masterElevation,
        masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));

    // Start filling in the observation error covariance matrix
    Cnn.zero();
    int nSatAvail = satAvail.size() - 1;
    int nSatAvailPhase = satAvailPhase.size() - 1;
    for (int i = 0; i < nSatAvail + nSatAvailPhase; i++) {
      for (int j = 0; j < nSatAvail + nSatAvailPhase; j++) {

        if (i < nSatAvail && j < nSatAvail)
          Cnn.set(i, j, goGPS.getStDevCode(roverObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
              * goGPS.getStDevCode(masterObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
              * (roverPivotWeight + masterPivotWeight));
        else if (i >= nSatAvail && j >= nSatAvail)
          Cnn.set(i, j, Math.pow(goGPS.getStDevPhase(), 2)
              * (roverPivotWeight + masterPivotWeight));
      }
    }

    // Satellite ID
    int id = 0;

    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (pos[i]!=null && gnssAvail.contains(checkAvailGnss)
          && i != pivot) {

        // Compute parameters obtained from linearization of observation equations
        double alphaX = diffRoverSat[i].get(0) / roverSatAppRange[i]
            - diffRoverPivot.get(0) / roverPivotAppRange;
        double alphaY = diffRoverSat[i].get(1) / roverSatAppRange[i]
            - diffRoverPivot.get(1) / roverPivotAppRange;
        double alphaZ = diffRoverSat[i].get(2) / roverSatAppRange[i]
            - diffRoverPivot.get(2) / roverPivotAppRange;

        // Fill in the A matrix
        A.set(k, 0, alphaX); /* X */
        A.set(k, 1, alphaY); /* Y */
        A.set(k, 2, alphaZ); /* Z */

        // Approximate code double difference
        double ddcApp = (roverSatAppRange[i] - masterSatAppRange[i])
            - (roverPivotAppRange - masterPivotAppRange);

        // Observed code double difference
        double ddcObs = (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
            - (roverPivotCodeObs - masterPivotCodeObs);

        // Observed phase double difference
        double ddpObs = (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
            - (roverPivotPhaseObs - masterPivotPhaseObs);

        // Compute troposphere and ionosphere residuals
        double tropoResiduals = (roverSatTropoCorr[i] - masterSatTropoCorr[i])
            - (roverPivotTropoCorr - masterPivotTropoCorr);
        double ionoResiduals = (roverSatIonoCorr[i] - masterSatIonoCorr[i])
            - (roverPivotIonoCorr - masterPivotIonoCorr);

        // Compute approximate ranges
        double appRangeCode;
        double appRangePhase;
        if (goGPS.getFreq() == 0) {
          appRangeCode = ddcApp + tropoResiduals + ionoResiduals;
          appRangePhase = ddcApp + tropoResiduals - ionoResiduals;
        } else {
          appRangeCode = ddcApp + tropoResiduals + ionoResiduals * Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(1)/roverObs.getSatByIDType(id, satType).getWavelength(0), 2);
          appRangePhase = ddcApp + tropoResiduals - ionoResiduals * Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(1)/roverObs.getSatByIDType(id, satType).getWavelength(0), 2);
        }

        // Fill in one row in the design matrix (for code)
        H.set(k, 0, alphaX);
        H.set(k, i1 + 1, alphaY);
        H.set(k, i2 + 1, alphaZ);

        // Fill in one element of the observation vector (for code)
        y0.set(k, 0, ddcObs - appRangeCode + alphaX * roverPos.getX()
            + alphaY * roverPos.getY() + alphaZ
            * roverPos.getZ());

        // Fill in the observation error covariance matrix (for code)
        double roverSatWeight = computeWeight(roverTopo[i].getElevation(), roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        double masterSatWeight = computeWeight(masterTopo[i].getElevation(), masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        double CnnBase = Cnn.get(k, k);
        Cnn.set(k, k, CnnBase + goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
            * goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq())
            * (roverSatWeight + masterSatWeight));

        if (gnssAvail.contains(checkAvailGnss)){
//        if (satAvailPhase.contains(id) && satTypeAvailPhase.contains(satType)) {

          // Fill in one row in the design matrix (for phase)
          H.set(nObsAvail + p, 0, alphaX);
          H.set(nObsAvail + p, i1 + 1, alphaY);
          H.set(nObsAvail + p, i2 + 1, alphaZ);
          H.set(nObsAvail + p, i3 + id, -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()));

          // Fill in one element of the observation vector (for phase)
          y0.set(nObsAvail + p, 0, ddpObs - appRangePhase + alphaX
              * roverPos.getX() + alphaY
              * roverPos.getY() + alphaZ
              * roverPos.getZ());

          // Fill in the observation error covariance matrix (for
          // phase)
          CnnBase = Cnn.get(nObsAvail + p, nObsAvail + p);
          Cnn.set(nObsAvail + p, nObsAvail + p, CnnBase
              + Math.pow(goGPS.getStDevPhase(), 2)
              * (roverSatWeight + masterSatWeight));

          // Increment satellites with phase counter
          p++;
        }

        // Increment available satellites counter
        k++;
      }
    }

    // Compute covariance matrix from A matrix [ECEF reference system]
    covXYZ = A.transpose().mult(A).invert();

    // Allocate and build rotation matrix
    SimpleMatrix R = new SimpleMatrix(3, 3);
    R = Coordinates.rotationMatrix(roverPos);

    // Propagate covariance from global system to local system
    covENU = R.mult(covXYZ).mult(R.transpose());

    //Compute DOP values
    roverPos.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    roverPos.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    roverPos.vDop = Math.sqrt(covENU.get(2, 2));
  }
  
  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  private void checkSatelliteConfigurationDoubleDiff(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    // Lists for keeping track of satellites that need ambiguity (re-)estimation
    ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
    ArrayList<Integer> slippedSatellites = new ArrayList<Integer>(0);

    // Check if satellites were lost since the previous epoch
    for (int i = 0; i < satOld.size(); i++) {

      // Set ambiguity of lost satellites to zero
//      if (!gnssAvailPhase.contains(satOld.get(i))) {
      if (!satAvailPhase.contains(satOld.get(i)) && satTypeAvailPhase.contains(satOld.get(i))) {

        if(goGPS.isDebug()) System.out.println("Lost satellite "+satOld.get(i));

        KFprediction.set(i3 + satOld.get(i), 0, 0);
      }
    }

    // Check if new satellites are available since the previous epoch
    int temporaryPivot = 0;
    boolean newPivot = false;
    for (int i = 0; i < pos.length; i++) {

      if (pos[i] != null && satAvailPhase.contains(pos[i].getSatID()) && satTypeAvailPhase.contains(pos[i].getSatType())
          && !satOld.contains(pos[i].getSatID()) && satTypeOld.contains(pos[i].getSatType())) {

        newSatellites.add(pos[i].getSatID());

        if (pos[i].getSatID() == pos[pivot].getSatID() && pos[i].getSatType() == pos[pivot].getSatType()) {
          newPivot = true;
          if(goGPS.isDebug()) System.out.println("New satellite "+pos[i].getSatID()+" (new pivot)");
        } else {
          if(goGPS.isDebug()) System.out.println("New satellite "+pos[i].getSatID());
        }
      }
    }

    // If a new satellite is going to be the pivot, its ambiguity needs to be estimated before switching pivot
    if (newPivot) {
      // If it is not the only satellite with phase
      if (satAvailPhase.size() > 1) {
        // If the former pivot is still among satellites with phase
        if (satAvailPhase.contains(oldPivotId) && satTypeAvailPhase.contains(oldPivotType)) {
          // Find the index of the old pivot
          for (int j = 0; j < pos.length; j ++) {
            if (pos[j] != null && pos[j].getSatID() == oldPivotId && pos[j].getSatType() == oldPivotType) {
              temporaryPivot = j;
            }
          }
        } else {
          double maxEl = 0;
          // Find a temporary pivot with phase
          for (int j = 0; j < pos.length; j ++) {
            if (pos[j] != null && satAvailPhase.contains(pos[j].getSatID()) && satTypeAvailPhase.contains(pos[j].getSatType())
                && j != pivot
                && roverTopo[j].getElevation() > maxEl) {
              temporaryPivot = j;
              maxEl = roverTopo[j].getElevation();
            }
          }
          // Reset the ambiguities of other satellites according to the temporary pivot
          newSatellites.clear();
          newSatellites.addAll(satAvailPhase);
          oldPivotId = pos[temporaryPivot].getSatID();
          oldPivotType = pos[temporaryPivot].getSatType();
          
        }
        // Estimate the ambiguity of the new pivot and other (new) satellites, using the temporary pivot
        estimateAmbiguitiesDoubleDiff(roverObs, masterObs, masterPos, newSatellites, temporaryPivot, false);
        newSatellites.clear();
      }
    }

    // Check if pivot satellite changed since the previous epoch
    if (oldPivotId != pos[pivot].getSatID() && oldPivotType == pos[pivot].getSatType()  && satAvailPhase.size() > 1) {

      if(goGPS.isDebug()) System.out.println("Pivot change from satellite "+oldPivotId+" to satellite "+pos[pivot].getSatID());

      // Matrix construction to manage the change of pivot satellite
      SimpleMatrix A = new SimpleMatrix(o3 + nN, o3 + nN);

      //TODO: need to check below
      int pivotIndex = i3 + pos[pivot].getSatID();
      int pivotOldIndex = i3 + oldPivotId;
      for (int i = 0; i < o3; i++) {
        for (int j = 0; j < o3; j++) {
          if (i == j)
            A.set(i, j, 1);
        }
      }
      for (int i = 0; i < satAvailPhase.size(); i++) {
        for (int j = 0; j < satAvailPhase.size(); j++) {
          int satIndex = i3 + satAvailPhase.get(i);
          if (i == j) {
            A.set(satIndex, satIndex, 1);
          }
          A.set(satIndex, pivotIndex, -1);
        }
      }
      A.set(pivotOldIndex, pivotOldIndex, 0);
      A.set(pivotIndex, pivotIndex, 0);

      // Update predicted state
      KFprediction = A.mult(KFprediction);

      // Re-computation of the Cee covariance matrix at the previous epoch
      Cee = A.mult(Cee).mult(A.transpose());
    }

    // Cycle-slip detection
    boolean lossOfLockCycleSlipRover;
    boolean lossOfLockCycleSlipMaster;
    boolean dopplerCycleSlipRover;
    boolean dopplerCycleSlipMaster;
    boolean approxRangeCycleSlip;
    boolean cycleSlip;
    //boolean slippedPivot = false;
    
    // Pivot satellite ID
    int pivotId = pos[pivot].getSatID();
    
    // Rover-pivot and master-pivot observed phase
    char satType = roverObs.getGnssType(0);
    double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
    
    // Rover-pivot and master-pivot approximate pseudoranges
    double roverPivotAppRange = roverSatAppRange[pivot];
    double masterPivotAppRange = masterSatAppRange[pivot];
    
    for (int i = 0; i < roverObs.getNumSat(); i++) {

      int satID = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(satID);

      if (gnssAvailPhase.contains(checkAvailGnss)) {

        // cycle slip detected by loss of lock indicator (disabled)
        lossOfLockCycleSlipRover = roverObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
        lossOfLockCycleSlipMaster = masterObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
        lossOfLockCycleSlipRover = false;
        lossOfLockCycleSlipMaster = false;

        // cycle slip detected by Doppler predicted phase range
        if (goGPS.getCycleSlipDetectionStrategy() == GoGPS.DOPPLER_PREDICTED_PHASE_RANGE) {
          dopplerCycleSlipRover = this.getRoverDopplerPredictedPhase(satID) != 0.0 && (Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - this.getRoverDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
          dopplerCycleSlipMaster = this.getMasterDopplerPredictedPhase(satID) != 0.0 && (Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
              - this.getMasterDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
        } else {
          dopplerCycleSlipRover = false;
          dopplerCycleSlipMaster = false;
        }

        // cycle slip detected by approximate pseudorange
        approxRangeCycleSlip = false;
        if (goGPS.getCycleSlipDetectionStrategy() == GoGPS.APPROX_PSEUDORANGE && satID != pivotId) {

          // Rover-satellite and master-satellite approximate pseudorange
          double roverSatCodeAppRange = roverSatAppRange[i];
          double masterSatCodeAppRange = masterSatAppRange[i];

          // Rover-satellite and master-satellite observed phase
          double roverSatPhaseObs = roverObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());
          double masterSatPhaseObs = masterObs.getSatByIDType(satID, satType).getPhaserange(goGPS.getFreq());

          // Estimated code pseudorange double differences
          double codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange) - (roverPivotAppRange - masterPivotAppRange);

          // Observed phase double differences
          double phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs) - (roverPivotPhaseObs - masterPivotPhaseObs);

          // Store estimated ambiguity combinations and their covariance
          double estimatedAmbiguityComb = (codeDoubleDiffApprox - phaseDoubleDiffObserv) / roverObs.getSatByIDType(satID, satType).getWavelength(goGPS.getFreq());

          approxRangeCycleSlip = (Math.abs(KFprediction.get(i3+satID) - estimatedAmbiguityComb)) > goGPS.getCycleSlipThreshold();

        } else {
          approxRangeCycleSlip = false;
        }

        cycleSlip = (lossOfLockCycleSlipRover || lossOfLockCycleSlipMaster || dopplerCycleSlipRover || dopplerCycleSlipMaster || approxRangeCycleSlip);

        if (satID != pos[pivot].getSatID() && !newSatellites.contains(satID) && cycleSlip) {

          slippedSatellites.add(satID);

          //        if (satID != pos[pivot].getSatID()) {
          if (dopplerCycleSlipRover)
            if(goGPS.isDebug()) System.out.println("[ROVER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - this.getRoverDopplerPredictedPhase(satID))+")");
          if (dopplerCycleSlipMaster)
            if(goGPS.isDebug()) System.out.println("[MASTER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
                - this.getMasterDopplerPredictedPhase(satID))+")");
          //        } else {
          //          boolean slippedPivot = true;
          //          if (dopplerCycleSlipRover)
          //            System.out.println("[ROVER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(roverObs.getGpsByID(satID).getPhase(goGPS.getFreq())
          //                - this.getRoverDopplerPredictedPhase(satID))+")");
          //          if (dopplerCycleSlipMaster)
          //            System.out.println("[MASTER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(masterObs.getGpsByID(satID).getPhase(goGPS.getFreq())
          //                - this.getMasterDopplerPredictedPhase(satID))+")");
          //        }
        }
      }
    }

//    // If the pivot satellites slipped, the ambiguities of all the other satellites must be re-estimated
//    if (slippedPivot) {
//      // If it is not the only satellite with phase
//      if (satAvailPhase.size() > 1) {
//        // Reset the ambiguities of other satellites
//        newSatellites.clear();
//        slippedSatellites.clear();
//        slippedSatellites.addAll(satAvailPhase);
//      }
//    }

    // Ambiguity estimation
    if (newSatellites.size() != 0 || slippedSatellites.size() != 0) {
      // List of satellites that need ambiguity estimation
      ArrayList<Integer> satAmb = newSatellites;
      satAmb.addAll(slippedSatellites);
      estimateAmbiguitiesDoubleDiff(roverObs, masterObs, masterPos, satAmb, pivot, false);
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   */
  void computeDopplerPredictedPhase(Observations roverObs, Observations masterObs) {

    this.roverDopplerPredPhase = new double[32];
    if (masterObs != null)
      this.masterDopplerPredPhase = new double[32];

    for (int i = 0; i < satAvailPhase.size(); i++) {

      int satID = satAvailPhase.get(i);
      char satType = satTypeAvailPhase.get(i);
      
      double roverPhase = roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
      float roverDoppler = roverObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
      if (!Double.isNaN(roverPhase) && !Float.isNaN(roverDoppler))
        this.setRoverDopplerPredictedPhase(satAvailPhase.get(i), roverPhase - roverDoppler);
      
      if (masterObs != null) {
        double masterPhase = masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
        float masterDoppler = masterObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
        if (!Double.isNaN(masterPhase) && !Float.isNaN(masterDoppler))
          this.setMasterDopplerPredictedPhase(satAvailPhase.get(i), masterPhase - masterDoppler);
      }
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   *
   */
  public void loop( Observations roverObs, Observations masterObs, Coordinates masterPos) {

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU;
    covENU = new SimpleMatrix(3, 3);

    // Set linearization point (approximate coordinates by KF prediction at previous step)
    roverPos.setXYZ(KFprediction.get(0), KFprediction.get(i1 + 1), KFprediction.get(i2 + 1));

    // Save previous list of available satellites with phase
    satOld = satAvailPhase;
    satTypeOld = satTypeAvailPhase;

    // Save the ID and index of the previous pivot satellite
    try {
      oldPivotId   = pos[pivot].getSatID();
      oldPivotType = pos[pivot].getSatType();
    } catch(ArrayIndexOutOfBoundsException e) {
      oldPivotId = 0;
    }

    // Select satellites for standalone
    selectSatellitesStandalone(roverObs);

    if( satAvail.size() >= 4)
      // Estimate receiver clock error by code stand-alone
      new LS_SA_code(goGPS).codeStandalone( roverObs, true, false);

    int obsReduction = 0;
    
    if (masterObs != null) {
      // Select satellites for double differences
      selectSatellitesDoubleDiff(roverObs, masterObs, masterPos);
      obsReduction = 1;
    }

    // Number of observations (code and phase)
    int nObs = satAvail.size();

    // Double differences with respect to pivot satellite reduce number of observations by 1
    nObs = nObs - obsReduction;

    if( satAvailPhase.size() != 0) {
      // Add number of satellites with phase (minus 1 for double diff)
      nObs = nObs + satAvailPhase.size() - obsReduction;
    }

    if( satAvail.size() >= goGPS.getMinNumSat()) {
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
        SimpleMatrix R = new SimpleMatrix(3, 3);
        R = Coordinates.rotationMatrix(roverPos);

        // Build 3x3 diagonal matrix with variances
        SimpleMatrix diagonal = new SimpleMatrix(3, 3);
        diagonal.zero();
        diagonal.set(0, 0, Math.pow(goGPS.getStDevE(), 2));
        diagonal.set(1, 1, Math.pow(goGPS.getStDevN(), 2));
        diagonal.set(2, 2, Math.pow(goGPS.getStDevU(), 2));

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
      
      if (masterObs != null) {
        // Fill in Kalman filter transformation matrix, observation vector and observation error covariance matrix
        setupKalmanFilterInputDoubleDiff(roverObs, masterObs, masterPos);
        // Check if satellite configuration changed since the previous epoch
        checkSatelliteConfigurationDoubleDiff(roverObs, masterObs, masterPos);
      } else {
        // Fill in Kalman filter transformation matrix, observation vector and observation error covariance matrix
        setupKalmanFilterInputStandalone(roverObs);
        // Check if satellite configuration changed since the previous epoch
        checkSatelliteConfigurationStandalone(roverObs);
      }

      // Identity matrix
      SimpleMatrix I = SimpleMatrix.identity(o3 + nN);

      // Kalman filter equations
      K = T.mult(Cee).mult(T.transpose()).plus(Cvv);
      G = K.mult(H.transpose()).mult(H.mult(K).mult(H.transpose()).plus(Cnn).invert());
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
    roverPos.setXYZ(KFstate.get(0), KFstate.get(i1 + 1), KFstate.get(i2 + 1));

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
    SimpleMatrix R = new SimpleMatrix(3, 3);
    R = Coordinates.rotationMatrix(roverPos);

    // Propagate covariance from global system to local system
    covENU = R.mult(positionCovariance).mult(R.transpose());

    // Kalman filter DOP computation
    roverPos.kpDop = Math.sqrt(positionCovariance.get(0, 0) + positionCovariance.get(1, 1) + positionCovariance.get(2, 2));
    roverPos.khDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    roverPos.kvDop = Math.sqrt(covENU.get(2, 2));

    // Compute positioning in geodetic coordinates
    roverPos.computeGeodetic();
  }
}
