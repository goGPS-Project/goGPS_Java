package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.Coordinates;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Observations;

public class LS_SA_code extends Core {

  public LS_SA_code( GoGPS goGPS ){
    super( goGPS );
  }
  
  public void codeStandalone( Observations roverObs, boolean estimateOnlyClock, boolean ignoreTopocentricParameters ) {

    // Number of GNSS observations without cutoff
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 4;
    
    // Add one unknown for each constellation in addition to the first (to estimate Inter-System Biases - ISBs)
    String sys = getAvailGnssSystems();
    if (sys.length()>0) {
      sys = sys.substring(1);
      nUnknowns = nUnknowns + sys.length();
    }

    // Define least squares matrices
    SimpleMatrix A;
    SimpleMatrix b;
    SimpleMatrix y0;
    SimpleMatrix Q;
    SimpleMatrix x;
    SimpleMatrix vEstim;
    SimpleMatrix tropoCorr;
    SimpleMatrix ionoCorr;

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    SimpleMatrix covXYZ;
    covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU;
    covENU = new SimpleMatrix(3, 3);

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();

    // Least squares design matrix
    A = new SimpleMatrix(nObsAvail, nUnknowns);

    // Vector for approximate pseudoranges
    b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix (initialized to identity)
    Q = SimpleMatrix.identity(nObsAvail);

    // Solution vector
    x = new SimpleMatrix(nUnknowns, 1);

    // Vector for observation error
    vEstim = new SimpleMatrix(nObsAvail, 1);

    // Vectors for troposphere and ionosphere corrections
    tropoCorr = new SimpleMatrix(nObsAvail, 1);

    ionoCorr = new SimpleMatrix(nObsAvail, 1);
    
    // Counter for available satellites
    int k = 0;

    // Satellite ID
    int id = 0;

    // Set up the least squares matrices
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);   
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
      
      if( pos[i]!=null && gnssAvail.contains(checkAvailGnss)) {
//      if (pos[i]!=null && satAvail.contains(id)  && satTypeAvail.contains(satType)) {
//        System.out.println("####" + checkAvailGnss  + "####");

        // Fill in one row in the design matrix
        A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i]); /* X */
        A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i]); /* Y */
        A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i]); /* Z */
        A.set(k, 3, 1); /* clock error */
        for (int c = 0; c < sys.length(); c++) {
          A.set(k, 4+c, sys.indexOf(satType)==c?1:0); /* inter-system bias */
        }

        // Add the approximate pseudorange value to b
        b.set(k, 0, roverSatAppRange[i] - pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT);

        // Add the clock-corrected observed pseudorange value to y0
        y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));

        if (!ignoreTopocentricParameters) {
          // Fill in troposphere and ionosphere double differenced
          // corrections
          tropoCorr.set(k, 0, roverSatTropoCorr[i]);
          ionoCorr.set(k, 0, roverSatIonoCorr[i]);

          // Fill in the cofactor matrix
          double weight = Q.get(k, k)
              + computeWeight(roverTopo[i].getElevation(),
                  roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
          Q.set(k, k, weight);
        }

        // Increment available satellites counter
        k++;
      }
      
    }

    if (!ignoreTopocentricParameters) {
      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);
    }

    // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
    x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose())
        .mult(Q.invert()).mult(y0.minus(b));

    // Receiver clock error
    roverPos.receiverClockError = x.get(3) / Constants.SPEED_OF_LIGHT;

    if(estimateOnlyClock)
      return;

    // Receiver position
    //roverPos.coord.ecef.set(roverPos.coord.ecef.plus(x.extractMatrix(0, 3, 0, 1)));
    roverPos.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));

    // Estimation of the variance of the observation error
    vEstim = y0.minus(A.mult(x).plus(b));
    double varianceEstim = (vEstim.transpose().mult(Q.invert())
        .mult(vEstim)).get(0)
        / (nObsAvail - nUnknowns);

    // Covariance matrix of the estimation error
    if (nObsAvail > nUnknowns) {
      SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
      .scale(varianceEstim);
      positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
    }else{
      positionCovariance = null;
    }

    // Compute covariance matrix from A matrix [ECEF reference system]
    covXYZ = A.transpose().mult(A).invert();
    covXYZ = covXYZ.extractMatrix(0, 3, 0, 3);

    // Allocate and build rotation matrix
    SimpleMatrix R = Coordinates.rotationMatrix(roverPos);

    // Propagate covariance from global system to local system
    covENU = R.mult(covXYZ).mult(R.transpose());

    //Compute DOP values
    roverPos.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    roverPos.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    roverPos.vDop = Math.sqrt(covENU.get(2, 2));

    // Compute positioning in geodetic coordinates
    roverPos.computeGeodetic();
  }

  
}
