package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Coordinates;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Observations;

public class LS_DD_code extends Core {

  public LS_DD_code(GoGPS goGPS) {
    super(goGPS);
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  public void codeDoubleDifferences( Observations roverObs,Observations masterObs, Coordinates masterPos) {

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 3;

    // Define least squares matrices
    SimpleMatrix A;
    SimpleMatrix Adop;
    SimpleMatrix b;
    SimpleMatrix y0;
    SimpleMatrix Q;
    SimpleMatrix x;
    SimpleMatrix vEstim;
    SimpleMatrix tropoCorr;
    SimpleMatrix ionoCorr;

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    SimpleMatrix covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU;
    covENU = new SimpleMatrix(3, 3);

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();

    // Full design matrix for DOP computation
    Adop = new SimpleMatrix(nObsAvail, 3);

    // Double differences with respect to pivot satellite reduce
    // observations by 1
    nObsAvail--;

    // Least squares design matrix
    A = new SimpleMatrix(nObsAvail, nUnknowns);

    // Vector for approximate pseudoranges
    b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix
    Q = new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    x = new SimpleMatrix(nUnknowns, 1);

    // Vector for observation error
    vEstim = new SimpleMatrix(nObsAvail, 1);

    // Vectors for troposphere and ionosphere corrections
    tropoCorr = new SimpleMatrix(nObsAvail, 1);
    ionoCorr = new SimpleMatrix(nObsAvail, 1);

    // Counter for available satellites (without pivot)
    int k = 0;

    // Counter for available satellites (with pivot)
    int d = 0;

    // Pivot satellite index
    int pivotId = roverObs.getSatID(pivot);
    char satType = roverObs.getGnssType(pivot);      
    
    // Store rover-pivot and master-pivot observed pseudoranges
    double roverPivotObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[pivot];
    double roverPivotAppRange   = rover.satAppRange[pivot];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[pivot];

    // Computation of rover-pivot troposphere correction
    double roverPivotTropoCorr = rover.satTropoCorr[pivot];

    // Computation of master-pivot troposphere correction
    double masterPivotTropoCorr = master.satTropoCorr[pivot];;

    // Computation of rover-pivot ionosphere correction
    double roverPivotIonoCorr = rover.satIonoCorr[pivot];

    // Computation of master-pivot ionosphere correction
    double masterPivotIonoCorr = master.satIonoCorr[pivot];

    // Compute rover-pivot and master-pivot weights
    double roverPivotWeight = computeWeight(rover.topo[pivot].getElevation(),
        roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
    double masterPivotWeight = computeWeight(master.topo[pivot].getElevation(),
        masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
    Q.set(roverPivotWeight + masterPivotWeight);

    // Satellite ID
    int id = 0;

    // Set up the least squares matrices
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (pos[i] !=null && gnssAvail.contains(checkAvailGnss) && i != pivot) {
//      if (pos[i] !=null && satAvail.contains(id) && satTypeAvail.contains(satType) && i != pivot) {

        // Fill in one row in the design matrix
        A.set(k, 0, rover.diffSat[i].get(0) / rover.satAppRange[i]
            - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

        A.set(k, 1, rover.diffSat[i].get(1) / rover.satAppRange[i]
            - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

        A.set(k, 2, rover.diffSat[i].get(2) / rover.satAppRange[i]
            - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

        // Add the differenced approximate pseudorange value to b
        b.set(k, 0, (rover.satAppRange[i] - master.satAppRange[i])
            - (roverPivotAppRange - masterPivotAppRange));

        // Add the differenced observed pseudorange value to y0
        y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
            - (roverPivotObs - masterPivotObs));

        // Fill in troposphere and ionosphere double differenced
        // corrections
        tropoCorr.set(k, 0, (rover.satTropoCorr[i] - master.satTropoCorr[i])
            - (roverPivotTropoCorr - masterPivotTropoCorr));
        ionoCorr.set(k, 0, (rover.satIonoCorr[i] - master.satIonoCorr[i])
            - (roverPivotIonoCorr - masterPivotIonoCorr));

        // Fill in the cofactor matrix
        double roverSatWeight = computeWeight(rover.topo[i].getElevation(),
            roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        double masterSatWeight = computeWeight(master.topo[i].getElevation(),
            masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
        Q.set(k, k, Q.get(k, k) + roverSatWeight + masterSatWeight);

        // Increment available satellites counter
        k++;
      }

      // Design matrix for DOP computation
      if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)) {
//      if (pos[i] != null && satAvail.contains(id) && satTypeAvail.contains(satType)) {
        // Fill in one row in the design matrix (complete one, for DOP)
        Adop.set(d, 0, rover.diffSat[i].get(0) / rover.satAppRange[i]); /* X */
        Adop.set(d, 1, rover.diffSat[i].get(1) / rover.satAppRange[i]); /* Y */
        Adop.set(d, 2, rover.diffSat[i].get(2) / rover.satAppRange[i]); /* Z */
        d++;
      }
    }

    // Apply troposphere and ionosphere correction
    b = b.plus(tropoCorr);
    b = b.plus(ionoCorr);

    // Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
    x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose())
        .mult(Q.invert()).mult(y0.minus(b));

    // Receiver position
    //this.coord.ecef.set(this.coord.ecef.plus(x));
    rover.setPlusXYZ(x);

    // Estimation of the variance of the observation error
    vEstim = y0.minus(A.mult(x).plus(b));
    double varianceEstim = (vEstim.transpose().mult(Q.invert())
        .mult(vEstim)).get(0)
        / (nObsAvail - nUnknowns);

    // Covariance matrix of the estimation error
    if (nObsAvail > nUnknowns){
      SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
          .scale(varianceEstim);
      positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
    }else{
      positionCovariance = null;
    }

    // Compute covariance matrix from A matrix [ECEF reference system]
    covXYZ = Adop.transpose().mult(Adop).invert();

    // Allocate and build rotation matrix
    SimpleMatrix R = new SimpleMatrix(3, 3);
    R = Coordinates.rotationMatrix(rover);

    // Propagate covariance from global system to local system
    covENU = R.mult(covXYZ).mult(R.transpose());

    //Compute DOP values
    rover.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    rover.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    rover.vDop = Math.sqrt(covENU.get(2, 2));

    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();
  }
}
