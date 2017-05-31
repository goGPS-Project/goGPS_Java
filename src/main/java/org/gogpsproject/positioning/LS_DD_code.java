package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.GoGPS;
import org.gogpsproject.producer.Observations;

public class LS_DD_code extends LS_SA_code {

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

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();

    // Full design matrix for DOP computation
    SimpleMatrix Adop = new SimpleMatrix(nObsAvail, 3);

    // Double differences with respect to pivot satellite reduce
    // observations by 1
    nObsAvail--;

    // Least squares design matrix
    SimpleMatrix A = new SimpleMatrix(nObsAvail, nUnknowns);

    // Vector for approximate pseudoranges
    SimpleMatrix b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    SimpleMatrix y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix
    SimpleMatrix Q = new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    SimpleMatrix x = new SimpleMatrix(nUnknowns, 1);

    // Vector for observation error
    SimpleMatrix vEstim = new SimpleMatrix(nObsAvail, 1);

    // Vectors for troposphere and ionosphere corrections
    SimpleMatrix tropoCorr = new SimpleMatrix(nObsAvail, 1);
    SimpleMatrix ionoCorr = new SimpleMatrix(nObsAvail, 1);

    // Counter for available satellites (without pivot)
    int k = 0;

    // Counter for available satellites (with pivot)
    int d = 0;

    // Pivot satellite index
    int pivotId = roverObs.getSatID(sats.pivot);
    char satType = roverObs.getGnssType(sats.pivot);      
    
    // Store rover-pivot and master-pivot observed pseudoranges
    double roverPivotObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
    double masterPivotObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

    // Rover-pivot approximate pseudoranges
    SimpleMatrix diffRoverPivot = rover.diffSat[sats.pivot];
    double roverPivotAppRange   = rover.satAppRange[sats.pivot];

    // Master-pivot approximate pseudoranges
    double masterPivotAppRange = master.satAppRange[sats.pivot];

    // Computation of rover-pivot troposphere correction
    double roverPivotTropoCorr = rover.satTropoCorr[sats.pivot];

    // Computation of master-pivot troposphere correction
    double masterPivotTropoCorr = master.satTropoCorr[sats.pivot];;

    // Computation of rover-pivot ionosphere correction
    double roverPivotIonoCorr = rover.satIonoCorr[sats.pivot];

    // Computation of master-pivot ionosphere correction
    double masterPivotIonoCorr = master.satIonoCorr[sats.pivot];

    // Compute rover-pivot and master-pivot weights
    double roverPivotWeight = computeWeight(rover.topo[sats.pivot].getElevation(),
        roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
    double masterPivotWeight = computeWeight(master.topo[sats.pivot].getElevation(),
        masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
    Q.set(roverPivotWeight + masterPivotWeight);

    // Satellite ID
    int id = 0;

    // Set up the least squares matrices
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      satType = roverObs.getGnssType(i);
      String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

      if (sats.pos[i] !=null && sats.gnssAvail.contains(checkAvailGnss) && i != sats.pivot) {
//      if (sats.pos[i] !=null && sats.avail.contains(id) && satTypeAvail.contains(satType) && i != pivot) {

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
      if (sats.pos[i] !=null && sats.gnssAvail.contains(checkAvailGnss)) {
//      if (sats.pos[i] != null && sats.avail.contains(id) && satTypeAvail.contains(satType)) {
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

    updateDops(A);

    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();
  }
}
