package org.gogpsproject.positioning;

import java.util.Arrays;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Status;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public class LS_SA_code_coarse_time extends LS_SA_code_snapshot {

  /** Float code ambiguities for modular case, should be between 0 and 1 */
  double[] codeAmbiguities;

  final int MINSV = 4;

  public LS_SA_code_coarse_time(GoGPS goGPS) {
    super(goGPS);
  }

  /**
   * @param roverObs
   * @param MODULO module in meters (
   * @return
   */
  public double codeStandaloneCoarseTime( Observations roverObs, final double MODULO ) {
    long unixTime = roverObs.getRefTime().getMsec();

    rover.status = Status.None;

    // Number of GNSS observations without cutoff
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 5;

    // Define least squares matrices
    SimpleMatrix A = null;  // aka H or G, // Least squares design matrix
    SimpleMatrix b;  // Vector for approximate (estimated, predicted) pseudoranges
    SimpleMatrix y0; // Observed pseudoranges
    SimpleMatrix Q = null;  // Cofactor (Weighted) Matrix
    SimpleMatrix x;  // Solution (Update) Vector
    SimpleMatrix vEstim; // Observation Errors
    SimpleMatrix tropoCorr; 
    SimpleMatrix ionoCorr;

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    SimpleMatrix covXYZ;
    covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU;
    covENU = new SimpleMatrix(3, 3);

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();
    
    nObsAvail++; // add DTM / height soft constraint

    // Least squares design matrix
    A = new SimpleMatrix( nObsAvail, nUnknowns );

    // Vector for approximate pseudoranges
    b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix
    Q = new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    x = new SimpleMatrix(nUnknowns, 1);

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

      if( sats.pos[i] == null  || !sats.avail.contains(id) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
//              l.warning( "ERROR, sats.pos[i]==null?" );
//              this.setXYZ(0, 0, 0);
//              return null;
        int satId = roverObs.getSatID(k);
        ObservationSet os = roverObs.getSatByID(satId);
        os.inUse(false);

        continue;
      }

      // Line Of Sight vector units (ECEF)
      SimpleMatrix e = new SimpleMatrix(1,3);

      // Line Of Sight vector units (ECEF)
      e.set( 0,0, rover.diffSat[i].get(0) / rover.satAppRange[i] );
      e.set( 0,1, rover.diffSat[i].get(1) / rover.satAppRange[i] );
      e.set( 0,2, rover.diffSat[i].get(2) / rover.satAppRange[i] );

      // scalar product of speed vector X unit vector
      float doppler = roverObs.getSatByID(id).getDoppler(ObservationSet.L1);
      double rodot;
      if( Float.isNaN( doppler ))
        rodot = -e.mult( sats.pos[i].getSpeed() ).get(0);
      else
        // scalar product of speed vector X unit vector
        rodot = -doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;

      // Fill in one row in the design matrix
      A.set(k, 0, e.get(0) ); /* X */
      A.set(k, 1, e.get(1) ); /* Y */
      A.set(k, 2, e.get(2) ); /* Z */

      A.set(k, 3, 1); /* clock error */
      A.set(k, 4, rodot );

      // Add the approximate pseudorange value to b
//      b.set(k, 0, (rover.satAppRange[i] - sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) % MODULO );
      b.set(k, 0, (rover.satAppRange[i] - sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) );

      ObservationSet os = roverObs.getSatByID(id);
      
      // Add the clock-corrected observed pseudorange value to y0
//      y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));
//      y0.set(k, 0, os.getCodeC(0) % MODULO );
      y0.set(k, 0, os.getCodeC(0) );

//      if (!ignoreTopocentricParameters) {
      // cap tropo correction
      if( Double.isNaN( rover.satTropoCorr[i] ))
        rover.satTropoCorr[i] = 0;
      
      if(  rover.satTropoCorr[i]>30 )
        rover.satTropoCorr[i] = 30;
      if(  rover.satTropoCorr[i]<-30 )
        rover.satTropoCorr[i] = -30;
      
      tropoCorr.set(k, 0, rover.satTropoCorr[i]);
      ionoCorr.set(k, 0, rover.satIonoCorr[i]);
   
      // Fill in the cofactor matrix
      double weight;
      
      if( rover.topo[i].getElevation()<15 )
        weight = 1;
      else
        weight = Q.get(k, k)
          + computeWeight(rover.topo[i].getElevation(),
              roverObs.getSatByIDType(id, 'G').getSignalStrength(goGPS.getFreq()));
      if( weight>5 )
        weight = 5;
      
//      Q.set(k, k, weight);
      Q.set(k, k, 1);
//          if( weight > maxWeight )
//            maxWeight = weight;

      // Increment available satellites counter
      k++;
          
     } // i loop

    {
      // Add height soft constraint
      double lam = Math.toRadians(rover.getGeodeticLongitude());
      double phi = Math.toRadians(rover.getGeodeticLatitude());
      double hR_app = rover.getGeodeticHeight();
      //double hR_app = 0;
      //double h_dtm = recpos.getGeodeticHeight();
      
//    %extraction from the dtm of the height correspondent to the approximated position
//    [h_dtm] = grid_bilin_interp(E_app, N_app, tile_buffer, tile_header.ncols*3, tile_header.nrows*3, tile_header.cellsize, Ell, Nll, tile_header.nodata);
      double h_dtm = hR_app>0? hR_app : 30; // initialize to something above sea level
      if( h_dtm > goGPS.getMaxHeight() )
        h_dtm = goGPS.getMaxHeight();
      
//      if( goGPS.useDTM() ){
//        try {
//          ElevationResult eres = ElevationApi.getByPoint( getContext(), new LatLng(this.getGeodeticLatitude(), this.getGeodeticLongitude())).await();
//          if( eres.elevation > 0 )
//            h_dtm = eres.elevation;
//        } catch (Exception e1) {
//          // TODO Auto-generated catch block
//  //        e.printStackTrace();
//        }
//      }

      double cosLam = Math.cos(lam);
      double cosPhi = Math.cos(phi);
      double sinLam = Math.sin(lam);
      double sinPhi = Math.sin(phi);
      double[][] data = new double[1][3];
      data[0][0] = cosPhi * cosLam;
      data[0][1] = cosPhi * sinLam;
      data[0][2] = sinPhi;
      
      k = nObsAvail-1;
      A.set(k, 0, data[0][0] );
      A.set(k, 1, data[0][1] ); 
      A.set(k, 2, data[0][2] ); 
      A.set(k, 3, 0 ); 
      A.set(k, 4, 0 );

//    %Y0 vector computation for DTM constraint
//    y0_dtm = h_dtm  - hR_app + cos(phiR_app)*cos(lamR_app)*X_app + cos(phiR_app)*sin(lamR_app)*Y_app + sin(phiR_app)*Z_app;
      double y0_dtm = h_dtm  - hR_app;
      y0.set(k, 0, y0_dtm );
//      double max = Q.elementMaxAbs();
//      Q.set(k, k, max);
      Q.set(k, k, 1);
    }
    
//  if (!ignoreTopocentricParameters) {
    b = b.plus(tropoCorr);
    b = b.plus(ionoCorr);

    rover.satsInUse = 0;
    SimpleMatrix resid = y0.minus(b);

    
// use smallest resid
  double pivot = 0;
  double pivot_test;
  int pivot_map = 0;
  int goodSat;
  int badSat;
  int maxGoodSat = 0;
  
  // Find pivot satellite whose group has more 'valid' one at the end. The loop test is done at most half of 'sats.avail.size' in idea case
  do {
    pivot_test = Double.MAX_VALUE;
    int pivot_pos = 0;
    for( k=0; k<sats.avail.size(); k++){
      if ((pivot_map & (1 << k)) == 0) {
        double d = resid.get(k);
        if( Math.abs(d)<Math.abs(pivot_test)) {
          pivot_test = d;
          pivot_pos = k;
        }
      }
    }
    
    pivot_map |= (1 << pivot_pos);
      
// use highest sat    
//    double pivot = 0;
//    double pivotEl = 0;
//    for( k=0; k<sats.avail.size(); k++){
//      int satId = roverObs.getSatID(k);
//      ObservationSet os = roverObs.getSatByID(satId);
//      if( rover.topo[k].getElevation() > pivotEl ){
//        pivotEl = rover.topo[k].getElevation();
//        pivot = resid.get(k);
//      }
//    }
    
    goodSat = 0;
    badSat = 0;   
    k = 0;
    for (int i = 0; i < nObs; i++) {
      int satId = roverObs.getSatID(i);

      if( sats.pos[i] == null  || !sats.avail.contains(satId) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
        continue;
      }

      double d = resid.get(k);

      if( d-pivot_test>MODULO/2 ){
        d-=MODULO;
      }
      if( d-pivot_test<-MODULO/2){
        d+=MODULO;
      }

      // check again, if fails, exclude this satellite
      double dms = Math.abs(d-pivot_test)/Constants.SPEED_OF_LIGHT*1000;
      if( Math.abs(dms) > goGPS.getResidThreshold() ){      
        badSat++;
      }
      else {      
        goodSat++;
      } 
      
      k++;
    }
    
    if (maxGoodSat < goodSat) {
      maxGoodSat = goodSat;
      pivot = pivot_test;
    }
    
  } while ((badSat > goodSat) && ((1 << sats.avail.size()) > (pivot_map + 1)));
    
    System.out.println( String.format( "* Residuals -> Adjusted Residuals (ms) - Pivot = %7.4f (ms)",  pivot/Constants.SPEED_OF_LIGHT*1000));
    
    // Officially check again
    k = 0;
    for (int i = 0; i < nObs; i++) {
      int satId = roverObs.getSatID(i);

      if( sats.pos[i] == null  || !sats.avail.contains(satId) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
        continue;
      }

      ObservationSet os = roverObs.getSatByID(satId);
      
      double d = resid.get(k);
      System.out.print( String.format( "%2d) C:%8.3f (%8.5f); %9.5f -> ", 
          satId, 
          roverObs.getSatByID(satId).getCodeC(0), 
          roverObs.getSatByID(satId).getCodeC(0)/Constants.SPEED_OF_LIGHT*1000, 
          d/Constants.SPEED_OF_LIGHT*1000));
      
      if( d-pivot>MODULO/2 ){
        d-=MODULO;
      }
      if( d-pivot<-MODULO/2){
        d+=MODULO;
      }
      System.out.print( String.format( "%9.5f", d/Constants.SPEED_OF_LIGHT*1000));
      System.out.print( String.format( "  Q:%3.1f", Q.get(k,k)));
      
      // check again, if fails, exclude this satellite
      double dms = Math.abs(d-pivot)/Constants.SPEED_OF_LIGHT*1000;
      if( Math.abs(dms) > goGPS.getResidThreshold() )
      {
        if( goGPS.isDebug() ) System.out.println( String.format( " Excluding d:%8.3f", dms));
        resid.set(k, 0);
        A.set(k, 0, 0);
        A.set(k, 1, 0);
        A.set(k, 2, 0);
        A.set(k, 3, 0);
        A.set(k, 4, 0);
        os.inUse(false);
      }
      else 
      {
        resid.set(k,d);
        rover.satsInUse++;
        os.inUse(true);
        os.el = rover.topo[i].getElevation();
        System.out.println();
      }
      k++;
    }
    
    if( rover.satsInUse < nUnknowns-1 ){
      System.out.println("Not enough satellites for " + roverObs.getRefTime() );
      rover.setXYZ(0, 0, 0);
      if( rover.status == Status.None ){
        rover.status = Status.NotEnoughSats;
      }
      return 0;
    }
      
//       Weighted Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
      x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(resid);

     double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                        Math.pow( x.get(1), 2 ) +
                                        Math.pow( x.get(2), 2 ) );

     // Common bias in meters and ms
     double cbias   = x.get(3);
     double cbiasms = cbias * 1000d / Constants.SPEED_OF_LIGHT;
     // x.get(4) = time update in seconds
     double tg = x.get(4); 

     // compute eRes 
     rover.eRes = 0;
     for( k=0; k<sats.avail.size(); k++){
       int satId = roverObs.getSatID(k);
       ObservationSet os = roverObs.getSatByID(satId);
       if( !os.isInUse() )
         continue;
       
       double d = resid.get(k);
       os.eRes = Math.abs(d-cbias);
       rover.eRes += Math.pow( os.eRes, 2); 
     }
     rover.eRes = Math.sqrt(rover.eRes/rover.satsInUse);
     System.out.println(String.format("eRes = %5.3f\r\n", rover.eRes));
     
     // expected
     System.out.println( String.format( "pos update:  %5.1f, %5.1f, %5.1f; Mag: %5d(m)", x.get(0), x.get(1), x.get(2), (long)correction_mag ));
     System.out.println( String.format( "common bias: %2.4f (ms)", cbiasms ));
     System.out.println( String.format( "time update: %3.3f (s)", tg ));

     // Receiver clock error
     rover.clockError = x.get(3) / Constants.SPEED_OF_LIGHT;

     // apply correction to Rx position estimate
     rover.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));
     rover.computeGeodetic();

     // update refTime
     if( correction_mag < 10 ){
       unixTime += tg * 1000;
       Time newTime = new Time( unixTime);
       roverObs.setRefTime( newTime );
       rover.setRefTime( newTime );
     }
     
     System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
         rover.getGeodeticLatitude(), 
         rover.getGeodeticLongitude(), 
         rover.getGeodeticHeight(), 
         new Time(unixTime).toString() ));
     
     // Estimation of the variance of the observation error
     vEstim = y0.minus(A.mult(x).plus(b));
     double varianceEstim = (vEstim.transpose().mult(Q.invert())
         .mult(vEstim)).get(0)
         / (nObsAvail - nUnknowns);

     // Covariance matrix of the estimation error
     if (nObsAvail > nUnknowns) {
       SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
       .scale(varianceEstim);
       this.positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
     }else{
       this.positionCovariance = null;
     }
     
     // Compute covariance matrix from A matrix [ECEF reference system]
     covXYZ = A.transpose().mult(A).invert().extractMatrix(0, 3, 0, 3);

     // Allocate and build rotation matrix
     SimpleMatrix R = new SimpleMatrix(3, 3);
     R = Coordinates.rotationMatrix(rover);

     // Propagate covariance from global system to local system
     covENU = R.mult(covXYZ).mult(R.transpose());

     //Compute DOP values
     rover.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
     rover.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
     rover.vDop = Math.sqrt(covENU.get(2, 2));
     
     return correction_mag; // return correction_mag
  }
  
  /**
   * Coarse time processing with additional Nk variables for code slips
   * @param roverObs
   * @param MODULO module in meters 
   * @return
   */
  public double codeStandaloneCoarseTimeCodeAmbs( Observations roverObs, final double MODULO ) {
    long unixTime = roverObs.getRefTime().getMsec();

    rover.status = Status.None;

    // Number of GNSS observations without cutoff
    int nObs = roverObs.getNumSat();

    // Define least squares matrices
    SimpleMatrix A = null;  // aka H or G, // Least squares design matrix
    SimpleMatrix b;  // Vector for approximate (estimated, predicted) pseudoranges
    SimpleMatrix y0; // Observed pseudoranges
    SimpleMatrix Q = null;  // Cofactor (Weighted) Matrix
    SimpleMatrix x;  // Solution (Update) Vector
    SimpleMatrix vEstim; // Observation Errors
    SimpleMatrix tropoCorr; 
    SimpleMatrix ionoCorr;

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    SimpleMatrix covXYZ;
    covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU;
    covENU = new SimpleMatrix(3, 3);

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();
    
    nObsAvail++; // add DTM / height soft constraint

    // Number of unknown parameters
    int nUnknowns = sats.avail.size();

    // Least squares design matrix
    A = new SimpleMatrix( nObsAvail, nUnknowns );

    // Vector for approximate pseudoranges
    b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix
    Q = new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    x = new SimpleMatrix(nUnknowns, 1);

    // Vectors for troposphere and ionosphere corrections
    tropoCorr = new SimpleMatrix(nObsAvail, 1);
    ionoCorr = new SimpleMatrix(nObsAvail, 1);

    // Counter for available satellites
    int k = 0;

    // Satellite ID
    int id = 0;

    // assign code ambiguities
    if( codeAmbiguities == null ){
      codeAmbiguities =  new double[nObs];
      Arrays.fill( codeAmbiguities, 0 );
    }
    
    // Set up the least squares matrices
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);

      if( sats.pos[i] == null  || !sats.avail.contains(id) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
//              l.warning( "ERROR, sats.pos[i]==null?" );
//              this.setXYZ(0, 0, 0);
//              return null;
        int satId = roverObs.getSatID(k);
        ObservationSet os = roverObs.getSatByID(satId);
        os.inUse(false);

        continue;
      }

      A.set(k, k, -MODULO );

      b.set(k, 0, (rover.satAppRange[i] - sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) );

      ObservationSet os = roverObs.getSatByID(id);
      
      // Add the observed pseudorange value to y0
      y0.set(k, 0, os.getCodeC(0) );

      // cap tropo correction
      if( Double.isNaN( rover.satTropoCorr[i] ))
        rover.satTropoCorr[i] = 0;
      
      if(  rover.satTropoCorr[i]>30 )
        rover.satTropoCorr[i] = 30;
      if(  rover.satTropoCorr[i]<-30 )
        rover.satTropoCorr[i] = -30;
      
      tropoCorr.set(k, 0, rover.satTropoCorr[i]);
      ionoCorr.set(k, 0, rover.satIonoCorr[i]);
   
      Q.set(k, k, 1);

      k++;
     } 

    {
      // Add height soft constraint
      double lam = Math.toRadians(rover.getGeodeticLongitude());
      double phi = Math.toRadians(rover.getGeodeticLatitude());
      double hR_app = rover.getGeodeticHeight();
      double h_dtm = hR_app>0? hR_app : 30; // initialize to something above sea level
      if( h_dtm > 3000 )
        h_dtm = 3000;
      
      double cosLam = Math.cos(lam);
      double cosPhi = Math.cos(phi);
      double sinLam = Math.sin(lam);
      double sinPhi = Math.sin(phi);
      
      k = nObsAvail-1;
      A.set(k, 0, cosPhi * cosLam );
      A.set(k, 1, cosPhi * sinLam ); 
      A.set(k, 2, sinPhi ); 

      y0.set(k, 0, h_dtm  - hR_app );
      Q.set(k, k, 1);
    }
    
    b = b.plus(tropoCorr);
    b = b.plus(ionoCorr);

    rover.satsInUse = 0;
    SimpleMatrix resid = y0.minus(b);
    
    double pivot = MODULO;
    for( k=0; k<sats.avail.size(); k++){
      int satId = roverObs.getSatID(k);
      
      if( !sats.avail.contains(satId) || sats.pos[k] == null || rover.topo[k] == null )
        continue;
      
      if( Math.abs(resid.get(k)) < pivot ){
        pivot = Math.abs(resid.get(k));
      }
    }

    System.out.println( String.format( "* Residuals -> Adjusted Residuals (ms) - Pivot = %7.4f (ms)",  pivot/Constants.SPEED_OF_LIGHT*1000));
    
    for( k=0; k<sats.avail.size(); k++){
      int satId = roverObs.getSatID(k);
      ObservationSet os = roverObs.getSatByID(satId);
      
      double d = resid.get(k);
      System.out.print( String.format( "%2d) C:%8.3f (%8.5f); %9.5f -> ", 
          satId, 
          roverObs.getSatByID(satId).getCodeC(0), 
          roverObs.getSatByID(satId).getCodeC(0)/Constants.SPEED_OF_LIGHT*1000, 
          d/Constants.SPEED_OF_LIGHT*1000));
      
      d += codeAmbiguities[k]*MODULO;
      
      System.out.print( String.format( "%9.5f", d/Constants.SPEED_OF_LIGHT*1000));
      System.out.print( String.format( "  Q:%3.1f; N:%6.2f", Q.get(k, k), codeAmbiguities[k]));
      
      resid.set(k,d);
    
      rover.satsInUse++;
      os.inUse(true);
      System.out.println();
      rover.eRes += Math.pow( d-pivot, 2); 
    }
    rover.eRes = Math.sqrt(rover.eRes/rover.satsInUse);
    System.out.println(String.format("eRes = %5.3f\r\n", rover.eRes));
    
    if( rover.satsInUse < nUnknowns-1 ){
      System.out.println("Not enough satellites for " + roverObs.getRefTime() );
      rover.setXYZ(0, 0, 0);
      if( rover.status == Status.None ){
        rover.status = Status.NotEnoughSats;
      }
      return 0;
    }
      
//       Weighted Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
      x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(resid);

     double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                        Math.pow( x.get(1), 2 ) +
                                        Math.pow( x.get(2), 2 ) );

     // x.get(3) = Receiver clock error in meters
     double cbiasms = x.get(3) * 1000d / Constants.SPEED_OF_LIGHT;
     double tg = x.get(4); // time update in seconds

     // expected
     System.out.println( String.format( "pos update:  %5.1f, %5.1f, %5.1f; Mag: %5d(m)", x.get(0), x.get(1), x.get(2), (long)correction_mag ));
     System.out.println( String.format( "common bias: %2.4f (ms)", cbiasms ));
     System.out.println( String.format( "time update: %3.3f (s)", tg ));
     
     System.out.println( "ambiguities: " );
     for( k = 0; k<codeAmbiguities.length; k++){
       System.out.print( String.format("%5.2f ->", x.get(k) ));
       codeAmbiguities[k] = x.get(k); 
       System.out.println( String.format("%5.2f", codeAmbiguities[k] ));
     }
     
     // only update position when ambiguities have converged
     {
     // Receiver clock error
//     this.receiverClockError = x.get(3) / Constants.SPEED_OF_LIGHT;

     // apply correction to Rx position estimate
//     this.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));
//     this.computeGeodetic();
     }
     
     // update refTime
     if( correction_mag < 10 ){
       unixTime += tg * 1000;
       Time newTime = new Time( unixTime);
       roverObs.setRefTime( newTime );
       rover.setRefTime( newTime );
     }
     
     System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
         rover.getGeodeticLatitude(), 
         rover.getGeodeticLongitude(), 
         rover.getGeodeticHeight(), 
         new Time(unixTime).toString() ));
     
     // Estimation of the variance of the observation error
     vEstim = y0.minus(A.mult(x).plus(b));
     double varianceEstim = (vEstim.transpose().mult(Q.invert())
         .mult(vEstim)).get(0)
         / (nObsAvail - nUnknowns);

     // Covariance matrix of the estimation error
     if (nObsAvail > nUnknowns) {
       SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
       .scale(varianceEstim);
       this.positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
     }else{
       this.positionCovariance = null;
     }
     
     // Compute covariance matrix from A matrix [ECEF reference system]
     covXYZ = A.transpose().mult(A).invert().extractMatrix(0, 3, 0, 3);

     // Allocate and build rotation matrix
     SimpleMatrix R = new SimpleMatrix(3, 3);
     R = Coordinates.rotationMatrix(rover);

     // Propagate covariance from global system to local system
     covENU = R.mult(covXYZ).mult(R.transpose());

     //Compute DOP values
     rover.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
     rover.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
     rover.vDop = Math.sqrt(covENU.get(2, 2));
     
     return correction_mag; // return correction_mag
  }

  /**
   * Modular case for only 3 satellites (don't compensate for coarse time error)
   * @param roverObs
   * @param MODULO module in meters 
   * @return
   */
  public double codeStandaloneDTM( Observations roverObs, final double MODULO ) {
    long unixTime = roverObs.getRefTime().getMsec();

    rover.status = Status.None;

    // Number of GNSS observations without cutoff
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 4;

    // Define least squares matrices
    SimpleMatrix A = null;  // aka H or G, // Least squares design matrix
    SimpleMatrix b;  // Vector for approximate (estimated, predicted) pseudoranges
    SimpleMatrix y0; // Observed pseudoranges
    SimpleMatrix Q = null;  // Cofactor (Weighted) Matrix
    SimpleMatrix x;  // Solution (Update) Vector
    SimpleMatrix vEstim; // Observation Errors
    SimpleMatrix tropoCorr; 
    SimpleMatrix ionoCorr;

    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    SimpleMatrix covXYZ;
    covXYZ = new SimpleMatrix(3, 3);

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU;
    covENU = new SimpleMatrix(3, 3);

    // Number of available satellites (i.e. observations)
    int nObsAvail = sats.avail.size();
    
    nObsAvail++; // add DTM / height soft constraint

    // Least squares design matrix
    A = new SimpleMatrix( nObsAvail, nUnknowns );

    // Vector for approximate pseudoranges
    b = new SimpleMatrix(nObsAvail, 1);

    // Vector for observed pseudoranges
    y0 = new SimpleMatrix(nObsAvail, 1);

    // Cofactor matrix
    Q = new SimpleMatrix(nObsAvail, nObsAvail);

    // Solution vector
    x = new SimpleMatrix(nUnknowns, 1);

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

      if( sats.pos[i] == null  || !sats.avail.contains(id) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
        int satId = roverObs.getSatID(k);
        ObservationSet os = roverObs.getSatByID(satId);
        os.inUse(false);

        continue;
      }

      // Line Of Sight vector units (ECEF)
      SimpleMatrix e = new SimpleMatrix(1,3);

      // Line Of Sight vector units (ECEF)
      e.set( 0,0, rover.diffSat[i].get(0) / rover.satAppRange[i] );
      e.set( 0,1, rover.diffSat[i].get(1) / rover.satAppRange[i] );
      e.set( 0,2, rover.diffSat[i].get(2) / rover.satAppRange[i] );

      // scalar product of speed vector X unit vector
      float doppler = roverObs.getSatByID(id).getDoppler(ObservationSet.L1);
      double rodot;
      if( Float.isNaN( doppler ))
        rodot = -e.mult( sats.pos[i].getSpeed() ).get(0);
      else
        // scalar product of speed vector X unit vector
        rodot = -doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;

      // Fill in one row in the design matrix
      A.set(k, 0, e.get(0) ); /* X */
      A.set(k, 1, e.get(1) ); /* Y */
      A.set(k, 2, e.get(2) ); /* Z */

      A.set(k, 3, 1); /* clock error */
     // A.set(k, 4, rodot );

      // Add the approximate pseudorange value to b
//      b.set(k, 0, (rover.satAppRange[i] - sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) % MODULO );
      b.set(k, 0, (rover.satAppRange[i] - sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) );

      ObservationSet os = roverObs.getSatByID(id);
      
      // Add the clock-corrected observed pseudorange value to y0
//      y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));
//      y0.set(k, 0, os.getCodeC(0) % MODULO );
      y0.set(k, 0, os.getCodeC(0) );

//      if (!ignoreTopocentricParameters) {
      // cap tropo correction
      if( Double.isNaN( rover.satTropoCorr[i] ))
        rover.satTropoCorr[i] = 0;
      
      if(  rover.satTropoCorr[i]>30 )
        rover.satTropoCorr[i] = 30;
      if(  rover.satTropoCorr[i]<-30 )
        rover.satTropoCorr[i] = -30;
      
      tropoCorr.set(k, 0, rover.satTropoCorr[i]);
      ionoCorr.set(k, 0, rover.satIonoCorr[i]);
   
      // Fill in the cofactor matrix
      double weight;
      
      if( rover.topo[i].getElevation()<15 )
        weight = 1;
      else
        weight = Q.get(k, k)
          + computeWeight(rover.topo[i].getElevation(),
              roverObs.getSatByIDType(id, 'G').getSignalStrength(goGPS.getFreq()));
      if( weight>5 )
        weight = 5;
      
//      Q.set(k, k, weight);
      Q.set(k, k, 1);
//          if( weight > maxWeight )
//            maxWeight = weight;

      // Increment available satellites counter
      k++;
          
     } // i loop

    {
      // Add height soft constraint
      double lam = Math.toRadians(rover.getGeodeticLongitude());
      double phi = Math.toRadians(rover.getGeodeticLatitude());
      double hR_app = rover.getGeodeticHeight();
      //double hR_app = 0;
      //double h_dtm = recpos.getGeodeticHeight();
      
//    %extraction from the dtm of the height correspondent to the approximated position
//    [h_dtm] = grid_bilin_interp(E_app, N_app, tile_buffer, tile_header.ncols*3, tile_header.nrows*3, tile_header.cellsize, Ell, Nll, tile_header.nodata);
      double h_dtm = hR_app>0? hR_app : 30; // initialize to something above sea level
      if( h_dtm > goGPS.getMaxHeight() )
        h_dtm = goGPS.getMaxHeight();
      
//      if( goGPS.useDTM() ){
//        try {
//          ElevationResult eres = ElevationApi.getByPoint( getContext(), new LatLng(this.getGeodeticLatitude(), this.getGeodeticLongitude())).await();
//          if( eres.elevation > 0 )
//            h_dtm = eres.elevation;
//        } catch (Exception e1) {
//          // TODO Auto-generated catch block
//  //        e.printStackTrace();
//        }
//      }

      double cosLam = Math.cos(lam);
      double cosPhi = Math.cos(phi);
      double sinLam = Math.sin(lam);
      double sinPhi = Math.sin(phi);
      double[][] data = new double[1][3];
      data[0][0] = cosPhi * cosLam;
      data[0][1] = cosPhi * sinLam;
      data[0][2] = sinPhi;
      
      k = nObsAvail-1;
      A.set(k, 0, data[0][0] );
      A.set(k, 1, data[0][1] ); 
      A.set(k, 2, data[0][2] ); 
      A.set(k, 3, 0 ); 
//      A.set(k, 4, 0 );

//    %Y0 vector computation for DTM constraint
//    y0_dtm = h_dtm  - hR_app + cos(phiR_app)*cos(lamR_app)*X_app + cos(phiR_app)*sin(lamR_app)*Y_app + sin(phiR_app)*Z_app;
      double y0_dtm = h_dtm  - hR_app;
      y0.set(k, 0, y0_dtm );
//      double max = Q.elementMaxAbs();
//      Q.set(k, k, max);
      Q.set(k, k, 1);
    }
    
//  if (!ignoreTopocentricParameters) {
    b = b.plus(tropoCorr);
    b = b.plus(ionoCorr);

    rover.satsInUse = 0;
    SimpleMatrix resid = y0.minus(b);
    
// use smallest resid
    double pivot = MODULO;
    for( k=0; k<sats.avail.size(); k++){
      double d = resid.get(k);
      if( Math.abs(d)<Math.abs(pivot))
        pivot = d;
    }
      
// use highest sat    
//    double pivot = 0;
//    double pivotEl = 0;
//    for( k=0; k<sats.avail.size(); k++){
//      int satId = roverObs.getSatID(k);
//      ObservationSet os = roverObs.getSatByID(satId);
//      if( rover.topo[k] == null )
//        continue;
//      
//      if( rover.topo[k].getElevation() > pivotEl ){
//        pivotEl = rover.topo[k].getElevation();
//        pivot = resid.get(k);
//      }
//    }
    
    System.out.println( String.format( "* Residuals -> Adjusted Residuals (ms) - Pivot = %7.4f (ms)",  pivot/Constants.SPEED_OF_LIGHT*1000));
    
    for( k=0; k<sats.avail.size(); k++){
      int satId = roverObs.getSatID(k);
      ObservationSet os = roverObs.getSatByID(satId);
      
      double d = resid.get(k);
      System.out.print( String.format( "%2d) C:%8.3f (%8.5f); %9.5f -> ", 
          satId, 
          roverObs.getSatByID(satId).getCodeC(0), 
          roverObs.getSatByID(satId).getCodeC(0)/Constants.SPEED_OF_LIGHT*1000, 
          d/Constants.SPEED_OF_LIGHT*1000));
      
      if( d-pivot>MODULO/2 ){
        d-=MODULO;
      }
      if( d-pivot<-MODULO/2){
        d+=MODULO;
      }
      System.out.print( String.format( "%9.5f", d/Constants.SPEED_OF_LIGHT*1000));
      System.out.print( String.format( "  Q:%3.1f", Q.get(k,k)));
      
      // check again, if fails, exclude this satellite
      double dms = Math.abs(d-pivot)/Constants.SPEED_OF_LIGHT*1000;
      if( Math.abs(dms) > goGPS.getResidThreshold() )
      {
        if( goGPS.isDebug() ) System.out.println( String.format( " Excluding d:%8.3f", dms));
        resid.set(k, 0);
        A.set(k, 0, 0);
        A.set(k, 1, 0);
        A.set(k, 2, 0);
        A.set(k, 3, 0);
        os.inUse(false);
        os.eRes = d-pivot;
        os.el = rover.topo[k].getElevation();
      }
      else 
      {
        resid.set(k,d);
        rover.satsInUse++;
        os.inUse(true);
        System.out.println();
        rover.eRes += Math.pow( d-pivot, 2); 
      }
    }
    rover.eRes = Math.sqrt(rover.eRes/rover.satsInUse);
    System.out.println(String.format("eRes = %5.3f\r\n", rover.eRes));
    
    if( rover.satsInUse < nUnknowns-1 ){
      System.out.println("Not enough satellites for " + roverObs.getRefTime() );
      rover.setXYZ(0, 0, 0);
      if( rover.status == Status.None ){
        rover.status = Status.NotEnoughSats;
      }
      return 0;
    }
      
//       Weighted Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
      x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert()).mult(resid);

     double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                        Math.pow( x.get(1), 2 ) +
                                        Math.pow( x.get(2), 2 ) );

     // x.get(3) = Receiver clock error in meters
     double cbiasms = x.get(3) * 1000d / Constants.SPEED_OF_LIGHT;
     // x.get(4) = time update in seconds
//     double tg = x.get(4); 

     // expected
     System.out.println( String.format( "pos update:  %5.1f, %5.1f, %5.1f; Mag: %5d(m)", x.get(0), x.get(1), x.get(2), (long)correction_mag ));
     System.out.println( String.format( "common bias: %2.4f (ms)", cbiasms ));
//     System.out.println( String.format( "time update: %3.3f (s)", tg ));

     // Receiver clock error
     rover.clockError = x.get(3) / Constants.SPEED_OF_LIGHT;

     // apply correction to Rx position estimate
     rover.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));
     rover.computeGeodetic();

     // update refTime
//     if( correction_mag < 10 ){
//       unixTime += tg * 1000;
//       Time newTime = new Time( unixTime);
//       roverObs.setRefTime( newTime );
//       this.setRefTime( newTime );
//     }
     
     System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
         rover.getGeodeticLatitude(), 
         rover.getGeodeticLongitude(), 
         rover.getGeodeticHeight(), 
         new Time(unixTime).toString() ));
     
     // Estimation of the variance of the observation error
     vEstim = y0.minus(A.mult(x).plus(b));
     double varianceEstim = (vEstim.transpose().mult(Q.invert())
         .mult(vEstim)).get(0)
         / (nObsAvail - nUnknowns);

     // Covariance matrix of the estimation error
     if (nObsAvail > nUnknowns) {
       SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
       .scale(varianceEstim);
       this.positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
     }else{
       this.positionCovariance = null;
     }
     
     // Compute covariance matrix from A matrix [ECEF reference system]
     covXYZ = A.transpose().mult(A).invert().extractMatrix(0, 3, 0, 3);

     // Allocate and build rotation matrix
     SimpleMatrix R = new SimpleMatrix(3, 3);
     R = Coordinates.rotationMatrix(rover);

     // Propagate covariance from global system to local system
     covENU = R.mult(covXYZ).mult(R.transpose());

     //Compute DOP values
     rover.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
     rover.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
     rover.vDop = Math.sqrt(covENU.get(2, 2));
     
     return correction_mag; // return correction_mag
  }
}
