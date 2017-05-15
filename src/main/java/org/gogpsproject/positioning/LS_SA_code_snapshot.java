package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.Coordinates;
import org.gogpsproject.GoGPS;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Status;
import org.gogpsproject.Time;

public class LS_SA_code_snapshot extends Core {

  private static final int MINSV = 4;

  public LS_SA_code_snapshot(GoGPS goGPS) {
    super(goGPS);
  }
  
  /**
   * Run snapshot processing on an observation, use as pivot the satellite with the passed index 
   * @param satId
   * @return computed eRes
   */
  public static class SnapshotPivotResult {
    int satIndex;
    int satId;
    double elevation;
    Coordinates roverPos;
    public long unixTime;
    public Double eRes;
    public double hDop;
    public int nObs;
    public double cbiasms;
    
    public SnapshotPivotResult() {
      this.eRes = Constants.SPEED_OF_LIGHT/1000/2;
    }

    public SnapshotPivotResult( int satIndex, int satId, double elevation,
        Coordinates roverPos, long unixTime, Double eRes, double hDop, int nObs, double cbiasms ){
      this.satIndex = satIndex;
      this.satId = satId;
      this.elevation = elevation;
      this.roverPos = (Coordinates) roverPos.clone();
      this.unixTime = unixTime;
      this.eRes = eRes;
      this.hDop = hDop;
      this.nObs = nObs;
      this.cbiasms = cbiasms;
    }
  }
  
  SnapshotPivotResult snapshotProcessPivot( Observations roverObs, int pivotIndex, int max_iterations, double cutOffEl, Double residCutOff ){
    
    int savedIndex = pivotIndex;
    int nObs = roverObs.getNumSat();

    // Number of unknown parameters
    int nUnknowns = 5;

    Coordinates refPos = (Coordinates) roverPos.clone();
    
    long refTime = roverObs.getRefTime().getMsec();
    long unixTime = refTime;

    // Define least squares matrices
    SimpleMatrix A = null;  // aka H or G, // Least squares design matrix
    SimpleMatrix b = null;  // Vector for approximate (estimated, predicted) pseudoranges
    SimpleMatrix y0 = null; // Observed pseudoranges
    SimpleMatrix Q = null;  // Cofactor (Weighted) Matrix
    SimpleMatrix x = null;  // Solution (Update) Vector
    SimpleMatrix tropoCorr = null; 
    SimpleMatrix ionoCorr = null;

    int pivotSatId = roverObs.getSatID(pivotIndex);

    final double POS_TOL = 1.0;    // meters
    final double TG_TOL = 1;  // milliseconds

    int nObsAvail = 0;
    double pivotElevation = 0;
    roverPos.eRes = 300;

    // common bias in ms
    double cbiasms = 1;
    
    for (int itr = 0; itr < max_iterations; itr++) {
      if( goGPS.isDebug()) System.out.println(">> Itr " + itr);
      
//      if( goGPS.isDebug() && goGPS.truePos != null ){
//        System.out.println( String.format( "\r\n* True Pos: %8.4f, %8.4f, %8.4f", 
//            goGPS.truePos.getGeodeticLatitude(),
//            goGPS.truePos.getGeodeticLongitude(),
//            goGPS.truePos.getGeodeticHeight()
//            ));
//        goGPS.truePos.selectSatellitesStandaloneFractional( roverObs, -100, GoGPS.MODULO1MS );
//      }
//      if( goGPS.isDebug()) System.out.println();
      
      if( roverObs.getNumSat()>5 )
        selectSatellitesStandaloneFractional( roverObs, cutOffEl, GoGPS.MODULO1MS ); 
      else
        selectSatellitesStandaloneFractional( roverObs, -10, GoGPS.MODULO1MS );
        
      nObsAvail = getSatAvailNumber();
  
      if( nObsAvail<MINSV ){
        if( goGPS.isDebug()) System.out.println("\r\nNot enough satellites for " + roverObs.getRefTime() );
        roverPos.setXYZ(0, 0, 0);
        if( nObsAvail>0 ){
          roverPos.satsInUse = nObsAvail;
          roverPos.status = Status.NotEnoughSats;
        }
        return null;
      }
    
      if( pos[savedIndex]==null || roverTopo[savedIndex] == null  || !satAvail.contains(pivotSatId)) {
        if( goGPS.isDebug()) System.out.println("\r\nCan't use pivot with satId " + pivotSatId );
        return null; 
      }
      
      pivotElevation = roverTopo[savedIndex].getElevation();
      
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
  
      int k = 0;
  
      // Initialize the cofactor matrix
  //    Q.set(1);
  
      // Set up the least squares matrices
      for (int i = 0; i < nObs; i++) {
        int satId = roverObs.getSatID(i);

        if (pos[i]==null || !satAvail.contains(satId)) 
          continue; // i loop
      
        ObservationSet os = roverObs.getSatByID(satId);

        if( satId == pivotSatId  && pivotIndex != k ) {
          pivotIndex = k;
        }

        // Line Of Sight vector units (ECEF)
        SimpleMatrix e = new SimpleMatrix(1,3);
  
        // Line Of Sight vector units (ECEF)
        e.set( 0,0, diffRoverSat[i].get(0) / roverSatAppRange[i] );
        e.set( 0,1, diffRoverSat[i].get(1) / roverSatAppRange[i] );
        e.set( 0,2, diffRoverSat[i].get(2) / roverSatAppRange[i] );
  
        // scalar product of speed vector X unit vector
        float doppler = roverObs.getSatByID(satId).getDoppler(ObservationSet.L1);
        double rodot;
        double rodotSatSpeed   = -e.mult( pos[i].getSpeed() ).get(0);
        double dopplerSatSpeed = -rodotSatSpeed*Constants.FL1/Constants.SPEED_OF_LIGHT;

        if( Float.isNaN( doppler )){
          rodot = rodotSatSpeed;
        }
        else {
          // scalar product of speed vector X unit vector
          rodot = -doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;
          if( goGPS.isDebug() ) System.out.println( String.format( "%2d) doppler:%6.0f; satSpeed:%6.0f; D:%6.0f", 
              satId,
              doppler, 
              dopplerSatSpeed,
              doppler - dopplerSatSpeed ));
          
//          if( Math.abs(doppler - dopplerSatSpeed)>200)
//            System.out.println("diff here");
          rodot = rodotSatSpeed;
        }
        
        
        // Fill in one row in the design matrix
        A.set(k, 0, e.get(0) ); /* X */
        A.set(k, 1, e.get(1) ); /* Y */
        A.set(k, 2, e.get(2) ); /* Z */
  
        A.set(k, 3, 1); /* clock error */
        A.set(k, 4, rodot );
  
        // Add the approximate pseudorange value to b
        b.set(k, 0, (roverSatAppRange[i] - pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) % GoGPS.MODULO1MS );
  
        // Add the clock-corrected observed pseudorange value to y0
  //      y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));
        y0.set(k, 0, os.getCodeC(0)  );
    
        // cap tropo correction
        if( Double.isNaN( roverSatTropoCorr[i] ))
          roverSatTropoCorr[i] = 0;
   
        if(  roverSatTropoCorr[i]>30 )
          roverSatTropoCorr[i] = 30;
        if(  roverSatTropoCorr[i]<-30 )
          roverSatTropoCorr[i] = -30;
    
        tropoCorr.set(k, 0, roverSatTropoCorr[i]);
        ionoCorr.set(k, 0, roverSatIonoCorr[i]);
     
        // Fill in the cofactor matrix
        double weight = Q.get(k, k)
            + computeWeight(roverTopo[i].getElevation(),
                roverObs.getSatByIDType(satId, 'G').getSignalStrength(goGPS.getFreq()));
        Q.set(k, k, weight);
  
        // Increment available satellites counter
        k++;
       } // i loop
    
      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);
  
      // Add height soft constraint
      double lam = Math.toRadians(roverPos.getGeodeticLongitude());
      double phi = Math.toRadians(roverPos.getGeodeticLatitude());
      double hR_app = roverPos.getGeodeticHeight();
        
    //  %extraction from the dtm of the height correspondent to the approximated position
    //  [h_dtm] = grid_bilin_interp(E_app, N_app, tile_buffer, tile_header.ncols*3, tile_header.nrows*3, tile_header.cellsize, Ell, Nll, tile_header.nodata);
      double h_dtm = hR_app>0? 
          hR_app 
          : 30; // initialize to something above sea level
      if( h_dtm > 2000 )
        h_dtm = 2000;
        
//        if( goGPS.useDTM() ){
//          try {
//            ElevationResult elres = ElevationApi.getByPoint( getContext(), new LatLng(getGeodeticLatitude(), getGeodeticLongitude())).await();
//            if( elres.elevation > 0 )
//              h_dtm = elres.elevation;
//          } catch (Exception e) {
//            // TODO Auto-generated catch block
//    //        e.printStackTrace();
//          }
//        }
        
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
    
    //  %Y0 vector computation for DTM constraint
    //  y0_dtm = h_dtm  - hR_app + cos(phiR_app)*cos(lamR_app)*X_app + cos(phiR_app)*sin(lamR_app)*Y_app + sin(phiR_app)*Z_app;
        double y0_dtm = h_dtm  - hR_app;
        y0.set(k, 0, y0_dtm );
    
        double maxWeight = Q.elementMaxAbs();
        Q.set(k, k, maxWeight );
    
        SimpleMatrix resid = y0.minus(b);
        
        roverPos.satsInUse = 0;
        
        // adjust residuals based on satellite with pivotIndex
        {
          ObservationSet os = roverObs.getSatByID(pivotSatId);
  //        if( !Float.isNaN( os.getSignalStrength(0)) && os.getSignalStrength(0)<18 ){
  //          return null;
  //        }
          double pivot = resid.get(pivotIndex);
          
          if( goGPS.isDebug()) System.out.println( String.format( "\r\n\r\nResiduals -> Adjusted Residuals (ms) - Pivot = %7.4f (ms)",  pivot/Constants.SPEED_OF_LIGHT*1000));
          int i = 0;
          for( k=0; k<roverObs.getNumSat(); k++){
            Integer satId = roverObs.getSatID(k);
            os = roverObs.getSatByID(satId);
      
            if( !satAvail.contains(satId) || pos[k] == null || roverTopo[k] == null )
              continue;
      
            os.el = roverTopo[k].getElevation();
      
            double d = resid.get(i);
            if( goGPS.isDebug()) System.out.print( String.format( "%2d) %7.4f -> ", satId, d/Constants.SPEED_OF_LIGHT*1000));
            if( d-pivot>GoGPS.MODULO1MS/2 ){
              d-=GoGPS.MODULO1MS;
            }
            if( d-pivot<-GoGPS.MODULO1MS/2){
              d+=GoGPS.MODULO1MS;
            }
            if( goGPS.isDebug()) System.out.print( String.format( "%7.4f", d/Constants.SPEED_OF_LIGHT*1000));
      
            resid.set(i,d);
      
            // check again, if fails, exclude this satellite
            // TODO there could be a problem if the pivot is bogus!
            double dms = Math.abs(d-pivot)/Constants.SPEED_OF_LIGHT*1000;
            if( residCutOff != null &&
                (roverPos.eRes<residCutOff*Constants.SPEED_OF_LIGHT/1000) &&
                dms>residCutOff) {
              if( goGPS.isDebug() ) System.out.println( String.format( "; D:%6.4f; Excluding sat:%2d; C:%6.4f; El:%6.4f; rodot:%7.2f; ", 
                  dms, 
                  roverObs.getSatID(k), 
                  roverObs.getSatByID(satId).getCodeC(0)/Constants.SPEED_OF_LIGHT*1000, 
                  os.el,
                  A.get(i, 4) ));
              
              A.set(i, 0, 0);
              A.set(i, 1, 0);
              A.set(i, 2, 0);
              A.set(i, 3, 0);
              A.set(i, 4, 0);
              os.inUse(false);
            }
            else {
              roverPos.satsInUse++;
              os.inUse(true);
      
              if( goGPS.isDebug() ) System.out.println( String.format( "; D:%6.4f; snr:%5.1f; El:%5.1f; rodot:%10.2f; ", 
                  dms, 
                  os.getSignalStrength(0),
                  os.el, 
                  A.get(i, 4) ));
            }
            i++;
          }
        } 
      
        if( roverPos.satsInUse < MINSV ){
          if( goGPS.isDebug()) System.out.println("Not enough satellites for " + roverObs.getRefTime() );
          roverPos.setXYZ(0, 0, 0);
          if( roverPos.status == Status.None ){
            roverPos.status = Status.NotEnoughSats;
          }
          return null;
        }
        SimpleMatrix B = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert());
        x = B.mult(resid);
  
        double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                Math.pow( x.get(1), 2 ) +
                                Math.pow( x.get(2), 2 ) );

        double cbias = x.get(3); // Receiver clock error in meters
        cbiasms  = cbias * 1000d / Constants.SPEED_OF_LIGHT;
        double tg = x.get(4); // time update in seconds
      
        if( goGPS.isDebug()) System.out.println( String.format( "\r\npos update:  %5.0f (m)", correction_mag ));
        if( goGPS.isDebug()) System.out.println( String.format( "time update: %3.3f (s)", tg ));
        if( goGPS.isDebug()) System.out.println( String.format( "common bias: %2.4f (ms)", cbiasms ));

        // Receiver clock error in seconds
        roverPos.receiverClockError = x.get(3)/ Constants.SPEED_OF_LIGHT;
       
        // apply correction to Rx position estimate
        roverPos.setPlusXYZ( x.extractMatrix(0, 3, 0, 1) );
        roverPos.computeGeodetic();

        // update refTime
        unixTime += tg * 1000;
        Time newTime = new Time( unixTime);
        roverObs.setRefTime( newTime );
          
        if( goGPS.isDebug()) System.out.println( String.format( "recpos (%d): %5.3f, %5.3f, %5.3f, %s", 
            itr, 
            roverPos.getGeodeticLatitude(), 
            roverPos.getGeodeticLongitude(), 
            roverPos.getGeodeticHeight(), 
            new Time(unixTime).toString() ));
        
       // average eRes 
       SimpleMatrix eResM = A.mult(x).minus(resid);
       roverPos.eRes = 0;
       for( k=0; k<satAvail.size(); k++){
         int satId = roverObs.getSatID(k);
         ObservationSet os = roverObs.getSatByID(satId);
         os.eRes = Math.abs( eResM.get(k));
         if( os.isInUse() )
           roverPos.eRes += Math.pow( os.eRes, 2); 
       }
       roverPos.eRes = Math.sqrt(roverPos.eRes/roverPos.satsInUse);
       if( goGPS.isDebug()) System.out.println(String.format("eRes = %5.3f\r\n", roverPos.eRes));
   
       // if correction is small enough, we're done, exit loop
       // TODO check also eRes
       if(( correction_mag< POS_TOL || roverPos.eRes<POS_TOL ) && Math.abs(tg*1000) < TG_TOL )
         break; 
    } // itr

    double correction_mag = refPos.minusXYZ(roverPos).normF();
    double tg = (roverObs.getRefTime().getMsec() - refTime)/1000;

    if( Double.isNaN(correction_mag) || correction_mag>goGPS.getPosLimit() ||  Math.abs(tg) > goGPS.getTimeLimit() ){
      
      if(goGPS.isDebug()) System.out.println("Correction exceeds the limits: dist = " + (long)correction_mag +"m; t offset = " + (long)tg +"s" );

      roverPos.setXYZ(0, 0, 0);
      roverPos.status = Status.MaxCorrection;

      return null;
    } 
    else {
      if( goGPS.isDebug()) 
        System.out.println( String.format( "recpos: %5.3f, %5.3f, %5.3f, %s, eRes = %5.3f", 
            roverPos.getGeodeticLatitude(), 
            roverPos.getGeodeticLongitude(), 
            roverPos.getGeodeticHeight(), 
            new Time(unixTime).toString(), 
            roverPos.eRes, cbiasms ));
    }
    
    
    SimpleMatrix vEstim; // Observation Errors
    // Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
    SimpleMatrix covXYZ = null;

    // Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
    SimpleMatrix covENU = null;

    
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
//    covXYZ = A.extractMatrix(0, nObsAvail, 0, 3).transpose().mult(A.extractMatrix(0, nObsAvail, 0, 3)).invert();
     covXYZ = A.transpose().mult(A).invert().extractMatrix(0, 3, 0, 3);

    // Allocate and build rotation matrix
    SimpleMatrix R = new SimpleMatrix(3, 3);
    R = Coordinates.rotationMatrix(roverPos);

    // Propagate covariance from global system to local system
    covENU = R.mult(covXYZ).mult(R.transpose());

     //Compute DOP values
    roverPos.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    roverPos.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    roverPos.vDop = Math.sqrt(covENU.get(2, 2));
    
//    if( Math.abs(cbiasms)>0.5 )
//      System.out.println( pivotSatId + ") cbiasms " + cbiasms); 
    
    return new SnapshotPivotResult( savedIndex, pivotSatId, pivotElevation, 
        roverPos, unixTime, roverPos.eRes, roverPos.hDop, getSatAvailNumber(), cbiasms );
  }
  
  /**
   * @param roverObs
   * return computed time offset in milliseconds
   */
  public Long snapshotPos( Observations roverObs ) {
    Double residCutOff = 0.5;
    double elCutOff = 5.0;
    roverPos.status = Status.None;

    // Number of GPS observations
    int nObs = roverObs.getNumSat();
    if( nObs < MINSV ){
      if(goGPS.isDebug()) System.out.println("Not enough satellites for " + roverObs.getRefTime() );
      roverPos.setXYZ(0, 0, 0);
      roverPos.satsInUse = nObs;
      roverPos.status = Status.NotEnoughSats;
      return null;
    }

    SnapshotPivotResult result = null;
    ReceiverPosition refPos = new ReceiverPosition();

    // save this position before trying
    roverPos.cloneInto(refPos);
    Time refTime = roverObs.getRefTime();
    
    for( int satIdx = 0; satIdx<roverObs.getNumSat(); satIdx++ ){
      if(goGPS.isDebug()) System.out.println( "\r\n===> Try Pivot " + satIdx );
      
      // restore this position before trying
      refPos.cloneInto(roverPos);
      roverObs.setRefTime(refTime);
      
      roverPos.status = Status.None;
      // select a pivot with at least elCutOff elevation
      int maxIterations = 3;
      residCutOff = 0.001;
      SnapshotPivotResult pivotRes = snapshotProcessPivot( roverObs, satIdx, maxIterations, elCutOff, residCutOff  );
      
      if( pivotRes == null && roverPos.status == Status.EphNotFound ){
        // don't keep requesting Ephemeris if they're not ready yet
        roverPos.setXYZ(0, 0, 0);
        return null;
//        continue;
      }
      
      if( pivotRes != null && Math.abs( pivotRes.cbiasms )<=0.5 &&( result == null || pivotRes.eRes<result.eRes )){
        result = pivotRes;
      }
//      if( pivotRes != null && ( result == null || Math.abs(pivotRes.cbiasms)<Math.abs( result.cbiasms ))){
//        result = pivotRes;
//      }
    }
    if( result == null ){
      roverPos.setXYZ(0, 0, 0);
      return null;
    }
    if(goGPS.isDebug()) 
      System.out.println( String.format( "\r\n>> Selected Pivot SatId = %d; SatIdx = %d; eRes = %5.2f;  cbias = %5.2f; elevation = %5.2f\r\n", 
        result.satId, 
        result.satIndex,
        result.eRes,
        result.cbiasms,
        result.elevation
        ));
   
    // restore this position after trying
    refPos.cloneInto(roverPos);
    roverObs.setRefTime(refTime);
    
    // reprocess with selected pivot and more iterations
 
    roverPos.status = Status.None;
    if( result.nObs<5 )
//      residCutOff = 0.0002;
      elCutOff = -5;
      
    result = snapshotProcessPivot( roverObs, result.satIndex, 100, elCutOff, residCutOff );
    if( result == null ){
      roverPos.setXYZ(0, 0, 0);
      return null;
    }
    
    if( result.eRes > 350 ){
//      if(goGPS.isDebug()) 
        System.out.println("eRes too large = " + roverPos.eRes );

      roverPos.setXYZ(0, 0, 0);
      roverPos.status = Status.MaxEres;

      return null;
    }

    if( result.hDop>this.goGPS.getHdopLimit() ){
      if(goGPS.isDebug()) System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
          roverPos.getGeodeticLatitude(), 
          roverPos.getGeodeticLongitude(), 
          roverPos.getGeodeticHeight(), 
          new Time(result.unixTime).toString() ));
//      if(goGPS.isDebug()) 
        System.out.println( String.format( "hDOP too large: %3.1f", result.hDop ));
      roverPos.setXYZ(0, 0, 0);
      roverPos.status = Status.MaxHDOP;
      return null;
    }
    
    roverPos.status = Status.Valid;
    result.roverPos.cloneInto(roverPos);
    roverPos.setRefTime(new Time(result.unixTime));

    if(goGPS.isDebug()) System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
        roverPos.getGeodeticLatitude(), 
        roverPos.getGeodeticLongitude(), 
        roverPos.getGeodeticHeight(), 
        new Time(result.unixTime).toString() ));

    long offsetms = result.unixTime - refTime.getMsec();
    
    if( offsetms == 0 )
      System.out.println("Check here");
    
    return offsetms;
  }
  
  
  
}
