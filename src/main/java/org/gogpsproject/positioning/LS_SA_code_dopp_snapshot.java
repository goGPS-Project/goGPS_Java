package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Status;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public class LS_SA_code_dopp_snapshot extends LS_SA_code_snapshot {

  // Number of unknown parameters (here adds the clock error rate)
  public static final int nUnknowns = 6;

  /** max time update for a valid fix */
  private long maxTimeUpdateSec = 30; // s

  /** Set aPrioriPos in thread mode */
  public Coordinates aPrioriPos;

  public LS_SA_code_dopp_snapshot(GoGPS goGPS) {
    super(goGPS);
  }
  
  public long getTimeLimit(){
    return maxTimeUpdateSec;
  }

  public GoGPS setTimeLimit(long maxTimeUpdateSec) {
    this.maxTimeUpdateSec  = maxTimeUpdateSec;
    return goGPS;
  }

  SnapshotPivotResult snapshotProcessPivot( Observations roverObs, int pivotIndex, int max_iterations, double cutOffEl, Double residCutOff ){
    
    int savedIndex = pivotIndex;
    int nObs = roverObs.getNumSat();

    Coordinates refPos = (Coordinates) rover.clone();
    
    long refTime = roverObs.getRefTime().getMsec();
    long unixTime = refTime;

    SimpleMatrix A = null;  // design matrix
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
    rover.eRes = 300;

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
      
      if( 2*roverObs.getNumSat()+1>nUnknowns )
        selectSatellites( roverObs, cutOffEl, GoGPS.MODULO1MS ); 
      else
        selectSatellites( roverObs, -10, GoGPS.MODULO1MS );
      
      int ns = sats.getAvailNumber();
      nObsAvail = 2*sats.getAvailNumber() + 1; // use doppler and DTM / height soft constraint

      if( nObsAvail<nUnknowns ){
        if( goGPS.isDebug()) System.out.println("\r\nNot enough satellites for " + roverObs.getRefTime() );
        rover.setXYZ(0, 0, 0);
        if( nObsAvail>0 ){
          rover.satsInUse = nObsAvail;
          rover.status = Status.NotEnoughSats;
        }
        return null;
      }
    
      if( sats.pos[savedIndex]==null || rover.topo[savedIndex] == null  || !sats.avail.contains(pivotSatId)) {
        if( goGPS.isDebug()) System.out.println("\r\nCan't use pivot with satId " + pivotSatId );
        return null; 
      }
      
      pivotElevation = rover.topo[savedIndex].getElevation();
      
      // Least squares design matrix
      A = new SimpleMatrix( nObsAvail, nUnknowns );
  
      // Vector for approximate pseudoranges
      b = new SimpleMatrix( nObsAvail, 1 );
  
      // Vector for observed pseudoranges
      y0 = new SimpleMatrix( nObsAvail, 1 );
  
      // Cofactor matrix
      Q = new SimpleMatrix( nObsAvail, nObsAvail );
  
      // Solution vector
      x = new SimpleMatrix( nUnknowns, 1 );
  
      // Vectors for troposphere and ionosphere corrections
      tropoCorr = new SimpleMatrix(nObsAvail, 1);
      ionoCorr = new SimpleMatrix(nObsAvail, 1);
  
      int k = 0;
  
      // Initialize the cofactor matrix
  //    Q.set(1);
  
      // Set up the least squares matrices
      for (int i = 0; i < nObs; i++) {
        int satId = roverObs.getSatID(i);

        if (sats.pos[i]==null || !sats.avail.contains(satId)) 
          continue; // i loop

        float doppler = roverObs.getSatByID(satId).getDoppler(ObservationSet.L1);

        if( Float.isNaN( doppler ))
          // TODO revert to code-only case
          return null;

        ObservationSet os = roverObs.getSatByID(satId);

        if( satId == pivotSatId  && pivotIndex != k ) {
          pivotIndex = k;
        }

        // Line Of Sight vector units (ECEF)
        SimpleMatrix e = new SimpleMatrix(1,3);
  
        // Line Of Sight vector units (ECEF)
        e.set( 0,0, rover.diffSat[i].get(0) / rover.satAppRange[i] );
        e.set( 0,1, rover.diffSat[i].get(1) / rover.satAppRange[i] );
        e.set( 0,2, rover.diffSat[i].get(2) / rover.satAppRange[i] );
  
        /** range rate = scalar product of speed vector X unit vector */
        double rodot = doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;

        /** computed satspeed: scalar product of speed vector X LOS unit vector */
        double rodotSatSpeed   = e.mult( sats.pos[i].getSpeed() ).get(0);
        
        /** Design Matrix */
        A.set( k, 0, e.get(0) ); /* X */
        A.set( k, 1, e.get(1) ); /* Y */
        A.set( k, 2, e.get(2) ); /* Z */
        A.set( k, 3, 1 );       // clock error
        A.set( k, 4, -rodot + rover.getClockErrorRate() );   // common bias
        A.set( ns+k, 0, sats.pos[i].getSpeed().get(0)/rover.satAppRange[i] ); /* VX */
        A.set( ns+k, 1, sats.pos[i].getSpeed().get(1)/rover.satAppRange[i] ); /* VY */
        A.set( ns+k, 2, sats.pos[i].getSpeed().get(2)/rover.satAppRange[i] ); /* VZ */
        A.set( ns+k, 5, 1 );     // clock error rate
  
        /** residuals */
        // Add the approximate pseudorange value to b
        b.set(k, 0, (rover.satAppRange[i] - sats.pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT) % GoGPS.MODULO1MS );
  
        // Add the clock-corrected observed pseudorange value to y0
  //      y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));
        y0.set(    k, 0, os.getCodeC(0) );
        y0.set( ns+k, 0, rodot  - rodotSatSpeed - rover.getClockErrorRate() );

        // cap tropo correction
        if( Double.isNaN( rover.satTropoCorr[i] ))
          rover.satTropoCorr[i] = 0;
   
        if(  rover.satTropoCorr[i]>30 )
          rover.satTropoCorr[i] = 30;
        if(  rover.satTropoCorr[i]<-30 )
          rover.satTropoCorr[i] = -30;
    
        tropoCorr.set(k, 0, rover.satTropoCorr[i]);
        ionoCorr.set( k, 0, rover.satIonoCorr[i]);
     
        // Fill in the cofactor matrix
        double weight = Q.get(k, k)
            + computeWeight(rover.topo[i].getElevation(),
                roverObs.getSatByIDType(satId, 'G').getSignalStrength(goGPS.getFreq()));
        Q.set(k, k, weight);
  
        // Increment available satellites counter
        k++;
       } // i loop
    
      // Apply troposphere and ionosphere correction
      b = b.plus(tropoCorr);
      b = b.plus(ionoCorr);
  
      SimpleMatrix resid = y0.minus(b);
      
      rover.satsInUse = 0;
        
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
    
          if( !sats.avail.contains(satId) || sats.pos[k] == null || rover.topo[k] == null )
            continue;
    
          os.el = rover.topo[k].getElevation();
    
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
              (rover.eRes<residCutOff*Constants.SPEED_OF_LIGHT/1000) &&
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
            rover.satsInUse++;
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
      
      if( 2*rover.satsInUse + 1 < nUnknowns ){
        if( goGPS.isDebug()) System.out.println("Not enough satellites for " + roverObs.getRefTime() );
        rover.setXYZ(0, 0, 0);
        if( rover.status == Status.None ){
          rover.status = Status.NotEnoughSats;
        }
        return null;
      }
      
      // Add height soft constraint
      double hR_app = rover.getGeodeticHeight();
        
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
        
      double lam = Math.toRadians(rover.getGeodeticLongitude());
      double phi = Math.toRadians(rover.getGeodeticLatitude());

      double cosLam = Math.cos(lam);
      double cosPhi = Math.cos(phi);
      double sinLam = Math.sin(lam);
      double sinPhi = Math.sin(phi);
        
      k = nObsAvail-1;
      A.set( k, 0, cosPhi * cosLam );
      A.set( k, 1, cosPhi * sinLam ); 
      A.set( k, 2, sinPhi ); 
      A.set( k, 3, 0 ); 
      A.set( k, 4, 0 );
    
    //  %Y0 vector computation for DTM constraint
    //  y0_dtm = h_dtm  - hR_app + cos(phiR_app)*cos(lamR_app)*X_app + cos(phiR_app)*sin(lamR_app)*Y_app + sin(phiR_app)*Z_app;
      double y0_dtm = h_dtm  - hR_app;
      y0.set(k, 0, y0_dtm );
    
      double maxWeight = Q.elementMaxAbs();
      Q.set(k, k, maxWeight );
  
//      SimpleMatrix B = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose()).mult(Q.invert());
      SimpleMatrix B = A.transpose().mult(A).invert().mult(A.transpose());
      x = B.mult(resid);
  
      double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                         Math.pow( x.get(1), 2 ) +
                                         Math.pow( x.get(2), 2 ) );

      // Receiver clock error: m -> seconds
      rover.clockError = x.get(3)/ Constants.SPEED_OF_LIGHT;
      
      // time update seconds -> ms
      cbiasms = x.get(4) * 1000;
     
      // Update receiver clock error rate
      rover.clockErrorRate += x.get(5);
      
      if( goGPS.isDebug()) System.out.println( String.format( "\r\npos update:  %5.0f (m)", correction_mag ));
      if( goGPS.isDebug()) System.out.println( String.format( "clock error: %2.4f (us)", rover.clockError*1000000 ));
      if( goGPS.isDebug()) System.out.println( String.format( "common bias: %2.4f (ms)", cbiasms ));
      if( goGPS.isDebug()) System.out.println( String.format( "clock error rate: %2.4f", rover.clockErrorRate ));

      // apply correction to Rx position estimate
      rover.setPlusXYZ( x.extractMatrix(0, 3, 0, 1) );
      rover.computeGeodetic();

      // update refTime
      unixTime += cbiasms;
      Time newTime = new Time( unixTime);
      roverObs.setRefTime( newTime );
          
      if( goGPS.isDebug()) System.out.println( String.format( "recpos (%d): %5.3f, %5.3f, %5.3f, %s", 
          itr, 
          rover.getGeodeticLatitude(), 
          rover.getGeodeticLongitude(), 
          rover.getGeodeticHeight(), 
          new Time(unixTime).toString() ));
        
       // average eRes 
       SimpleMatrix eResM = A.mult(x).minus(resid);
       rover.eRes = 0;
       for( k=0; k<sats.avail.size(); k++){
         int satId = roverObs.getSatID(k);
         ObservationSet os = roverObs.getSatByID(satId);
         os.eRes = Math.abs( eResM.get(k));
         if( os.isInUse() )
           rover.eRes += Math.pow( os.eRes, 2); 
       }
       rover.eRes = Math.sqrt(rover.eRes/rover.satsInUse);
       if( goGPS.isDebug()) System.out.println(String.format("eRes = %5.3f\r\n", rover.eRes));
   
       // if correction is small enough, we're done, exit loop
       // TODO check also eRes
       if(( correction_mag< POS_TOL || rover.eRes<POS_TOL ) && Math.abs(cbiasms) < TG_TOL )
         break; 
    } // itr

    double correction_mag = refPos.minusXYZ(rover).normF();
    double tg = (roverObs.getRefTime().getMsec() - refTime)/1000;

    if( Double.isNaN(correction_mag) || correction_mag>goGPS.getPosLimit() ||  Math.abs(tg) > this.getTimeLimit() ){
      
      if(goGPS.isDebug()) System.out.println("Correction exceeds the limits: dist = " + (long)correction_mag +"m; t offset = " + (long)tg +"s" );

      rover.setXYZ(0, 0, 0);
      rover.status = Status.MaxCorrection;

      return null;
    } 
    else {
      if( goGPS.isDebug()) 
        System.out.println( String.format( "recpos: %5.3f, %5.3f, %5.3f, %s, eRes = %5.3f", 
            rover.getGeodeticLatitude(), 
            rover.getGeodeticLongitude(), 
            rover.getGeodeticHeight(), 
            new Time(unixTime).toString(), 
            rover.eRes, cbiasms ));
    }
    
//    /** Observation Errors */
//    SimpleMatrix vEstim = y0.minus(A.mult(x).plus(b));
//    double varianceEstim = (vEstim.transpose().mult(Q.invert())
//        .mult(vEstim)).get(0)
//        / (nObsAvail - nUnknowns);
//   
//    // Covariance matrix of the estimation error
//    if (nObsAvail > nUnknowns) {
//      SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
//      .scale(varianceEstim);
//      positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
//    }else{
//      positionCovariance = null;
//   }

     /** Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates] */
     // Compute covariance matrix from A matrix [ECEF reference system]
//    covXYZ = A.extractMatrix(0, nObsAvail, 0, 3).transpose().mult(A.extractMatrix(0, nObsAvail, 0, 3)).invert();
    SimpleMatrix covXYZ = A.transpose().mult(A).invert().extractMatrix(0, 3, 0, 3);

    // Allocate and build rotation matrix
    SimpleMatrix R = Coordinates.rotationMatrix(rover);

    /** Covariance matrix obtained from matrix A (satellite geometry) [local coordinates] */
    // Propagate covariance from global system to local system
    SimpleMatrix covENU = R.mult(covXYZ).mult(R.transpose());

     //Compute DOP values
    rover.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
    rover.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
    rover.vDop = Math.sqrt(covENU.get(2, 2));
    
//    if( Math.abs(cbiasms)>0.5 )
//      System.out.println( pivotSatId + ") cbiasms " + cbiasms); 
    
    return new SnapshotPivotResult( savedIndex, pivotSatId, pivotElevation, 
        rover, unixTime, rover.eRes, rover.hDop, sats.getAvailNumber(), cbiasms );
  }
  
  /**
   * @param roverObs
   * return computed time offset in milliseconds
   */
  public Long snapshotPos( Observations roverObs ) {
    Double residCutOff = 0.5;
    double elCutOff = 5.0;
    rover.status = Status.None;

    // Number of GPS observations
    int nObs = roverObs.getNumSat();
    if( 2*nObs + 1 < nUnknowns ){
      if(goGPS.isDebug()) System.out.println("Not enough satellites for " + roverObs.getRefTime() );
      rover.setXYZ(0, 0, 0);
      rover.satsInUse = nObs;
      rover.status = Status.NotEnoughSats;
      return null;
    }

    SnapshotPivotResult result = null;
    RoverPosition refPos = new RoverPosition();

    // save this position before trying
    rover.cloneInto(refPos);
    Time refTime = roverObs.getRefTime();
    
    for( int satIdx = 0; satIdx<roverObs.getNumSat(); satIdx++ ){
      if(goGPS.isDebug()) System.out.println( "\r\n===> Try Pivot " + satIdx );
      
      // restore this position before trying
      refPos.cloneInto(rover);
      roverObs.setRefTime(refTime);
      
      rover.status = Status.None;
      // select a pivot with at least elCutOff elevation
      int maxIterations = 3;
//      residCutOff = 0.05;
      SnapshotPivotResult pivotRes = snapshotProcessPivot( roverObs, satIdx, maxIterations, elCutOff, residCutOff  );
      
      if( pivotRes == null && rover.status == Status.EphNotFound ){
        // don't keep requesting Ephemeris if they're not ready yet
        rover.setXYZ(0, 0, 0);
        return null;
      }
      
      if( pivotRes != null  && ( result == null ||  pivotRes.eRes<result.eRes )){
        result = pivotRes;
      }
    }
    if( result == null ){
      rover.setXYZ(0, 0, 0);
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
    refPos.cloneInto(rover);
    roverObs.setRefTime(refTime);
    
    // reprocess with selected pivot and more iterations
    rover.status = Status.None;
    if( result.nObs<5 )
//      residCutOff = 0.0002;
      elCutOff = -5;
      
    result = snapshotProcessPivot( roverObs, result.satIndex, 100, elCutOff, residCutOff );
    if( result == null ){
      rover.setXYZ(0, 0, 0);
      return null;
    }
    
    if( result.eRes > 350 ){
//      if(goGPS.isDebug()) 
        System.out.println("eRes too large = " + rover.eRes );

      rover.setXYZ(0, 0, 0);
      rover.status = Status.MaxEres;

      return null;
    }

    if( result.hDop>this.goGPS.getHdopLimit() ){
      if(goGPS.isDebug()) System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
          rover.getGeodeticLatitude(), 
          rover.getGeodeticLongitude(), 
          rover.getGeodeticHeight(), 
          new Time(result.unixTime).toString() ));
//      if(goGPS.isDebug()) 
        System.out.println( String.format( "hDOP too large: %3.1f", result.hDop ));
      rover.setXYZ(0, 0, 0);
      rover.status = Status.MaxHDOP;
      return null;
    }
    
    rover.status = Status.Valid;
    result.roverPos.cloneInto(rover);
    rover.setRefTime(new Time(result.unixTime));

    if(goGPS.isDebug()) System.out.println( String.format( "recpos: %5.4f, %5.4f, %5.4f, %s", 
        rover.getGeodeticLatitude(), 
        rover.getGeodeticLongitude(), 
        rover.getGeodeticHeight(), 
        new Time(result.unixTime).toString() ));

    long offsetms = result.unixTime - refTime.getMsec();
    
    return offsetms;
  }
  
  Long runOffset( Observations obsR, long offsetms ){
    if( rover == null || obsR == null )
      return null;
  
    if(goGPS.isDebug()) System.out.println( "\r\n>>Try offset = " + offsetms/1000 + " (s)");

    rover.setXYZ( aPrioriPos.getX(), aPrioriPos.getY(), aPrioriPos.getZ() );
    rover.computeGeodetic();
    if(goGPS.isDebug()) System.out.println( "A priori pos: " + aPrioriPos );

    Time refTime = new Time( obsR.getRefTime().getMsec() + offsetms );
    obsR.setRefTime( refTime );
    
    if( !rover.isValidXYZ() && aPrioriPos.isValidXYZ() ){
      aPrioriPos.cloneInto(rover);
    }
    
    Long updatedms = snapshotPos(obsR);
    
    if( updatedms == null && rover.status == Status.MaxCorrection ){
      if(goGPS.isDebug()) System.out.println("Reset aPrioriPos");        
      aPrioriPos.cloneInto(rover);
    }
    
    return updatedms!=null? 
        offsetms + updatedms : 
        null ;
  }

  
  public void tryOffset( Coordinates aPrioriPos, Observations obsR ) throws Exception{
    this.aPrioriPos = aPrioriPos;
    
  //  Long offsetsec = Math.round(offsetms/1000.0);
    Long offsetms = 0l;
  
    Long updatems = runOffset( obsR, offsetms );
    
    if( updatems == null && 
        (rover.status == Status.EphNotFound 
      || rover.status == Status.MaxHDOP 
      || rover.status == Status.MaxEres 
      || rover.status == Status.NotEnoughSats 
      )){
      return;
  }

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 2*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms + 2*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 4*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms + 4*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 6*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms + 6*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 8*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms + 8*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 10*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms + 10*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 12*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 14*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 16*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 18*maxTimeUpdateSec*1000 );

  if(  updatems == null  )
    updatems = runOffset( obsR, offsetms - 20*maxTimeUpdateSec*1000 );

  if( updatems == null )
    rover.setXYZ(0, 0, 0);
}

  
  
}
