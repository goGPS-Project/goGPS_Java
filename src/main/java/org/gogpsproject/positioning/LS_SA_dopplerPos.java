package org.gogpsproject.positioning;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Status;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public class LS_SA_dopplerPos extends LS_SA_code {

  public LS_SA_dopplerPos(GoGPS goGPS) {
    super(goGPS);
  }

  /**
   * Estimate full pseudorange and satellite position from a priori rover position and fractional pseudoranges
   * @param roverObs
   * @param cutoff
   */
  public void selectSatellites( Observations roverObs, double cutoff, final double MODULO ) {
    
    NavigationProducer navigation = goGPS.getNavigation();

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Allocate an array to store GPS satellite positions
    sats.pos = new SatellitePosition[nObs];

    // Allocate an array to store receiver-satellite vectors
    rover.diffSat = new SimpleMatrix[nObs];

    // Allocate an array to store receiver-satellite approximate range
    rover.satAppRange = new double[nObs];

    // Allocate arrays to store receiver-satellite atmospheric corrections
    rover.satTropoCorr = new double[nObs];
    rover.satIonoCorr = new double[nObs];

    // Create a list for available satellites after cutoff
    sats.avail = new ArrayList<Integer>(0);
    sats.typeAvail = new ArrayList<Character>(0);
    sats.gnssAvail = new ArrayList<String>(0);

    // Create a list for available satellites with phase
    sats.availPhase = new ArrayList<Integer>(0);
    sats.typeAvailPhase = new ArrayList<Character>(0);
    sats.gnssAvailPhase = new ArrayList<String>(0);
    
    // Allocate array of topocentric coordinates
    rover.topo = new TopocentricCoordinates[nObs];

    rover.satsInUse = 0;
    // Satellite ID
    int id = 0;

    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    for (int i = 0; i < nObs; i++) {
      id = roverObs.getSatID(i);
      ObservationSet os = roverObs.getSatByID(id);
      char satType = roverObs.getGnssType(i);

      sats.pos[i] = goGPS.getNavigation()
          .getGpsSatPosition( roverObs, id, 'G', rover.getClockError() );
      
      if(  sats.pos[i] == SatellitePosition.UnhealthySat ) {
        sats.pos[i] = null;
        continue;
      }
      
      if( sats.pos[i] == null  || Double.isNaN(sats.pos[i].getX() )) {
//        if(goGPS.isDebug()) System.out.println("Not useful sat "+roverObs.getSatID(i));
        if( i == 0 || sats.avail.size()> 0 )
          continue;
        else {
          rover.status = Status.EphNotFound;
          return;
        }
      }

      // remember cph and restore it later
      double code = os.getCodeC(0);

      // Compute rover-satellite approximate pseudorange
      rover.diffSat[i] = rover.minusXYZ(sats.pos[i]); // negative, for LOS vectors

      rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
                                    + Math.pow(rover.diffSat[i].get(1), 2)
                                    + Math.pow(rover.diffSat[i].get(2), 2));

      // recompute satpos now with estimatedPR
      if( Double.isNaN(rover.satAppRange[i])){
        if( goGPS.isDebug() ) System.out.println("Error NaN");
      }
      else {
        os.setCodeC(0, rover.satAppRange[i]);
  
        // Compute GPS satellite positions getGpsByIdx(idx).getSatType()
        sats.pos[i] = goGPS.getNavigation().getGpsSatPosition(roverObs, id, 'G', rover.getClockError());
        
        // restore code observation
        os.setCodeC(0, code);
  
        if( sats.pos[i] == null  ) {
  //        if(debug) System.out.println("Not useful sat "+roverObs.getSatID(i));
          if( (i== 0) || sats.avail.size()> 0 )
            continue;
          else {
            rover.status = Status.EphNotFound;
            continue;
          }
        }
      }
      // Compute rover-satellite approximate pseudorange
      rover.diffSat[i] = rover.minusXYZ(sats.pos[i]);
      rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
                           + Math.pow(rover.diffSat[i].get(1), 2)
                           + Math.pow(rover.diffSat[i].get(2), 2));

      double R = rover.satAppRange[i]/Constants.SPEED_OF_LIGHT*1000;
      double C = roverObs.getSatByID(id).getCodeC(0)/Constants.SPEED_OF_LIGHT*1000;
      if( goGPS.isDebug() ) System.out.print( String.format( "%2d) SR:%8.5f C:%8.5f D:%9.5f ", 
          id, 
          R%(MODULO*1000/Constants.SPEED_OF_LIGHT), 
          C,
          C-(MODULO*1000/Constants.SPEED_OF_LIGHT)));

      // Compute azimuth, elevation and distance for each satellite
      rover.topo[i] = new TopocentricCoordinates();
      rover.topo[i].computeTopocentric( rover, sats.pos[i]);

      // Correct approximate pseudorange for troposphere
      rover.satTropoCorr[i] = sats.computeTroposphereCorrection(rover.topo[i].getElevation(), rover.getGeodeticHeight());

      // Correct approximate pseudorange for ionosphere
      rover.satIonoCorr[i] = sats.computeIonosphereCorrection(navigation, rover, rover.topo[i].getAzimuth(), rover.topo[i].getElevation(), roverObs.getRefTime());

      if( goGPS.isDebug()) System.out.print( String.format( " El:%4.1f ", rover.topo[i].getElevation() ));

//        System.out.println("getElevation: " + id + "::" + rover.topo[i].getElevation() ); 
      // Check if satellite elevation is higher than cutoff
      if( rover.topo[i].getElevation() >= cutoff ) {
          
        sats.avail.add(id);
        sats.typeAvail.add(satType);
        sats.gnssAvail.add(String.valueOf(satType) + String.valueOf(id));
  
        // Check if also phase is available
        if (!Double.isNaN(roverObs.getSatByIDType(id, 'G').getPhaseCycles(goGPS.getFreq()))) {
          sats.availPhase.add(id);
          sats.typeAvailPhase.add('G');
          sats.gnssAvailPhase.add(String.valueOf('G') + String.valueOf(id));       
         }
      }
      else{
        os.el = rover.topo[i].getElevation();
        if(goGPS.isDebug()) System.out.print( String.format( " Not useful sat %2d  for too low elevation %3.1f < %3.1f", roverObs.getSatID(i), rover.topo[i].getElevation(), cutoff ));
      }
      if( goGPS.isDebug()) System.out.println();
    }
  }
  
  /**
   * A port of FastGPS's doppler positioning algorithm 
   * See http://fastgps.sourceforge.net/
   * spectrum.library.concordia.ca/973909/1/Othieno_MASc_S2012.pdf
   * @param obs
   */
  @Deprecated
  public void dopplerPosHill( Observations obs ) {
    int MINSV = 5;

    // Number of unknown parameters
    int nUnknowns = 4;
    final double DOPP_POS_TOL = 1.0;    

    double max_iterations = 20; 

    for (int itr = 0; itr < max_iterations; itr++) {

      selectSatellites( obs, -20, GoGPS.MODULO1MS ); 
//      sats.selectStandalone(obs, -100);

      // Number of available satellites (i.e. observations)
      int nObsAvail = sats.avail.size();
      if( nObsAvail < MINSV ){
        if( goGPS.isDebug() ) System.out.println("dopplerPos, not enough satellites for " + obs.getRefTime() );
        if( rover.status == Status.None ){
          rover.status = Status.NotEnoughSats;
        }
        rover.setXYZ(0, 0, 0);
        return;
      }
      
//      nObsAvail++; // add DTM / height soft constraint

      /** range rate */
      double[] rodot = new double[nObsAvail];

      /** Least squares design matrix */
      SimpleMatrix A = new SimpleMatrix( nObsAvail, nUnknowns );

      // Set up the least squares matrices
      SimpleMatrix b = new SimpleMatrix( nObsAvail, 1 );

      double pivotSNR = 0;
      double pivot = 0;
      for (int i = 0, k = 0; i < obs.getNumSat(); i++) {
        int satId = obs.getSatID(i);

        if( sats.pos[i] == null  || !sats.avail.contains(satId) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
          continue;
        }

        ObservationSet os = obs.getSatByID(satId);

        // scalar product of speed vector X unit vector
        float doppler = os.getDoppler(ObservationSet.L1);

        // Line Of Sight vector units (ECEF)
        SimpleMatrix e = new SimpleMatrix(1,3);
        e.set( 0,0, rover.diffSat[i].get(0) / rover.satAppRange[i] );
        e.set( 0,1, rover.diffSat[i].get(1) / rover.satAppRange[i] );
        e.set( 0,2, rover.diffSat[i].get(2) / rover.satAppRange[i] );
        double rodotSatSpeed   = -e.mult( sats.pos[i].getSpeed() ).get(0);
        double dopplerSatSpeed = -rodotSatSpeed*Constants.FL1/Constants.SPEED_OF_LIGHT;

        if( Float.isNaN( doppler )){
          rodot[k] = rodotSatSpeed;
        }
        else {
          rodot[k] = doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;
          
          os.getDoppler(ObservationSet.L1);
          System.out.println( String.format( "%2d) snr:%2.0f doppler:%6.0f; satSpeed:%6.0f; D:%6.0f", 
              satId,
              os.getSignalStrength(ObservationSet.L1),
              doppler, 
              dopplerSatSpeed,
              doppler - dopplerSatSpeed ));
        }
        
        // build A matrix
        A.set(k, 0, sats.pos[i].getSpeed().get(0) ); /* X */
        A.set(k, 1, sats.pos[i].getSpeed().get(1) ); /* Y */
        A.set(k, 2, sats.pos[i].getSpeed().get(2) ); /* Z */
        
        double satpos_norm = Math.sqrt(Math.pow(sats.pos[i].getX(), 2)
                                     + Math.pow(sats.pos[i].getY(), 2)
                                     + Math.pow(sats.pos[i].getZ(), 2));
        A.set( k, 3, satpos_norm ); 

        SimpleMatrix tempv = rover.minusXYZ(sats.pos[i]);

        /** range */
        double ro = Math.sqrt(Math.pow(tempv.get(0), 2)
                  + Math.pow(tempv.get(1), 2)
                  + Math.pow(tempv.get(2), 2));

        SimpleMatrix satposxyz = new SimpleMatrix(1,3);
        satposxyz.set(0, 0, sats.pos[i].getX());
        satposxyz.set(0, 1, sats.pos[i].getY());
        satposxyz.set(0, 2, sats.pos[i].getZ());

        SimpleMatrix satvelxyz = new SimpleMatrix(1,3);
        satvelxyz.set(0, 0, sats.pos[i].getSpeed().get(0));
        satvelxyz.set(0, 1, sats.pos[i].getSpeed().get(1));
        satvelxyz.set(0, 2, sats.pos[i].getSpeed().get(2));

        /** satpos times satspeed*/
        double posvel = satposxyz.mult(satvelxyz.transpose()).get(0,0);

        // B[j] = posvel + rodot[j]*ro + ClockErrorRate*(satpos_norm-ro);
        double bval = posvel + rodot[k]*ro + rover.getClockErrorRate()*( satpos_norm - ro );
        b.set(k, 0, bval);

        double snr = os.getSignalStrength(ObservationSet.L1);
        if( snr > pivotSNR ){
          pivotSNR = snr;
          pivot = Math.abs(bval);
        }
        k++;
     }
    
     SimpleMatrix x = A.transpose().mult(A).invert().mult(A.transpose()).mult(b);

     System.out.println( String.format( "Update %d: x: %3.3f, y: %3.3f, z: %3.3f, br: %3.3f", itr, 
                                                    x.get(0), x.get(1), x.get(2), x.get(3) ));

     double correction_mag = Math.sqrt( Math.pow( x.get(0) - rover.getX(), 2 ) + 
                                        Math.pow( x.get(1) - rover.getY(), 2 ) +
                                        Math.pow( x.get(2) - rover.getZ(), 2 ) );

     // expected
     System.out.println( String.format( "pos diff mag %f (m)", correction_mag ));

     // Update receiver clock error rate
     rover.clockErrorRate = x.get(3);

     // Update Rx position estimate
     rover.setXYZ( x.get(0), 
                   x.get(1), 
                   x.get(2));

     rover.computeGeodetic();
     
     // clamp it to the ground, not very elegant
//     if( rover.getGeodeticHeight()<30 || rover.getGeodeticHeight() > 100 ){
//       rover.setGeod( rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), 30 );
//       rover.computeECEF();
//     }

     System.out.println( "recpos (" + itr +")");
     System.out.println( String.format( "%10.6f,%10.6f,%10.6f", 
         rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight() ));
     System.out.println();
     
     // if correction is small enough, we're done, exit loop
     if( correction_mag< DOPP_POS_TOL )
       break;
    }
    System.out.println( rover );
  }

  /**
   * A simpler version, based on Van Diggelen (8.3.2.1)
   * This system returns a position update so it can be plugged in the standard design matrix
   * @param obs
   */
  public void dopplerPos( Observations obs ) {
    int nUnknowns = 4;
    final double DOPP_POS_TOL = 1.0;    

    double max_iterations = 20; 

    /** Least squares design matrix */
    SimpleMatrix A = null;

    for (int itr = 0; itr < max_iterations; itr++) {

      selectSatellites( obs, -100, GoGPS.MODULO1MS ); 
//      sats.selectStandalone(obs, -100);

      // Number of available satellites (i.e. observations)
      int nObsAvail = sats.avail.size() + 1; // add DTM / height soft constraint

      if( nObsAvail < nUnknowns ){
//        if( goGPS.isDebug() ) 
        System.out.println("dopplerPos, not enough satellites for " + obs.getRefTime() );
        if( rover.status == Status.None ){
          rover.status = Status.NotEnoughSats;
        }
        rover.setXYZ(0, 0, 0);
        return;
      }
      
      A = new SimpleMatrix( nObsAvail, nUnknowns );

      // Set up the least squares matrices
      SimpleMatrix b = new SimpleMatrix( nObsAvail, 1 );

      for (int i = 0, k = 0; i < obs.getNumSat(); i++) {
        int satId = obs.getSatID(i);

        if( sats.pos[i] == null  || !sats.avail.contains(satId) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
          continue;
        }

        ObservationSet os = obs.getSatByID(satId);

        A.set( k, 0, sats.pos[i].getSpeed().get(0)/rover.satAppRange[i] ); /* VX */
        A.set( k, 1, sats.pos[i].getSpeed().get(1)/rover.satAppRange[i] ); /* VY */
        A.set( k, 2, sats.pos[i].getSpeed().get(2)/rover.satAppRange[i] ); /* VZ */
        A.set( k, 3, 1 ); // clock error rate

        // Line Of Sight vector units (ECEF)
        SimpleMatrix e = new SimpleMatrix(1,3);
        e.set( 0,0, rover.diffSat[i].get(0) / rover.satAppRange[i] );
        e.set( 0,1, rover.diffSat[i].get(1) / rover.satAppRange[i] );
        e.set( 0,2, rover.diffSat[i].get(2) / rover.satAppRange[i] );

        /** computed satspeed: scalar product of speed vector X LOS unit vector */
        double rodotSatSpeed   = e.mult( sats.pos[i].getSpeed() ).get(0);
        
        float doppler = os.getDoppler(ObservationSet.L1);

        /** observed range rate */
        double rodot = doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;
        
        /** residuals */
        b.set(k, 0, rodot  - rodotSatSpeed - rover.getClockErrorRate() );

        k++;
     }
     
     // Add height soft constraint
     double hR_app = rover.getGeodeticHeight();
     double h_dtm = hR_app>0? 
        hR_app 
        : 30; // initialize to something above sea level
     if( h_dtm > 2000 )
      h_dtm = 2000;
    
     double lam = Math.toRadians(rover.getGeodeticLongitude());
     double phi = Math.toRadians(rover.getGeodeticLatitude());
     
     double cosLam = Math.cos(lam);
     double cosPhi = Math.cos(phi);
     double sinLam = Math.sin(lam);
     double sinPhi = Math.sin(phi);
    
     int k = nObsAvail-1;
     A.set(k, 0, cosPhi * cosLam );
     A.set(k, 1, cosPhi * sinLam ); 
     A.set(k, 2, sinPhi ); 
     A.set(k, 3, 0 ); 
     double y0_dtm = h_dtm  - hR_app;
     b.set(k, 0, y0_dtm );
      
     SimpleMatrix x = A.transpose().mult(A).invert().mult(A.transpose()).mult(b);

     System.out.println( String.format( "Update %d: x: %3.3f, y: %3.3f, z: %3.3f, cr: %3.3f", itr, 
                                                    x.get(0), x.get(1), x.get(2), x.get(3) ));

     double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                        Math.pow( x.get(1), 2 ) +
                                        Math.pow( x.get(2), 2 ) );

     // expected
     System.out.println( String.format( "pos diff mag %f (m)", correction_mag ));

     // Update Rx position estimate
     rover.setPlusXYZ( x.extractMatrix(0, 3, 0, 1) );
     rover.computeGeodetic();
     
     // Update receiver clock error rate
     rover.clockErrorRate += x.get(3);

     System.out.println( "recpos (" + itr +")");
     System.out.println( String.format( "%10.6f,%10.6f,%10.6f cr:%10.6f", 
                         rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), rover.getGeodeticHeight(),
                         rover.clockErrorRate ));
     System.out.println();
     
     // if correction is small enough, we're done, exit loop
     if( correction_mag< DOPP_POS_TOL )
       break;
    }
    
    updateDops(A);
    
    System.out.println( rover );
  }
}

