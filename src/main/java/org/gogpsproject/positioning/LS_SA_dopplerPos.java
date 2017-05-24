package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Status;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

public class LS_SA_dopplerPos extends LS_SA_code {

  public LS_SA_dopplerPos(GoGPS goGPS) {
    super(goGPS);
  }

  public void dopplerPos( Observations obs ) {
    int MINSV = 5;

    // Number of unknown parameters
    int nUnknowns = 4;
    final double DOPP_POS_TOL = 1.0;    

    double max_iterations = 20; 

    for (int itr = 0; itr < max_iterations; itr++) {

      //    sats.selectSatellitesStandaloneFractional(obs, -100);
      sats.selectStandalone(obs, -100);

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
      
      /** range rate */
      double[] rodot = new double[nObsAvail];

      /** Least squares design matrix */
      SimpleMatrix A = new SimpleMatrix( nObsAvail, nUnknowns );

      // Set up the least squares matrices
      for (int i = 0; i < nObsAvail; i++) {

        int satId = obs.getSatID(i);

        if( sats.pos[i] == null  || !sats.avail.contains(satId) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
          continue;
        }

        ObservationSet os = obs.getSatByID(satId);

        // scalar product of speed vector X unit vector
        float doppler = os.getDoppler(ObservationSet.L1);

        ////
        // Line Of Sight vector units (ECEF)
        SimpleMatrix e = new SimpleMatrix(1,3);
        e.set( 0,0, rover.diffSat[i].get(0) / rover.satAppRange[i] );
        e.set( 0,1, rover.diffSat[i].get(1) / rover.satAppRange[i] );
        e.set( 0,2, rover.diffSat[i].get(2) / rover.satAppRange[i] );
        double rodotSatSpeed   = -e.mult( sats.pos[i].getSpeed() ).get(0);
        double dopplerSatSpeed = -rodotSatSpeed*Constants.FL1/Constants.SPEED_OF_LIGHT;
        ///

        if( Float.isNaN( doppler )){
          // Line Of Sight vector units (ECEF)
          rodot[i] = rodotSatSpeed;
        }
        else {
          // scalar product of speed vector X unit vector
          rodot[i] = doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;
          
//          if( Math.abs(doppler - dopplerSatSpeed)>200)
//            System.out.println("diff here");
//          rodot[i] = rodotSatSpeed;
          os.getDoppler(ObservationSet.L1);
          System.out.println( String.format( "%2d) snr:%2.0f doppler:%6.0f; satSpeed:%6.0f; D:%6.0f", 
              satId,
              os.getSignalStrength(ObservationSet.L1),
              doppler, 
              dopplerSatSpeed,
              doppler - dopplerSatSpeed ));
        }
        
        // build A matrix
        A.set(i, 0, sats.pos[i].getSpeed().get(0) ); /* X */
        A.set(i, 1, sats.pos[i].getSpeed().get(1) ); /* Y */
        A.set(i, 2, sats.pos[i].getSpeed().get(2) ); /* Z */
        
        double satpos_norm = Math.sqrt(Math.pow(sats.pos[i].getX(), 2)
                                     + Math.pow(sats.pos[i].getY(), 2)
                                     + Math.pow(sats.pos[i].getZ(), 2));
        A.set(i, 3, satpos_norm ); 
      }
      
      SimpleMatrix b = new SimpleMatrix( nObsAvail, 1 );

      double pivotSNR = 0;
      double pivot = 0;

      for (int i = 0; i<nObsAvail; i++) {
        int satId = obs.getSatID(i);
        if( sats.pos[i] == null  || !sats.avail.contains(satId) ) {//|| recpos.ecef==null || sats.pos[i].ecef==null ){
        //        l.warning( "ERROR, sats.pos[i]==null?" );
        //        this.setXYZ(0, 0, 0);
        //        return null;
          continue;
      }
        
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

        /** sat pos times speed*/
        double posvel = satposxyz.mult(satvelxyz.transpose()).get(0,0);

        // B[j] = posvel + rodot[j]*ro + Xc[3]*(A[j][3]-ro);
        double satpos_norm = A.get(i, 3);
        double bval = posvel + rodot[i]*ro + rover.getReceiverClockErrorRate()*( satpos_norm - ro);
        b.set(i, 0, bval);

        ObservationSet os = obs.getSatByIdx(i);
        double snr = os.getSignalStrength(ObservationSet.L1);
        if( snr > pivotSNR ){
          pivotSNR = snr;
          pivot = Math.abs(bval);
        }
     }
///////////////
//     b.normF()
//     System.out.println( String.format( "Residuals -> Adjusted Residuals (ms) - Pivot = %7.4f (ms)",  pivot/Constants.SPEED_OF_LIGHT*1000));
//     for( int k=0; k<sats.avail.size(); k++){
//       double d = Math.abs(b.get(k))-pivot;
//       System.out.print( String.format( "%7.4f\r\n", d/Constants.SPEED_OF_LIGHT*1000));
//     }
///////////////     
      
     SimpleMatrix x = A.transpose().mult(A).invert().mult(A.transpose()).mult(b);

     System.out.println( String.format( "Update %d: x: %3.3f, y: %3.3f, z: %3.3f, br: %3.3f", itr, 
         x.get(0), x.get(1), x.get(2), x.get(3) ));

     double correction_mag = Math.sqrt( Math.pow( x.get(0) - rover.getX(), 2 ) + 
                                        Math.pow( x.get(1) - rover.getY(), 2 ) +
                                        Math.pow( x.get(2) - rover.getZ(), 2 ) );

     // expected
     System.out.println( String.format( "pos diff mag %f (m)", correction_mag ));

     // Update Rx position estimate
//   rover.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));
     rover.setXYZ( x.get(0), 
                   x.get(1), 
                   x.get(2));

     rover.computeGeodetic();
     // clamp it to the ground, not very elegant
     if( rover.getGeodeticHeight()<30 || rover.getGeodeticHeight() > 100 ){
       rover.setGeod( rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), 30 );
       rover.computeECEF();
     }

     // Update receiver clock error rate
     rover.receiverClockErrorRate = x.get(3);
//   rover.receiverClockErrorRate += x.get(3);
     
     System.out.println( "recpos (" + itr +")");
     System.out.println( String.format( "%10.6f,%10.6f", rover.getGeodeticLatitude(), rover.getGeodeticLongitude() ));
     System.out.println();
     
     // if correction is small enough, we're done, exit loop
     if( correction_mag< DOPP_POS_TOL )
       break;
    }
    
  }
}

