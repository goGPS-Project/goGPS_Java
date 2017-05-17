package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Status;

public class LS_SA_dopplerPos extends Core {

  public LS_SA_dopplerPos(GoGPS goGPS) {
    super(goGPS);
  }

  public void dopplerPos( Observations obs ) {
    int MINSV = 5;

    // Number of unknown parameters
    int nUnknowns = 4;
    final double DOPP_POS_TOL = 1.0;    

    // Number of available satellites (i.e. observations)
    int nObsAvail = satAvail.size();
    if( nObsAvail < MINSV ){
      if( goGPS.isDebug() ) System.out.println("dopplerPos, not enough satellites for " + obs.getRefTime() );
      if( rover.status == Status.None ){
        rover.status = Status.NotEnoughSats;
      }
      rover.setXYZ(0, 0, 0);
      return;
    }
    
    double[] rodot = new double[nObsAvail];

//    RinexNavigation navigation = (RinexNavigation) goGPS.getNavigation();
//    long unixTime = obs.getRefTime().getMsec();
    
    double max_iterations = 20; 

    // Least squares design matrix
    SimpleMatrix A = new SimpleMatrix( nObsAvail, nUnknowns );

    // Set up the least squares matrices
    for (int i = 0; i < nObsAvail; i++) {

      int satId = obs.getSatID(i);

      if( pos[i] == null  || !satAvail.contains(satId) ) {//|| recpos.ecef==null || pos[i].ecef==null ){
//              l.warning( "ERROR, pos[i]==null?" );
//              this.setXYZ(0, 0, 0);
//              return null;
        continue;
      }

      ObservationSet os = obs.getSatByID(satId);

//      EphGps eph = navigation.findEph( unixTime, satId, 'G' );
//      if( eph == null ){
//        l.warning( "SvID " + satId + " eph is null at " + unixTime );
//        continue;
//      }
//      if( eph.getSvHealth() != 0 ){
//        l.warning( "SvID " + satId + " health: " + eph.getSvHealth() + " at " + unixTime );
//        continue;
//      }

      float doppler = os.getDoppler(ObservationSet.L1);
      rodot[i] = doppler * Constants.SPEED_OF_LIGHT/Constants.FL1;

      // build A matrix
      A.set(i, 0, pos[i].getSpeed().get(0) ); /* X */
      A.set(i, 1, pos[i].getSpeed().get(1) ); /* Y */
      A.set(i, 2, pos[i].getSpeed().get(2) ); /* Z */
      
      double satpos_norm = Math.sqrt(Math.pow(pos[i].getX(), 2)
                                   + Math.pow(pos[i].getY(), 2)
                                   + Math.pow(pos[i].getZ(), 2));
      A.set(i, 3, satpos_norm ); 
    }
    
    for (int itr = 0; itr < max_iterations; itr++) {

      SimpleMatrix b = new SimpleMatrix( nObsAvail, 1 );

      double pivot = Double.MAX_VALUE;

      for (int i = 0; i<nObsAvail; i++) {
        int satId = obs.getSatID(i);
        if( pos[i] == null  || !satAvail.contains(satId) ) {//|| recpos.ecef==null || pos[i].ecef==null ){
        //        l.warning( "ERROR, pos[i]==null?" );
        //        this.setXYZ(0, 0, 0);
        //        return null;
          continue;
      }
        
        SimpleMatrix tempv = rover.minusXYZ(pos[i]);

        double Ym = Math.sqrt(Math.pow(tempv.get(0), 2)
            + Math.pow(tempv.get(1), 2)
            + Math.pow(tempv.get(2), 2));

        SimpleMatrix satposxyz = new SimpleMatrix(1,3);
        satposxyz.set(0, 0, pos[i].getX());
        satposxyz.set(0, 1, pos[i].getY());
        satposxyz.set(0, 2, pos[i].getZ());

        SimpleMatrix satvelxyz = new SimpleMatrix(1,3);
        satvelxyz.set(0, 0, pos[i].getSpeed().get(0));
        satvelxyz.set(0, 1, pos[i].getSpeed().get(1));
        satvelxyz.set(0, 2, pos[i].getSpeed().get(2));

        double tempd = satposxyz.mult(satvelxyz.transpose()).get(0,0);

        // B[j] = tempd+W[j]*Ym + Xc[3]*(A[j][3]-Ym);
        double val = tempd + rodot[i]*Ym + rover.getReceiverClockErrorRate()*( A.get(i, 3) - Ym);
        b.set(i, 0, val);
        if( Math.abs(val)<pivot)
          pivot = Math.abs(val);
     }
///////////////
//     b.normF()
     System.out.println( String.format( "Residuals -> Adjusted Residuals (ms) - Pivot = %7.4f (ms)",  pivot/Constants.SPEED_OF_LIGHT*1000));
     for( int k=0; k<satAvail.size(); k++){
//       int satId = obs.getSatID(k);
//       ObservationSet os = obs.getSatByID(satId);
       double d = Math.abs(b.get(k))-pivot;
//       System.out.print( String.format( "%7.4f\r\n", d/Constants.SPEED_OF_LIGHT*1000));
       System.out.print( String.format( "%7.4f\r\n", d/Constants.SPEED_OF_LIGHT*1000));
     }
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
     rover.setXYZ( x.get(0), 
                    x.get(1), 
                    x.get(2));

     rover.computeGeodetic();

     // Update receiver clock error rate
     rover.receiverClockErrorRate = x.get(3);
     
     System.out.println( "recpos (" + itr +")");
     System.out.println( String.format( "%10.6f,%10.6f", rover.getGeodeticLatitude(), rover.getGeodeticLongitude() ));
     System.out.println();
     
     // if correction is small enough, we're done, exit loop
     if( correction_mag< DOPP_POS_TOL )
       break;
    }
    
    // Compute positioning in geodetic coordinates
    rover.computeGeodetic();
    
    // clamp it to the ground, not very elegant
    if( rover.getGeodeticHeight()<30 || rover.getGeodeticHeight() > 100 ){
      rover.setGeod( rover.getGeodeticLatitude(), rover.getGeodeticLongitude(), 30 );
      rover.computeECEF();
    }
  }
}

