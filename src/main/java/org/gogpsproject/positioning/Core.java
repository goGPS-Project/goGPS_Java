package org.gogpsproject.positioning;

import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.Coordinates;
import org.gogpsproject.GoGPS;
import org.gogpsproject.IonoGps;
import org.gogpsproject.NavigationProducer;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.SatellitePosition;
import org.gogpsproject.Status;
import org.gogpsproject.Time;
import org.gogpsproject.TopocentricCoordinates;

//import com.google.maps.ElevationApi;
//import com.google.maps.GeoApiContext;
//import com.google.maps.model.ElevationResult;
//import com.google.maps.model.LatLng;

public class Core {

  GoGPS goGPS;
  ReceiverPosition rover;
  MasterPosition master;
  Satellites sats;
  
  /** Covariance matrix of the position estimation error */
  SimpleMatrix positionCovariance; 

//static GeoApiContext context;
//static GeoApiContext getContext(){
//  if( context == null )
//    context = new GeoApiContext().setApiKey("...Add your key here");
//  return context;
//}
  public Core( GoGPS goGPS) {
    this.goGPS  = goGPS;
    this.rover  = goGPS.getReceiverPosition();
    this.master = goGPS.getMasterPosition();
    this.sats   = goGPS.getSatellites();
  }

  /**
   * @param x
   * @param y
   * @return Lorentz inner product
   */
  static double lorentzInnerProduct(SimpleMatrix x, SimpleMatrix y) {

    double prod = x.get(0) * y.get(0) + x.get(1) * y.get(1) + x.get(2) * y.get(2) - x.get(3) * y.get(3);

    return prod;
  }
  
  /**
   * @param roverObs
   */
  public void selectSatellitesStandalone( Observations roverObs) {
    // Retrieve options from goGPS class
    double cutoff = goGPS.getCutoff();
    
    selectSatellitesStandalone( roverObs, cutoff);
  }

  /**
   * @param elevation
   * @param snr
   * @return weight computed according to the variable "goGPS.weights"
   */
  double computeWeight(double elevation, float snr) {

    double weight = 1;
    float Sa = Constants.SNR_a;
    float SA = Constants.SNR_A;
    float S0 = Constants.SNR_0;
    float S1 = Constants.SNR_1;

    if (Float.isNaN(snr) && (goGPS.getWeights() == GoGPS.WEIGHT_SIGNAL_TO_NOISE_RATIO ||
        goGPS.getWeights() == GoGPS.WEIGHT_COMBINED_ELEVATION_SNR)) {
      if(goGPS.isDebug()) System.out.println("SNR not available: forcing satellite elevation-based weights...");
      goGPS.setWeights(GoGPS.WEIGHT_SAT_ELEVATION);
    }

    switch (goGPS.getWeights()) {

      // Weight based on satellite elevation
      case GoGPS.WEIGHT_SAT_ELEVATION:
        weight = 1 / Math.pow(Math.sin(elevation * Math.PI / 180), 2);
        break;

      // Weight based on signal-to-noise ratio
      case GoGPS.WEIGHT_SIGNAL_TO_NOISE_RATIO:
        if (snr >= S1) {
          weight = 1;
        } else {
          weight = Math.pow(10, -(snr - S1) / Sa)
              * ((SA / Math.pow(10, -(S0 - S1) / Sa) - 1) / (S0 - S1)
                  * (snr - S1) + 1);
        }
        break;

      // Weight based on combined elevation and signal-to-noise ratio
      case GoGPS.WEIGHT_COMBINED_ELEVATION_SNR:
        if (snr >= S1) {
          weight = 1;
        } else {
          double weightEl = 1 / Math.pow(Math.sin(elevation * Math.PI / 180), 2);
          double weightSnr = Math.pow(10, -(snr - S1) / Sa)
              * ((SA / Math.pow(10, -(S0 - S1) / Sa) - 1) / (S0 - S1) * (snr - S1) + 1);
          weight = weightEl * weightSnr;
        }
        break;

      // Same weight for all observations or default
      case GoGPS.WEIGHT_EQUAL:
      default:
        weight = 1;
    }

    return weight;
  }

  /**
   * @param elevation
   * @param height
   * @return troposphere correction value by Saastamoinen model
   */
  double computeTroposphereCorrection(double elevation, double height) {

    double tropoCorr = 0;

    if (height < 5000) {

      elevation = Math.toRadians(Math.abs(elevation));
      if (elevation == 0){
        elevation = elevation + 0.01;
      }

      // Numerical constants and tables for Saastamoinen algorithm
      // (troposphere correction)
      double hr = 50.0;
      int[] ha = new int[9];
      double[] ba = new double[9];

      ha[0] = 0;
      ha[1] = 500;
      ha[2] = 1000;
      ha[3] = 1500;
      ha[4] = 2000;
      ha[5] = 2500;
      ha[6] = 3000;
      ha[7] = 4000;
      ha[8] = 5000;

      ba[0] = 1.156;
      ba[1] = 1.079;
      ba[2] = 1.006;
      ba[3] = 0.938;
      ba[4] = 0.874;
      ba[5] = 0.813;
      ba[6] = 0.757;
      ba[7] = 0.654;
      ba[8] = 0.563;

      // Saastamoinen algorithm
      double P = Constants.STANDARD_PRESSURE * Math.pow((1 - 0.0000226 * height), 5.225);
      double T = Constants.STANDARD_TEMPERATURE - 0.0065 * height;
      double H = hr * Math.exp(-0.0006396 * height);

      // If height is below zero, keep the maximum correction value
      double B = ba[0];
      // Otherwise, interpolate the tables
      if (height >= 0) {
        int i = 1;
        while (height > ha[i]) {
          i++;
        }
        double m = (ba[i] - ba[i - 1]) / (ha[i] - ha[i - 1]);
        B = ba[i - 1] + m * (height - ha[i - 1]);
      }

      double e = 0.01
          * H
          * Math.exp(-37.2465 + 0.213166 * T - 0.000256908
              * Math.pow(T, 2));

      tropoCorr = ((0.002277 / Math.sin(elevation))
          * (P - (B / Math.pow(Math.tan(elevation), 2))) + (0.002277 / Math.sin(elevation))
          * (1255 / T + 0.05) * e);
    }

    return tropoCorr;
  }

  /**
   * @param ionoParams
   * @param coord
   * @param time
   * @return ionosphere correction value by Klobuchar model
   */
  double computeIonosphereCorrection(NavigationProducer navigation,
      Coordinates coord, double azimuth, double elevation, Time time) {

    double ionoCorr = 0;

    IonoGps iono = navigation.getIono(time.getMsec());
    if(iono==null) return 0.0;
//    double a0 = navigation.getIono(time.getMsec(),0);
//    double a1 = navigation.getIono(time.getMsec(),1);
//    double a2 = navigation.getIono(time.getMsec(),2);
//    double a3 = navigation.getIono(time.getMsec(),3);
//    double b0 = navigation.getIono(time.getMsec(),4);
//    double b1 = navigation.getIono(time.getMsec(),5);
//    double b2 = navigation.getIono(time.getMsec(),6);
//    double b3 = navigation.getIono(time.getMsec(),7);

    elevation = Math.abs(elevation);

    // Parameter conversion to semicircles
    double lon = coord.getGeodeticLongitude() / 180; // geod.get(0)
    double lat = coord.getGeodeticLatitude() / 180; //geod.get(1)
    azimuth = azimuth / 180;
    elevation = elevation / 180;

    // Klobuchar algorithm
    double f = 1 + 16 * Math.pow((0.53 - elevation), 3);
    double psi = 0.0137 / (elevation + 0.11) - 0.022;
    double phi = lat + psi * Math.cos(azimuth * Math.PI);
    if (phi > 0.416){
      phi = 0.416;
    }
    if (phi < -0.416){
      phi = -0.416;
    }
    double lambda = lon + (psi * Math.sin(azimuth * Math.PI))
        / Math.cos(phi * Math.PI);
    double ro = phi + 0.064 * Math.cos((lambda - 1.617) * Math.PI);
    double t = lambda * 43200 + time.getGpsTime();
    while (t >= 86400)
      t = t - 86400;
    while (t < 0)
      t = t + 86400;
    double p = iono.getBeta(0) + iono.getBeta(1) * ro + iono.getBeta(2) * Math.pow(ro, 2) + iono.getBeta(3) * Math.pow(ro, 3);

    if (p < 72000)
      p = 72000;
    double a = iono.getAlpha(0) + iono.getAlpha(1) * ro + iono.getAlpha(2) * Math.pow(ro, 2) + iono.getAlpha(3) * Math.pow(ro, 3);
    if (a < 0)
      a = 0;
    double x = (2 * Math.PI * (t - 50400)) / p;
    if (Math.abs(x) < 1.57){
      ionoCorr = Constants.SPEED_OF_LIGHT
          * f
          * (5e-9 + a
              * (1 - (Math.pow(x, 2)) / 2 + (Math.pow(x, 4)) / 24));
    }else{
      ionoCorr = Constants.SPEED_OF_LIGHT * f * 5e-9;
    }
    return ionoCorr;
  }
  
  /**
   * @param roverObs
   * @param cutoff
   */
  public void selectSatellitesStandalone( Observations roverObs, double cutoff) {

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

    // Satellite ID
    int id = 0;

    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);

      // Compute GPS satellite positions getGpsByIdx(idx).getSatType()
      sats.pos[i] = navigation.getGpsSatPosition( roverObs, id, satType, rover.getReceiverClockError());
      
      if(sats.pos[i]!=null){

        // Compute rover-satellite approximate pseudorange
        rover.diffSat[i] = rover.minusXYZ(sats.pos[i]);
        rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
            + Math.pow(rover.diffSat[i].get(1), 2)
            + Math.pow(rover.diffSat[i].get(2), 2));

        // Compute azimuth, elevation and distance for each satellite
        rover.topo[i] = new TopocentricCoordinates();
        rover.topo[i].computeTopocentric(rover, sats.pos[i]);

        // Correct approximate pseudorange for troposphere
        rover.satTropoCorr[i] = computeTroposphereCorrection(rover.topo[i].getElevation(), rover.getGeodeticHeight());

        // Correct approximate pseudorange for ionosphere
        rover.satIonoCorr[i] = computeIonosphereCorrection(navigation, rover, rover.topo[i].getAzimuth(), rover.topo[i].getElevation(), roverObs.getRefTime());

//        System.out.println("getElevation: " + id + "::" + rover.topo[i].getElevation() ); 
        // Check if satellite elevation is higher than cutoff
        if (rover.topo[i].getElevation() > cutoff) {
          
          sats.avail.add(id);
          sats.typeAvail.add(satType);
          sats.gnssAvail.add(String.valueOf(satType) + String.valueOf(id));

          // Check if also phase is available
          if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {
            sats.availPhase.add(id);
            sats.typeAvailPhase.add(satType);
            sats.gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));       
            
          }
        }else{
          if(goGPS.isDebug()) System.out.println("Not useful sat "+roverObs.getSatID(i)+" for too low elevation "+rover.topo[i].getElevation()+" < "+cutoff);
        }
      }
      
    }
  }

  /**
   * Update roverPos to that observed satellites become visible
   * @param roverObs
   * @return 
   */
  public double selectSatellitesStandalonePositionUpdate(Observations roverObs) {
    
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

    // Least squares design matrix
    SimpleMatrix A = new SimpleMatrix( nObs+1, 3 );
    SimpleMatrix y0 = new SimpleMatrix( nObs+1, 1 );
    SimpleMatrix x = new SimpleMatrix(3, 1);

    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    System.out.println("Satellite Elevation");
    System.out.println( roverObs.getRefTime() );
    for (int i = 0; i < nObs; i++) {
      id = roverObs.getSatID(i);
      ObservationSet os = roverObs.getSatByID(id);
      char satType = roverObs.getGnssType(i);

      sats.pos[i] = goGPS.getNavigation().getGpsSatPosition( roverObs, id, 'G', 0 );

      if( sats.pos[i] == null  || Double.isNaN(sats.pos[i].getX() )) {
        rover.status = Status.EphNotFound;
        continue;
      }

      // Compute rover-satellite approximate pseudorange
      rover.diffSat[i]     = rover.minusXYZ(sats.pos[i]); // negative, for LOS vectors

      rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
                                    + Math.pow(rover.diffSat[i].get(1), 2)
                                    + Math.pow(rover.diffSat[i].get(2), 2));

      // Compute azimuth, elevation and distance for each satellite
      rover.topo[i] = new TopocentricCoordinates();
      rover.topo[i].computeTopocentric(rover, sats.pos[i]);

      double el = rover.topo[i].getElevation();
      
      // Correct approximate pseudorange for troposphere
      rover.satTropoCorr[i] = computeTroposphereCorrection(rover.topo[i].getElevation(), rover.getGeodeticHeight());

      // Correct approximate pseudorange for ionosphere
      rover.satIonoCorr[i] = computeIonosphereCorrection(navigation, rover, rover.topo[i].getAzimuth(), rover.topo[i].getElevation(), roverObs.getRefTime());

      sats.avail.add(id);
      sats.typeAvail.add(satType);
      sats.gnssAvail.add(String.valueOf(satType) + String.valueOf(id));

      SimpleMatrix R = Coordinates.rotationMatrix(rover);
      SimpleMatrix enu = R.mult(sats.pos[i].minusXYZ(rover));
      
      double U = enu.get(2);
      System.out.println( String.format( "%2d) C:%12.3f %5.1f(dg) %9.0f(up)", 
          id, 
          roverObs.getSatByID(id).getCodeC(0),
          el, 
          U));
/*
      System.out.println( String.format( "%2d) SR:%8.5f C:%8.5f D:%9.5f", 
          id, 
          R, 
          C,
          C-R));
 */
      
      A.set(i, 0, rover.diffSat[i].get(0) / rover.satAppRange[i]); /* X */
      A.set(i, 1, rover.diffSat[i].get(1) / rover.satAppRange[i]); /* Y */
      A.set(i, 2, rover.diffSat[i].get(2) / rover.satAppRange[i]); /* Z */

      if( U>0){
        y0.set(i, 0, 0);
      }
      else {
        y0.set(i, 0, U);
//        y0.set(i, 0, Constants.EARTH_RADIUS);
      }
    }
    System.out.println("");

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

    // it's like row[2], UP, of rotationMatrix(this)
    A.set(nObs, 0, cosPhi * cosLam );
    A.set(nObs, 1, cosPhi * sinLam ); 
    A.set(nObs, 2, sinPhi ); 

//    %Y0 vector computation for DTM constraint
    double y0_dtm = h_dtm  - hR_app;
    y0.set(nObs, 0, y0_dtm );
    
    x = A.transpose().mult(A).invert().mult(A.transpose()).mult(y0);

   double correction_mag = Math.sqrt( Math.pow( x.get(0), 2 ) + 
                                      Math.pow( x.get(1), 2 ) +
                                      Math.pow( x.get(2), 2 ) );

   System.out.println( String.format( "pos update:  %5.1f, %5.1f, %5.1f; Mag: %5d(m)", x.get(0), x.get(1), x.get(2), (long)correction_mag ));

   // apply correction to Rx position estimate
   rover.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));
   rover.computeGeodetic();

   System.out.println( "recpos: " + this );
   
   return correction_mag; // return correction_mag
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  public void selectSatellitesDoubleDiff(Observations roverObs, Observations masterObs, Coordinates masterPos) {

    NavigationProducer navigation = goGPS.getNavigation();

    // Retrieve options from goGPS class
    double cutoff = goGPS.getCutoff();

    // Number of GPS observations
    int nObs = roverObs.getNumSat();

    // Allocate an array to store GPS satellite positions
    sats.pos = new SatellitePosition[nObs];

    // Allocate arrays to store receiver-satellite vectors
    rover.diffSat = new SimpleMatrix[nObs];
    master.diffSat = new SimpleMatrix[nObs];

    // Allocate arrays to store receiver-satellite approximate range
    rover.satAppRange = new double[nObs];
    master.satAppRange = new double[nObs];

    // Allocate arrays to store receiver-satellite atmospheric corrections
    rover.satTropoCorr = new double[nObs];
    rover.satIonoCorr = new double[nObs];
    master.satTropoCorr = new double[nObs];
    master.satIonoCorr = new double[nObs];

    // Create a list for available satellites
    sats.avail = new ArrayList<Integer>(0);
    sats.typeAvail = new ArrayList<Character>(0);
    sats.gnssAvail = new ArrayList<String>(0);

    // Create a list for available satellites with phase
    sats.availPhase = new ArrayList<Integer>(0);
    sats.typeAvailPhase = new ArrayList<Character>(0);
    sats.gnssAvailPhase = new ArrayList<String>(0);

    // Allocate arrays of topocentric coordinates
    rover.topo = new TopocentricCoordinates[nObs];
    master.topo = new TopocentricCoordinates[nObs];

    // Variables to store highest elevation
    double maxElevCode = 0;
    double maxElevPhase = 0;

    // Variables for code pivot and phase pivot
    int pivotCode = -1;
    int pivotPhase = -1;

    // Satellite ID
    int id = 0;

    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    for (int i = 0; i < nObs; i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);

      // Compute GPS satellite positions
      sats.pos[i] = navigation.getGpsSatPosition(roverObs, id, satType, rover.getReceiverClockError());

      if(sats.pos[i]!=null){

        // Compute rover-satellite approximate pseudorange
        rover.diffSat[i] = rover.minusXYZ(sats.pos[i]);
        rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
            + Math.pow(rover.diffSat[i].get(1), 2)
            + Math.pow(rover.diffSat[i].get(2), 2));

        // Compute master-satellite approximate pseudorange
        master.diffSat[i] = masterPos.minusXYZ(sats.pos[i]);
        master.satAppRange[i] = Math.sqrt(Math.pow(master.diffSat[i].get(0), 2)
            + Math.pow(master.diffSat[i].get(1), 2)
            + Math.pow(master.diffSat[i].get(2), 2));

        // Compute azimuth, elevation and distance for each satellite from
        // rover
        rover.topo[i] = new TopocentricCoordinates();
        rover.topo[i].computeTopocentric(rover, sats.pos[i]);

        // Compute azimuth, elevation and distance for each satellite from
        // master
        master.topo[i] = new TopocentricCoordinates();
        master.topo[i].computeTopocentric(masterPos, sats.pos[i]);

        // Computation of rover-satellite troposphere correction
        rover.satTropoCorr[i] = computeTroposphereCorrection(rover.topo[i].getElevation(), rover.getGeodeticHeight());

        // Computation of master-satellite troposphere correction
        master.satTropoCorr[i] = computeTroposphereCorrection(master.topo[i].getElevation(), masterPos.getGeodeticHeight());

        // Computation of rover-satellite ionosphere correction
        rover.satIonoCorr[i] = computeIonosphereCorrection(navigation,
            rover, rover.topo[i].getAzimuth(), rover.topo[i].getElevation(), roverObs.getRefTime());

        // Computation of master-satellite ionosphere correction
        master.satIonoCorr[i] = computeIonosphereCorrection(navigation,
                masterPos, master.topo[i].getAzimuth(), master.topo[i].getElevation(), roverObs.getRefTime());

        // Check if satellite is available for double differences, after
        // cutoff
        if (masterObs.containsSatIDType(roverObs.getSatID(i), roverObs.getGnssType(i)) // gpsSat.get( // masterObs.gpsSat.contains(roverObs.getGpsSatID(i)
            && rover.topo[i].getElevation() > cutoff) {

          // Find code pivot satellite (with highest elevation)
          if (rover.topo[i].getElevation() > maxElevCode) {
            pivotCode = i;
            maxElevCode = rover.topo[i].getElevation();
          }

          sats.avail.add(id);
          sats.typeAvail.add(satType);
          sats.gnssAvail.add(String.valueOf(satType) + String.valueOf(id));  

          // Check if also phase is available for both rover and master
          if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq())) &&
              !Double.isNaN(masterObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {

            // Find code pivot satellite (with highest elevation)
            if (rover.topo[i].getElevation() > maxElevPhase) {
              pivotPhase = i;
              maxElevPhase = rover.topo[i].getElevation();
            }

            sats.availPhase.add(id);
            sats.typeAvailPhase.add(satType);
            sats.gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));
            
          }
        }
      }
    }

    // Select best pivot satellite
    if (pivotPhase != -1){
      sats.pivot = pivotPhase;
    }else{
      sats.pivot = pivotCode;
    }
  }
}
