package org.gogpsproject.positioning;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Status;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.IonoGps;

public class Satellites {
  
  GoGPS goGPS;
  RoverPosition rover;
  MasterPosition master; 
  NavigationProducer navigation;

  /** Absolute position of all visible satellites (ECEF) */
  SatellitePosition[] pos; 

  /** List of satellites available for processing */
  Map<Integer, SatellitePosition> avail; 
 
  /** List of satellites available for processing */
  ArrayList<Integer> availPhase; 
  
  /** List of satellite Types available for processing */
  ArrayList<Character> typeAvail; 
  
  /** List of satellite Type available for processing */
  ArrayList<Character> typeAvailPhase; 
  
  /** List of satellite Types & Id available for processing */
  ArrayList<String> gnssAvail;  
  
  /** List of satellite Types & Id available for processing */
  ArrayList<String> gnssAvailPhase;  

  /** Index of the satellite with highest elevation in satAvail list */
  int pivot;

  public Satellites(GoGPS goGPS) {
    this.goGPS = goGPS;
    this.rover = goGPS.getRoverPos();
    this.master = goGPS.getMasterPos();
    this.navigation = goGPS.getNavigation();
  }

  /** @return the number of available satellites */
  public int getAvailNumber() {
    return avail.size();
  }

  /** @return the number of available satellites (with phase) */
  public int getAvailPhaseNumber() {
    return availPhase.size();
  }
  
  public String getAvailGnssSystems(){
    if( typeAvail.isEmpty()) return "";
    String GnssSys = "";
    for(int i=0;i< typeAvail.size();i++) {
      if (GnssSys.indexOf(( typeAvail.get(i))) < 0)
        GnssSys = GnssSys + typeAvail.get(i);
    }
    return GnssSys;
  }
  
  /**
   * @param elevation
   * @param height
   * @return troposphere correction value by Saastamoinen model
   */
  static double computeTroposphereCorrection(double elevation, double height) {

    double tropoCorr = 0;

    if (height > 5000)
      return tropoCorr;
      
    elevation = Math.toRadians(Math.abs(elevation));
    if (elevation == 0){
      elevation = elevation + 0.01;
    }

    // Numerical constants and tables for Saastamoinen algorithm
    // (troposphere correction)
    final double hr = 50.0;
    final int[] ha = {0, 500, 1000, 1500, 2000, 2500, 3000, 4000, 5000 };
    final double[] ba = { 1.156, 1.079, 1.006, 0.938, 0.874, 0.813, 0.757, 0.654, 0.563 };

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

    return tropoCorr;
  }
  
  /**
   * @param ionoParams
   * @param coord
   * @param time
   * @return ionosphere correction value by Klobuchar model
   */
  static double computeIonosphereCorrection( NavigationProducer navigation,
      Coordinates coord, double azimuth, double elevation, Time time) {

    double ionoCorr = 0;

    IonoGps iono = navigation.getIono(time.getMsec());
    
    if(iono==null) 
      return 0.0;
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
    }
    else{
      ionoCorr = Constants.SPEED_OF_LIGHT * f * 5e-9;
    }
    
    return ionoCorr;
  }
  
  void init( Observations roverObs ) {

    int nObs = roverObs.getNumSat();

    // Allocate an array to store GPS satellite positions
    pos = new SatellitePosition[nObs];

    // Create a list for available satellites
    avail = new LinkedHashMap<>();
    typeAvail = new ArrayList<>(0);
    gnssAvail = new ArrayList<>(0);

    // Create a list for available satellites with phase
    availPhase = new ArrayList<>(0);
    typeAvailPhase = new ArrayList<>(0);
    gnssAvailPhase = new ArrayList<>(0);

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

    // Allocate arrays of topocentric coordinates
    rover.topo = new TopocentricCoordinates[nObs];
    master.topo = new TopocentricCoordinates[nObs];
    
    rover.satsInUse = 0;
  }
  
  /**
   * @param roverObs
   */
  public void selectStandalone( Observations roverObs ) {
    selectStandalone( roverObs, goGPS.getCutoff() );
  }

  /**
   * @param roverObs
   * @param cutoff
   */
  public void selectStandalone( Observations roverObs, double cutoff) {

    init( roverObs );
    
    // Compute topocentric coordinates and
    // select satellites above the cutoff level
    for( int i = 0; i < roverObs.getNumSat(); i++) {

      int id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);

      // Compute GPS satellite positions getGpsByIdx(idx).getSatType()
      pos[i] = navigation.getGpsSatPosition( roverObs, id, satType, rover.getClockError());
      
      if(pos[i]!=null){

    	  	if( pos[i].equals( SatellitePosition.UnhealthySat )) {
    	        pos[i] = null;
    	        continue;
    	  	}
    	  	
        // Compute rover-satellite approximate pseudorange
        rover.diffSat[i] = rover.minusXYZ(pos[i]);
        rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
            + Math.pow(rover.diffSat[i].get(1), 2)
            + Math.pow(rover.diffSat[i].get(2), 2));

        // Compute azimuth, elevation and distance for each satellite
        rover.topo[i] = new TopocentricCoordinates();
        rover.topo[i].computeTopocentric(rover, pos[i]);

        // Correct approximate pseudorange for troposphere
        rover.satTropoCorr[i] = computeTroposphereCorrection(rover.topo[i].getElevation(), rover.getGeodeticHeight());

        // Correct approximate pseudorange for ionosphere
        rover.satIonoCorr[i] = computeIonosphereCorrection(navigation, rover, rover.topo[i].getAzimuth(), rover.topo[i].getElevation(), roverObs.getRefTime());

//        System.out.println("getElevation: " + id + "::" + rover.topo[i].getElevation() ); 
        // Check if satellite elevation is higher than cutoff
        if (rover.topo[i].getElevation() > cutoff) {
          
          avail.put(id, pos[i]);
          typeAvail.add(satType);
          gnssAvail.add(String.valueOf(satType) + String.valueOf(id));

          // Check if also phase is available
          if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {
            availPhase.add(id);
            typeAvailPhase.add(satType);
            gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));       
            
          }
        }else{
          if(goGPS.isDebug()) System.out.println("Not useful sat "+roverObs.getSatID(i)+" for too low elevation "+rover.topo[i].getElevation()+" < "+cutoff);
        }
      }
    }
  }

  /**
   * @param roverObs
   * @param masterObs
   * @param masterPos
   */
  public void selectDoubleDiff( Observations roverObs, Observations masterObs, Coordinates masterPos ) {

    // Retrieve options from goGPS class
    double cutoff = goGPS.getCutoff();

    init( roverObs );
    
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
    for (int i = 0; i < roverObs.getNumSat(); i++) {

      id = roverObs.getSatID(i);
      char satType = roverObs.getGnssType(i);

      // Compute GPS satellite positions
      pos[i] = navigation.getGpsSatPosition(roverObs, id, satType, rover.getClockError());

      if(pos[i]!=null){

        // Compute rover-satellite approximate pseudorange
        rover.diffSat[i] = rover.minusXYZ(pos[i]);
        rover.satAppRange[i] = Math.sqrt(Math.pow(rover.diffSat[i].get(0), 2)
                             + Math.pow(rover.diffSat[i].get(1), 2)
                             + Math.pow(rover.diffSat[i].get(2), 2));

        // Compute master-satellite approximate pseudorange
        master.diffSat[i]     = masterPos.minusXYZ(pos[i]);
        master.satAppRange[i] = Math.sqrt(Math.pow(master.diffSat[i].get(0), 2)
                              + Math.pow(master.diffSat[i].get(1), 2)
                              + Math.pow(master.diffSat[i].get(2), 2));

        // Compute azimuth, elevation and distance for each satellite from rover
        rover.topo[i] = new TopocentricCoordinates();
        rover.topo[i].computeTopocentric(rover, pos[i]);

        // Compute azimuth, elevation and distance for each satellite from master
        master.topo[i] = new TopocentricCoordinates();
        master.topo[i].computeTopocentric(masterPos, pos[i]);

        // Computation of rover-satellite troposphere correction
        rover.satTropoCorr[i] = computeTroposphereCorrection( rover.topo[i].getElevation(), rover.getGeodeticHeight());

        // Computation of master-satellite troposphere correction
        master.satTropoCorr[i] = computeTroposphereCorrection( master.topo[i].getElevation(), masterPos.getGeodeticHeight());

        // Computation of rover-satellite ionosphere correction
        rover.satIonoCorr[i] = computeIonosphereCorrection( navigation, rover, rover.topo[i].getAzimuth(), rover.topo[i].getElevation(), roverObs.getRefTime());

        // Computation of master-satellite ionosphere correction
        master.satIonoCorr[i] = computeIonosphereCorrection(navigation,
                masterPos, master.topo[i].getAzimuth(), master.topo[i].getElevation(), roverObs.getRefTime());

        // Check if satellite is available for double differences, after cutoff
        if (masterObs.containsSatIDType(roverObs.getSatID(i), roverObs.getGnssType(i)) // gpsSat.get( // masterObs.gpsSat.contains(roverObs.getGpsSatID(i)
            && rover.topo[i].getElevation() > cutoff) {

          // Find code pivot satellite (with highest elevation)
          if (rover.topo[i].getElevation() > maxElevCode) {
            pivotCode = i;
            maxElevCode = rover.topo[i].getElevation();
          }

          avail.put(id,pos[i]);
          typeAvail.add(satType);
          gnssAvail.add(String.valueOf(satType) + String.valueOf(id));  

          // Check if also phase is available for both rover and master
          if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq())) &&
              !Double.isNaN(masterObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {

            // Find code pivot satellite (with highest elevation)
            if (rover.topo[i].getElevation() > maxElevPhase) {
              pivotPhase = i;
              maxElevPhase = rover.topo[i].getElevation();
            }

            availPhase.add(id);
            typeAvailPhase.add(satType);
            gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));
          }
        }
      }
    }

    // Select best pivot satellite
    if( pivotPhase != -1 ){
      pivot = pivotPhase;
    }else{
      pivot = pivotCode;
    }
  }
}
