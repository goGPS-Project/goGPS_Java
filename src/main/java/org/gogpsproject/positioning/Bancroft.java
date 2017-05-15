package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;
import org.gogpsproject.Observations;
import org.gogpsproject.SatellitePosition;

public class Bancroft extends Core {

  public Bancroft( GoGPS goGPS ){
    super(goGPS);
  }
  
  public void bancroft( Observations obs) {

    //roverPos.coord = null;
    //roverPos.coord = Coordinates.globalXYZInstance(0.0, 0.0, 0.0);

    double travelTime = 0;
    double angle;
    double a, b, c;
    double root;
    double[] r, omc;
    double cdt, calc;
    double rho;

    // Define matrices
    SimpleMatrix Binit;
    SimpleMatrix B;
    SimpleMatrix BBB;
    SimpleMatrix BBBe;
    SimpleMatrix BBBalpha;
    SimpleMatrix e;
    SimpleMatrix alpha;
    SimpleMatrix possiblePosA;
    SimpleMatrix possiblePosB;

    // Allocate vectors
    r = new double[2];
    omc = new double[2];

    // Number of GPS observations
    int nObs = obs.getNumSat();

    // Allocate an array to store GPS satellite positions
    pos = new SatellitePosition[nObs];

    // Allocate a 2D array to store Bancroft matrix data
    double[][] dataB = new double[nObs][4];

    int p=0;
    int id = 0;
    
    for (int i = 0; i < nObs; i++) {

      id = obs.getSatID(i);
      char satType = obs.getGnssType(i);
      
      // Create new satellite position object
      //pos[i] = new SatellitePosition(obs.getRefTime().getGpsTime(), obs.getGpsSatID(i), obs.getGpsByID(id).getPseudorange(goGPS.getFreq()));

      // Compute clock-corrected satellite position
      //pos[i].computePositionGps(goGPS.getNavigation());
    
      double obsPseudorange = obs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
      pos[i] = goGPS.getNavigation().getGpsSatPosition(obs, id, satType, roverPos.getReceiverClockError());
    
      try {
//      System.out.println("SatPos "+obs.getGpsSatID(i)+" x:"+pos[i].getX()+" y:"+pos[i].getY()+" z:"+pos[i].getZ());
        // Store Bancroft matrix data (X, Y, Z and clock-corrected
        // range)
        if(pos[i]!=null){
          dataB[p][0] = pos[i].getX();
          dataB[p][1] = pos[i].getY();
          dataB[p][2] = pos[i].getZ();
          dataB[p][3] = obsPseudorange + Constants.SPEED_OF_LIGHT * pos[i].getSatelliteClockError();
          p++;
        }else{
          if(goGPS.isDebug()) System.out.println("Error: satellite positions not computed for satID:"+obs.getSatID(i));
        }
      } catch (NullPointerException u) {
        u.printStackTrace();
        if(goGPS.isDebug()) System.out.println("Error: satellite positions not computed for satID:"+obs.getSatID(i));
        //return; // don't break eggs so quickly :-)
      }
    
//      }else{
//        
//        p++;
//      }
    }
    
    if(p<4) return;
    if(dataB.length != p){
      double[][] dataB1 = new double[p][4];
      for(int i=0;i<p;i++){
        dataB1[i]=dataB[i];
      }
      dataB = dataB1;
    }
    // Allocate matrices
    BBB = new SimpleMatrix(4, dataB.length);
    BBBe = new SimpleMatrix(4, 1);
    BBBalpha = new SimpleMatrix(4, 1);
    e = new SimpleMatrix(dataB.length, 1);
    alpha = new SimpleMatrix(dataB.length, 1);
    possiblePosA = new SimpleMatrix(4, 1);
    possiblePosB = new SimpleMatrix(4, 1);

    // Allocate initial B matrix
    Binit = new SimpleMatrix(dataB);

    // Make two iterations
    for (int iter = 0; iter < 2; iter++) {

      // Allocate B matrix
      B = new SimpleMatrix(Binit);

      for (int i = 0; i < dataB.length; i++) {

        double x = B.get(i, 0);
        double y = B.get(i, 1);

        if (iter == 0) {
          travelTime = Constants.GPS_APPROX_TRAVEL_TIME;
        } else {
          double z = B.get(i, 2);
          rho = Math.pow((x - roverPos.getX()), 2)
              + Math.pow((y - roverPos.getY()), 2)
              + Math.pow((z - roverPos.getZ()), 2);
          travelTime = Math.sqrt(rho) / Constants.SPEED_OF_LIGHT;
        }
        angle = travelTime * Constants.EARTH_ANGULAR_VELOCITY;
        B.set(i, 0, Math.cos(angle) * x + Math.sin(angle) * y);
        B.set(i, 1, -Math.sin(angle) * x + Math.cos(angle) * y);
      }

      if (dataB.length > 4) {
        BBB = B.transpose().mult(B).solve(B.transpose());
      } else {
        BBB = B.invert();
      }

      e.set(1);
      for (int i = 0; i < dataB.length; i++) {

        alpha.set(i, 0, lorentzInnerProduct(B.extractMatrix(i, i+1, 0, 4), B
            .extractMatrix(i, i+1, 0, 4)) / 2);
      }

      BBBe = BBB.mult(e);
      BBBalpha = BBB.mult(alpha);
      a = lorentzInnerProduct(BBBe, BBBe);
      b = lorentzInnerProduct(BBBe, BBBalpha) - 1;
      c = lorentzInnerProduct(BBBalpha, BBBalpha);
      root = Math.sqrt(b * b - a * c);
      r[0] = (-b - root) / a;
      r[1] = (-b + root) / a;
      possiblePosA = BBBalpha.plus(r[0], BBBe);
      possiblePosB = BBBalpha.plus(r[1], BBBe);
      possiblePosA.set(3, 0, -possiblePosA.get(3, 0));
      possiblePosB.set(3, 0, -possiblePosB.get(3, 0));
      for (int i = 0; i < dataB.length; i++) {
        cdt = possiblePosA.get(3, 0);
        calc = B.extractMatrix(i, i+1, 0, 3).transpose().minus(
            possiblePosA.extractMatrix(0, 3, 0, 1)).normF()
            + cdt;
        omc[0] = B.get(i, 3) - calc;
        cdt = possiblePosB.get(3, 0);
        calc = B.extractMatrix(i, i+1, 0, 3).transpose().minus(
            possiblePosB.extractMatrix(0, 3, 0, 1)).normF()
            + cdt;
        omc[1] = B.get(i, 3) - calc;
      }

      // Discrimination between roots (choose one of the possible
      // positions)
      if (Math.abs(omc[0]) > Math.abs(omc[1])) {
        //roverPos.coord.ecef = possiblePosB.extractMatrix(0, 3, 0, 1); // new SimpleMatrix(
        SimpleMatrix sm = possiblePosB.extractMatrix(0, 3, 0, 1);
        roverPos.setXYZ(sm.get(0),sm.get(1),sm.get(2));
        // Clock offset
        roverPos.receiverClockError = possiblePosB.get(3, 0) / Constants.SPEED_OF_LIGHT;
      } else {
        //roverPos.coord.ecef = possiblePosA.extractMatrix(0, 3, 0, 1); // new SimpleMatrix(
        SimpleMatrix sm = possiblePosA.extractMatrix(0, 3, 0, 1);
        roverPos.setXYZ(sm.get(0),sm.get(1),sm.get(2));
        // Clock offset
        roverPos.receiverClockError = possiblePosA.get(3, 0) / Constants.SPEED_OF_LIGHT;
      }
    }
//    System.out.println("## x: " + roverPos.getX() );
//    System.out.println("## y: " + roverPos.getY() );
//    System.out.println("## z: " + roverPos.getZ() );
    
    // Compute Bancroft's positioning in geodetic coordinates
    roverPos.computeGeodetic();
  }
}
