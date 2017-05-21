package org.gogpsproject.positioning;

import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Constants;
import org.gogpsproject.GoGPS;

//import com.google.maps.ElevationApi;
//import com.google.maps.GeoApiContext;
//import com.google.maps.model.ElevationResult;
//import com.google.maps.model.LatLng;

public class Core {

  GoGPS goGPS;
  RoverPosition rover;
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

}
