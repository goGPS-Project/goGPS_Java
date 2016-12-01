package org.gogpsproject;

/**
 * Augmented RoverPosition with extra information about the status of processed observations
 */
public class RoverPositionObs extends RoverPosition{
  public long index;
  public Time sampleTime;
  public Observations obs;
  public Status status;

  //Snapshot Variables to review
  public long satsInView = 0;
  public long satsInUse = 0;
  //
  /**
   * Average Residual Error
   */
  public double eRes;
  /**
   * Coarse Time Clock Error (in seconds)
   */
  public double cErr;
  
  public RoverPositionObs(Coordinates c) {
    super(c);
  }

  public RoverPositionObs(Coordinates c, int dopType, double pDop,
      double hDop, double vDop) {
    super(c, dopType, pDop, hDop, vDop);
  }
}