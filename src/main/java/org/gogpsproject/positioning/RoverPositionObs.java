package org.gogpsproject.positioning;

import org.gogpsproject.Coordinates;
import org.gogpsproject.Observations;
import org.gogpsproject.Status;
import org.gogpsproject.Time;

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
  
  /** clock error in ms */
  public long cErrMS = 0;

  /** average residual error */
  public double eRes;

  public RoverPositionObs(Coordinates c) {
    super(c);
  }

  public RoverPositionObs(Coordinates c, int dopType, double pDop,
      double hDop, double vDop) {
    super(c, dopType, pDop, hDop, vDop);
  }
}