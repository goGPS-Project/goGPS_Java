package org.gogpsproject;

import java.io.FileNotFoundException;

import org.gogpsproject.ephemeris.EphGps;

public class ObservationsSpeedBuffer extends ObservationsBuffer {

  public ObservationsSpeedBuffer() {
    super();
  }

  public ObservationsSpeedBuffer(StreamResource streamResource, String fileNameOutLog) throws FileNotFoundException {
    super( streamResource, fileNameOutLog );
  }

  public SatellitePosition computePositionGps(Observations obs, int satID, char satType, EphGps eph, double receiverClockError) {
    return computePositionSpeedGps(obs, satID, satType, eph, receiverClockError );
  }
}
