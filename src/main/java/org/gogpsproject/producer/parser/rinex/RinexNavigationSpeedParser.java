package org.gogpsproject.producer.parser.rinex;

import java.io.File;
import java.io.InputStream;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.rinex.RinexNavigationParser;
import org.gogpsproject.positioning.SatellitePosition;

public class RinexNavigationSpeedParser extends RinexNavigationParser {

  public RinexNavigationSpeedParser() {
   super(null);
  }

  public RinexNavigationSpeedParser(File fileNav) {
    super(fileNav);
  }

  public RinexNavigationSpeedParser(InputStream is, File cache) {
    super(is, cache);
  }

  public SatellitePosition computePositionGps(Observations obs, int satID, char satType, EphGps eph, double receiverClockError) {
    return computePositionSpeedGps(obs, satID, satType, eph, receiverClockError );
  }
  
}
