package org.gogpsproject.producer.parser.rinex;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.parser.rinex.RinexNavigation;
import org.gogpsproject.producer.parser.rinex.RinexNavigationParser;
import org.gogpsproject.positioning.SatellitePosition;

public class RinexNavigationSpeed extends RinexNavigation  {
  
  public RinexNavigationSpeed() {
    super(null);
  }

  public RinexNavigationSpeed(String garnerNavigationAuto) {
    super(garnerNavigationAuto);
  }

  @Override
  public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
    long unixTime = obs.getRefTime().getMsec();
    long requestedTime = unixTime + 2*3600L*1000L;
    EphGps eph = null;
    int maxBack = 3;
    while(eph==null && (maxBack--)>0){
  
      RinexNavigationParser rnp = getRNPByTimestamp(requestedTime, satType);
      if(rnp!=null){
        obs.rinexFileName = rnp.getFileName();
        
          if(rnp.isTimestampInEpocsRange(unixTime)){
  //        return rnp.getGpsSatPosition(obs, satID, satType, receiverClockError);
          eph = rnp.findEph(unixTime, satID, satType);
          if( eph == EphGps.UnhealthyEph )
            return SatellitePosition.UnhealthySat;
                
          if (eph != null) {
             // cache this rnp in case we've backed off the time
             put( obs.getRefTime().getMsec(), rnp, satType);

    //        char satType = eph.getSatType();
            SatellitePosition sp = rnp.computePositionSpeedGps(obs, satID, satType, eph, receiverClockError);
    //        SatellitePosition sp = computePositionGps(unixTime, satType, satID, eph, range, receiverClockError);
            //if(receiverPosition!=null) earthRotationCorrection(receiverPosition, sp);
            return sp;// new SatellitePosition(eph, unixTime, satID, range);
          }
        }
      }
      requestedTime -= (2L*3600L*1000L);
    }
    return null;
  }

}
