package org.gogpsproject.parser.rinex;

import org.gogpsproject.EphGps;
import org.gogpsproject.Observations;
import org.gogpsproject.SatellitePosition;

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
    long requestedTime = unixTime;
    EphGps eph = null;
    int maxBack = 3;
    while(eph==null && (maxBack--)>0){
  
      RinexNavigationParser rnp = getRNPByTimestamp(requestedTime);
      if(rnp!=null){
        obs.rinexFileName = rnp.getFileName();
        
        if(rnp.isTimestampInEpocsRange(unixTime)){
  //        return rnp.getGpsSatPosition(obs, satID, satType, receiverClockError);
          eph = rnp.findEph(unixTime, satID, satType);
          if( eph == EphGps.UnhealthyEph )
            return SatellitePosition.UnhealthySat;
                
          if (eph != null) {
             // cache this rnp in case we've backed off the time
             put( obs.getRefTime().getMsec(), rnp);

    //        char satType = eph.getSatType();
            SatellitePosition sp = rnp.computePositionSpeedGps(obs, satID, satType, eph, receiverClockError);
    //        SatellitePosition sp = computePositionGps(unixTime, satType, satID, eph, range, receiverClockError);
            //if(receiverPosition!=null) earthRotationCorrection(receiverPosition, sp);
            return sp;// new SatellitePosition(eph, unixTime, satID, range);
          }
        }
      }
      requestedTime -= (1L*3600L*1000L);
    }
    return null;
  }

}
