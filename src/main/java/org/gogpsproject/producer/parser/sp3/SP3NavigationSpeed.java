/*
 * Copyright (c) 2011 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 *
 * goGPS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * goGPS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with goGPS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
package org.gogpsproject.producer.parser.sp3;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.SimpleTimeZone;
import java.util.TimeZone;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.StreamResource;
import org.gogpsproject.producer.parser.IonoGps;
import org.gogpsproject.util.UncompressInputStream;

/**
 * @author Lorenzo Patocchi, cryms.com
 *
 * Still incomplete
 */
public class SP3NavigationSpeed extends SP3Navigation {

	public SP3NavigationSpeed(String urltemplate){
		super( urltemplate );
	}

  protected SP3Parser getSP3ByTimestamp( long unixTime ) {
    SP3Parser sp3p = null;
    long reqTime = unixTime;

    do{
      // found none, retrieve from urltemplate
      Time t = new Time(reqTime);
      //System.out.print("request: "+unixTime+" "+(new Date(t.getMsec()))+" week:"+t.getGpsWeek()+" "+t.getGpsWeekDay());

      String url = urltemplate.replaceAll("\\$\\{wwww\\}", (new DecimalFormat("0000")).format(t.getGpsWeek()));
      url = url.replaceAll("\\$\\{d\\}", (new DecimalFormat("0")).format(t.getGpsWeekDay()));
      int hh4 = t.getGpsHourInDay();
      if(0<=hh4&&hh4<6) hh4=0;
      if(6<=hh4&&hh4<12) hh4=6;
      if(12<=hh4&&hh4<18) hh4=12;
      if(18<=hh4&&hh4<24) hh4=18;
      url = url.replaceAll("\\$\\{hh4\\}", (new DecimalFormat("00")).format(hh4));

      if( !url.startsWith("ftp://") )
          throw new RuntimeException("Invalid url template " + url);
        
      try {
        if(pool.containsKey(url)){
          //System.out.println(url+" from memory cache.");
          sp3p = pool.get(url);
        }else{

          sp3p = getFromFTP(url);
        }
        
        if(sp3p != null){
          pool.put(url, sp3p);
          // file exist, look for epoch
          if(sp3p.isTimestampInEpocsRange(unixTime)){
            return sp3p;
          }else{
            return null;
          }
        }
     } catch( Exception e) {
          e.printStackTrace();
          return null;
     }
   } while( sp3p==null );

   return null;
 }
  
	
	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double)
	 */
	@Override
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
		long unixTime = obs.getRefTime().getMsec();
    long reqTime = unixTime + 2*3600L*1000L;
		SP3Parser sp3p = null;
    int maxBack = 3;
    while( sp3p==null && (maxBack--)>0 ){
      sp3p = getSP3ByTimestamp(reqTime);
      
      if( sp3p != null ) {
//        obs.rinexFileName = sp3p.getFileName();
//        return sp3p.getGpsSatPosition( obs, satID, satType, receiverClockError );
        SatellitePosition sp = sp3p.getGpsSatPosition(obs, satID, satType, receiverClockError );
        return sp;
			}
      else 
	      reqTime -= (2L*3600L*1000L);
		};

		return null;
	}
}
