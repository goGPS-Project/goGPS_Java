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
package org.gogpsproject.producer;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.PositionConsumer;
import org.gogpsproject.RoverPosition;
import org.gogpsproject.RoverPositionObs;
import org.gogpsproject.Status;

/**
 * <p>
 * Produces TXT file
 * </p>
 *
 * @author Eugenio Realini
 */

public class CsvProducer implements PositionConsumer, Runnable {

  private static DecimalFormat latlonf = new DecimalFormat("0.000000");
  private static DecimalFormat dopf = new DecimalFormat("0.0");

  private static DecimalFormat f = new DecimalFormat("0.000");
  private static DecimalFormat g = new DecimalFormat("0.00000000");

  private SimpleDateFormat dateTXT = new SimpleDateFormat("yyyy/MM/dd");
  private SimpleDateFormat timeTXT = new SimpleDateFormat("HH:mm:ss");

  private String filename = null;
  private boolean debug=false;

  public Thread t = null;

  private ArrayList<RoverPosition> positions = new ArrayList<RoverPosition>();

  public CsvProducer(String filename) throws IOException{
    TimeZone gmttz = TimeZone.getTimeZone("GMT");
    dateTXT.setTimeZone( gmttz );
    timeTXT.setTimeZone( gmttz );
    this.filename = filename;

    writeHeader();

    t = new Thread(this);
    t.start();
  }

  /* (non-Javadoc)
   * @see org.gogpsproject.producer.PositionConsumer#addCoordinate(org.gogpsproject.Coordinates)
   */
  @Override
  public void addCoordinate(RoverPosition coord) {
    if(debug) System.out.println("Lon:"+g.format(coord.getGeodeticLongitude()) + " " // geod.get(0)
        +"Lat:"+ g.format(coord.getGeodeticLatitude()) + " " // geod.get(1)
        +"H:"+ f.format(coord.getGeodeticHeight()) + "\t" // geod.get(2)
        +"P:"+ coord.getpDop()+" "
        +"H:"+ coord.gethDop()+" "
        +"V:"+ coord.getvDop()+" ");//geod.get(2)

    positions.add(coord);
  }

  /* (non-Javadoc)
   * @see org.gogpsproject.producer.PositionConsumer#addCoordinate(org.gogpsproject.Coordinates)
   */
  public void writeCoordinate(int index, RoverPosition coord, FileWriter out) {
    try {
//      out.write( "Index,Status,Date,UTC,Latitude [DD], Longitude [DD],HDOP,SVs in Use,SVs in View,SNR Avg [dB],Residual Error,Clock Error,Clock Error Total,\r\n" );
      RoverPositionObs c = (RoverPositionObs)coord;

      PrintWriter pw = new PrintWriter(out);

      pw.printf("%d,%s,", c.index, c.status.toString() );

      //date, time
      String d = dateTXT.format(new Date(c.getRefTime().getMsec()));
      String t = timeTXT.format(new Date(c.getRefTime().getMsec()));
      pw.printf("%s,%s,", d, t);
      
      double lat = c.getGeodeticLatitude();
      double lon = c.getGeodeticLongitude();
//      double hEllips = coord.getGeodeticHeight();
      
      if( c.status==Status.Valid )
        pw.printf("%s,%s,%s,", latlonf.format(lat), latlonf.format(lon), dopf.format( c.getpDop() ));
      else
        pw.printf("0,0,0,");
        
      
      pw.printf("%d,%d,", c.satsInUse, c.satsInView ); 
      pw.printf("%3.1f,%4.3f,\r\n", c.eRes, Math.round(c.cErrMS/1000.0)); 

      out.flush();

    } catch (NullPointerException e) {
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  /* (non-Javadoc)
   * @see org.gogpsproject.producer.PositionConsumer#startOfTrack()
   */
  public FileWriter writeHeader() {
    try {
      FileWriter out = new FileWriter(filename);

      out.write( "Index,Status,Date,UTC,Latitude [DD], Longitude [DD],HDOP,SVs in Use,SVs in View,SNR Avg [dB],Residual Error,Clock Error,\r\n" );

      out.flush();
      return out;
    } catch (IOException e) {
      e.printStackTrace();
    }
    return null;
  }

  /* (non-Javadoc)
   * @see org.gogpsproject.producer.PositionConsumer#event(int)
   */
  @Override
  public void event(int event) {
//    if(event == EVENT_START_OF_TRACK){
//      startOfTrack();
//    }
    if(event == EVENT_END_OF_TRACK){
      // finish writing
      t = null;
    }
  }

  /**
   * @param debug the debug to set
   */
  public void setDebug(boolean debug) {
    this.debug = debug;
  }

  /**
   * @return the debug
   */
  public boolean isDebug() {
    return debug;
  }

  /* (non-Javadoc)
   * @see java.lang.Runnable#run()
   */
  @Override
  public void run() {
    int last = 0;
    try {
      while(t!=null && Thread.currentThread()==t){
        if(last != positions.size()){ // check if we have more data to write
          last = positions.size();

//          goodDop = false;
          FileWriter out = writeHeader();
          if(out!=null){
            int index = 0;
            for(RoverPosition pos: (ArrayList<RoverPosition>) positions.clone()){
              writeCoordinate(index, pos, out);
              index++;
            }
          }

        }

        Thread.sleep(1000);
      }

      //flush the last coordinates
      if(last != positions.size()){ // check if we have more data to write
        last = positions.size();

//        goodDop = false;
        FileWriter out = writeHeader();
        if(out!=null){
          int index = 0;
          for(RoverPosition pos: (ArrayList<RoverPosition>) positions.clone()){
            writeCoordinate(index, pos, out);
            index++;
          }
        }

      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
  public void cleanStop(){
    t=null;
  }
}
