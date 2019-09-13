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
package org.gogpsproject.consumer;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.positioning.RoverPosition;
import org.gogpsproject.positioning.RoverPosition.DopType;
/**
 * <p>
 * Produces TXT file
 * </p>
 *
 * @author Eugenio Realini
 */

public class TxtProducer extends Thread implements PositionConsumer {

	private static DecimalFormat f = new DecimalFormat("0.000");
	private static DecimalFormat g = new DecimalFormat("0.00000000");

	private SimpleDateFormat dateTXT = new SimpleDateFormat("yy/MM/dd");
	private SimpleDateFormat timeTXT = new SimpleDateFormat("HH:mm:ss.SSS");

	private String filename = null;
	private boolean debug=false;

	private ArrayList<RoverPosition> positions = new ArrayList<RoverPosition>();
	
	private final static TimeZone TZ = TimeZone.getTimeZone("GMT");

	public TxtProducer(String filename) throws IOException{
    super("TxtProducer");
		this.filename = filename;

		writeHeader();
		
		dateTXT.setTimeZone(TZ);
		timeTXT.setTimeZone(TZ);

		start();
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
	public void writeCoordinate(RoverPosition coord,FileWriter out) {
		try {
			
			PrintWriter pw = new PrintWriter(out);

			//date, time
			String d = dateTXT.format(new Date(coord.getRefTime().getMsec()));
			String t = timeTXT.format(new Date(coord.getRefTime().getMsec()));
			
			pw.printf("%8s%16s", d, t);
			
			//GPS week
			int week = coord.getRefTime().getGpsWeek();
			
			pw.printf("%16d", week);
			
			//GPS time-of-week (tow)
			double tow = coord.getRefTime().getGpsTime();
			
			pw.printf("%16f", tow);
			
			//latitude, longitude, ellipsoidal height
			double lat = coord.getGeodeticLatitude();
			double lon = coord.getGeodeticLongitude();
			double hEllips = coord.getGeodeticHeight();
			
			pw.printf("%16.8f%16.8f%16.3f", lat, lon, hEllips);
			
			//ECEF coordinates (X, Y, Z)
			double X = coord.getX();
			double Y = coord.getY();
			double Z = coord.getZ();
			
			pw.printf("%16.3f%16.3f%16.3f", X, Y, Z);

			//UTM north, UTM east, orthometric height, UTM zone
			int noData = -9999;
			double utmNorth = noData;
			double utmEast = noData;
			double hOrtho = noData;
			String utmZone = "-9999";
			
			pw.printf("%16.3f%16.3f%16.3f%16s", utmNorth, utmEast, hOrtho, utmZone);
			
			//HDOP, KHDOP
			double hdop = noData;
			double khdop = noData;
			if (coord.getDopType() == DopType.KALMAN ) {
			  khdop = coord.gethDop();
			} else {
			  hdop = coord.gethDop();
			}
			
			pw.printf("%16.3f%16.3f%n", hdop, khdop);

//			out.write(lon + "," // geod.get(0)
//					+ lat + "," // geod.get(1)
//					+ h + "\n"); // geod.get(2)
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

			out.write("    Date        GPS time        GPS week         GPS tow    " +
					  "    Latitude       Longitude     h (ellips.)      " +
					  "    ECEF X          ECEF Y          ECEF Z   " +
					  "    UTM North        UTM East     h (orthom.)        UTM zone        " +
					  "    HDOP           KHDOP\n");
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
//		if(event == EVENT_START_OF_TRACK){
//			startOfTrack();
//		}
		if(event == EVENT_END_OF_TRACK){
			// finish writing
			interrupt();
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
    FileWriter out = writeHeader();
		while(!isInterrupted() || last < positions.size() ){
		  //	goodDop = false;
      for( ; last<positions.size(); last++ ) {
				writeCoordinate(positions.get(last), out);
			}
			try {
				Thread.sleep(200);
			}
			catch (InterruptedException e) {
			  interrupt();
			}
	  }
	}

	public void cleanStop(){
    interrupt();
	}
}
