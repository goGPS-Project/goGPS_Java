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
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.Status;
import org.gogpsproject.positioning.RoverPosition;
import org.gogpsproject.positioning.RoverPosition.DopType;

/**
 * <p>
 * Produces KML file
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com
 */

public class KmlProducer extends Thread implements PositionConsumer {

	private static DecimalFormat f = new DecimalFormat("0.000");
	private static DecimalFormat g = new DecimalFormat("0.00000000");

	private SimpleDateFormat timeKML = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");

	private String filename = null;
	private String timeline = null;
	private int num = 0;

	private boolean goodDop = false;
	private double goodDopThreshold = 0.0;
	private int timeSampleDelaySec = 0;

	private String circleColorLine = "FFCC99";
	private String circleOpacity = "88";
	private int circlePixelWidth = 2;

	private String goodColorLine = "00ff00";
	private String goodOpacity = "ff";
	private int goodLinePixelWidth = 3;
	private String worstColorLine = "0000ff";
	private String worstOpacity = "ff";
	private int worstLinePixelWidth = 3;
	private boolean debug=false;

	private ArrayList<RoverPosition> positions = new ArrayList<RoverPosition>();
	
	private final static TimeZone TZ = TimeZone.getTimeZone("GMT");

	public KmlProducer(String filename, double goodDopTreshold, int timeSampleDelaySec) throws IOException{
	  super("KmlProducer");
		this.filename = filename;
		this.goodDopThreshold = goodDopTreshold;
		this.timeSampleDelaySec = timeSampleDelaySec;
		
		timeKML.setTimeZone(TZ);

		goodDop = false;
		FileWriter out = startOfTrack();
		if(out!=null){

			endOfTrack(out);
		}
		start();
	}

	 public KmlProducer(String filename, double goodDopTreshold, int timeSampleDelaySec, String goodColorLine) throws IOException{
	    super("KmlProducer");
	    this.goodColorLine = goodColorLine;
//	   this.worstColorLine = goodColorLine;
	   
	    this.filename = filename;
	    this.goodDopThreshold = goodDopTreshold;
	    this.timeSampleDelaySec = timeSampleDelaySec;
	    
	    timeKML.setTimeZone(TZ);

	    goodDop = false;
	    FileWriter out = startOfTrack();
	    if(out!=null){

	      endOfTrack(out);
	    }
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
//      ReceiverPositionObs c = (ReceiverPositionObs)coord;
//      if( c.status != Status.Valid )
//        return;
      
			boolean prevDopResponse = goodDop;
			goodDop = coord.getpDop()<goodDopThreshold;
			if(prevDopResponse != goodDop){
				out.write("</coordinates></LineString></Placemark>\n"+
						"  <Placemark>\n"+
						"    <name></name>\n"+
						"    <description></description>\n"+
						"    <styleUrl>#LineStyle_"+(goodDop?"good":"worst")+"</styleUrl>\n"+
						"    <LineString>\n"+
						"      <tessellate>1</tessellate>\n"+
						"      <coordinates>\n");
			}

			String lon = g.format(coord.getGeodeticLongitude());
			String lat = g.format(coord.getGeodeticLatitude());
			String h = f.format(coord.getGeodeticHeight());

			out.write(lon + "," // geod.get(0)
					+ lat + "," // geod.get(1)
					+ h + "\n"); // geod.get(2)
			out.flush();

			String t = timeKML.format(new Date(coord.getRefTime().getMsec()));
//			System.out.print("T:" + t);
//			System.out.print(" Lon:" + lon);//geod.get(0)
//			System.out.print(" Lat:" + lat);//geod.get(1)
			String dopLabel = "DOP";
			if ( coord.getDopType() == DopType.KALMAN )
				dopLabel = "KDOP";

			if(timeSampleDelaySec>0 && (num++)%timeSampleDelaySec==0){

				timeline += "\n";
				timeline += "<Placemark>"+
		        "<TimeStamp>"+
		        "<when>"+t+"</when>"+
		        "</TimeStamp>"+
		        "<styleUrl>#dot-icon</styleUrl>"+
		        "<Point>"+
		        "<coordinates>"+lon + ","
				+ lat + ","
				+ h + "</coordinates>"+
		        "</Point>"+
		        "</Placemark>";
			}
		} catch (NullPointerException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.producer.PositionConsumer#startOfTrack()
	 */
	public FileWriter startOfTrack() {
		if(timeSampleDelaySec>0)timeline = "<Folder><open>1</open><Style><ListStyle><listItemType>checkHideChildren</listItemType></ListStyle></Style>";
		try {
			FileWriter out = new FileWriter(filename);

			out.write("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"+
				"<Document xmlns:kml=\"http://earth.google.com/kml/2.1\">\n"+
				"  <Style id=\"LineStyle_worst\"><LineStyle><color>"+worstOpacity+worstColorLine+"</color><width>"+worstLinePixelWidth+"</width></LineStyle><PolyStyle><color>"+worstOpacity+worstColorLine+"</color></PolyStyle></Style>\n"+
				"  <Style id=\"LineStyle_good\"><LineStyle><color>"+goodOpacity+goodColorLine+"</color><width>"+goodLinePixelWidth+"</width></LineStyle><PolyStyle><color>"+goodOpacity+goodColorLine+"</color></PolyStyle></Style>\n"+
				"  <Style id=\"CircleStyle\"><LineStyle><color>"+circleOpacity+circleColorLine+"</color><width>"+circlePixelWidth+"</width></LineStyle><PolyStyle><color>"+circleOpacity+circleColorLine+"</color></PolyStyle></Style>\n"+
				"  <Style id=\"dot-icon\"><IconStyle><Icon><href>https://storage.googleapis.com/support-kms-prod/SNP_2752129_en_v0</href></Icon><scale>0.3</scale></IconStyle></Style>\n"+
				"  <Folder><Placemark>\n"+
				"    <name></name>\n"+
				"    <description></description>\n"+
				"    <styleUrl>#LineStyle_"+(goodDop?"good":"worst")+"</styleUrl>\n"+
				"    <LineString>\n"+
				"      <tessellate>1</tessellate>\n"+
				"      <coordinates>\n");
			out.flush();
			return out;
		} catch (IOException e) {
			e.printStackTrace();
		}
		return null;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.producer.PositionConsumer#endOfTrack()
	 */
	public void endOfTrack(FileWriter out) {
		if(out!=null){
			// Write KML footer part
			try {
				String circle = null;
//				if(positions.size()>0){
//					ReceiverPosition last = positions.get(positions.size()-1);
//					circle = generateCircle(last.getGeodeticLatitude(), last.getGeodeticLongitude(), last.getGeodeticHeight(), 90, last.getpDop());
//				}

				out.write("</coordinates></LineString></Placemark></Folder>"+(timeline==null?"":timeline+"</Folder>")+(circle!=null?circle:"")+"</Document>\n");
				// Close FileWriter
				out.close();
			} catch (IOException e) {
				e.printStackTrace();
			}

		}
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

	private String generateCircle(double centerlat_form, double centerlong_form, double height, int num_points, double radius_form) {
		double lat1, long1;
		double d_rad;
		double d;
		double delta_pts;
		double radial, lat_rad, dlon_rad, lon_rad;

//		double degreeToRadian = Math.PI / 180.0;

		String result = "";

		// convert coordinates to radians
		lat1 = Math.toRadians(centerlat_form);
		long1 = Math.toRadians(centerlong_form);

		// Earth measures
		// Year Name a (meters) b (meters) 1/f Where Used
		// 1980 International 6,378,137 6,356,752 298.257 Worldwide
		d = radius_form;
		d_rad = d / 6378137;

		result = "<Folder>\n<name>Circle</name>\n<visibility>1</visibility>\n<Placemark>\n<name></name>\n<styleUrl>CircleStyle</styleUrl>\n<LinearRing>\n<coordinates>\n";
		// System.out.write(c);
		// System.out.println(c);

		delta_pts = 360/(double)num_points;

		// loop through the array and write path linestrings
		for (int i = 0; i < num_points; i++) {
			radial = Math.toRadians((double)i*delta_pts);
			//radial = Math.toRadians((double) i);

			// This algorithm is limited to distances such that dlon <pi/2
			lat_rad = Math.asin(Math.sin(lat1) * Math.cos(d_rad)
					+ Math.cos(lat1) * Math.sin(d_rad) * Math.cos(radial));
			dlon_rad = Math.atan2(Math.sin(radial) * Math.sin(d_rad)
					* Math.cos(lat1), Math.cos(d_rad) - Math.sin(lat1)
					* Math.sin(lat_rad));
			lon_rad = ((long1 + dlon_rad + Math.PI) % (2 * Math.PI))
					- Math.PI;

			// write results
			result += "" + Math.toDegrees(lon_rad) + ",";
			result += "" + Math.toDegrees(lat_rad) + ",";
			result += "" + height + "\n";
		}
		// output footer
		result += "</coordinates>\n</LinearRing>\n</Placemark>\n</Folder>";

		return result;
	}

	private String reverse(String string){
		return new StringBuffer(string).reverse().toString();
	}

	/**
	 * @return the goodColorLine in hex format RRGGBB
	 */
	public String getGoodColorLine() {
		return reverse(goodColorLine);
	}

	/**
	 * @param goodColorLine the goodColorLine to set in hex format RRGGBB
	 */
	public void setGoodColorLine(String goodColorLine) {
		this.goodColorLine = reverse(goodColorLine);
	}

	/**
	 * @return the goodOpacity in hex format (range 00..FF)
	 */
	public String getGoodOpacity() {
		return goodOpacity;
	}

	/**
	 * @param goodOpacity the goodOpacity to set in hex format (range 00..FF)
	 */
	public void setGoodOpacity(String goodOpacity) {
		this.goodOpacity = goodOpacity;
	}

	/**
	 * @return the goodLinePixelWidth
	 */
	public int getGoodLinePixelWidth() {
		return goodLinePixelWidth;
	}

	/**
	 * @param goodLinePixelWidth the goodLinePixelWidth to set
	 */
	public void setGoodLinePixelWidth(int goodLinePixelWidth) {
		this.goodLinePixelWidth = goodLinePixelWidth;
	}

	/**
	 * @return the worstColorLine in hex format RRGGBB
	 */
	public String getWorstColorLine() {
		return reverse(worstColorLine);
	}

	/**
	 * @param worstColorLine the worstColorLine to set in hex format RRGGBB
	 */
	public void setWorstColorLine(String worstColorLine) {
		this.worstColorLine = reverse(worstColorLine);
	}

	/**
	 * @return the worstOpacity in hex format (range 00..FF)
	 */
	public String getWorstOpacity() {
		return worstOpacity;
	}

	/**
	 * @param worstOpacity the worstOpacity to set in hex format (range 00..FF)
	 */
	public void setWorstOpacity(String worstOpacity) {
		this.worstOpacity = worstOpacity;
	}

	/**
	 * @return the worstLinePixelWidth
	 */
	public int getWorstLinePixelWidth() {
		return worstLinePixelWidth;
	}

	/**
	 * @param worstLinePixelWidth the worstLinePixelWidth to set
	 */
	public void setWorstLinePixelWidth(int worstLinePixelWidth) {
		this.worstLinePixelWidth = worstLinePixelWidth;
	}

	/**
	 * @param debug the debug to set
	 */
	public void setDebug(boolean debug) {
		this.debug = debug;
	}

	public boolean isDebug() {
		return debug;
	}

	@Override
	public void run() {
		int last = 0;
    FileWriter out = startOfTrack();
		while(!isInterrupted() || last<positions.size()){
//					goodDop = false;
			for( ; last<positions.size(); last++ ) {
				writeCoordinate(positions.get(last), out);
			}
			try {
        Thread.sleep(200);
      } catch (InterruptedException e) {
        interrupt();
      }
		}
    endOfTrack(out);
	}
	
	public void cleanStop(){
		interrupt();
	}
}

