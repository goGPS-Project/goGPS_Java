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

import java.io.FileOutputStream;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.Map;
import java.util.TimeZone;
import java.util.concurrent.ConcurrentHashMap;

import javax.xml.bind.JAXBContext;
import javax.xml.bind.JAXBException;
import javax.xml.bind.Marshaller;
import javax.xml.stream.XMLOutputFactory;
import javax.xml.stream.XMLStreamWriter;

import org.gogpsproject.Status;
import org.gogpsproject.positioning.RoverPosition;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;

import de.micromata.opengis.kml.v_2_2_0.AltitudeMode;
import de.micromata.opengis.kml.v_2_2_0.BalloonStyle;
import de.micromata.opengis.kml.v_2_2_0.ColorMode;
import de.micromata.opengis.kml.v_2_2_0.Data;
import de.micromata.opengis.kml.v_2_2_0.ExtendedData;
import de.micromata.opengis.kml.v_2_2_0.Folder;
import de.micromata.opengis.kml.v_2_2_0.Icon;
import de.micromata.opengis.kml.v_2_2_0.IconStyle;
import de.micromata.opengis.kml.v_2_2_0.Kml;
import de.micromata.opengis.kml.v_2_2_0.LabelStyle;
import de.micromata.opengis.kml.v_2_2_0.LineStyle;
import de.micromata.opengis.kml.v_2_2_0.Placemark;
import de.micromata.opengis.kml.v_2_2_0.Point;
import de.micromata.opengis.kml.v_2_2_0.Schema;
import de.micromata.opengis.kml.v_2_2_0.SimpleField;
import de.micromata.opengis.kml.v_2_2_0.TimeStamp;
import de.micromata.opengis.kml.v_2_2_0.gx.Track;
import txw2.output.IndentingXMLStreamWriter;

/**
 * <p>
 * Produces KML file
 * </p>
 *
 * @author Emanuele Ziglioli 
 */

public class JakKmlProducer implements PositionConsumer {

  public static final String ATOMNS = "http://www.w3.org/2005/Atom";
  public static final String KMLNS = "http://www.opengis.net/kml/2.2";
  public static final String GXNS = "http://www.google.com/kml/ext/2.2"; 
  public static final String XALNS = "urn:oasis:names:tc:ciq:xsdschema:xAL:2.0";
  
  private DecimalFormat cf = new DecimalFormat("#.00000");

  private SimpleDateFormat timeKML = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");

  private String filename = null;
  private double goodDopThreshold = 3.0;
  private double goodEResThreshold = 10.0;
  private String goodColorLine = "00ff00";
  private String goodOpacity = "ff";
  private int goodLinePixelWidth = 3;
  private String worstColorLine = "0000ff";
  private String worstOpacity = "ff";
  private int worstLinePixelWidth = 3;
  private boolean debug=false;

  private final static TimeZone TZ = TimeZone.getTimeZone("GMT");

  private transient JAXBContext jc = null;
  private transient Marshaller m = null;
  XMLStreamWriter xmlOut;
  Track track = null;
  Placemark trackPlacemark = null;
  static final String GREEN = "FF00FF55";
  static final String RED   = "ff6775fd";

  SimpleDateFormat sdf = Observations.getGMTdf();
  int last = 0;

  public AltitudeMode altitudeMode = AltitudeMode.CLAMP_TO_GROUND;

	public JakKmlProducer(String filename, double goodDopTreshold) throws IOException{

	  m = createMarshaller();
	  this.filename = filename;
		this.goodDopThreshold = goodDopTreshold;
		timeKML.setTimeZone(TZ);
	}

  private JAXBContext getJaxbContext() throws JAXBException {
    if (this.jc == null) {
      this.jc = JAXBContext.newInstance(new Class[] { Kml.class });
    }
    return this.jc;
  }

  /** Improves performance by caching contexts which are expensive to create. */
  private final static Map<String, JAXBContext> contexts = new ConcurrentHashMap<String, JAXBContext>();

  public static synchronized JAXBContext getContext(String contextPath,
      ClassLoader classLoader) throws JAXBException {
  // Contexts are thread-safe so reuse those.
  JAXBContext result = contexts.get(contextPath);

  if (result == null) {
      result = (classLoader == null) ? JAXBContext
              .newInstance(contextPath) : JAXBContext.newInstance(
              contextPath, classLoader);
      contexts.put(contextPath, result);
  }

  return result;
}
  
  private Marshaller createMarshaller() {
    // contexPath = de.micromata.opengis.kml.v_2_2_0
    
    if (this.m == null) {
      try {
        this.m = getJaxbContext().createMarshaller();
        m.setProperty( Marshaller.JAXB_FORMATTED_OUTPUT, true );
        m.setProperty( Marshaller.JAXB_FRAGMENT, true );
      } catch (JAXBException e) {
        throw new RuntimeException(e);
      }
    }
    return this.m;
  }

	/* (non-Javadoc)
	 * @see org.gogpsproject.producer.PositionConsumer#startOfTrack()
	 */
	public XMLStreamWriter startOfTrack() {
		try {
		    FileOutputStream fout = new FileOutputStream(filename);

		    XMLOutputFactory f = XMLOutputFactory.newInstance();
		    f.setProperty("escapeCharacters", false);
		    f.setProperty(XMLOutputFactory.IS_REPAIRING_NAMESPACES, true);
		    XMLStreamWriter xmlOut1 = f.createXMLStreamWriter( fout, "utf-8" );
        xmlOut = new IndentingXMLStreamWriter( xmlOut1 );
        xmlOut.writeStartDocument("UTF-8", "1.0");
        xmlOut.writeStartElement("kml");
        xmlOut.writeDefaultNamespace( KMLNS );
        xmlOut.writeNamespace("atom", ATOMNS );
        xmlOut.writeNamespace("gx", GXNS );
        xmlOut.writeNamespace("xal", XALNS );
            
        xmlOut.writeStartElement("Document");
        
        Icon icon = new Icon().withHref("http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png");
        IconStyle validIconStyle = new IconStyle()
                                .withIcon( icon )
                                .withScale(.25)
                                .withColor(GREEN);
        IconStyle invalidIconStyle = new IconStyle()
                                .withIcon( icon )
                                .withScale(.25)
                                .withColor(RED);
        
        Icon trackIcon = new Icon().withHref("http://earth.google.com/images/kml-icons/track-directional/track-0.png");
        IconStyle trackIconStyle = new IconStyle()
            .withIcon( trackIcon )
            .withScale(.5)
            .withColor(GREEN);
        
        LabelStyle labelStyle = new LabelStyle().withScale(0.7);
        LabelStyle idLabelStyle = new LabelStyle()
                                .withColor("00000000")
                                .withColorMode( ColorMode.NORMAL )
                                .withScale(1);
        LineStyle validLineStyle   = new LineStyle().withColor(GREEN);
        LineStyle invalidLineStyle = new LineStyle().withColor(RED);

        LineStyle lineStyle_t = new LineStyle().withColor(GREEN).withWidth(1d);
        
        Schema schema = new Schema().withId("schema1")
                        .withSimpleField(Arrays.asList(
                            new SimpleField().withName("index").withType("uint"),
                            new SimpleField().withName("coord").withType("string"),
                            new SimpleField().withName("hDop").withType("float"),
                            new SimpleField().withName("eRes").withType("float"),
                            new SimpleField().withName("RTC Time").withType("string"),
                            new SimpleField().withName("FIX Time").withType("string"),
                            new SimpleField().withName("inUse").withType("string"),
                            new SimpleField().withName("sats").withType("string")
        ));
        m.marshal(schema, xmlOut);

//      Style baloonStyle = new Style().withId("baloonStyle");
        String text = "\r\n<![CDATA[\r\n"
            + "<b>Point $[index]</b><br/><br/>"
            + "<b>$[coord]</b><br/><br/>"
            + "<b>RTC Time:</b> $[RTC Time]<br/>\r\n"
            + "<b>FIX TIme:</b> $[FIX Time]<br/>\r\n"
            + "<b>hDop:</b> $[hDop]<br/>\r\n"
            + "<b>eRes:</b> $[eRes]<br/>\r\n"
            + "<b>Sats:</b> $[inUse]<br/>\r\n"
            + "<table>\r\n" 
            + "<tr align=right><th>satId</th><th>Code</th><th>Doppler</th><th>SNR</th><th>el</th><th>eRes</th><th>inUse</th></tr>"
            + "$[sats]<br/>\r\n" 
            + "</table>\r\n" 
            + "]]>\r\n";
        BalloonStyle balloonStyle = new BalloonStyle().withText(text);
        
        Folder sf = new Folder().withName("Styles");
        sf.setVisibility(false);
        
        sf.createAndAddStyle().withId("Valid")
          .withIconStyle( validIconStyle )
          .withLabelStyle(labelStyle)
          .withLabelStyle(idLabelStyle)
          .withLineStyle(validLineStyle)
          .withBalloonStyle(balloonStyle);

        sf.createAndAddStyle().withId("Invalid")
          .withIconStyle( invalidIconStyle )
          .withLabelStyle( labelStyle )
          .withLabelStyle( idLabelStyle )
          .withLineStyle( invalidLineStyle )
          .withBalloonStyle( balloonStyle );
        
        sf.createAndAddStyle().withId("track")
          .withLineStyle( lineStyle_t )
          .withIconStyle( trackIconStyle )
          .withLabelStyle( labelStyle )
          .withLabelStyle( idLabelStyle );
        
        m.marshal(sf, xmlOut);
        
        xmlOut.writeStartElement("Folder");
        xmlOut.writeStartElement( "name");
        xmlOut.writeCharacters("Points");
        xmlOut.writeEndElement();  // name
        
        xmlOut.flush();
        
        track = new Track().withAltitudeMode(altitudeMode);
        trackPlacemark = new Placemark()
            .withName( "Track" )
            .withStyleUrl("#track")
            .withGeometry(track);

        return xmlOut;
		} catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}

	/* (non-Javadoc)
   * @see org.gogpsproject.producer.PositionConsumer#addCoordinate(org.gogpsproject.Coordinates)
   */
  @Override
  public void addCoordinate( RoverPosition c ) {
  	if(debug) System.out.println("Lon:"+cf.format(c.getGeodeticLongitude()) + " " // geod.get(0)
  			+"Lat:"+ cf.format(c.getGeodeticLatitude()) + " " // geod.get(1)
  			+"H:"+ cf.format(c.getGeodeticHeight()) + "\t" // geod.get(2)
  			+"P:"+ c.getpDop()+" "
  			+"H:"+ c.gethDop()+" "
  			+"V:"+ c.getvDop()+" ");//geod.get(2)
  
      if( c.status != Status.Valid )
        return;
      
      String t = timeKML.format(new Date(c.getRefTime().getMsec()));
      String name = c.obs.index + ": " + sdf.format(new Date(c.getRefTime().getMsec()));
  
      TimeStamp ts = new TimeStamp();
      ts.setWhen(t);
      
      String coordStr = String.format ("%8.5f, %8.5f, %8.5f",
                      c.getGeodeticLatitude(), c.getGeodeticLongitude(), c.getGeodeticHeight() );
      String sats = "<![CDATA[";
      for( int i=0; i<c.obs.getNumSat(); i++ ){
        ObservationSet os = c.obs.getSatByIdx(i);
          sats += String.format( "<tr align=right>" + 
            "<td>" + os.getSatID() + "</td>" 
           +"<td>"+ (long)(os.getCodeC(0)) + "</td>"
           +"<td>"+(long)(os.getDoppler(0)) + "</td>"
           +"<td>%3.1f</td>"
           +"<td>%3.1f</td>"
           +"<td>%4.1f</td>"
           +"<td>%c</td>"
           +"</tr>", 
          Float.isNaN(os.getSignalStrength(0))?0:os.getSignalStrength(0),
          os.el,
          os.eRes,
          os.inUse()?'Y':'N');
      }
      sats+="]]>";
      ExtendedData ed = new ExtendedData();
      ed.setData(Arrays.asList(
          new Data( Long.toString(c.obs.index) ).withName("index"),
          new Data( coordStr ).withName("coord"),
          new Data( sdf.format(new Date(c.sampleTime.getMsec()))).withName("RTC Time"),
          new Data( sdf.format(new Date(c.getRefTime().getMsec()))).withName("FIX Time"),
          new Data( String.format("%4.1f", c.gethDop() )).withName("hDop"),
          new Data( String.format("%4.1f", c.eRes )).withName("eRes"),
          new Data( c.satsInUse + "/" + c.obs.getNumSat()).withName("inUse"),
          new Data( sats ).withName("sats")
      ));
      
      Placemark p = new Placemark()
          .withName(name)
          .withTimePrimitive(ts)
          .withExtendedData(ed)
          .withStyleUrl( ( c.getpDop()<goodDopThreshold && c.eRes<goodEResThreshold )? 
              "#Valid" : "#Invalid");
      
      Point pt = p.createAndSetPoint();
      pt.getCoordinates().add( new Coordinate(c.getGeodeticLongitude(), c.getGeodeticLatitude(), c.getGeodeticHeight()));
      pt.withAltitudeMode(altitudeMode);
  
      if( c.getpDop()<goodDopThreshold ){
        track.addToWhen( t );
        track.addToCoord( cf.format( c.getGeodeticLongitude() ) + " " + cf.format(c.getGeodeticLatitude()) + " " + cf.format(c.getGeodeticHeight()) );
      }
      
      try {
        m.marshal( p, xmlOut );
      } catch (JAXBException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
  }

  /* (non-Javadoc)
	 * @see org.gogpsproject.producer.PositionConsumer#endOfTrack()
	 */
	public void endOfTrack() {
			try {
        xmlOut.writeEndElement(); // dataPointFolder
        m.marshal(trackPlacemark, xmlOut);
        xmlOut.writeEndElement(); // Document
        xmlOut.writeEndElement(); // kml

				String circle = null;
//				if(positions.size()>0){
//					ReceiverPosition last = positions.get(positions.size()-1);
//					circle = generateCircle(last.getGeodeticLatitude(), last.getGeodeticLongitude(), last.getGeodeticHeight(), 90, last.getpDop());
//				}
//				out.write("</coordinates></LineString></Placemark></Folder>"+(timeline==null?"":timeline+"</Folder>")+(circle!=null?circle:"")+"</Document>\n");
				// Close FileWriter
			} catch (Exception e) {
				e.printStackTrace();
			}
			finally{
        try {
          xmlOut.flush();
          xmlOut.close();
        } catch (Exception e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
			}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.producer.PositionConsumer#event(int)
	 */
	@Override
	public void event(int event) {
	  switch( event ){
	  case EVENT_START_OF_TRACK: 
	    startOfTrack(); 
	    break;
	  case EVENT_END_OF_TRACK:			
	    endOfTrack();
	    break;
		}
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

	/**
	 * @return the debug
	 */
	public boolean isDebug() {
		return debug;
	}

	public static class Coordinate extends de.micromata.opengis.kml.v_2_2_0.Coordinate {
	  private DecimalFormat df = new DecimalFormat("#.00000");

	  public Coordinate(final double longitude, final double latitude) {
	    super( longitude, latitude );
	 }

	  public Coordinate(double lon, double lat, double alt ) {
	    super( lon, lat, alt );
    }

    @Override
	  public String toString() {
	      StringBuilder sb = new StringBuilder();
	      sb.append( df.format(longitude));
	      sb.append(",");
	      sb.append( df.format(latitude));
	      if (altitude!= 0.0D) {
	          sb.append(",");
	          sb.append(df.format(altitude));
	      }
	      return sb.toString();
	  }
	}
}
