package org.gogpsproject.consumer;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.SVInfo;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.TopocentricCoordinates;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.SVInfoListener;
import org.gogpsproject.producer.parser.IonoGps;
import org.gogpsproject.producer.parser.rinex.RinexNavigationParser;

public class SVInfoLogger implements SVInfoListener {
	HashMap<Integer,SVInfo> satpos = new HashMap<>();
	PrintWriter pw;
	RinexNavigationParser nav;
	private Coordinates rover;
	
	public SVInfoLogger( String filename, RinexNavigationParser nav ) throws Exception {
    File csvOutputFile = new File(filename);
    pw = new PrintWriter(csvOutputFile);
    this.nav = nav;
    this.nav.init();
	}
	
	public String convertToCSV(String[] data) {
    return Stream.of(data)
//      .map(this::escapeSpecialCharacters)
      .collect(Collectors.joining(","));
	}
	
//	@Override
//	public void addObservations(Observations o) {
//		for( int i=0; i<o.getNumSat(); i++ ) {
//			ObservationSet os = o.getSatByIdx(i);
//
//			SVInfo sv = satpos.get(os.getSatID());
//			if( sv == null || !sv.isValid())
//				continue;
//			
//			int diff = o.getRefTime().getGpsWeekSec() - sv.getRefTime().getGpsWeekSec();
//			
//			if( diff>2)
//				continue;
//			
//			String[] line = new String[] {
//					Integer.toString( o.getRefTime().getGpsWeekSec()),
//					Integer.toString( os.getSatID()),
//					Integer.toString((int)sv.az), //* 255.0f / 360.0f,
//					Integer.toString((int)sv.el),
//					Double.toString( os.getCodeC(0)),
//					Double.toString( os.getDoppler(0)),
//					Double.toString( os.getPhaserange(0)),
//					Double.toString( os.getSignalStrength(0))
//			};
//			
//			String csvl = convertToCSV(line);
//			pw.println(csvl);
//		}
//		System.out.println( o.getRefTime().getGpsWeekSec());
////	pw.flush();
//	}

	@Override
	public void addObservations(Observations o) {
		long unixTime = o.getRefTime().getMsec();

		for( int i=0; i<o.getNumSat(); i++ ) {
			ObservationSet os = o.getSatByIdx(i);

			int    satID = os.getSatID();
			char satType = os.getSatType();
			
			EphGps eph = nav.findEph( unixTime, satID, satType);
			if( eph == null ) // || !eph.getSvHealth() )
				continue;
			
      SatellitePosition satpos = nav.getGpsSatPosition( o, satID, satType, 0);
			
      if(satpos==null || satpos.equals( SatellitePosition.UnhealthySat )) {
        continue;
	  	}

      TopocentricCoordinates topo = new TopocentricCoordinates();
      topo.computeTopocentric(rover, satpos);
      
      if( topo.getElevation()<0 )
      	continue;
      
			String[] line = new String[] {
					Integer.toString( o.getRefTime().getGpsWeekSec()),
					Integer.toString( os.getSatID()),
					Integer.toString((int)topo.getAzimuth()), //* 255.0f / 360.0f,
					Integer.toString((int)topo.getElevation()),
					Double.toString( os.getCodeC(0)),
					Double.toString( os.getDoppler(0)),
					Double.toString( os.getPhaserange(0)),
					Double.toString( os.getSignalStrength(0))
			};
			
			String csvl = convertToCSV(line);
			pw.println(csvl);
		}
		System.out.println( o.getRefTime().getGpsWeekSec() + " " + o.getNumSat());
//	pw.flush();
	}
	
	@Override
	public void addSVInfo(List<SVInfo> spl) {
		spl.forEach( sp -> {
			satpos.put(sp.getSatID(), sp);
		});
	}
	
	@Override
	public void streamClosed() {
		pw.close();
	}

	@Override
	public void addIonospheric(IonoGps iono) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void addEphemeris(EphGps eph) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void setDefinedPosition(Coordinates definedPosition) {
		this.rover = definedPosition;
		rover.computeECEF();
	}

	@Override
	public Observations getCurrentObservations() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void pointToNextObservations() {
	}


}
