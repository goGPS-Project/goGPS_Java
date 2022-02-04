package org.gogpsproject.consumer;

import java.io.File;
import java.io.PrintWriter;
import java.util.HashMap;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.RoverPosition;
import org.gogpsproject.positioning.SVInfo;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.TopocentricCoordinates;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.RinexNavigationProducer;
import org.gogpsproject.producer.StreamEventListener;
import org.gogpsproject.producer.parser.IonoGps;

/** Log satellite observables + azimuth/elevation to csv  */
public class SVLogger /*extends EphemerisSystem*/ implements StreamEventListener, PositionConsumer {
	HashMap<Integer,SVInfo> satpos = new HashMap<>();
	PrintWriter pw;
	RinexNavigationProducer nav;
	private Coordinates rover;

	/** standard subms clock error in s*/
  double clockError = 0;
  
	double clockErrorRate = 0; 
	
	public SVLogger( String filename, RinexNavigationProducer nav ) throws Exception {
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
	
	@Override
	public void addObservations(Observations o) {
		long unixTime = o.getRefTime().getMsec();

		for( int i=0; i<o.getNumSat(); i++ ) {
			ObservationSet os = o.getSatByIdx(i);

			if( !os.inUse() )
				continue;
			
			int    satID = os.getSatID();
			char satType = os.getSatType();
			
			EphGps eph = nav.findEph( unixTime, satID, satType);
			if( eph == null ) // || !eph.getSvHealth() )
				continue;
			
      SatellitePosition satpos = nav.getGpsSatPosition( o, satID, satType, clockError);
		
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
					String.format("%3.2f", topo.getAzimuth()), //* 255.0f / 360.0f,
					String.format("%3.2f", topo.getElevation()),
					String.format("%10.3f", os.getCodeC(0)),
          String.format("%10.3f", os.getDoppler(0)),
          String.format("%10.3f", os.getPhaseCycles(0)),
          String.format("%3.1f", os.getSignalStrength(0)),
          String.format("%10.9f", clockError ),
          String.format("%10.9f", clockErrorRate )
//					os.inUse()?"Y":"N"
			};
			
			String csvl = convertToCSV(line);
			pw.println(csvl);
		}
		System.out.println( o.getRefTime().getGpsWeekSec() + " " + o.getNumSat());
//	pw.flush();
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

	@Override
	public void addCoordinate(RoverPosition coord) {
		this.clockError = coord.getClockError();
		this.clockErrorRate = coord.getClockErrorRate();
	}

	@Override
	public void event(int event) {
		// TODO Auto-generated method stub
		
	}
}
