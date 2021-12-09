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
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.SVInfoListener;
import org.gogpsproject.producer.parser.IonoGps;

public class SVInfoLogger implements SVInfoListener {
	HashMap<Integer,SVInfo> satpos = new HashMap<>();
	PrintWriter pw;
	
	public SVInfoLogger( String filename ) throws IOException {
    File csvOutputFile = new File(filename);
    pw = new PrintWriter(csvOutputFile);
	}
	
	public String convertToCSV(String[] data) {
    return Stream.of(data)
//      .map(this::escapeSpecialCharacters)
      .collect(Collectors.joining(","));
	}
	
	@Override
	public void addObservations(Observations o) {
		for( int i=0; i<o.getNumSat(); i++ ) {
			ObservationSet os = o.getSatByIdx(i);
			SVInfo sv = satpos.get(os.getSatID());

			String[] line = new String[] {
					Integer.toString( o.getRefTime().getGpsWeekSec()),
					Integer.toString( os.getSatID()),
					Integer.toString((int)sv.az), //* 255.0f / 360.0f,
					Integer.toString((int)sv.el),
					Double.toString( os.getCodeC(0)),
					Double.toString( os.getDoppler(0)),
					Double.toString( os.getPhaserange(0)),
					Double.toString(os.getSignalStrength(0))
			};
			
			String csvl = convertToCSV(line);
			pw.println(csvl);
		}
		System.out.println( o.getRefTime().getGpsWeekSec());
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
		// TODO Auto-generated method stub
		
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
