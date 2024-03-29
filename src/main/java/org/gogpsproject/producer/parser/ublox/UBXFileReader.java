/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
package org.gogpsproject.producer.parser.ublox;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Vector;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.ephemeris.EphemerisSystem;
import org.gogpsproject.positioning.Coordinates;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.ObservationsProducer;
import org.gogpsproject.producer.StreamEventListener;
import org.gogpsproject.producer.StreamEventProducer;
import org.gogpsproject.producer.StreamResource;
import org.gogpsproject.producer.parser.IonoGps;

/**
 * <p>
 * Read an UBX File and implement Observation and Navigation producer (if AID-HUI and AID-EPH has been recorded)
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com
 */

public class UBXFileReader extends EphemerisSystem implements ObservationsProducer, NavigationProducer, StreamResource, StreamEventProducer {

	private InputStream in;
	private UBXReader reader;
	private File file;
	private Observations obs = null;
	private IonoGps iono = null;
	// TODO support past times, now keep only last broadcast data
	private HashMap<Integer,EphGps> ephs = new HashMap<Integer,EphGps>();
	
	boolean gpsEnable = true;  // enable GPS data reading
	boolean qzsEnable = true;  // enable QZSS data reading
  boolean gloEnable = true;  // enable GLONASS data reading	
  boolean galEnable = true;  // enable Galileo data reading
  boolean bdsEnable = true;  // enable BeiDou data reading
	
	Boolean[] multiConstellation = {gpsEnable, qzsEnable, gloEnable, galEnable, bdsEnable};

	Coordinates definedPosition = Coordinates.globalXYZInstance(0.0, 0.0, 0.0);
	
	public UBXFileReader(File file) {
		this.file = file;
	}
	
	public UBXFileReader(File file, Boolean[] multiConstellation) {
		this.file = file;
		this.multiConstellation = multiConstellation;		
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#getApproxPosition()
	 */
	@Override
	public Coordinates getDefinedPosition() {
		return definedPosition;
	}

	public void setDefinedPosition( Coordinates definedPosition ) {
		definedPosition.cloneInto( this.definedPosition );
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#getCurrentObservations()
	 */
	@Override
	public Observations getCurrentObservations() {
		return obs;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#hasMoreObservations()
	 */
	public boolean hasMoreObservations() {
		boolean moreObs = false;
		try {
			moreObs = in.available()>0;
		} catch (IOException e) {
		}
		return moreObs;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#init()
	 */
	@Override
	public void init() throws Exception {
		this.in = new FileInputStream(file);
		this.reader = new UBXReader(in, multiConstellation, null);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#nextObservations()
	 */
	@Override
	public Observations getNextObservations() {
		try{
			while(in.available()>0){
				try{
					int data = in.read();
					if(data == 0xB5){
						Object o = reader.readMessage();
						if(o instanceof Observations){
							return (Observations)o;
						}else
							if(o instanceof IonoGps){
								iono = (IonoGps)o;
							}
						if(o instanceof EphGps){

							EphGps e = (EphGps)o;
							ephs.put(new Integer(e.getSatID()), e);
						}
					}else if(data == 0x24){
						//System.out.println("NMEA detected");
						//no warning, may be NMEA
						//System.out.println("Wrong Sync char 1 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
					}
				}catch(UBXException ubxe){
					System.err.println(ubxe);
					//					ubxe.printStackTrace();
				}
			}
		}catch(IOException e){
			e.printStackTrace();
		}
		return null;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#release()
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs) throws InterruptedException {
		try {
			in.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double)
	 */
	@Override
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
		EphGps eph = ephs.get(new Integer(satID));
		if (eph != null) {
//			char satType = eph.getSatType();
			SatellitePosition sp = computePositionGps(obs, satID, satType, eph, receiverClockError);
			return sp;
		}
		return null ;
	}

	public void enableDebugMode(Boolean enableDebug) {
		reader.enableDebugMode(enableDebug);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getIono(long)
	 */
	@Override
	public IonoGps getIono(long unixTime) {
		return iono;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#addStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void addStreamEventListener(StreamEventListener streamEventListener) {
		reader.addStreamEventListener(streamEventListener);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#getStreamEventListeners()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Vector<StreamEventListener> getStreamEventListeners() {
		return reader.getStreamEventListeners();
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void removeStreamEventListener(StreamEventListener streamEventListener) {
		reader.removeStreamEventListener(streamEventListener);
	}
}
