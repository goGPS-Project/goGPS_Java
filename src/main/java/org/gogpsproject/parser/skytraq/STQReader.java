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
package org.gogpsproject.parser.skytraq;

import java.io.IOException;
import java.io.InputStream;
import java.util.Vector;

import org.gogpsproject.Observations;
import org.gogpsproject.StreamEventListener;
import org.gogpsproject.StreamEventProducer;
/**
 * <p>
 * Read and parse STQ messages
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com, Eugenio Realini
 */
public class STQReader implements StreamEventProducer {
	private InputStream in;
	private Vector<StreamEventListener> streamEventListeners = new Vector<StreamEventListener>();
	private Boolean debugModeEnabled = false;
	//	private StreamEventListener streamEventListener;

	public STQReader(InputStream is){
		this(is,null);
	}
	public STQReader(InputStream is, StreamEventListener eventListener){
		this.in = is;
		addStreamEventListener(eventListener);
	}

	public Object readMessage(Observations o) throws IOException, STQException{

	//	int data = in.read();
	//	if(data == 0xA0){
		int data = in.read();
		if(data == 0xA1){

			// parse big endian data
			int[] length = new int[2];

			length[0] = in.read();
			length[1] = in.read();

			int len = length[0]*256+length[1];

			if (len == 0) {
				throw new STQException("Zero-length SkyTraq message");
			}

			data = in.read(); // message type
			boolean parsed = false;
			if (data == 0xDD && o!=null) { //RAW-MEAS
				// RAW-MEAS
				DecodeRAWMEAS decodegps = new DecodeRAWMEAS(in, o);
				parsed = true;

				o = decodegps.decode(len);
				if (o!=null && this.debugModeEnabled) {
					System.out.println("Decoded observations");
				}
				if(streamEventListeners!=null && o!=null){
					for(StreamEventListener sel:streamEventListeners){
						Observations oc = (Observations)o.clone();
						sel.addObservations(oc);
					}
				}
			}else
			if (data == 0xDC) { //MEAS-TIME
				// MEAS-TIME (measurement time)
				DecodeMEASTIME decodegps = new DecodeMEASTIME(in);
				parsed = true;

				o = decodegps.decode(len);
				if (o!=null && this.debugModeEnabled) {
					System.out.println("Decoded time message");
				}
				return o;
//			}else
//			if (data == 0xB1) {
//				// GPS-EPH (ephemerides)
//				DecodeGPSEPH decodegps = new DecodeGPSEPH(in);
//				parsed = true;
//
//				EphGps eph = decodegps.decode();
//				if (eph!=null && this.debugModeEnabled) {
//					System.out.println("Decoded ephemeris for satellite " + eph.getSatID());
//				}
//				if(streamEventListeners!=null && eph!=null){
//					for(StreamEventListener sel:streamEventListeners){
//						sel.addEphemeris(eph);
//					}
//				}
//				return eph;
			}
			if(!parsed){

				if (this.debugModeEnabled) {
					System.out.println("Warning: STQ message not decoded; skipping "+len+" bytes");
				}
				for (int b = 0; b < len+2; b++) {
					in.read();
				}
			}
		}else{
			if (this.debugModeEnabled) {
				System.out.println("Warning: wrong sync char 2 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
			}
			return o;
		}
	//	}else{
	//		//no warning, may be NMEA
	//		//System.out.println("Warning: wrong sync char 1 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
	//	}
		return null;
	}
	/**
	 * @return the streamEventListener
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Vector<StreamEventListener> getStreamEventListeners() {
		return (Vector<StreamEventListener>)streamEventListeners.clone();
	}
	/**
	 * @param streamEventListener the streamEventListener to set
	 */
	@Override
	public void addStreamEventListener(StreamEventListener streamEventListener) {
		if(streamEventListener==null) return;
		if(!streamEventListeners.contains(streamEventListener))
			this.streamEventListeners.add(streamEventListener);
	}
	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void removeStreamEventListener(
			StreamEventListener streamEventListener) {
		if(streamEventListener==null) return;
		if(streamEventListeners.contains(streamEventListener))
			this.streamEventListeners.remove(streamEventListener);
	}
	public void enableDebugMode(Boolean enableDebug) {
		this.debugModeEnabled = enableDebug;
	}

}
