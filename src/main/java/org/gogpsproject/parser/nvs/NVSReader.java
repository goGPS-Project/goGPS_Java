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
package org.gogpsproject.parser.nvs;

import java.io.BufferedInputStream;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

import org.gogpsproject.EphGps;
import org.gogpsproject.IonoGps;
import org.gogpsproject.Observations;
import org.gogpsproject.StreamEventListener;
import org.gogpsproject.StreamEventProducer;
/**
 * <p>
 * Read and parse NVS messages
 * </p>
 *
 * @author Daisuke Yoshida (Osaka City University), Lorenzo Patocchi (cryms.com)
 */
public class NVSReader implements StreamEventProducer {
	private InputStream is = null;
	private Vector<StreamEventListener> streamEventListeners = new Vector<StreamEventListener>();
	private Boolean debugModeEnabled = false;
	
	boolean gpsEnable = true;  // enable GPS data reading
	boolean qzsEnable = false;  // enable QZSS data reading
    boolean gloEnable = true;  // enable GLONASS data reading	
    boolean galEnable = true;  // enable Galileo data reading
    boolean bdsEnable = false;  // enable BeiDou data reading

    private Boolean[] multiConstellation = {gpsEnable, qzsEnable, gloEnable, galEnable, bdsEnable};

	//TODO
	public NVSReader(BufferedInputStream is, Boolean[] multiConstellation){
		this(is,null, null);		
	}
	
	public NVSReader(BufferedInputStream is, StreamEventListener eventListener){
		this.is = is;
		addStreamEventListener(eventListener);
	}
	
	public NVSReader(BufferedInputStream is, Boolean[] multiConstellation, StreamEventListener eventListener){
		this.is = is;
		this.multiConstellation = multiConstellation;
		addStreamEventListener(eventListener);
	}

	public Object readMessage() throws IOException, NVSException{

			int data = is.read();
			
			if(data == 0xf7){ // F7
				
				ByteArrayInputStream msg = new ByteArrayInputStream(removeDouble0x10(findMessageEnd()));

				DecodeF7 decodeF7 = new DecodeF7(msg, multiConstellation);
				
				EphGps eph = decodeF7.decode();
				//if(streamEventListeners!=null && eph!=null){
					//for(StreamEventListener sel:streamEventListeners){
						//sel.addEphemeris(eph);
					//}
				//}

				return eph;
						
			}else
			if (data == 0xf5){  // F5
				
				byte[] byteArray = removeDouble0x10(findMessageEnd());
				ByteArrayInputStream msg = new ByteArrayInputStream(byteArray);

				DecodeF5 decodeF5 = new DecodeF5(msg, multiConstellation);										

				Observations o = decodeF5.decode(null, byteArray.length*8);
				if(streamEventListeners!=null && o!=null){
					for(StreamEventListener sel:streamEventListeners){
						Observations oc = (Observations)o.clone();
						sel.addObservations(oc);
					}
				}

				return o;

			}else
			if (data == 0x4a){ // 4A
				
				ByteArrayInputStream msg = new ByteArrayInputStream(removeDouble0x10(findMessageEnd()));
				
				Decode4A decode4A = new Decode4A(msg);
				
				IonoGps iono = decode4A.decode();
				//if(streamEventListeners!=null && iono!=null){
					//for(StreamEventListener sel:streamEventListeners){
						//sel.addIonospheric(iono);
					//}
				//}

				return iono;

			}else
			if (data == 0x62){
				findMessageEnd();
				return -1;				
			}else
			if (data == 0x70){
				findMessageEnd();
				return -1;				
			}else
			if (data == 0x4b){
				findMessageEnd();
				return -1;					
			}else
			if (data == 0xF6){
				findMessageEnd();
				return -1;						
			}
			else
			if (data == 0xe7){
				findMessageEnd();
				return -1;							
			}
			else
			if (debugModeEnabled) {
				System.out.println("Warning: wrong sync char 2 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
			}

			return null;
	}
	
	private Byte[] findMessageEnd() {
		List<Byte> data = new ArrayList<Byte>();
		boolean stop = false;
		try {
			while (!stop) {
				if(is.available()>0){
					byte value;
					value = (byte) is.read();
					data.add(value);
					if(value == 0x10){  // <dle>
						value = (byte) is.read();
						data.add(value);
						if(value == 0x03){  // <ETX>
							stop = true;
						}
					}
				} else {
					// no bytes to read, wait 1 msec
					try {
						Thread.sleep(1);
					} catch (InterruptedException e) {}
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		return data.toArray(new Byte[data.size()]);
	}

	private byte[] removeDouble0x10(Byte[] in) {
		byte[] out = new byte[in.length];
		int i, k;
		for(i = 0, k = 0; i < in.length; i++, k++) {
			out[k] = in[i];
			if (in[i] == 0x10 && in[i+1] == 0x10) {
				i++;
			}
		}
		return Arrays.copyOfRange(out,0,in.length-(i-k));
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
