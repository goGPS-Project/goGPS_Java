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

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Vector;

import org.gogpsproject.EphGps;
import org.gogpsproject.Observations;
import org.gogpsproject.StreamEventListener;
import org.gogpsproject.StreamEventProducer;
import org.gogpsproject.util.BufferedInputStreamCounter;

/**
 * <p>
 *
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com, Eugenio Realini
 */

public class NVSSerialReader implements Runnable,StreamEventProducer {

	private BufferedInputStreamCounter in;
	private OutputStream out;
	//private boolean end = false;
	private Thread t = null;
	private boolean stop = false;
	private Vector<StreamEventListener> streamEventListeners = new Vector<StreamEventListener>();
	//private StreamEventListener streamEventListener;
	private NVSReader reader;
	private String COMPort;
	private int measRate = 1;
	private boolean sysTimeLogEnabled = false;
	private String dateFile;
	private String outputDir = "./test";
	private boolean debugModeEnabled = false;

	public NVSSerialReader(InputStream in,OutputStream out, String COMPort, String outputDir) {
		this(in,out,COMPort,outputDir,null);
		this.COMPort = padCOMSpaces(COMPort);
	}
	
	public NVSSerialReader(InputStream in,OutputStream out,String COMPort,String outputDir,StreamEventListener streamEventListener) {
		FileOutputStream fos_nvs= null;
		COMPort = padCOMSpaces(COMPort);
		String COMPortStr = prepareCOMStringForFilename(COMPort);
		
		File file = new File(outputDir);
		if(!file.exists() || !file.isDirectory()){
		    boolean wasDirectoryMade = file.mkdirs();
		    if(wasDirectoryMade)System.out.println("Directory "+outputDir+" created");
		    else System.out.println("Could not create directory "+outputDir);
		}
		
		Date date = new Date();
		SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
		String date1 = sdf1.format(date);
		SimpleDateFormat sdfFile = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
		dateFile = sdfFile.format(date);
		
		try {
			System.out.println(date1+" - "+COMPort+" - Logging NVS stream in "+outputDir+"/"+ COMPortStr+ "_" + dateFile + ".bin");
			fos_nvs = new FileOutputStream(outputDir+"/"+COMPortStr+ "_" + dateFile + ".nvs");
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		
		this.in = new BufferedInputStreamCounter(in, fos_nvs);
		this.out = out;
		this.reader = new NVSReader(this.in,streamEventListener);
	}
	
	public boolean setBinrProtocol() throws IOException {
		Date date = new Date();
		SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
		String date1 = sdf1.format(date);
		
		NVSProtocolConfiguration msgcfg = new NVSProtocolConfiguration();
		out.write(msgcfg.getByte());
		in.skip(in.available());
		out.flush();

		int data = 0;
		try { Thread.sleep(100); } catch (InterruptedException e) {}
		if(in.available()>0){
			data = in.read();
			if(data == 0x10){
				data = in.read();
				if(data == 0x50 || data == 0xf5 || data == 0x0B){
					in.skip(in.available());
					System.out.println(date1+" - "+COMPort+" - raw data messages (F5h) enabled");
					return true;
				}
			} else {
				if (this.debugModeEnabled) {
					System.out.println("Warning: wrong sync char 1 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
				}
			}
		}
		
		String nmeacfg = "$PORZA,0,115200,3";
		nmeacfg = nmeacfg+"*"+computeNMEACheckSum(nmeacfg)+"\r\n";
		out.write(nmeacfg.getBytes());
		in.skip(in.available());
		out.flush();

		try { Thread.sleep(100); } catch (InterruptedException e) {}
		if(in.available()>0){
			data = in.read();
			if(data == 0x24){ // "$"
				data = in.read();
				if(data == 0x50){ // "P"
					in.skip(in.available());
					System.out.println(date1+" - "+COMPort+" - raw data messages (F5h) enabled");
					return true;
				}
			} else {
				if (this.debugModeEnabled) {
					System.out.println("Warning: wrong sync char 1 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
				}
			}
		}
		
		return false;
	}

	public void start()  throws IOException{
		t = new Thread(this);
		t.setName("NVSSerialReader");
		t.start();
		
		Date date = new Date();
		SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
		String date1 = sdf1.format(date);

		System.out.println(date1+" - "+COMPort+" - Measurement rate set at "+measRate+" Hz");
		NVSRateConfiguration ratecfg = new NVSRateConfiguration(measRate);
		out.write(ratecfg.getByte());
		out.flush();

		if (this.debugModeEnabled) {
			System.out.println(date1+" - "+COMPort+" - !!! DEBUG MODE !!!");
		}
	}
	public void stop(boolean waitForThread, long timeoutMs){
		stop = true;
		if(waitForThread && t!=null && t.isAlive()){
			try {
				t.join(timeoutMs);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}
	public void run() {

		int data = 0;
		FileOutputStream fos_tim = null;
		PrintStream psSystime = null;
		COMPort = padCOMSpaces(COMPort);
		String COMPortStr = prepareCOMStringForFilename(COMPort);

		Date date = new Date();
		SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
		String date1 = sdf1.format(date);
		SimpleDateFormat sdfFile = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
		dateFile = sdfFile.format(date);
		
		if (sysTimeLogEnabled) {
			System.out.println(date1+" - "+COMPort+" - System time logging enabled");
			try {
				System.out.println(date1+" - "+COMPort+" - Logging system time in "+outputDir+"/"+COMPortStr+ "_" + dateFile + "_systime.txt");
				fos_tim = new FileOutputStream(outputDir+"/"+COMPortStr+ "_" + dateFile + "_systime.txt");
				psSystime = new PrintStream(fos_tim);
				psSystime.println("GPS time                      System time");
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
		} else {
			System.out.println(date1+" - "+COMPort+" - System time logging disabled");
		}

		try {
			in.start();
			sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
			String dateSys = null;
			String dateGps = null;
			boolean f5hMsgReceived = false;
			boolean f7hMsgReceived = false;
			reader.enableDebugMode(this.debugModeEnabled);
			while (!stop) {
				if(in.available()>0){
					dateSys = sdf1.format(new Date());
					data = in.read();
					try{
						if(data == 0x10){
							Object o = reader.readMessage();
							try {
								if(o instanceof Observations){
									if(streamEventListeners!=null && o!=null){
										for(StreamEventListener sel:streamEventListeners){
											Observations co = sel.getCurrentObservations();
										    sel.pointToNextObservations();

										    f5hMsgReceived = true;
										    
										    if (this.sysTimeLogEnabled) {
										    	dateGps = sdf1.format(new Date(co.getRefTime().getMsec()));
										    	psSystime.println(dateGps +"       "+dateSys);
										    }
										}
									}
								} else if (o instanceof EphGps) {
									f7hMsgReceived = true;
								} else if (o instanceof Integer) {
									if (this.debugModeEnabled) {
										System.out.println("Unsupported message");
									}
								} else if (o == null) {
									if (this.debugModeEnabled) {
										System.out.println("Decoding error");
									}
								}
							} catch (NullPointerException e) {
							}
						} else {
							if (this.debugModeEnabled) {
								//System.out.println("Warning: wrong sync char 1 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
							}
						}
					}catch(NVSException nvse){
						nvse.printStackTrace();
					}
				}else{
					// no bytes to read, wait 1 msec
					try {
						Thread.sleep(1);
					} catch (InterruptedException e) {}
				}

				if (f5hMsgReceived) {
					int bps = in.getCurrentBps();
					if (bps != 0) {
						System.out.println(dateSys+" - "+COMPort+" - logging at "+String.format("%4d", bps)+" Bps -- total: "+in.getCounter()+" bytes");
					} else {
						System.out.println(dateSys+" - "+COMPort+" - log starting...     -- total: "+in.getCounter()+" bytes");
					}
					f5hMsgReceived = false;
				}
				if (f7hMsgReceived) {
					int bps = in.getCurrentBps();
					if (bps != 0) {
						System.out.println(dateSys+" - "+COMPort+" - logging at "+String.format("%4d", bps)+" Bps -- total: "+in.getCounter()+" bytes");
					} else {
						System.out.println(dateSys+" - "+COMPort+" - log starting...     -- total: "+in.getCounter()+" bytes");
					}
					f7hMsgReceived = false;
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		for(StreamEventListener sel:streamEventListeners){
			sel.streamClosed();
		}
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
		if(this.reader!=null)
			this.reader.addStreamEventListener(streamEventListener);
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
		this.reader.removeStreamEventListener(streamEventListener);
	}
	
	public void setRate(int measRate) {
		this.measRate = measRate;
	}
	
	public void enableSysTimeLog(Boolean enableTim) {
		this.sysTimeLogEnabled = enableTim;
	}
	
	public void enableDebugMode(Boolean enableDebug) {
		this.debugModeEnabled = enableDebug;
	}
	
	private String padCOMSpaces(String COMPortIn) {
		if (COMPortIn.substring(0, 3).equals("COM") && COMPortIn.length() == 4) {
			COMPortIn = COMPortIn + " ";
		}
		return COMPortIn;
	}
	
	private String prepareCOMStringForFilename(String COMPort) {
		String [] tokens = COMPort.split("/");
		if (tokens.length > 0) {
			COMPort = tokens[tokens.length-1].trim();          //for UNIX /dev/tty* ports
		}
		return COMPort;
	}
	
	private static String computeNMEACheckSum(String msg){
		// perform NMEA checksum calculation
		int chk = 0;

		for (int i = 1; i < msg.length(); i++){
			chk ^= msg.charAt(i);
		}
		String chk_s = Integer.toHexString(chk).toUpperCase();
		// checksum must be 2 characters!
		while (chk_s.length() < 2){
			chk_s = "0" + chk_s;
		}
		return chk_s;

	}
	
	public void setOutputDir(String outDir) {
		this.outputDir = outDir;
	}
}
