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

import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;
import java.util.Vector;

import org.gogpsproject.StreamEventListener;
import org.gogpsproject.StreamEventProducer;
import org.gogpsproject.StreamResource;

public class NVSSerialConnection  implements StreamResource, StreamEventProducer{
	private InputStream inputStream;
	private OutputStream outputStream;
	private boolean connected = false;

	private SerialPort serialPort;

	private NVSSerialReader nvsReader;
	//private StreamEventListener streamEventListener;

	private String portName;
	private int speed;
	private int setMeasurementRate = 1;
	private Boolean enableTimetag = true;
	private Boolean enableDebug = true;
	private String outputDir = "./test";

	public NVSSerialConnection(String portName, int speed) {
		this.portName = portName;
		this.speed = speed;
	}

	@SuppressWarnings("unchecked")
	public static Vector<String> getPortList(boolean showList) {
		Enumeration<CommPortIdentifier> portList;
		Vector<String> portVect = new Vector<String>();
		portList = CommPortIdentifier.getPortIdentifiers();

		CommPortIdentifier portId;
		while (portList.hasMoreElements()) {
			portId = portList.nextElement();
			if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
				portVect.add(portId.getName());
			}
		}
		if (showList) {
			System.out.println("Found the following ports:");
			for (int i = 0; i < portVect.size(); i++) {
				System.out.println(portVect.elementAt(i));
			}
		}

		return portVect;
	}

	public boolean isConnected() {
		return connected;
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamResource#init()
	 */
	@Override
	public void init() throws Exception {

		CommPortIdentifier portIdentifier;

//		boolean conn = false;
//		try {
			portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
			if (portIdentifier.isCurrentlyOwned()) {
				System.out.println("Error: Port is currently in use");
			} else {
				serialPort = (SerialPort) portIdentifier.open("Serial", 2000);
				
				boolean reply;
				
				//try with NMEA
				serialPort.setSerialPortParams(speed, SerialPort.DATABITS_8,
						SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

				inputStream = serialPort.getInputStream();
				outputStream = serialPort.getOutputStream();

				nvsReader = new NVSSerialReader(inputStream,outputStream,portName,outputDir);
				nvsReader.enableDebugMode(this.enableDebug);
				reply = nvsReader.setBinrProtocol();

				Thread.sleep(100);
				serialPort.setSerialPortParams(speed, SerialPort.DATABITS_8,
						SerialPort.STOPBITS_1, SerialPort.PARITY_ODD);
				
				if (!reply) {
					//try with BINR
					inputStream = serialPort.getInputStream();
					outputStream = serialPort.getOutputStream();

					nvsReader = new NVSSerialReader(inputStream,outputStream,portName,outputDir);
					nvsReader.enableDebugMode(this.enableDebug);
					reply = nvsReader.setBinrProtocol();
				}
				
				connected = true;
				System.out.println("Connection on " + portName + " established");
				
				//nvsReader.setStreamEventListener(streamEventListener);
				nvsReader.setRate(this.setMeasurementRate);
				nvsReader.enableSysTimeLog(this.enableTimetag);
				nvsReader.enableDebugMode(this.enableDebug);
				nvsReader.start();
			}
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamResource#release(boolean, long)
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs)
			throws InterruptedException {

		if(nvsReader!=null){
			nvsReader.stop(waitForThread, timeoutMs);
		}

		try {
			inputStream.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		try {
			outputStream.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		serialPort.close();


		connected = false;
		System.out.println("Connection disconnected");

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#addStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void addStreamEventListener(StreamEventListener streamEventListener) {
		nvsReader.addStreamEventListener(streamEventListener);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#getStreamEventListeners()
	 */
	@Override
	public Vector<StreamEventListener> getStreamEventListeners() {
		return nvsReader.getStreamEventListeners();
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void removeStreamEventListener(
			StreamEventListener streamEventListener) {
		nvsReader.removeStreamEventListener(streamEventListener);
	}
	
	public void setMeasurementRate(int measRate) {
		if(nvsReader!=null){
			nvsReader.setRate(measRate);
		} else {
			this.setMeasurementRate = measRate;
		}
	}

	public void enableTimetag(Boolean enableTim) {
		if(nvsReader!=null){
			nvsReader.enableSysTimeLog(enableTim);
		} else {
			this.enableTimetag = enableTim;
		}
	}
	
	public void enableDebug(Boolean enableDebug) {
		if(nvsReader!=null){
			nvsReader.enableDebugMode(enableDebug);
		} else {
			this.enableDebug = enableDebug;
		}
	}
	
	public void setOutputDir(String outDir) {
		if(nvsReader!=null){
			nvsReader.setOutputDir(outDir);
		} else {
			this.outputDir = outDir;
		}
	}
}
