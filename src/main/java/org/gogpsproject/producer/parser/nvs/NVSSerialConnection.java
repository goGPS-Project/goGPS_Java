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

package org.gogpsproject.producer.parser.nvs;

import org.gogpsproject.producer.parser.AbstractSerialConnection;

import com.fazecast.jSerialComm.*;

public class NVSSerialConnection extends AbstractSerialConnection<NVSSerialReader> {

	// private StreamEventListener streamEventListener;

	public NVSSerialConnection(String portName, int speed) {
		this.portName = portName;
		this.speed = speed;
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.gogpsproject.StreamResource#init()
	 */
	@Override
	public void init() throws Exception {

//		boolean conn = false;
//		try {
		super.init();

		boolean reply;

		// try with NMEA

		prod = new NVSSerialReader(inputStream, outputStream, portName, outputDir);
		prod.enableDebugMode(this.enableDebug);
		reply = prod.setBinrProtocol();

		Thread.sleep(100);
		serialPort.setComPortParameters(speed, 8, SerialPort.ONE_STOP_BIT, SerialPort.ODD_PARITY);

		if (!reply) {
			// try with BINR
			inputStream = serialPort.getInputStream();
			outputStream = serialPort.getOutputStream();

			prod = new NVSSerialReader(inputStream, outputStream, portName, outputDir);
			prod.enableDebugMode(this.enableDebug);
			reply = prod.setBinrProtocol();
		}

		connected = true;
		System.out.println("Connection on " + portName + " established");

		// nvsReader.setStreamEventListener(streamEventListener);
		prod.setRate(this.setMeasurementRate);
		prod.enableSysTimeLog(this.enableTimetag);
		prod.enableDebugMode(this.enableDebug);
		prod.start();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.gogpsproject.StreamResource#release(boolean, long)
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs) throws InterruptedException {

		if (prod != null) {
			prod.stop(waitForThread, timeoutMs);
		}

		super.release();
	}

	public NVSSerialConnection setMeasurementRate(int measRate) {
		if (prod != null) {
			prod.setRate(measRate);
		} else {
			this.setMeasurementRate = measRate;
		}
		return this;
	}

	public NVSSerialConnection enableTimetag(Boolean enableTim) {
		if (prod != null) {
			prod.enableSysTimeLog(enableTim);
		} else {
			this.enableTimetag = enableTim;
		}
		return this;
	}

	public NVSSerialConnection enableDebug(Boolean enableDebug) {
		if (prod != null) {
			prod.enableDebugMode(enableDebug);
		} else {
			this.enableDebug = enableDebug;
		}
		return this;
	}

	public NVSSerialConnection setOutputDir(String outDir) {
		if (prod != null) {
			prod.setOutputDir(outDir);
		} else {
			this.outputDir = outDir;
		}
		return this;
	}
}
