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

import java.util.ArrayList;
import java.util.List;

import org.gogpsproject.producer.parser.AbstractSerialConnection;

public class UBXSerialConnection extends AbstractSerialConnection<UBXSerialReader> {

	protected int setEphemerisRate = 10;
	protected int setIonosphereRate = 60;
	protected Boolean enableDebug = false;
	protected Boolean enableTimetag = false;
	protected String outputDir = "./out";
	protected List<String> enableNmeaList = new ArrayList<String>();

	public UBXSerialConnection(String portName, int speed) {
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
		prod = new UBXSerialReader(inputStream, outputStream, portName, outputDir);
		prod.setRate(this.setMeasurementRate);
		prod.enableAidEphMsg(this.setEphemerisRate);
		prod.enableAidHuiMsg(this.setIonosphereRate);
		prod.enableSysTimeLog(this.enableTimetag);
		prod.enableDebugMode(this.enableDebug);
		prod.enableNmeaMsg(this.enableNmeaList);
		prod.start();

		connected = true;
		System.out.println("Connection on " + portName + " established");
		// conn = true;

//		} catch (NoSuchPortException e) {
//			System.out.println("The connection could not be made");
//			e.printStackTrace();
//		} catch (PortInUseException e) {
//			System.out.println("The connection could not be made");
//			e.printStackTrace();
//		} catch (UnsupportedCommOperationException e) {
//			System.out.println("The connection could not be made");
//			e.printStackTrace();
//		} catch (IOException e) {
//			System.out.println("The connection could not be made");
//			e.printStackTrace();
//		}
//		return conn;
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

	public void setMeasurementRate(int measRate) {
		if (prod != null) {
			prod.setRate(measRate);
		} else {
			this.setMeasurementRate = measRate;
		}
	}

	public void enableEphemeris(Integer ephRate) {
		if (prod != null) {
			prod.enableAidEphMsg(ephRate);
		} else {
			this.setEphemerisRate = ephRate;
		}
	}

	public void enableIonoParam(Integer ionRate) {
		if (prod != null) {
			prod.enableAidHuiMsg(ionRate);
		} else {
			this.setIonosphereRate = ionRate;
		}
	}

	public void enableNmeaSentences(List<String> nmeaList) {
		if (prod != null) {
			prod.enableNmeaMsg(nmeaList);
		} else {
			this.enableNmeaList = nmeaList;
		}
	}

	public void enableTimetag(Boolean enableTim) {
		if (prod != null) {
			prod.enableSysTimeLog(enableTim);
		} else {
			this.enableTimetag = enableTim;
		}
	}

	public void enableDebug(Boolean enableDebug) {
		if (prod != null) {
			prod.enableDebugMode(enableDebug);
		} else {
			this.enableDebug = enableDebug;
		}
	}

	public void setOutputDir(String outDir) {
		if (prod != null) {
			prod.setOutputDir(outDir);
		} else {
			this.outputDir = outDir;
		}
	}
}
