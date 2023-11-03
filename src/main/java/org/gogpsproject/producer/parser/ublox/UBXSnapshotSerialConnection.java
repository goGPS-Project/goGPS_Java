package org.gogpsproject.producer.parser.ublox;

import com.fazecast.jSerialComm.*;

public class UBXSnapshotSerialConnection extends UBXSerialConnection {

	public UBXSnapshotSerialConnection(String portName, int speed) {
		super(portName, speed);
	}

	public void serialinit() throws Exception {
		serialPort = SerialPort.getCommPort(portName);
		if (serialPort.isOpen()) {
			System.out.println("Error: Port is currently in use");
		} else {
			boolean openedSuccessfully = serialPort.openPort(2000);
			if (!openedSuccessfully) {
				System.out.println("Error code was " + serialPort.getLastErrorCode() + " at Line "
						+ serialPort.getLastErrorLocation());
				throw new RuntimeException("Can't open port");
			}

			serialPort.setComPortParameters(speed, 8, SerialPort.ONE_STOP_BIT, SerialPort.NO_PARITY);

			inputStream = serialPort.getInputStream();
			outputStream = serialPort.getOutputStream();
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.gogpsproject.StreamResource#init()
	 */
	@Override
	public void init() throws Exception {
		serialinit();

		prod = new UBXSnapshotSerialReader(inputStream, outputStream, portName, outputDir);
		prod.setRate(super.setMeasurementRate);
		prod.enableAidEphMsg(super.setEphemerisRate);
		prod.enableAidHuiMsg(super.setIonosphereRate);
		prod.enableSysTimeLog(super.enableTimetag);
		prod.enableDebugMode(super.enableDebug);
		prod.enableNmeaMsg(super.enableNmeaList);
		prod.start();

		connected = true;
		System.out.println("Connection on " + portName + " established");
		// conn = true;
	}

}
