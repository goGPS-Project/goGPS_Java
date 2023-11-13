package org.gogpsproject.producer.parser;

import com.fazecast.jSerialComm.*;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Vector;

import org.gogpsproject.producer.StreamEventListener;
import org.gogpsproject.producer.StreamEventProducer;
import org.gogpsproject.producer.StreamResource;

public abstract class AbstractSerialConnection<T extends StreamEventProducer>
		implements StreamResource, StreamEventProducer {
	protected InputStream inputStream;
	protected OutputStream outputStream;
	protected boolean connected = false;

	protected SerialPort serialPort;

	protected String portName;
	protected int speed;

	protected int setMeasurementRate = 1;
	protected boolean enableTimetag = true;
	protected Boolean enableDebug = true;
	protected String outputDir = "./test";

	protected T prod;

	@Override
	public void init() throws Exception {

		serialPort = SerialPort.getCommPort(portName);
		if (serialPort.isOpen()) {
			System.out.println("Error: Port is currently in use");
		} else {
			boolean openedSuccessfully = serialPort.openPort(2000);
			if (!openedSuccessfully)
			{
				System.out.println("Error code was " + serialPort.getLastErrorCode() + " at Line " + serialPort.getLastErrorLocation());
				throw new RuntimeException("Can't open port");
			}
			
			serialPort.setComPortParameters(speed, 
					8, 
					SerialPort.ONE_STOP_BIT, 
					SerialPort.NO_PARITY);
			
			inputStream = serialPort.getInputStream();
			outputStream = serialPort.getOutputStream();
			
//			serialPort.setComPortTimeouts(SerialPort.TIMEOUT_NONBLOCKING, 1000, 0);
//			serialPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_BLOCKING, 0, 0);
			serialPort.setComPortTimeouts(SerialPort.TIMEOUT_READ_SEMI_BLOCKING, 0, 0);
		}
	}

	public void release() throws InterruptedException {
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
		serialPort.closePort();

		connected = false;
		System.out.println("Connection disconnected");
	}

	public static Vector<String> getPortList(boolean showList) {
		SerialPort[] ports = SerialPort.getCommPorts();
		Vector<String> portVect = new Vector<String>();

		for (int i = 0; i < ports.length; ++i) {
			portVect.add(ports[i].getSystemPortName());
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

	/*
	 * (non-Javadoc)
	 * 
	 * @see
	 * org.gogpsproject.StreamEventProducer#addStreamEventListener(org.gogpsproject.
	 * StreamEventListener)
	 */
	@Override
	public void addStreamEventListener(StreamEventListener streamEventListener) {
		prod.addStreamEventListener(streamEventListener);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.gogpsproject.StreamEventProducer#getStreamEventListeners()
	 */
	@Override
	public Vector<StreamEventListener> getStreamEventListeners() {
		return prod.getStreamEventListeners();
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.
	 * gogpsproject.StreamEventListener)
	 */
	@Override
	public void removeStreamEventListener(StreamEventListener streamEventListener) {
		prod.removeStreamEventListener(streamEventListener);
	}

}
