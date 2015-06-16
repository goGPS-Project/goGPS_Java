package org.gogpsproject.parser;

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

public abstract class AbstractSerialConnection<T extends StreamEventProducer> implements StreamResource, StreamEventProducer {
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
    CommPortIdentifier portIdentifier;

    portIdentifier = CommPortIdentifier.getPortIdentifier(portName);
    if (portIdentifier.isCurrentlyOwned()) {
      System.out.println("Error: Port is currently in use");
    } else {
      serialPort = (SerialPort) portIdentifier.open("Serial", 2000);
      serialPort.setSerialPortParams(speed, SerialPort.DATABITS_8,
          SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);

      inputStream = serialPort.getInputStream();
      outputStream = serialPort.getOutputStream();
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
    serialPort.close();

    connected = false;
    System.out.println("Connection disconnected");
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
   * @see org.gogpsproject.StreamEventProducer#addStreamEventListener(org.gogpsproject.StreamEventListener)
   */
  @Override
  public void addStreamEventListener(StreamEventListener streamEventListener) {
    prod.addStreamEventListener(streamEventListener);
  }

  /* (non-Javadoc)
   * @see org.gogpsproject.StreamEventProducer#getStreamEventListeners()
   */
  @Override
  public Vector<StreamEventListener> getStreamEventListeners() {
    return prod.getStreamEventListeners();
  }

  /* (non-Javadoc)
   * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.gogpsproject.StreamEventListener)
   */
  @Override
  public void removeStreamEventListener(
      StreamEventListener streamEventListener) {
    prod.removeStreamEventListener(streamEventListener);
  }

}
