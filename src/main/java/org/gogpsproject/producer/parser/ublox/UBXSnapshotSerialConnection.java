package org.gogpsproject.producer.parser.ublox;

import java.util.List;

import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;

public class UBXSnapshotSerialConnection extends UBXSerialConnection {

  public UBXSnapshotSerialConnection(String portName, int speed) {
    super( portName, speed );
  }

  public void serialinit() throws Exception {
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

  /* (non-Javadoc)
   * @see org.gogpsproject.StreamResource#init()
   */
  @Override
  public void init() throws Exception {
        serialinit();
        
        prod = new UBXSnapshotSerialReader(inputStream,outputStream,portName,outputDir);
        prod.setRate(super.setMeasurementRate);
        prod.enableAidEphMsg(super.setEphemerisRate);
        prod.enableAidHuiMsg(super.setIonosphereRate);
        prod.enableSysTimeLog(super.enableTimetag);
        prod.enableDebugMode(super.enableDebug);
        prod.enableNmeaMsg(super.enableNmeaList);
        prod.start();

        connected = true;
        System.out.println("Connection on " + portName + " established");
        //conn = true;
  }

}

