package org.gogpsproject.producer.parser.ublox;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.PrintStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.logging.Level;
import java.util.logging.Logger;

import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.StreamEventListener;

public class UBXSnapshotSerialReader extends UBXSerialReader {

  private static final Logger logger = Logger.getLogger(UBXSnapshotSerialReader.class.getName());

  public UBXSnapshotSerialReader(InputStream in,OutputStream out, String COMPort, String outputDir) {
    super(in,out, COMPort, outputDir);
  }
  
  public UBXSnapshotSerialReader(InputStream in,OutputStream out,String COMPort,String outputDir,StreamEventListener streamEventListener) {
    super(in,out,COMPort,outputDir,streamEventListener);
  }
  
  public void start()  throws IOException{
    
    super.start();

  }

  public void run() {

    int data = 0;
    long aidEphTS = System.currentTimeMillis();
    long aidHuiTS = System.currentTimeMillis();
    //long sysOutTS = System.currentTimeMillis();
    UBXMsgConfiguration msgcfg = null;
    FileOutputStream fos_tim = null;
    FileOutputStream fos_nmea = null;
    PrintStream psSystime = null;
    PrintStream psNmea = null;

    Date date = new Date();
    SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
    String date1 = sdf1.format(date);
    String COMPortStr = prepareCOMStringForFilename(COMPort);
    
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
    
    if (!requestedNmeaMsgs.isEmpty()) {
      try {
        System.out.println(date1+" - "+COMPort+" - Logging NMEA sentences in "+outputDir+"/"+COMPortStr+ "_" + dateFile + "_NMEA.txt");
        fos_nmea = new FileOutputStream(outputDir+"/"+COMPortStr+ "_" + dateFile + "_NMEA.txt");
        psNmea = new PrintStream(fos_nmea);
      } catch (FileNotFoundException e) {
        e.printStackTrace();
      }
    }

    try {
      int msg[] = {};
      if (msgAidHuiRate > 0) {
        System.out.println(date1+" - "+COMPort+" - AID-HUI message polling enabled (rate: "+msgAidHuiRate+"s)");
        msgcfg = new UBXMsgConfiguration(UBXMessageType.CLASS_AID, UBXMessageType.AID_HUI, msg);
        out.write(msgcfg.getByte());
        out.flush();

      }
      if (msgAidEphRate > 0) {
        System.out.println(date1+" - "+COMPort+" - AID-EPH message polling enabled (rate: "+msgAidEphRate+"s)");
        msgcfg = new UBXMsgConfiguration(UBXMessageType.CLASS_AID, UBXMessageType.AID_EPH, msg);
        out.write(msgcfg.getByte());
        out.flush();
      }

      in.start();
      sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
      String dateSys = null;
      String dateGps = null;
      boolean rxmRawMsgReceived = false;
      boolean truncatedNmea = false;
      reader.enableDebugMode(this.debugModeEnabled);
      while (!stop) {
        if(in.available()>0){
          dateSys = sdf1.format(new Date());
          if (!truncatedNmea) {
            data = in.read();
          }else{
            truncatedNmea = false;
          }
          try{
            if(data == 0xB5){
              Object o = reader.readMessage();
                if(o instanceof Observations){
                  if(streamEventListeners!=null && o!=null){
                    for(StreamEventListener sel:streamEventListeners){
                      try {
                      Observations co = sel.getCurrentObservations();
                      if( co == null )
                        continue;
                      
//                        sel.pointToNextObservations();

                        rxmRawMsgReceived = true;

                        if (this.sysTimeLogEnabled) {
                          dateGps = sdf1.format(new Date(co.getRefTime().getMsec()));
                          psSystime.println(dateGps +"       "+dateSys);
                        }
                      } catch (Exception e) {
                        logger.log( Level.SEVERE, e.getMessage(), e);
                      }
                    }
                  }
                }
            }else if(data == 0x24){
              if (!requestedNmeaMsgs.isEmpty()) {
                String sentence = "" + (char) data;
                data = in.read();
                if(data == 0x47) {
                  sentence = sentence + (char) data;
                  data = in.read();
                  if(data == 0x50) {
                    sentence = sentence + (char) data;
                    data = in.read();
                    while (data != 0x0A && data != 0xB5) {
                      sentence = sentence + (char) data;
                      data = in.read();
                    }
                    sentence = sentence + (char) data;
                    psNmea.print(sentence);
//                    if (this.debugModeEnabled) {
//                      System.out.print(sentence);
//                    }
                    if (data == 0xB5) {
                      truncatedNmea = true;
                      if (this.debugModeEnabled) {
                        System.out.println("Warning: truncated NMEA message");
                      }
                    }
                  }
                }
              }
            } else {
              if (this.debugModeEnabled) {
//                System.out.println("Warning: wrong sync char 1 "+data+" "+Integer.toHexString(data)+" ["+((char)data)+"]");
              }
            }
          }catch(UBXException ubxe){
            ubxe.printStackTrace();
          }
        }else{
          // no bytes to read, wait 1 msec
          try {
            Thread.sleep(1);
          } catch (InterruptedException e) {}
        }
        long curTS = System.currentTimeMillis();
        
        if(msgAidEphRate > 0 && curTS-aidEphTS >= msgAidEphRate*1000){
          System.out.println(dateSys+" - "+COMPort+" - Sending AID-INI message");
          msgcfg = new UBX_INI_MsgConfiguration();
          out.write(msgcfg.getByte());
          out.flush();
          aidEphTS = curTS;
        }
        
//        if(msgAidEphRate > 0 && curTS-aidEphTS >= msgAidEphRate*1000){
//          System.out.println(dateSys+" - "+COMPort+" - Polling AID-EPH message");
//          msgcfg = new UBXMsgConfiguration(UBXMessageType.CLASS_AID, UBXMessageType.AID_EPH, msg);
//          out.write(msgcfg.getByte());
//          out.flush();
//          aidEphTS = curTS;
//        }
//        
//        if(msgAidHuiRate > 0 && curTS-aidHuiTS >= msgAidHuiRate*1000){
//          System.out.println(dateSys+" - "+COMPort+" - Polling AID-HUI message");
//          msgcfg = new UBXMsgConfiguration(UBXMessageType.CLASS_AID, UBXMessageType.AID_HUI, msg);
//          out.write(msgcfg.getByte());
//          out.flush();
//          aidHuiTS = curTS;
//          
//        }
        if (rxmRawMsgReceived/*curTS-sysOutTS >= 1*1000*/) {
          int bps = in.getCurrentBps();
          if (bps != 0) {
            System.out.println(dateSys+" - "+COMPort+" - logging at "+String.format("%4d", bps)+" Bps -- total: "+in.getCounter()+" bytes");
          } else {
            System.out.println(dateSys+" - "+COMPort+" - log starting...     -- total: "+in.getCounter()+" bytes");
          }
          //sysOutTS = curTS;
          rxmRawMsgReceived = false;
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    for(StreamEventListener sel:streamEventListeners){
      sel.streamClosed();
    }
    //if(streamEventListener!=null) streamEventListener.streamClosed();
  }
  
}
