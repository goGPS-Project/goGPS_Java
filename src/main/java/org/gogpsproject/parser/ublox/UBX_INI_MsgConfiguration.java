package org.gogpsproject.parser.ublox;

import java.util.Calendar;
import java.util.TimeZone;
import java.util.Vector;

import org.gogpsproject.Time;

/*
This message contains position, time and clock drift information. The position can be input
in either the ECEF X/Y/Z coordinate system or as lat/lon/height. The time can either be input
as inexact value via the standard communication interface, suffering from latency
depending on the baudrate, or using harware time synchronization where an accuracte
time pulse is input on the external interrupts. It is also possible to supply hardware
frequency aiding by connecting a continuous signal to an external interrupt.
*/
public class UBX_INI_MsgConfiguration extends UBXMsgConfiguration {

  private static int pl_length = 48;
  private static int[] msg = new int[pl_length];
  
  public static Time refTime;
  
  static int[] getMsg(){
    Calendar c = Calendar.getInstance();
    c.setTimeZone(TimeZone.getTimeZone("UTC"));
    refTime = new Time(c.getTimeInMillis());
    
    int i = 0;
    
    long ecefXOrLat = 442428378;
    msg[i++] = ((int)(ecefXOrLat & 0xFF)); 
    msg[i++] = ((int)((ecefXOrLat >>  8)& 0xFF)); 
    msg[i++] = ((int)((ecefXOrLat >> 16)& 0xFF)); 
    msg[i++] = ((int)((ecefXOrLat >> 24)& 0xFF)); 

    long ecefYOrLat = 71460808;
    msg[i++] = ((int)(ecefYOrLat & 0xFF)); 
    msg[i++] = ((int)((ecefYOrLat >>  8)& 0xFF)); 
    msg[i++] = ((int)((ecefYOrLat >> 16)& 0xFF)); 
    msg[i++] = ((int)((ecefYOrLat >> 24)& 0xFF)); 

    long ecefZOrLat = 452325426;
    msg[i++] = ((int)(ecefZOrLat & 0xFF)); 
    msg[i++] = ((int)((ecefZOrLat >>  8)& 0xFF)); 
    msg[i++] = ((int)((ecefZOrLat >> 16)& 0xFF)); 
    msg[i++] = ((int)((ecefZOrLat >> 24)& 0xFF)); 

    long posAcc = 1220;
    msg[i++] = ((int)(posAcc & 0xFF)); 
    msg[i++] = ((int)((posAcc >>  8)& 0xFF)); 
    msg[i++] = ((int)((posAcc >> 16)& 0xFF)); 
    msg[i++] = ((int)((posAcc >> 24)& 0xFF)); 
    
    int tmCfg = 1;
    msg[i++] = ((int)(tmCfg & 0xFF)); 
    msg[i++] = ((int)((tmCfg >>  8)& 0xFF)); 

    // I2 week 
//    int week = 1853;
    int week = refTime.getGpsWeek();
    msg[i++] = ((int)(week & 0xFF)); 
    msg[i++] = ((int)((week >>  8)& 0xFF)); 
    
    // I4 iTOW 
//    long iTOW = 53557000l; //ms
    long iTOW = (long) refTime.getGpsTime();
    
    msg[i++] = ((int)(iTOW & 0xFF)); 
    msg[i++] = ((int)((iTOW >>  8)& 0xFF)); 
    msg[i++] = ((int)((iTOW >> 16)& 0xFF)); 
    msg[i++] = ((int)((iTOW >> 24)& 0xFF)); 

    long towNs = 0l; 
    msg[i++] = ((int)(towNs & 0xFF)); 
    msg[i++] = ((int)((towNs >>  8)& 0xFF)); 
    msg[i++] = ((int)((towNs >> 16)& 0xFF)); 
    msg[i++] = ((int)((towNs >> 24)& 0xFF)); 

    long tAccMs = 0l; 
    msg[i++] = ((int)(tAccMs & 0xFF)); 
    msg[i++] = ((int)((tAccMs >>  8)& 0xFF)); 
    msg[i++] = ((int)((tAccMs >> 16)& 0xFF)); 
    msg[i++] = ((int)((tAccMs >> 24)& 0xFF)); 

    long tAccNs = 0l; 
    msg[i++] = ((int)(tAccNs & 0xFF)); 
    msg[i++] = ((int)((tAccNs >>  8)& 0xFF)); 
    msg[i++] = ((int)((tAccNs >> 16)& 0xFF)); 
    msg[i++] = ((int)((tAccNs >> 24)& 0xFF)); 

    long clkDOrFreq = 381; 
    msg[i++] = ((int)(clkDOrFreq & 0xFF)); 
    msg[i++] = ((int)((clkDOrFreq >>  8)& 0xFF)); 
    msg[i++] = ((int)((clkDOrFreq >> 16)& 0xFF)); 
    msg[i++] = ((int)((clkDOrFreq >> 24)& 0xFF)); 

    long clkDAccOrFreqAcc = 1; 
    msg[i++] = ((int)(clkDAccOrFreqAcc & 0xFF)); 
    msg[i++] = ((int)((clkDAccOrFreqAcc >>  8)& 0xFF)); 
    msg[i++] = ((int)((clkDAccOrFreqAcc >> 16)& 0xFF)); 
    msg[i++] = ((int)((clkDAccOrFreqAcc >> 24)& 0xFF)); 

    long flags = 11; 
    msg[i++] = ((int)(flags & 0xFF)); 
    msg[i++] = ((int)((flags >>  8)& 0xFF)); 
    msg[i++] = ((int)((flags >> 16)& 0xFF)); 
    msg[i++] = ((int)((flags >> 24)& 0xFF)); 

    return msg;
  }
  
  public UBX_INI_MsgConfiguration() {
    super(UBXMessageType.CLASS_AID, UBXMessageType.AID_INI, getMsg());
  }

}
