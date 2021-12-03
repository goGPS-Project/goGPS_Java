/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl, Daisuke Yoshida, Emanuele Ziglioli. All Rights Reserved.
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

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
//import java.text.SimpleDateFormat;
import java.util.Calendar;
//import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.Constants;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;
import static org.gogpsproject.util.UnsignedOperation.*;

public class DecodeTRKMEAS {
	private InputStream in;

//	private int[] fdata;
//	private int[] fbits;
//	private boolean end = true;
	
	private Boolean[] multiConstellation;

	public DecodeTRKMEAS(InputStream in) {
		this.in = in;
	}

	public DecodeTRKMEAS(InputStream in, Boolean[] multiConstellation) throws IOException {
		this.in = in;		
		this.multiConstellation = multiConstellation;
	}
	
	/**
	 * See https://github.com/tomojitakasu/RTKLIB/blob/master/src/rcv/ublox.c
	 * and https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rcv/ublox.c
	 * static int decode_trkmeas(raw_t *raw)
	 * 
	 * @param logos
	 * @param time 
	 * @return
	 * @throws IOException
	 * @throws UBXException
	 */
	public Observations decode(OutputStream logos, Time time) throws IOException, UBXException {
		double maxts = 0;		
		boolean anomalousValues = false;
		int gpsCounter = 0;
    Observations o = new Observations(new Time(0, 0),0);

		// parse little Endian data
		int[] length = new int[2];

		length[1] = in.read();
		length[0] = in.read();

		int CH_A = 0;
		int CH_B = 0;
		CH_A += 0x02;CH_B += CH_A;

		CH_A += 0x10;CH_B += CH_A;
		CH_A += length[1];CH_B += CH_A;
		CH_A += length[0];CH_B += CH_A;

		int len = length[0]*256+length[1];
		
		boolean gpsEnable = multiConstellation[0];
		boolean qzsEnable = multiConstellation[1];
		boolean gloEnable = multiConstellation[2];
		boolean galEnable = multiConstellation[3];
		boolean bdsEnable = multiConstellation[4];
		
		if (len == 0) {
			throw new UBXException("Zero-length TRK-MEAS message");
		}

		System.out.println("TRK-MEAS message Length : " + len);

/*0*/in.skip(2); //  *p=raw->buff+6

/*2*/int numSV = in.read(); // nch=U1(p+2);
		System.out.println("numSV :  " + numSV );
		in.read();
		
		if( len< 112 + numSV*56 - 8) {
			throw new UBXException("Length error TRK-MEAS message");
    }

/*4*/for (int i = 0; i < 5; i++) { // reserved
			System.out.print(  Integer.toHexString(in.read()) + " " );
		}
		for (int i = 5; i < 100; i++) { // reserved
			System.out.print(  Integer.toHexString(in.read()) + " " );
			if( (i + 5)  %16 == 0 )
				System.out.println();
		}
		System.out.println();

		for (int k = 0; k < numSV; k++) { // p=raw->buff+110

/*0*/	int chNum = in.read(); 
			System.out.println("\nchNum:  " + chNum );
			
/*1*/	int mesQI = in.read();  
			System.out.println("mesQI:  " + mesQI );
			if( mesQI<4 || mesQI >7) {
				System.out.println("Invalid");
				in.skip(53);
				continue;
			}
			in.skip(2);

/*4*/	int gnssId = in.read();
			System.out.println("gnssId:  " + gnssId );
			
/*5*/	int svId = in.read();
			System.out.println("svId:  " + svId );
			in.read();
			
/*7*/	int fcn = in.read()-7;
      // GLO frequency channel number+7
			System.out.println("fcn:  " + fcn );
			
/*8*/	int statusval = in.read();
			// tracking/lock status (bit3: half-cycle)
			System.out.print("status:  " + Integer.toHexString(statusval) + " " + Integer.toBinaryString(statusval) );
			boolean statusFlag = ((statusval & 0b01000000) != 0);
			if( statusFlag )
				System.out.println(" HALF");
			else
				System.out.println(" FULL");
				
			in.skip(7);

/*16*/int lock1 = in.read();
      // code lock count
			System.out.println("lock1:  " + lock1 );

/*17*/int lock2 = in.read();
      // carrier lock count
			System.out.println("lock2:  " + lock2 );
			in.skip(2);

/*20*/float cNo = U2(in);
			if( cNo <=0 ) {
				System.out.println("Invalid" );
				in.skip(34);
				continue;
			}
				
			cNo /= Math.pow(2, 8);
			System.out.println("cNo:  " + cNo );
			in.skip(2);

/*24*/double ts = I8(in);
      // transmission time
			ts = ts / Math.pow(2, 32)/1000;
			System.out.println("ts:  " + ts );

			if( gnssId==3 ) // SYS_CMP
				ts += 14.0;             /* bdt  -> gpst */
			else if ( gnssId == 6 ) // SYS_GLO 
				ts -= 10800.0 + time.getGpsWeekSec(); /* glot -> gpst */			
	
			// for now store ts as pseudorange
			double pseudoRange = ts;
			

			if( gnssId==0 ) { // SYS_GPS
				if( ts>maxts ) {
					maxts = ts;
			}
			
/*32*/double adr = I8(in);
		  // accumulated Doppler range
			adr /= Math.pow(2, 32);
			adr += statusFlag? 0.5: 0;
			System.out.println("adr:  " + adr );
		  double carrierPhase = -adr;

/*40*/float dopplerHz = I4(in);
//			dopplerHz *= 10*Math.pow(2, 10);
			dopplerHz /= Math.pow(2, 8);
			System.out.println("dopplerHz:  " + dopplerHz );

/*48*/in.skip(12);

			ObservationSet os = new ObservationSet();
					
			if (gnssId == 0 && gpsEnable == true && !anomalousValues){ 
				/* GPS */
				os.setSatType('G');
				os.setSatID(svId);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, dopplerHz);
				os.setSignalStrength(ObservationSet.L1, cNo);
				o.setGps(gpsCounter, os);
				gpsCounter++;
				
//			} else if (satType == 1) { // SBAS
//				os.setSatType('S'); 
//				os.setSatID(satId);
			
//			} else if (satType == 4) { // IMES
//				os.setSatType('I');
//				os.setSatID(satId);
			
			} else if (gnssId == 2 && galEnable == true && !anomalousValues) { 
				/* Galileo */
				os.setSatType('E');
				os.setSatID(svId);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, dopplerHz);
				os.setSignalStrength(ObservationSet.L1, cNo);
				o.setGps(gpsCounter, os);
				gpsCounter++;
				
			} else if (gnssId == 3 && bdsEnable == true && !anomalousValues) { 
				/* BeiDou */
				os.setSatType('C');
				os.setSatID(svId);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, dopplerHz);
				os.setSignalStrength(ObservationSet.L1, cNo);
				o.setGps(gpsCounter, os);
				gpsCounter++;
				
			} else if (gnssId == 5 && qzsEnable == true && !anomalousValues) { 
				/* QZSS*/
				os.setSatType('J');
				os.setSatID(svId);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, dopplerHz);
				os.setSignalStrength(ObservationSet.L1, cNo);
				o.setGps(gpsCounter, os);
				gpsCounter++;
						
			} else if (gnssId == 6 && gloEnable == true && !anomalousValues) { 
				/* GLONASS */
				os.setSatType('R');
				os.setSatID(svId);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, dopplerHz);
				os.setSignalStrength(ObservationSet.L1,cNo);
				o.setGps(gpsCounter, os);
				gpsCounter++;
				
			}
			anomalousValues = false;
			
		 }
	  }
		// / Checksum
		CH_A = CH_A & 0xFF;
		CH_B = CH_B & 0xFF;

		int c1 = in.read();
		if(logos!=null) logos.write(c1);

		int c2 = in.read();
		if(logos!=null) logos.write(c2);

//	if(CH_A != c1 || CH_B!=c2)
//	throw new UBXException("Wrong message checksum");

		if (o != null && o.getNumSat() == 0) {
			return null;
		}
		
		/* time-tag = max(transmission time + 0.08) rounded by 100 ms */
		double tr = Math.round((maxts+0.08)/0.1)*0.1;
		System.out.println("tr " + tr );
		
//		/* adjust week handover */
//		time2gpst( raw->time, week );
		int week = time.getGpsWeek();
		int t = time.getGpsWeekSec();
		
		if (t<tr-302400.0) 
			week--;
		else if (t>tr+302400.0) 
			week++;
		
		Time time2 = new Time( week, tr );
		long gpsTow = time2.getGpsWeekSec(); 
		//utc_gpst = timediff(gpst2utc(time), time);
		
		System.out.println("gpsTow:  " + gpsTow );
		
		// compute all PRs
		Observations o2 = new Observations( time2,0);
		for( int i=0; i<o.getNumSat(); i++ ) {
			ObservationSet os = o.getSatByIdx(i);
			
			double ts = os.getPseudorange(0);
			
			/* signal travel time */
      double tau = tr - ts;
      if( tau<-302400.0 ) 
      	tau += 604800.0;
      else if( tau>302400.0 ) 
      	tau-=604800.0;				
	
			os.setCodeC(ObservationSet.L1, tau * Constants.SPEED_OF_LIGHT );
			o2.setGps(i, os);
		}
		
		o2.sortBySatID();
		return o2;
	}

	private long getGMTTS(long tow, long week) {
		Calendar c = Calendar.getInstance();
		c.setTimeZone(TimeZone.getTimeZone("GMT Time"));
		c.set(Calendar.YEAR, 1980);
		c.set(Calendar.MONTH, Calendar.JANUARY);
		c.set(Calendar.DAY_OF_MONTH, 6);
		c.set(Calendar.HOUR_OF_DAY, 0);
		c.set(Calendar.MINUTE, 0);
		c.set(Calendar.SECOND, 0);
		c.set(Calendar.MILLISECOND, 0);

//		c.add(Calendar.DATE, week*7);
//		c.add(Calendar.MILLISECOND, tow/1000*1000);

		//SimpleDateFormat sdf = new SimpleDateFormat("yyyy MM dd HH mm ss.SSS");
		//System.out.println(sdf.format(c.getTime()));
		//ubx.log( (c.getTime().getTime())+" "+c.getTime()+" "+week+" "+tow+"\n\r");

		return c.getTimeInMillis() + week*7*24*3600*1000 + tow ;
	}
}
