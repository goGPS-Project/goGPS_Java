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


import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.Constants;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;


public class DecodeF5 {
	
	private InputStream in;
//	private BufferedInputStream in;
	private Boolean[] multiConstellation;
	
	int leng;	
	int[] data;
	
	public DecodeF5(InputStream in, Boolean[] multiConstellation) throws IOException {
		this.in = in;		
		this.multiConstellation = multiConstellation;
	}

	public Observations decode(OutputStream logos,int leng) throws IOException, NVSException {
		
		byte bytes[];
		
		boolean[] bits;
		int indice;
		boolean[] temp1;
		this.leng = leng;
		
		boolean gpsEnable = multiConstellation[0];
		boolean qzsEnable = multiConstellation[1];
		boolean gloEnable = multiConstellation[2];
		boolean galEnable = multiConstellation[3];
		boolean bdsEnable = multiConstellation[4];
		
		int signInt;
		String signStr; 
		int espInt;
		String espStr;
		long mantInt;
		String mantStr; 
		double mantInt2;
		
		//System.out.println("Num of Satellite: "+ nsv);	
		
		/*  TOW_UTC, 8 bytes  */
		bytes = new byte[8];
		in.read(bytes, 0, bytes.length);
		double tow = Bits.byteToIEEE754Double(bytes);
	
		/*  Week Number, 2 bytes  */
		bytes = new byte[2];
		in.read(bytes, 0, bytes.length);
		long weekN = Bits.byteToLong(bytes);
		
		/*  GPS Time Shift, 8 bytes  */
		bytes = new byte[8];
		in.read(bytes, 0, bytes.length);
		double gpsTimeShift = Bits.byteToIEEE754Double(bytes);
		
		/*  GLONASS Time Shift, 8 bytes  */
		bytes = new byte[8];
		in.read(bytes, 0, bytes.length);
		double glonassTimeShift = Bits.byteToIEEE754Double(bytes);
		
		/* Time Correction, 1 bytes */
		//bytes = new byte[1];
		//in.read(bytes, 0, bytes.length);	
		int timeCorrection = in.read();
		
//		tow = (tow + gpsTimeShift) / 1000 ;
		weekN = weekN + 1024 ;
		
		//long gmtTS = getGMTTS(tow, weekN);
		Time refTime = new Time((int)weekN, (tow+gpsTimeShift)/1000);
		Observations o = new Observations(refTime,0);
		
//		System.out.println("+----------------  Start of F5  ------------------+");
//		System.out.println("TOW_UTC: "+ utc);			        
//		System.out.println("Week No.: " + weekN);
//		System.out.println("GPS-UTC TimeShift: "+ gpsTimeShift);		
//		System.out.println("GLONASS-UTC TimeShift: "+ glonassTimeShift);	
//		System.out.println("Time_Correction: "+ timeCorrection);	
		
		
		int nsv = (leng - 224) / 240 ;
//		System.out.println("nsv: " + nsv );
		
		int gpsCounter = 0;
		boolean anomalousValues = false;
		for(int i=0; i< nsv ; i++){
						
				ObservationSet os = new ObservationSet();
			
				/* 1:GLONASS, 2: GPS/QZSS, 4: SBAS, 8:Galileo */
				bytes = new byte[1];
				in.read(bytes, 0, bytes.length);
				int satType = Bits.byteToInt(bytes);					
				
				/* Satellite Number, 1 byte */				
				/* satID: 33 is QZSS  */
				bytes = new byte[1];
				in.read(bytes, 0, bytes.length);
				int satID = Bits.byteToInt(bytes);
				if (satID <= 0) {
					anomalousValues = true;
				}
				
				/* A carrier Number for GLONASS, 1 bytes */
				bytes = new byte[1];
				in.read(bytes, 0, bytes.length);
				int carrierNum = Bits.byteToInt(bytes);
				
				/* SNR (dB-Hz), 1 byte */
				bytes = new byte[1];
				in.read(bytes, 0, bytes.length);
				int snr = Bits.byteToInt(bytes);
				
				/*  Carrier Phase (cycles), 8 bytes  */		
				bytes = new byte[8];
                in.read(bytes, 0, bytes.length);
                
                String binL1 = "";
                for (int j = 7; j >= 0; j--) {    // for little endian
                        String temp0 = Integer.toBinaryString(bytes[j] & 0xFF);  // & 0xFF is for converting to unsigned 
                        temp0 = String.format("%8s",temp0).replace(' ', '0');
                        binL1 =  binL1 + temp0  ;
                }
                
                signStr = binL1.substring(0,1);
		        signInt = Integer.parseInt(signStr, 2);
		        espStr = binL1.substring(1,12);
		        espInt = Integer.parseInt(espStr, 2);
		        mantStr = binL1.substring(12,64);
		        mantInt = Long.parseLong(mantStr, 2);
		        mantInt2 = mantInt / Math.pow(2, 52);
		        double carrierPhase = Math.pow(-1, signInt) * Math.pow(2, (espInt - 1023)) * (1 + mantInt2);   // FP64
		        
		        
		        /*  cannot use below code due to surpass the max value of Long  */
//				bytes = new byte[8];
//				//in.read(bytes, 0, 8);
//				in.read(bytes, 0, bytes.length);
//				double carrierPhase = Bits.byteToIEEE754Double(bytes);
						
		        

		        /* C/A Pseudo Range (ms), 8 bytes  */
//		        bytes = new byte[8];
//		                in.read(bytes, 0, bytes.length);
//		
//		                String binpR = "";
//		                for (int j = 7; j >= 0; j--) {    // for little endian
//		                        String temp0 = Integer.toBinaryString(bytes[j] & 0xFF);  // & 0xFF is for converting to unsigned
//		                        temp0 = String.format("%8s",temp0).replace(' ', '0');
//		                        binpR =  binpR + temp0  ;
//		                }
//		
//		                signStr = binpR.substring(0,1);
//		        signInt = Integer.parseInt(signStr, 2);
//		        espStr = binpR.substring(1,12);
//		        espInt = Integer.parseInt(espStr, 2);
//		        mantStr = binpR.substring(12,64);
//		        mantInt = Long.parseLong(mantStr, 2);
//		        mantInt2 = mantInt / Math.pow(2, 52);
//		        double pseudoRange = Math.pow(-1, signInt) * Math.pow(2, (espInt - 1023)) * (1 + mantInt2);   // FP64
//		        pseudoRange = pseudoRange * 299792458 * 0.001;   // velocity of light in the void [m/s]
//		        os.setCodeC(ObservationSet.L1, pseudoRange);
							       							        
		        
				/* C/A Pseudo Range (ms), 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double pseudoRange = Bits.byteToIEEE754Double(bytes);
		        pseudoRange = pseudoRange * Constants.SPEED_OF_LIGHT * 1e-3;   // velocity of light in the void [m/s]
		        if (pseudoRange < 1e6 || pseudoRange > 6e7) {
		        	anomalousValues = true;
		        }
				
				/*  Doppler Frequency(Hz), 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double dopplerFrequency = Bits.byteToIEEE754Double(bytes);
				float d1 = (float)dopplerFrequency; 
				if (Math.abs(d1) > 1e5) {
					d1 = 0;
				}
				
				/* Raw Data Flags, 1 byte */
				bytes = new byte[1];
				in.read(bytes, 0, bytes.length);
				int rawDataFlags = Bits.byteToInt(bytes);
				
				/* Reserved, 1 byte*/
				bytes = new byte[1];
				in.read(bytes, 0, bytes.length);
				
//				in.read(); // DLE
//		        in.read(); // ETX
									
//				if (os.getSatID() <= 32) {
//					
//				}
									
//				System.out.println("reserved: "+ reserved);
									
//				System.out.println("##### Satellite:  "+ i );
//				System.out.println("SatType: "+ satType);
//				System.out.println("Satellite Number: "+ satID);
//				System.out.println("Carrier Number: "+ carrierNum);
//				System.out.println("SNR: "+ snr);
//				System.out.println("Carrier Phase: "+ carrierPhase);	
//				System.out.println("Pseudo Range: "+ pseudoRange);
//				System.out.println("Doppler Frequency: "+ dopperFrequency);
//				System.out.println("Raw Data Flags: "+ rawDataFlags);
//				System.out.println("			");
									
									
				if (satType == 2 && satID != 33 && gpsEnable == true && !anomalousValues){  
				/* signalType 1:GLONASS, 2: GPS/QZSS, 4: SBAS, 8:Galileo */
					os.setSatID(satID);
					os.setSatType('G');
					os.setCodeC(ObservationSet.L1, pseudoRange);
			        os.setPhaseCycles(ObservationSet.L1, carrierPhase); 			
					os.setSignalStrength(ObservationSet.L1, snr);
					os.setDoppler(ObservationSet.L1, d1);
					o.setGps(gpsCounter, os);
					gpsCounter ++ ;
					
				}else if(satType == 2 && satID == 33 && qzsEnable == true && !anomalousValues) {
				/* QZSS: satID 33 */				
					satID = 1;
					os.setSatID(satID);
					os.setSatType('J');
					os.setCodeC(ObservationSet.L1, pseudoRange);
			        os.setPhaseCycles(ObservationSet.L1, carrierPhase); 			
					os.setSignalStrength(ObservationSet.L1, snr);
					os.setDoppler(ObservationSet.L1, d1);
					o.setGps(gpsCounter, os);
					gpsCounter ++ ;				
					
				}else if(satType == 1 && gloEnable == true && !anomalousValues){
				/* GLONASS */	
					os.setSatID(satID);
					os.setSatType('R');
					os.setCodeC(ObservationSet.L1, pseudoRange);
			        os.setPhaseCycles(ObservationSet.L1, carrierPhase); 			
					os.setSignalStrength(ObservationSet.L1, snr);
					os.setDoppler(ObservationSet.L1, d1);
					o.setGps(gpsCounter, os);
					gpsCounter ++ ;				
				}else if(satType == 8 && galEnable == true && !anomalousValues){
				/* Galileo */	
					os.setSatID(satID);
					os.setSatType('E');
					os.setCodeC(ObservationSet.L1, pseudoRange);
			        os.setPhaseCycles(ObservationSet.L1, carrierPhase); 			
					os.setSignalStrength(ObservationSet.L1, snr);
					os.setDoppler(ObservationSet.L1, d1);
					o.setGps(gpsCounter, os);
					gpsCounter ++ ;				
				}
				anomalousValues = false;
		}
//		System.out.println("+-----------------  End of F5  -------------------+");

		if (o.getNumSat() == 0) {
			o = null;
		}

		return o;		
		
	}

	private long getGMTTS(double tow, long week) {
		Calendar c = Calendar.getInstance();
		c.setTimeZone(TimeZone.getTimeZone("GMT Time"));
//		c.setTimeZone(c.getTimeZone());
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

		return (long)(c.getTimeInMillis() + week*7*24*3600*1000 + tow);
	}
}
