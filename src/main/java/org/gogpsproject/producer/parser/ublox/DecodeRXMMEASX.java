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


public class DecodeRXMMEASX {
	private InputStream in;

//	private int[] fdata;
//	private int[] fbits;
//	private boolean end = true;
	
	private Boolean[] multiConstellation;

	public DecodeRXMMEASX(InputStream in) {
		this.in = in;
	}

	public DecodeRXMMEASX(InputStream in, Boolean[] multiConstellation) throws IOException {
		this.in = in;		
		this.multiConstellation = multiConstellation;
	}
	
	public Observations decode(OutputStream logos) throws IOException, UBXException {
		// parse little Endian data
		int[] length = new int[2];
		int[] data;

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
			throw new UBXException("Zero-length RXM-MEASX message");
		}

//		System.out.println("Length : " + len);

		int ver = in.read();
//		System.out.println("ver:  " + ver );
		
		for (int i = 0; i < 3; i++) { // reserved1
			in.read(); 
		}

		data = new int[4]; // gpsTow (U4)
		for (int i = 0; i < 4; i++) {
			data[i] = in.read();
		}
		boolean[] bits = new boolean[4 * 8]; 
		int indice = 0;
		for (int j = 3; j >= 0 ; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		int gpsTow = UnsignedOperation.toInt(Bits.tobytes(bits));
//		System.out.println("gpsTow:  " + gpsTow );

		data = new int[4]; // gloTow (U4)
		for (int i = 0; i < 4; i++) {
			data[i] = in.read();
		}
		bits = new boolean[4 * 8]; 
		indice = 0;
		for (int j = 3; j >= 0 ; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		int gloTow = UnsignedOperation.toInt(Bits.tobytes(bits));
//		System.out.println("gloTow:  " + gloTow );
		
		data = new int[4]; // bdsTOW (U4)
		for (int i = 0; i < 4; i++) {
			data[i] = in.read();
		}
		bits = new boolean[4 * 8]; 
		indice = 0;
		for (int j = 3; j >= 0 ; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		int bdsTOW = UnsignedOperation.toInt(Bits.tobytes(bits));
//		System.out.println("bdsTOW:  " + bdsTOW );
		
		for (int i = 0; i < 4; i++) { // reserved2
			in.read(); 
		}
	
		data = new int[4]; // qzssTOW (U4)
		for (int i = 0; i < 4; i++) {
			data[i] = in.read();
		}
		bits = new boolean[4 * 8]; 
		indice = 0;
		for (int j = 3; j >= 0 ; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		int qzssTOW = UnsignedOperation.toInt(Bits.tobytes(bits));
//		System.out.println("qzssTOW:  " + qzssTOW );

		data = new int[2]; // gpsTOWacc (U2)
		for (int i = 0; i < 2; i++) {
			data[i] = in.read();
		}
		bits = new boolean[2 * 8];  
		indice = 0;
		for (int j = 1; j >= 0; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		float gpsTOWacc = UnsignedOperation.toShort(Bits.tobytes(bits));
		gpsTOWacc *= Math.pow(2, -4);
//		System.out.println("gpsTOWacc:  " + gpsTOWacc );
		
		data = new int[2]; // gloTOWacc (U2)
		for (int i = 0; i < 2; i++) {
			data[i] = in.read();
		}
		bits = new boolean[2 * 8];  
		indice = 0;
		for (int j = 1; j >= 0; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		float gloTOWacc = UnsignedOperation.toShort(Bits.tobytes(bits));
		gloTOWacc *= Math.pow(2, -4);
//		System.out.println("gloTOWacc:  " + gloTOWacc );

		data = new int[2]; // bdsTOWacc (U2)
		for (int i = 0; i < 2; i++) {
			data[i] = in.read();
		}
		bits = new boolean[2 * 8];  
		indice = 0;
		for (int j = 1; j >= 0; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		float bdsTOWacc = UnsignedOperation.toShort(Bits.tobytes(bits));
		bdsTOWacc *= Math.pow(2, -4);
//		System.out.println("bdsTOWacc:  " + bdsTOWacc );

		data = new int[2]; // qzssTOWacc (U2)
		for (int i = 0; i < 2; i++) {
			data[i] = in.read();
		}
		bits = new boolean[2 * 8];  
		indice = 0;
		for (int j = 1; j >= 0; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		float qzssTOWacc = UnsignedOperation.toShort(Bits.tobytes(bits));
		qzssTOWacc *= Math.pow(2, -4);
//		System.out.println("qzssTOWacc:  " + qzssTOWacc );
		
		for (int i = 0; i < 2; i++) { // reserved3
			in.read(); 
		}
		
		bits = new boolean[8]; // numSV (U1)
		indice = 0;
		boolean temp1[] = Bits.intToBits(in.read(), 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}		
		int numSV = (int)Bits.bitsToUInt(bits);
//		System.out.println("numSV :  " + numSV );
		
		int flags = in.read();
//		System.out.println("flags :  " + Integer.toString(flags,2) );

		for (int i = 0; i < 8; i++) { // reserved4
			in.read(); 
		}

		boolean anomalousValues = false;
		int gpsCounter = 0;

		Observations o = new Observations(new Time(0, gpsTow),0);
		
		for (int k = 0; k < numSV; k++) {

			int gnssId = in.read();
//			System.out.println("gnssId:  " + gnssId );

			int svId = in.read();
//			System.out.println("svId:  " + svId );

			int cNo = in.read();
//			System.out.println("cNo:  " + cNo );

			int mpathIndic = in.read();
//			System.out.println("mpathIndic:  " + mpathIndic );
			
			data = new int[4]; // dopplerMS [m/s] (I4)
			for (int i = 0; i < 4; i++) {
				data[i] = in.read();
			}
			bits = new boolean[4 * 8]; 
			indice = 0;
			for (int j = 3; j >= 0 ; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			float dopplerMS = (float)UnsignedOperation.toInt(Bits.tobytes(bits));
			dopplerMS *= 0.04;
//			System.out.println("dopplerMS:  " + dopplerMS );

			data = new int[4]; // dopplerHz (I4)
			for (int i = 0; i < 4; i++) {
				data[i] = in.read();
			}
			bits = new boolean[4 * 8]; 
			indice = 0;
			for (int j = 3; j >= 0 ; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			float dopplerHz = (float)UnsignedOperation.toInt(Bits.tobytes(bits));
			dopplerHz *= 0.2;
//			System.out.println("dopplerHz:  " + dopplerHz );

			data = new int[2]; // wholeChips [0..1022] (U2)
			for (int i = 0; i < 2; i++) {
				data[i] = in.read();
			}
			bits = new boolean[8 * 2];  
			indice = 0;
			for (int j = 1; j >= 0; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			int wholeChips = (int)Bits.bitsTwoComplement(bits);
//			System.out.println("wholeChips :  " + wholeChips );

			data = new int[2]; // fracChips [0..1023] (U2)
			for (int i = 0; i < 2; i++) {
				data[i] = in.read();
			}
			bits = new boolean[8 * 2];  
			indice = 0;
			for (int j = 1; j >= 0; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			int fracChips = (int)Bits.bitsTwoComplement(bits);
//			System.out.println("fracChips :  " + fracChips );

			data = new int[4]; // codePhase [ms] (U4)
			for (int i = 0; i < 4; i++) {
				data[i] = in.read();
			}
			bits = new boolean[4 * 8]; 
			indice = 0;
			for (int j = 3; j >= 0 ; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			float codePhase = UnsignedOperation.toInt(Bits.tobytes(bits));
			codePhase *= Math.pow(2,-21);
//			System.out.println("codePhase:  " + codePhase );
		
			bits = new boolean[8]; // intCodePhase [ms] (U1)
			indice = 0;
			temp1 = Bits.intToBits(in.read(), 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}		
			int intCodePhase = (int)Bits.bitsToUInt(bits);
//			System.out.println("intCodePhase :  " + intCodePhase );

			bits = new boolean[8]; // pseuRangeRMSErr [0..63] (U1)
			indice = 0;
			temp1 = Bits.intToBits(in.read(), 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}		
			int pseuRangeRMSErr = (int)Bits.bitsToUInt(bits);
//			System.out.println("pseuRangeRMSErr :  " + pseuRangeRMSErr );

			for (int i = 0; i < 2; i++) { //reserved5
				in.read();
			}

			ObservationSet os = new ObservationSet();

			// TODO Check conversion with constellations other than GPS
			double pseudoRange = (intCodePhase + codePhase)/ 1000.0 * Constants.SPEED_OF_LIGHT;
//          System.out.println("Range " + pseudoRange );
					
//			System.out.print("SV" + k +"\tPhase: " + carrierPhase + "  ");
			double carrierPhase = 0;
			
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
		// / Checksum
		CH_A = CH_A & 0xFF;
		CH_B = CH_B & 0xFF;

		int c1 = in.read();
		if(logos!=null) logos.write(c1);

		int c2 = in.read();
		if(logos!=null) logos.write(c2);

//		if(CH_A != c1 || CH_B!=c2)
//			throw new UBXException("Wrong message checksum");
		if (o.getNumSat() == 0) {
			o = null;
		}
		
		return o;
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
