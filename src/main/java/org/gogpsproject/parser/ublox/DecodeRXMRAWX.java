/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl, Daisuke Yoshida. All Rights Reserved.
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

package org.gogpsproject.parser.ublox;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
//import java.text.SimpleDateFormat;
import java.util.Calendar;
//import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;


public class DecodeRXMRAWX {
	private InputStream in;

//	private int[] fdata;
//	private int[] fbits;
//	private boolean end = true;
	
	private Boolean[] multiConstellation;

	public DecodeRXMRAWX(InputStream in) {
		this.in = in;
	}

	public DecodeRXMRAWX(InputStream in, Boolean[] multiConstellation) throws IOException {
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
//		boolean galEnable = multiConstellation[3];
//		boolean bdsEnable = multiConstellation[4];
		
		
		if (len == 0) {
			throw new UBXException("Zero-length RXM-RAWX message");
		}

		//System.out.println("Length : " + len);
		data = new int[16];
		//System.out.print("\n Header ");
		for (int i = 0; i < 16; i++) {
			data[i] = in.read();
			CH_A += data[i];CH_B += CH_A;
			if(logos!=null) logos.write(data[i]);
//			System.out.print("0x" + Integer.toHexString(data[i]) + " ");
		}
		
		boolean[] bits = new boolean[8 * 8]; // rcvTow (R8)
		int indice = 0;
		for (int j = 7; j >= 0 ; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
//		System.out.println("tow :  " + UnsignedOperation.toDouble(Bits.tobytes(bits)) );
		double tow = UnsignedOperation.toDouble(Bits.tobytes(bits));
//		tow = tow * 1000; // convert to milliseconds 
//		System.out.println("tow :  " + tow );

				
		bits = new boolean[8 * 2]; // week (U2)
		indice = 0;
		for (int j = 9; j >= 8; j--) {
			boolean[] temp1 = Bits.intToBits(data[j], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
		}
		int week = (int)Bits.bitsTwoComplement(bits);
//		System.out.println("Week :  " + week );
		
		
		bits = new boolean[8]; // leapS (I1)
		indice = 0;
		boolean[] temp1 = Bits.intToBits(data[10], 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}			
		long leapS = Bits.bitsTwoComplement(bits);
		leapS = leapS * 1000; // convert to milliseconds 
//		tow = tow - leapS ;
//		System.out.println("leapS :  " + leapS );
		
		
		bits = new boolean[8]; // numMeas (U1)
		indice = 0;
		temp1 = Bits.intToBits(data[11], 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}		
		int numMeas = (int)Bits.bitsToUInt(bits);
//		System.out.println("numMeas :  " + numMeas + " S ");

		
		bits = new boolean[8]; // recStat (X1)
		indice = 0;
		temp1 = Bits.intToBits(data[12], 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}

		bits = new boolean[8]; // reserved1 (U1)
		indice = 0;
		temp1 = Bits.intToBits(data[13], 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}
//		System.out.println("Res1 :  " + Bits.bitsToUInt(bits) + "  ");
		
		
		bits = new boolean[8]; // reserved2 (U1)
		indice = 0;
		temp1 = Bits.intToBits(data[14], 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}
//		System.out.println("Res2 :  " + Bits.bitsToUInt(bits) + "  ");

		
		bits = new boolean[8]; // reserved3 (U1)
		indice = 0;
		temp1 = Bits.intToBits(data[15], 8);
		for (int i = 0; i < 8; i++) {
			bits[indice] = temp1[i];
			indice++;
		}
//		System.out.println("Res3 :  " + Bits.bitsToUInt(bits) + "  ");
		
		data = new int[len - 16];

		for (int i = 0; i < len - 16; i++) {
			data[i] = in.read();
			CH_A += data[i];CH_B += CH_A;
			if(logos!=null) logos.write(data[i]);
			//System.out.print("0x" + Integer.toHexString(data[i]) + " ");
		}
		//System.out.println();

//		long gmtTS = getGMTTS((long) tow, week);
//		gmtTS = gmtTS - leapS ;
//		Observations o = new Observations(new Time(gmtTS),0);
		Observations o = new Observations(new Time(week, tow),0);
//		Time refTime = new Time((int)week, tow/1000);
//		Observations o = new Observations(refTime,0);

		//System.out.println(gmtTS+" GPS time "+o.getRefTime().getGpsTime());
		//ubx.log( o.getRefTime().getGpsTime()+" "+tow+"\n\r");

		boolean anomalousValues = false;

		int gpsCounter = 0;

		for (int k = 0; k < (len - 16) / 32; k++) {

			ObservationSet os = new ObservationSet();

			int offset = k * 32;
			
			bits = new boolean[8 * 8]; // preMes (R8)
			indice = 0;
			for (int j = offset + 7; j >= 0 + offset; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			double pseudoRange = UnsignedOperation.toDouble(Bits.tobytes(bits));
			if (pseudoRange < 1e6 || pseudoRange > 6e7) {
				anomalousValues = true;
			}
//			System.out.print("SV" + k +"\tPhase: "
//					+ carrierPhase + "  ");
			
			bits = new boolean[8 * 8]; // cpMes (R8)
			indice = 0;
			for (int j = offset + 7 + 8; j >= 8 + offset; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			double carrierPhase = UnsignedOperation.toDouble(Bits.tobytes(bits));
//			System.out.print(" Code: "
//					+ pseudoRange + "  ");

			
			bits = new boolean[8 * 4]; // doMes (R4)
			indice = 0;
			for (int j = offset + 7 + 8 + 4; j >= 8 + 8 + offset; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
			float d1 = UnsignedOperation.toFloat(Bits.tobytes(bits));
//			System.out.print(" Doppler: "
//					+ d1 + "  ");
			
			bits = new boolean[8]; // gnssId (U1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			int satType = (int)Bits.bitsToUInt(bits) ;		
//			System.out.print(" gnssID: "
//					+ satType + "  ");		
			

			bits = new boolean[8];  // svId (U1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}

			int satID = (int)Bits.bitsToUInt(bits);
			if (satID <= 0) {
				anomalousValues = true;
			}
//			System.out.print(" satID: "
//					+ satID + "  ");
			
			bits = new boolean[8];  // reserved2 (U1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
//			System.out.print(" reserved2: "
//					+ Bits.bitsToUInt(bits) + "  ");
			
						
			bits = new boolean[8]; //freqId (U1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
//			System.out.print(" freqId: "
//					+ Bits.bitsToUInt(bits) + "  ");

			
			bits = new boolean[8 * 2]; // locktime (U2)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2], 8);			
			for (int j = offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2; j >=  1 + 1 + 1 + 1 + 4 + 8 + 8 + offset; j--) {
				temp1 = Bits.intToBits(data[j], 8);
				for (int i = 0; i < 8; i++) {
					bits[indice] = temp1[i];
					indice++;
				}
			}
//			System.out.print(" locktime: "
//					+ (int)Bits.bitsTwoComplement(bits) + "  ");
			
			
			bits = new boolean[8]; // cno (U1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			long snr = Bits.bitsToUInt(bits);	
//			System.out.print(" cno: "
//					+ snr + "  ");
			
			
			bits = new boolean[8]; // prStdev (X1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			
			
			bits = new boolean[8]; // cpStdev (X1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			
			
			bits = new boolean[8]; // doStdev (X1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			
			
			bits = new boolean[8]; // trkStat (X1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1 + 1 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			
			
			bits = new boolean[8]; // prStdev (U1)
			indice = 0;
			temp1 = Bits.intToBits(data[offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1 + 1 + 1 + 1 + 1], 8);
			for (int i = 0; i < 8; i++) {
				bits[indice] = temp1[i];
				indice++;
			}
			
			int total = offset + 7 + 8 + 4 + 1 + 1 + 1 + 1 + 2 + 1 + 1 + 1 + 1 + 1;
			//System.out.println("Offset " + total);
//			System.out.println();
			
			
			if (satType == 0 && gpsEnable == true && !anomalousValues){ 
				/* GPS */
				os.setSatType('G');
				os.setSatID(satID);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, d1);
				os.setSignalStrength(ObservationSet.L1, snr);
				o.setGps(gpsCounter, os);
				gpsCounter++;
				
//			} else if (satType == 1) { // SBAS
//				os.setSatType('S'); 
//				os.setSatID(satId);
			
//			} else if (satType == 2) { // Galileo
//				os.setSatType('E');
//				os.setSatID(satId);
				
//			} else if (satType == 3) { // BeiDou
//				os.setSatType('C');
//				os.setSatID(satId);
			
//			} else if (satType == 4) { // IMES
//				os.setSatType('I');
//				os.setSatID(satId);
				
			} else if (satType == 5 && qzsEnable == true && !anomalousValues) { 
				/* QZSS*/
				os.setSatType('J');
				os.setSatID(satID);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, d1);
				os.setSignalStrength(ObservationSet.L1, snr);
				o.setGps(gpsCounter, os);
				gpsCounter++;
						
			} else if (satType == 6 && gloEnable == true && !anomalousValues) { 
				/* GLONASS */
				os.setSatType('R');
				os.setSatID(satID);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, d1);
				os.setSignalStrength(ObservationSet.L1, snr);
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
