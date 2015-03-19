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

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.EphGps;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;


public class DecodeF7 {

	InputStream in;
	private Boolean[] multiConstellation;

	public DecodeF7(InputStream _in, Boolean[] multiConstellation) {
		in = _in;
		this.multiConstellation = multiConstellation;
	}

	public EphGps decode() throws IOException,NVSException {
		// parse little Endian data


		EphGps eph = new EphGps();
		int satType = in.read();  // satType 1 = GPS, 2 = GLONASS		
		int satId = in.read();
		
		byte bytes[];
		
		boolean gpsEnable = multiConstellation[0];
		boolean qzsEnable = multiConstellation[1];
		boolean gloEnable = multiConstellation[2];
		boolean galEnable = multiConstellation[3];
		boolean bdsEnable = multiConstellation[4];
				
		if (satType == 1){   // GPS: 138(-2) bytes	

				int signInt;
				String signStr; 
				int espInt;
				String espStr;
				long mantInt;
				String mantStr; 
				double mantInt2;

				/*  Crs, 4 bytes  */		
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float crs = Bits.byteToIEEE754Float(bytes);		        
		        
		        /*  deltaN, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float deltaN = Bits.byteToIEEE754Float(bytes);
				deltaN = deltaN*1000;

		        /*  M0, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double m0 = Bits.byteToIEEE754Double(bytes);

		        /*  Cuc, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float cuc = Bits.byteToIEEE754Float(bytes);

		        /*  E, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);	
				double e = Bits.byteToIEEE754Double(bytes);
			
		        /*  Cus, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float cus = Bits.byteToIEEE754Float(bytes);

		        /*  SqrtA, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double rootA = Bits.byteToIEEE754Double(bytes);

		        /*  Toe, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double toe = Bits.byteToIEEE754Double(bytes);
				toe = toe/1000;

		        /*  Cic, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float cic = Bits.byteToIEEE754Float(bytes);

		        /*  Omega0, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double omega0 = Bits.byteToIEEE754Double(bytes);

				/*  Cis, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float cis = Bits.byteToIEEE754Float(bytes);

		        /*  I0, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double i0 = Bits.byteToIEEE754Double(bytes);

				/*  Crc, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float crc = Bits.byteToIEEE754Float(bytes);

		        /*  W, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double omega = Bits.byteToIEEE754Double(bytes);

				/*  OmegaR(OmegaDot), 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double omegaDot = Bits.byteToIEEE754Double(bytes);
				omegaDot = omegaDot*1000;

				/*  IDOT, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double iDot = Bits.byteToIEEE754Double(bytes);
				iDot = iDot*1000;
				
				/*  Tgd, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);
				float tgd = Bits.byteToIEEE754Float(bytes);
				tgd = tgd/1000;

		        /*  Toc, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double toc = Bits.byteToIEEE754Double(bytes);
				toc = toc/1000;
		        			
				/*  Af2, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);

		        /*  cannot use below code due to surpass the max value of Long  */
//				long af2 = Bits.byteToIEEE754Float(bytes);
								
				long longAf2 = Bits.byteToLong(bytes);
				String binAf2 = Long.toBinaryString(longAf2);
				binAf2 = String.format("%32s",binAf2).replace(' ', '0');
				
				signStr = binAf2.substring(0,1);
		        signInt = Integer.parseInt(signStr, 2);
		        espStr = binAf2.substring(1,9);
		        espInt = Integer.parseInt(espStr, 2);
		        mantStr = binAf2.substring(9);
		        mantInt = Integer.parseInt(mantStr, 2);
		        mantInt2 = mantInt / Math.pow(2, 23);
		        double af2 = (double) (Math.pow(-1, signInt) * Math.pow(2, (espInt - 127)) * (1 + mantInt2));  //FP32
		        af2 = af2*1000;

		        /*  Af1, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);			
				float af1 = Bits.byteToIEEE754Float(bytes);

		        /*  Af0, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);				
				float af0 = Bits.byteToIEEE754Float(bytes);
				af0 = af0/1000;

		        /*  URA(svaccur), 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long svAccur = Bits.byteToLong(bytes);	

				/*  IODE, 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long iode = Bits.byteToLong(bytes);
				
				/*  IODC, 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long iodc = Bits.byteToLong(bytes);		

				/*  CodeL2, 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long l2Code = Bits.byteToLong(bytes);

				/*  L2_Pdata_flag, 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long l2Flag = Bits.byteToLong(bytes);	

				/*  WeekN, 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long week = Bits.byteToLong(bytes);

				if (gpsEnable == true) {
					
					eph.setSatType('G');
					eph.setSatID((int)satId);
			        eph.setCrs(crs);
			        eph.setDeltaN(deltaN);
					eph.setM0(m0);
			        eph.setCuc(cuc);
					eph.setE(e);
			        eph.setCus(cus);
					eph.setRootA(rootA);
					eph.setToe(toe);
			        eph.setCic(cic);
					eph.setOmega0(omega0);
			        eph.setCis(cis);
					eph.setI0(i0);
			        eph.setCrc(crc);
					eph.setOmega(omega);
					eph.setOmegaDot(omegaDot);
					eph.setiDot(iDot);
			        eph.setTgd(tgd);
					eph.setToc(toc);
			        eph.setAf2(af2);
			        eph.setAf1(af1);
			        eph.setAf0(af0);
					eph.setSvAccur((int)svAccur);
					eph.setIode((int)iode);
					eph.setIodc((int)iodc);
					eph.setL2Code((int)l2Code);
					eph.setL2Flag((int)l2Flag);
					eph.setWeek((int)week);
				}
			
//				System.out.println("+----------------  Start of F7 (GPS) ------------------+");
//				System.out.println("satType: " + satType);  
//				System.out.println("GPS PRN: " + satId);  
//		        System.out.println("Crs: "+ crs);		
//		        System.out.println("deltaN: "+ deltaN);	   
//				System.out.println("M0: "+ m0);			        
//		        System.out.println("Cuc: "+ cuc);	        
//				System.out.println("E: "+ e);				
//		        System.out.println("Cus: "+ cus);        
//				System.out.println("SqrtA: "+ rootA);		
//				System.out.println("Toe: "+ toe);			
//		        System.out.println("Cis: "+ cis);
//		        System.out.println("Cic: "+ cic);        
//				System.out.println("Omega0: "+ omega0);			
//				System.out.println("I0: "+ i0);			
//		        System.out.println("Crc: "+ crc);  
//				System.out.println("W: "+ omega);	
//				System.out.println("OmegaR: "+ omegaDot);				
//				System.out.println("IDOT: "+ iDot);	
//		        System.out.println("Tgd: "+ tgd);
//				System.out.println("toc: "+ toc);		
//		        System.out.println("Af2: "+ af2);
//		        System.out.println("Af1: "+ af1);	        
//		        System.out.println("Af0: "+ af0);  
//				System.out.println("URA: " + svAccur);
//				System.out.println("IODE: " + iode);
//				System.out.println("IODC: " + iodc);
//				System.out.println("CodeL2: " + l2Code);
//				System.out.println("L2_Pdata_flag: " + l2Flag);
//				System.out.println("WeekN: " + week);
//				System.out.println("+-----------------  End of F7  ----------------------+");
						        
        
		}else if (satType == 2 && gloEnable == true){  // GLONASS: 93 (-2) bytes

				/*  Carrier Number, 1 bytes  */
				int carrierNum = in.read();
				
				/*  X, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double X = Bits.byteToIEEE754Double(bytes);
				X = X * 1e3;
	
				/*  Y, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Y = Bits.byteToIEEE754Double(bytes);
				Y = Y * 1e3;
				
				/*  Z, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Z = Bits.byteToIEEE754Double(bytes);
				Z = Z * 1e3;

				/*  Xv, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Xv = Bits.byteToIEEE754Double(bytes);
				Xv = Xv * 1e3;

				/*  Yv, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Yv = Bits.byteToIEEE754Double(bytes);
				Yv = Yv * 1e3;

				/*  Zv, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Zv = Bits.byteToIEEE754Double(bytes);
				Zv = Zv * 1e3;

				/*  Xa, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Xa = Bits.byteToIEEE754Double(bytes);
				Xa = Xa * 1e3;

				/*  Ya, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Ya = Bits.byteToIEEE754Double(bytes);
				Ya = Ya * 1e3;

				/*  Za, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double Za = Bits.byteToIEEE754Double(bytes);
				Za = Za * 1e3;

				/*  tb, 8 bytes  */
				bytes = new byte[8];
				in.read(bytes, 0, bytes.length);
				double tb = Bits.byteToIEEE754Double(bytes);
				tb = tb * 1e-3;

				/*  gammaN, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);				
				float gammaN = Bits.byteToIEEE754Float(bytes);

				 /*  tn, 4 bytes  */
				bytes = new byte[4];
				in.read(bytes, 0, bytes.length);				
				float tn = Bits.byteToIEEE754Float(bytes);

				/*  En, 2 bytes  */
				bytes = new byte[2];
				in.read(bytes, 0, bytes.length);
				long En = Bits.byteToLong(bytes);

				/* tb is a time interval within the current day (UTC + 3 hours)*/
				double tk = tb - 10800;

//				System.out.println("+--------------  Start of F7 (GLONASS)  -------------+");
//				System.out.println("satType: " + satType);  
//				System.out.println("GLONASS PRN: " + satId);
//				System.out.println("carrierNum: " + carrierNum);  
//		        System.out.println("Xm: "+ Xm);		
//		        System.out.println("Ym: "+ Ym);	   
//				System.out.println("Zm: "+ Zm);			        
//		        System.out.println("Vx: "+ Vx);	        
//				System.out.println("Vy: "+ Vy);				
//		        System.out.println("Vz: "+ Vz);        
//				System.out.println("Ax: "+ Ax);		
//				System.out.println("Ay: "+ Ay);			
//		        System.out.println("Az: "+ Az);
//		        System.out.println("tb: "+ tb);       
//				System.out.println("tb: "+ tb);		
//				System.out.println("tk: "+ tk);			

//				System.out.println("gammaN: "+ gammaN);			
//		        System.out.println("tn: "+ tn);  
//				System.out.println("En: "+ En);	
//				System.out.println("+-----------------  End of F7  -----------------------+");
			
				if (gloEnable) {
					eph.setSatType('R');
					eph.setSatID((int)satId);
					eph.setX(X);
					eph.setY(Y);
					eph.setZ(Z);
					eph.setXv(Xv);
					eph.setYv(Yv);
					eph.setZv(Zv);
					eph.setXa(Xa);
					eph.setYa(Ya);
					eph.setZa(Za);
					eph.settb(tb);
					eph.setfreq_num(carrierNum);
					eph.setGammaN(gammaN);
					eph.setTauN(tn);
					eph.setEn(En);
					eph.settk(tk);
				}
		}
		
//		in.read(); // DLE
//      in.read(); // ETX

		return eph;
	}


}
