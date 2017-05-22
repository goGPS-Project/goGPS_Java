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

package org.gogpsproject.producer.parser.skytraq;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;


public class DecodeGPSEPH {

	private final static double TWO_P_4   = 16.0; // 2^4
	private final static double TWO_P_M5  = 0.03125; // 2^-5
	private final static double TWO_P_M19 = 0.0000019073486328125; // 2^-19
	private final static double TWO_P_M29 = 0.00000000186264514923095703125; // 2^-29
	private final static double TWO_P_M31 = 0.0000000004656612873077392578125; // 2^-31
	private final static double TWO_P_M33 = 1.16415321826934814453125e-10; // 2^-33
	private final static double TWO_P_M43 = 1.136868377216160297393798828125e-13; // 2^-43
	private final static double TWO_P_M55 = 2.7755575615628913510590791702271e-17; // 2^-55
	private final static double PI        = 3.141592653589793238462643383279502884197169399375105820974944592;

	InputStream in;

	public DecodeGPSEPH(InputStream _in) {
		in = _in;
	}

	public EphGps decode() throws IOException,STQException {
		// parse little Endian data

		EphGps eph = new EphGps();

		int[] length = new int[2];

		length[1] = in.read();
		length[0] = in.read();

		int CH_A = 0;
		int CH_B = 0;
		CH_A += 0x0B;CH_B += CH_A;
		CH_A += 0x31;CH_B += CH_A;
		CH_A += length[1];CH_B += CH_A;
		CH_A += length[0];CH_B += CH_A;

		int len = length[0]*256+length[1];
		if(len == 8){
			for (int b = 0; b < len+2; b++) {
				in.read();
			}
			return null;
		}

		ByteArrayOutputStream baos = new ByteArrayOutputStream(len);

		//System.out.println("AID-EPH "+len);

		byte bytes[] = new byte[4];
		in.read(bytes, 0, bytes.length);
		long svid=0;
		for(int i=3;i>=0;i--){
			svid=svid<<8;
			svid = svid | Bits.getUInt(bytes[i]);
		}
		eph.setSatID((int)svid);
		//System.out.print("svid "+svid);
		baos.write(bytes);

		int how =0;
		bytes = new byte[4];
		in.read(bytes, 0, bytes.length);
		boolean bits[] = new boolean[bytes.length*8];
		for (int j = 3; j >= 0; j--) {
			boolean[] temp1 = Bits.intToBits(Bits.getUInt(bytes[j]), 8);
			how=(int)Bits.bitsToUInt(temp1);
			//System.out.print((sf1d[b]<=0xf?"0":"")+Integer.toHexString(sf1d[b])+" ");
			for(int i=0;i<8;i++){
				//System.out.println(" "+(b*8*4+((3-j)*8)+i)+" ");
				bits[((3-j)*8)+i] = temp1[i];
			}
		}
		//System.out.print(svid+" how "+Bits.bitsToStr(bits));
		//System.out.println(" tow "+Bits.bitsToUInt(Bits.subset(bits, 0, 17)));
// TODO UNDERSTAND HOW CONTENT
		baos.write(bytes);

//		in.read(bytes, 0, bytes.length);
//		long how=0;
//		for(int i=3;i>=0;i--){
//			how=how<<8;
//			how = how | Bits.getUInt(bytes[i]);
//		}
//		//System.out.println(" how "+how);
//		baos.write(bytes);

		int sf1d[] = new int[8];
		bytes = new byte[sf1d.length*4];
		in.read(bytes, 0, bytes.length);
		bits = new boolean[bytes.length*8];
		for(int b=0;b<sf1d.length;b++){
			for (int j = 3; j >= 0; j--) {
				boolean[] temp1 = Bits.intToBits(Bits.getUInt(bytes[b*4+j]), 8);
				sf1d[b]=(int)Bits.bitsToUInt(temp1);
				//System.out.print((sf1d[b]<=0xf?"0":"")+Integer.toHexString(sf1d[b])+" ");
				for(int i=0;i<8;i++){
					//System.out.println(" "+(b*8*4+((3-j)*8)+i)+" ");
					bits[b*8*4+((3-j)*8)+i] = temp1[i];
				}
			}
		}
		baos.write(bytes);

		//eph.setWeek(Bits.bitsToUInt(Bits.subset(bits, 9, 10)));
//		System.out.println();
		eph.setWeek((int)Bits.bitsToUInt(Bits.subset(bits, 8, 10)));
		eph.setL2Code((int)Bits.bitsToUInt(Bits.subset(bits, 18, 2)));
		eph.setSvAccur((int)Bits.bitsToUInt(Bits.subset(bits, 20, 4)));
		eph.setSvHealth((int)Bits.bitsToUInt(Bits.subset(bits, 24, 6)));
		eph.setIodc((int)Bits.bitsToUInt(Bits.concat(Bits.subset(bits, 30, 2), Bits.subset(bits, 5*32+8, 8))));
		eph.setL2Flag((int)Bits.bitsToUInt(Bits.subset(bits, 1*32+8, 1)));
		eph.setTgd(Bits.bitsTwoComplement(Bits.subset(bits, 4*32+24, 8))*TWO_P_M31);
		eph.setToc(Bits.bitsToUInt(Bits.subset(bits, 5*32+16, 16))*TWO_P_4);
		eph.setAf2(Bits.bitsTwoComplement(Bits.subset(bits, 6*32+8, 8))*TWO_P_M55);
		eph.setAf1(Bits.bitsTwoComplement(Bits.subset(bits, 6*32+16, 16))*TWO_P_M43);
		eph.setAf0(Bits.bitsTwoComplement(Bits.subset(bits, 7*32+8, 22))*TWO_P_M31);

//		int msb = Bits.bitsToUInt(Bits.subset(bits, 30, 2));
//		int lsb = Bits.bitsToUInt(Bits.subset(bits, 5*32+8, 8));
//		System.out.println("week "+Bits.bitsToUInt(Bits.subset(bits, 8, 10)));
//		System.out.println("L2 code "+Bits.bitsToUInt(Bits.subset(bits, 18, 2)));
//		System.out.println("svaccur "+Bits.bitsToUInt(Bits.subset(bits, 20, 4)));
//		System.out.println("svhealth "+Bits.bitsToUInt(Bits.subset(bits, 24, 6)));
//		System.out.println("msb "+msb+" lsb "+lsb+" IODC "+(msb*256+lsb));
//		System.out.println("L2flag "+Bits.bitsToUInt(Bits.subset(bits, 1*32+8, 1)));
//		System.out.println("tgd "+(Bits.bitsTwoComplement(Bits.subset(bits, 4*32+24, 8))*TWO_P_M31)); // 2^-31
//		System.out.println("toc "+(Bits.bitsToUInt(Bits.subset(bits, 5*32+16, 16))*TWO_P_4)); // 2^4
//		System.out.println("af2 "+(Bits.bitsTwoComplement(Bits.subset(bits, 6*32+8, 8))*TWO_P_M55)); // 2^-55
//		System.out.println("af1 "+(Bits.bitsTwoComplement(Bits.subset(bits, 6*32+16, 16))*TWO_P_M43)); // 2^-43
//		System.out.println("af0 "+(Bits.bitsTwoComplement(Bits.subset(bits, 7*32+8, 22))*TWO_P_M31)); // 2^-31

//		System.out.println();
		int sf2d[] = new int[8];
		bytes = new byte[sf2d.length*4];
		in.read(bytes, 0, bytes.length);
		//System.out.println(Bits.toHexString(bytes, bytes.length));
		for(int b=0;b<sf2d.length;b++){
			for (int j = 3; j >= 0; j--) {
				boolean[] temp1 = Bits.intToBits(Bits.getUInt(bytes[b*4+j]), 8);
				sf2d[b]=(int)Bits.bitsToUInt(temp1);
				for(int i=0;i<8;i++){
					//System.out.println(" "+(b*8*4+((3-j)*8)+i)+" ");
					bits[b*8*4+((3-j)*8)+i] = temp1[i];
				}
				//System.out.print((sf2d[b]<=0xf?"0":"")+Integer.toHexString(sf2d[b])+" ");
			}
		}
		baos.write(bytes);

		int IODE2 = (int)Bits.bitsToUInt(Bits.subset(bits, 8, 8));
		eph.setCrs(Bits.bitsTwoComplement(Bits.subset(bits, 16, 16))*TWO_P_M5);
		eph.setDeltaN(Bits.bitsTwoComplement(Bits.subset(bits, 1*32+8, 16)) * PI * TWO_P_M43);
		eph.setM0(Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 1*32+24, 8), Bits.subset(bits, 2*32+8, 24)) ) * PI * TWO_P_M31);
		eph.setCuc(Bits.bitsTwoComplement(Bits.subset(bits, 3*32+8, 16)) * TWO_P_M29);
		eph.setE(Bits.bitsToULong( Bits.concat(Bits.subset(bits, 3*32+24, 8), Bits.subset(bits, 4*32+8, 24)) ) * TWO_P_M33);
		eph.setCus(Bits.bitsTwoComplement(Bits.subset(bits, 5*32+8, 16)) * TWO_P_M29);
		eph.setRootA(Bits.bitsToULong( Bits.concat(Bits.subset(bits, 5*32+24, 8), Bits.subset(bits, 6*32+8, 24)) ) * TWO_P_M19);
		eph.setToe(Bits.bitsToUInt(Bits.subset(bits, 7*32+8, 16)) * TWO_P_4);
		eph.setFitInt(Bits.bitsToUInt(Bits.subset(bits, 7*32+24, 1)));

//		System.out.println();
//		System.out.println("IODE2 "+Bits.bitsToUInt(Bits.subset(bits, 8, 8)));
//		System.out.println("Crs "+Bits.bitsTwoComplement(Bits.subset(bits, 16, 16)));//2^-5
//		System.out.println("delta_n "+Bits.bitsTwoComplement(Bits.subset(bits, 1*32+8, 16)));//pi * 2^-43
//		System.out.println("M0 "+Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 1*32+24, 8), Bits.subset(bits, 2*32+8, 24)) ));//pi * 2^-31
//		System.out.println("Cuc "+Bits.bitsTwoComplement(Bits.subset(bits, 3*32+8, 16)));// 2^-29
//		System.out.println("e "+Bits.bitsToULong( Bits.concat(Bits.subset(bits, 3*32+24, 8), Bits.subset(bits, 4*32+8, 24)) ));// 2^-33
//		System.out.println("Cus "+Bits.bitsTwoComplement(Bits.subset(bits, 5*32+8, 16)));// 2^-29
//		System.out.println("root_A "+Bits.bitsToULong( Bits.concat(Bits.subset(bits, 5*32+24, 8), Bits.subset(bits, 6*32+8, 24)) ));// 2^-19
//		System.out.println("toe "+Bits.bitsToUInt(Bits.subset(bits, 7*32+8, 16))); // 2^4
//		System.out.println("fit_int "+Bits.bitsToUInt(Bits.subset(bits, 7*32+24, 1)));
//		System.out.println();



		int sf3d[] = new int[8];
		bytes = new byte[sf3d.length*4];
		in.read(bytes, 0, bytes.length);
		//System.out.println(Bits.toHexString(bytes, bytes.length));
		for(int b=0;b<sf3d.length;b++){
			for (int j = 3; j >= 0; j--) {
				boolean[] temp1 = Bits.intToBits(Bits.getUInt(bytes[b*4+j]), 8);
				sf3d[b]=(int)Bits.bitsToUInt(temp1);
				//System.out.print((sf3d[b]<=0xf?"0":"")+Integer.toHexString(sf3d[b])+" ");
				for(int i=0;i<8;i++){
					//System.out.println(" "+(b*8*4+((3-j)*8)+i)+" ");
					bits[b*8*4+((3-j)*8)+i] = temp1[i];
				}
			}
		}
		baos.write(bytes);

		eph.setCic(Bits.bitsTwoComplement(Bits.subset(bits, 8, 16)) * TWO_P_M29);
		eph.setOmega0(Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 0*32+24, 8), Bits.subset(bits, 1*32+8, 24)) ) * PI * TWO_P_M31);
		eph.setCis(Bits.bitsTwoComplement(Bits.subset(bits, 2*32+8, 16)) * TWO_P_M29);
		eph.setI0(Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 2*32+24, 8), Bits.subset(bits, 3*32+8, 24)) ) * PI * TWO_P_M31);
		eph.setCrc(Bits.bitsTwoComplement(Bits.subset(bits, 4*32+8, 16)) * TWO_P_M5);
		eph.setOmega(Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 4*32+24, 8), Bits.subset(bits, 5*32+8, 24)) ) * PI * TWO_P_M31);
		eph.setOmegaDot(Bits.bitsTwoComplement(Bits.subset(bits, 6*32+8, 24)) * PI * TWO_P_M43);
		int IODE3 = (int)Bits.bitsToUInt(Bits.subset(bits, 7*32+8, 8));
		eph.setiDot(Bits.bitsTwoComplement(Bits.subset(bits, 7*32+16, 14)) * PI * TWO_P_M43);

//		System.out.println();
//		System.out.println("Cic "+Bits.bitsTwoComplement(Bits.subset(bits, 8, 16)));//2^-29
//		System.out.println("omega0 "+Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 0*32+24, 8), Bits.subset(bits, 1*32+8, 24)) ));//pi * 2^-31
//		System.out.println("Cis "+Bits.bitsTwoComplement(Bits.subset(bits, 2*32+8, 16)));//2^-29
//		System.out.println("i0 "+Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 2*32+24, 8), Bits.subset(bits, 3*32+8, 24)) ));//pi * 2^-31
//		System.out.println("Crc "+Bits.bitsTwoComplement(Bits.subset(bits, 4*32+8, 16)));//2^-5
//		System.out.println("omega "+Bits.bitsTwoComplement( Bits.concat(Bits.subset(bits, 4*32+24, 8), Bits.subset(bits, 5*32+8, 24)) ));//pi * 2^-31
//		System.out.println("omegadot "+Bits.bitsTwoComplement(Bits.subset(bits, 6*32+8, 24)));//pi * 2^-43
//		System.out.println("IODE3 "+Bits.bitsToUInt(Bits.subset(bits, 7*32+8, 8))); //
//		System.out.println("IDOT "+Bits.bitsTwoComplement(Bits.subset(bits, 7*32+16, 14)));//pi * 2^-43


		byte data[] = baos.toByteArray();
		for (int i = 0; i < data.length; i++) {
			CH_A += Bits.getUInt(data[i]);
			CH_B += CH_A;
			//System.out.print("0x" + Integer.toHexString(data[i]) + " ");
		}

		CH_A = CH_A & 0xFF;
		CH_B = CH_B & 0xFF;
		int a= in.read();
		int b= in.read();

		if(CH_A != a && CH_B!=b)
			throw new STQException("Wrong message checksum for SVID "+svid);

		// TODO Ref Time Time Ref
		eph.setRefTime(new Time(System.currentTimeMillis()));

		if(eph.getIodc() == IODE2 && eph.getIodc() == IODE3){
			eph.setIode(IODE3);
			return eph;
		}
		System.out.println("IODC("+eph.getIodc()+"), IODE2("+IODE2+"), IODE3("+IODE3+") not matching for SVID "+svid);

		return null;
	}


}
