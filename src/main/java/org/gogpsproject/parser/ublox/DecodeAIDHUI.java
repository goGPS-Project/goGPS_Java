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

package org.gogpsproject.parser.ublox;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.IonoGps;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;


public class DecodeAIDHUI {
	//private boolean[] bits;
	InputStream in;

//	int[] fdata;
//	int[] fbits;
//	boolean end = true;

	public DecodeAIDHUI(InputStream _in) {
		in = _in;
	}

	public IonoGps decode() throws IOException,UBXException {
		// parse little Endian data

		IonoGps iono = new IonoGps();

		int[] length = new int[2];

		length[1] = in.read();
		length[0] = in.read();

		int CH_A = 0;
		int CH_B = 0;
		CH_A += 0x0B;CH_B += CH_A;
		CH_A += 0x02;CH_B += CH_A;
		CH_A += length[1];CH_B += CH_A;
		CH_A += length[0];CH_B += CH_A;

		int len = length[0]*256+length[1];
		ByteArrayOutputStream baos = new ByteArrayOutputStream(len);

		//System.out.println("AID-HUI "+len);

		byte bits[] = new byte[4];
		in.read(bits, 0, bits.length);
		long health=0;
		for(int i=3;i>=0;i--){
			health=health<<8;
			health = health | Bits.getUInt(bits[i]);
		}
		iono.setHealth(health);
		baos.write(bits);

		bits = new byte[8];
		in.read(bits, 0, bits.length);
		iono.setUtcA1(Bits.byteToIEEE754Double(bits));
		baos.write(bits);

		in.read(bits, 0, bits.length);
		iono.setUtcA0(Bits.byteToIEEE754Double(bits));
		baos.write(bits);

		bits = new byte[4];
		in.read(bits, 0, bits.length);
		iono.setUtcTOW(Bits.byteToLong(bits));
		baos.write(bits);

		bits = new byte[2];
		in.read(bits, 0, bits.length);
		iono.setUtcWNT(Bits.byteToInt(bits));
		baos.write(bits);

		in.read(bits, 0, bits.length);
		iono.setUtcLS(Bits.byteToInt(bits));
		baos.write(bits);

		in.read(bits, 0, bits.length);
		iono.setUtcWNF(Bits.byteToInt(bits));
		baos.write(bits);

		in.read(bits, 0, bits.length);
		iono.setUtcDN(Bits.byteToInt(bits));
		baos.write(bits);

		in.read(bits, 0, bits.length);
		iono.setUtcLSF(Bits.byteToInt(bits));
		baos.write(bits);

		in.read(bits, 0, bits.length);
		baos.write(bits);
		//long utcSpare = Bits.byteToInt(bits);

		bits = new byte[4];
		float alpha[] = new float[4];
		for(int i=0;i<alpha.length;i++){
			in.read(bits, 0, bits.length);
			alpha[i] = Bits.byteToIEEE754Float(bits);
			baos.write(bits);
		}
		iono.setAlpha(alpha);

		float beta[] = new float[4];
		for(int i=0;i<alpha.length;i++){
			in.read(bits, 0, bits.length);
			beta[i] = Bits.byteToIEEE754Float(bits);
			baos.write(bits);
		}
		iono.setBeta(beta);

		in.read(bits, 0, bits.length);
		int flags=0;
		for(int i=3;i>=0;i--){
			flags=flags<<8;
			flags = flags | Bits.getUInt(bits[i]);
		}
		iono.setValidHealth((flags & 0x1) == 1);
		iono.setValidUTC((flags & 0x2) == 0x2);
		iono.setValidKlobuchar((flags & 0x4) == 0x4);
		baos.write(bits);

		Time refTime = new Time((int)iono.getUtcWNT(),(int)iono.getUtcTOW());
		iono.setRefTime(refTime);
		//System.out.println(new Date(refTime.getMsec()));

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
			throw new UBXException("Wrong message checksum");

		return iono;
	}



}
