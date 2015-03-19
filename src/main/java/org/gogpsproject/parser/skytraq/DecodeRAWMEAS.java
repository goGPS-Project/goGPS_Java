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

package org.gogpsproject.parser.skytraq;

import java.io.IOException;
import java.io.InputStream;

import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.util.Bits;


public class DecodeRAWMEAS {
	private InputStream in;
	private Observations o;

	public DecodeRAWMEAS(InputStream in, Observations o) {
		this.in = in;
		this.o = o;
	}

	public Observations decode(int len) throws IOException, STQException {

		byte bytes[];
		
		/* IOD, 1 byte */				
		bytes = new byte[1];
		in.read(bytes, 0, bytes.length);
		int IOD = Bits.byteToIntBigEndian(bytes);

		bytes = new byte[1];
		in.read(bytes, 0, bytes.length);
		int NMEAS = Bits.byteToIntBigEndian(bytes);

		int gpsCounter = 0;
		boolean anomalousValues = false;
		for (int k = 0; k < NMEAS; k++) {

			ObservationSet os = new ObservationSet();

			/* Satellite Number, 1 byte */				
			bytes = new byte[1];
			in.read(bytes, 0, bytes.length);
			int satID = Bits.byteToIntBigEndian(bytes);
			os.setSatID(satID);

			/* signal-to-noise ratio, 1 byte */				
			bytes = new byte[1];
			in.read(bytes, 0, bytes.length);
			int CN0 = Bits.byteToIntBigEndian(bytes);

			/* C/A Pseudorange (m), 8 bytes  */
			bytes = new byte[8];
			in.read(bytes, 0, bytes.length);
			double pseudoRange = Bits.byteToIEEE754DoubleBigEndian(bytes);
//			if (pseudoRange < 1e6 || pseudoRange > 6e7) {
//	        	anomalousValues = true;
//	        }

			/* Carrier phase (cycles), 8 bytes  */
			bytes = new byte[8];
			in.read(bytes, 0, bytes.length);
			double carrierPhase = Bits.byteToIEEE754DoubleBigEndian(bytes);

			/*  Doppler Frequency(Hz), 4 bytes  */
			bytes = new byte[4];
			in.read(bytes, 0, bytes.length);
			float doppler = Bits.byteToIEEE754FloatBigEndian(bytes);
			
			/* channel indicator, 1 byte */				
			bytes = new byte[1];
			in.read(bytes, 0, bytes.length);
			
			if (o.getIssueOfData() == IOD && os.getSatID() <= 32 && !anomalousValues) {
				os.setSatType('G');
				os.setSignalStrength(ObservationSet.L1, CN0);
				os.setCodeC(ObservationSet.L1, pseudoRange);
				os.setPhaseCycles(ObservationSet.L1, carrierPhase);
				os.setDoppler(ObservationSet.L1, doppler);
				o.setGps(gpsCounter, os);
				gpsCounter++;
			}
		}

		if (o.getNumSat() == 0) {
			o = null;
		}
		
		return o;
	}
}
