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

package org.gogpsproject.parser.rtcm3;

import org.gogpsproject.Coordinates;
import org.gogpsproject.util.Bits;

public class Decode1006Msg implements Decode {


	private RTCM3Client client;
	public Decode1006Msg(RTCM3Client client) {
		this.client = client;
	}


	public Object decode(boolean[] bits, int referenceTS) {
		int start = 12;
		//System.out.println("Debug : Decode 1006");
		StationaryAntenna stationaryantenne = new StationaryAntenna();

		stationaryantenne.setStationID((int)Bits.bitsToUInt(Bits.subset(bits, start, 12)));
		start += 12;
		stationaryantenne.setItrl((int)Bits.bitsToUInt(Bits.subset(bits, start, 6)));
		start += 6;
		stationaryantenne.setGpsIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;
		stationaryantenne.setGlonassIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;
		stationaryantenne.setRgalileoIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;
		stationaryantenne.setRstationIndicator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;
		//System.out.println("x"+Bits.bitsToStr(Bits.subset(bits, start, 38)));
		stationaryantenne.setAntennaRefPointX(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
		start += 38;
		stationaryantenne.setSreceiverOscillator((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;
		stationaryantenne.setReserved1((int)Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		start += 1;
		//System.out.println("y"+Bits.bitsToStr(Bits.subset(bits, start, 38)));
		stationaryantenne.setAntennaRefPointY(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
		start += 38;
		stationaryantenne.setReserved2((int)Bits.bitsToUInt(Bits.subset(bits, start, 2)));
		start += 2;
		//System.out.println("z"+Bits.bitsToStr(Bits.subset(bits, start, 38)));
		stationaryantenne.setAntennaRefPointZ(Bits.bitsTwoComplement(Bits.subset(bits, start, 38)) * 0.0001);
		start += 38;
		stationaryantenne.setAntennaHeight(Bits.bitsToUInt(Bits.subset(bits, start, 16)) * 0.0001);
		start += 16;

		Coordinates c = Coordinates.globalXYZInstance(stationaryantenne.getAntennaRefPointX(), stationaryantenne.getAntennaRefPointY(), stationaryantenne.getAntennaRefPointZ());
		client.setMasterPosition(c);
		//System.out.println(stationaryantenne);
		//System.out.println("Debug length: " + start);
		return c;
	}

}
