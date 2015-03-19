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

import org.gogpsproject.util.Bits;

public class Decode1008Msg implements Decode {

	private RTCM3Client client;
	public Decode1008Msg(RTCM3Client client) {
		this.client = client;
	}

	public Object decode(boolean[] bits, int referenceTS) {
		AntennaDescriptor antenna = new AntennaDescriptor();
		int start = 12;
		String desc = "";
		String serial = "";
		antenna.setStationID((int)Bits.bitsToUInt(Bits.subset(bits, start, 12)));
		start += 12;
		int cnt = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
		start += 8;
		for (int i = 0; i < cnt; i++) {
			char value = (char) Bits.bitsToUInt(Bits.subset(bits, start, 8));
			desc += Character.toString(value);
			start += 8;
		}
		antenna.setAntennaDescriptor(desc);
		antenna.setSetupID((int)Bits.bitsToUInt(Bits.subset(bits, start, 8)));
		start += 8;

		cnt = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
		start += 8;
		for (int i = 0; i < cnt; i++) {
			char value = (char) Bits.bitsToUInt(Bits.subset(bits, start, 8));
			serial += Character.toString(value);
			start += 8;
		}
		antenna.setAntennaSerial(serial);

		client.setAntennaDescriptor(antenna);
		//System.out.println(antenna);
		//System.out.println(start);
		return antenna;
	}


}
