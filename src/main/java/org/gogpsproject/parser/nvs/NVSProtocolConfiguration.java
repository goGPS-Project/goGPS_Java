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

import java.nio.ByteBuffer;
import java.util.Vector;

import org.gogpsproject.util.UnsignedOperation;


public class NVSProtocolConfiguration {

	public final int nvsPrefix = 0x10;
	public final  int nvsSuffix = 0x03;

	private Vector<Integer> msg;

	public NVSProtocolConfiguration() {
		msg = new Vector();
		msg.addElement(new Integer(nvsPrefix));
		msg.addElement(new Integer(0x0B)); // 0Bh
		msg.addElement(new Integer(0x00)); // 00 --> "current port"
		//byte[] baudRateBytes = ByteBuffer.allocate(4).putInt(115200).array();
		msg.addElement(new Integer(0x00));
		msg.addElement(new Integer(0xC2));
		msg.addElement(new Integer(0x01));
		msg.addElement(new Integer(0x00));
		msg.addElement(new Integer(0x04)); // protocol --> BINR
		msg.addElement(new Integer(nvsPrefix));
		msg.addElement(new Integer(nvsSuffix));
	}

	public byte[] getByte() {
		byte[] bytes = new byte[msg.size()];
		for (int i = 0; i < msg.size(); i++) {
			bytes[i] = UnsignedOperation.unsignedIntToByte(((Integer)msg.elementAt(i)).intValue());
		}
		return bytes;
	}

}
