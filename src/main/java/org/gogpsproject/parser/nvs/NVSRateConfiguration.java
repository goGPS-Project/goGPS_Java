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

import java.util.Vector;
import org.gogpsproject.util.UnsignedOperation;


public class NVSRateConfiguration {

	public final int nvsPrefix = 0x10;
	public final  int nvsSuffix = 0x03;

	private Vector<Integer> msg;

	public NVSRateConfiguration(int rate) {
		msg = new Vector();
		msg.addElement(new Integer(nvsPrefix));
		msg.addElement(new Integer(0xD7)); // D7h
		msg.addElement(new Integer(0x02)); // 02 --> navigation rate
		msg.addElement(new Integer(rate));
		msg.addElement(new Integer(nvsPrefix));
		msg.addElement(new Integer(nvsSuffix));
		
		msg.addElement(new Integer(nvsPrefix));
		msg.addElement(new Integer(0xF4)); // F4h
		rate = 1; //to be further checked...
		msg.addElement(new Integer((int) ((1/(double) rate)/0.1)));
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
