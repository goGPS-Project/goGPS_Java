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

package org.gogpsproject.producer.parser.ublox;

import java.util.Vector;

import org.gogpsproject.util.UnsignedOperation;


public class UBXMsgConfiguration {

	public final int uBloxPrefix1 = 0xB5;
	public final int uBloxPrefix2 = 0x62;
	private int classid;
	private int msgval;
	private UBXMessageType msgid;

	//private int rate;
	private int CK_A;
	private int CK_B;
	private Vector<Integer> msg;

	public UBXMsgConfiguration(int classtype, int msgtype, boolean enable) {
		msgid = new UBXMessageType(classtype, msgtype);
		classid = msgid.getClassOut();
		msgval = msgid.getIdOut();
		// System.out.println("ID 1 >>:" + ID1 + "ID 2 >>:" + ID2);
		msg = new Vector();
		msg.addElement(new Integer(uBloxPrefix1));
		msg.addElement(new Integer(uBloxPrefix2));
		msg.addElement(new Integer(0x06)); // CFG
		msg.addElement(new Integer(0x01)); // MSG
		msg.addElement(new Integer(3)); // length low
		msg.addElement(new Integer(0)); // length hi
		msg.addElement(new Integer(classid));
		msg.addElement(new Integer(msgval));
		msg.addElement(new Integer(enable?0x01:0x00));
		checkSum();
		msg.addElement(new Integer(CK_A));
		msg.addElement(new Integer(CK_B));
	}
	public UBXMsgConfiguration(int classtype, int msgtype, int smsg[]) {
		msgid = new UBXMessageType(classtype, msgtype);
		classid = msgid.getClassOut();
		msgval = msgid.getIdOut();
		// System.out.println("ID 1 >>:" + ID1 + "ID 2 >>:" + ID2);

		msg = new Vector();
		msg.addElement(new Integer(uBloxPrefix1));
		msg.addElement(new Integer(uBloxPrefix2));
		msg.addElement(new Integer(classid));
		msg.addElement(new Integer(msgval));
		int length1 = (smsg.length)/0xff;
		int length2 = (smsg.length)&0xff;
		msg.addElement(new Integer(length2));
		msg.addElement(new Integer(length1));
		for(int i=0;i<smsg.length;i++){
			msg.addElement(new Integer(smsg[i]));
		}
		checkSum();
		msg.addElement(new Integer(CK_A));
		msg.addElement(new Integer(CK_B));

	}

	private void checkSum() {
		CK_A = 0;
		CK_B = 0;
		for (int i = 2; i < msg.size(); i++) {
			CK_A = CK_A + ((Integer) msg.elementAt(i)).intValue();
			CK_B = CK_B + CK_A;

		}
		CK_A = CK_A & 0xFF;
		CK_B = CK_B & 0xFF;
	}

	public byte[] getByte() {
		byte[] bytes = new byte[msg.size()];
		for (int i = 0; i < msg.size(); i++) {
			bytes[i] = UnsignedOperation.unsignedIntToByte(((Integer)msg.elementAt(i)).intValue());
		}
		return bytes;
	}

}
