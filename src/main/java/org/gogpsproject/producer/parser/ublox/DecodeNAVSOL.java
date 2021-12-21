/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl, Daisuke Yoshida, Emanuele Ziglioli. All Rights Reserved.
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

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
//import java.text.SimpleDateFormat;
import java.util.Calendar;
//import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.Constants;
import org.gogpsproject.positioning.ReceiverPosition;
import org.gogpsproject.positioning.RoverPosition;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.ObservationSet;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;
import static org.gogpsproject.util.UnsignedOperation.*;

public class DecodeNAVSOL {
	private InputStream in;

//	private int[] fdata;
//	private int[] fbits;
//	private boolean end = true;
	
	private Boolean[] multiConstellation;

	public DecodeNAVSOL(InputStream in) {
		this.in = in;
	}

	public DecodeNAVSOL(InputStream in, Boolean[] multiConstellation) throws IOException {
		this.in = in;		
		this.multiConstellation = multiConstellation;
	}
	
	/**
	 /* decode ubx-nav-sol: navigation solution 
	 * See https://github.com/tomojitakasu/RTKLIB/blob/master/src/rcv/ublox.c
	 * and https://github.com/rtklibexplorer/RTKLIB/blob/demo5/src/rcv/ublox.c
	 * static int decode_navsol(raw_t *raw)* 
	 * 
	 * @param logos
	 * @return
	 * @throws IOException
	 * @throws UBXException
	 */
	public ReceiverPosition decode(OutputStream logos) throws IOException, UBXException {
		// parse little Endian data
		int[] length = new int[2];

		length[1] = in.read();
		length[0] = in.read();
		int len = length[0]*256+length[1];
		
		if (len == 0) {
			throw new UBXException("Zero-length NAV-SOL message");
		}

//		System.out.println("NAV-SOL message Length : " + len);

    int itow = U4(in); 
    int ftow = I4(in);
    int week = U2(in);
    
    int gpsFix = in.read();
    int flags  = in.read();
    
    in.skip(40);
    
    if(( flags & 0x0C) == 0x0C ) {
        Time t = new Time( week, itow*1E-3 + ftow*1E-9 );
//    		System.out.println(t);

        RoverPosition p = new RoverPosition();
        p.setRefTime(t);
        return p;
    }
		
		int c1 = in.read();
		if(logos!=null) logos.write(c1);

		int c2 = in.read();
		if(logos!=null) logos.write(c2);

		return null; 
	}

}
