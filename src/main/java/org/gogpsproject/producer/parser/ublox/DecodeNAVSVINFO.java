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
import java.util.ArrayList;
import java.util.List;
import org.gogpsproject.positioning.SVInfo;
import org.gogpsproject.positioning.Time;
import static org.gogpsproject.util.UnsignedOperation.*;

public class DecodeNAVSVINFO {
	private InputStream in;

//	private int[] fdata;
//	private int[] fbits;
//	private boolean end = true;
	
	private Boolean[] multiConstellation;

	public DecodeNAVSVINFO(InputStream in) {
		this.in = in;
	}

	public DecodeNAVSVINFO(InputStream in, Boolean[] multiConstellation) throws IOException {
		this.in = in;		
		this.multiConstellation = multiConstellation;
	}
	
	/**
	 /* decode ubx-nav-info: 0x01 0x30
s  * 
	 * @param logos
	 * @param time 
	 * @return
	 * @throws IOException
	 * @throws UBXException
	 */
	public List<SVInfo> decode(OutputStream logos, Time time) throws IOException, UBXException {
		int[] length = new int[2];
		length[1] = in.read();
		length[0] = in.read();
		int len = length[0]*256+length[1];
		
		if (len == 0) {
			throw new UBXException("Zero-length NAV-SVINFO message");
		}
		System.out.println("NAV-SVINFO message Length : " + len);

		int week = time.getGpsWeek();  
/*0*/int itow  = U4(in);
		time = new Time( week, itow/1000 );

/*4*/int numCh = U1(in);
		System.out.println("numCh: " + numCh);
		if( len< 8 + numCh*12 ) {
			throw new UBXException("Length error NAV-SVINFO message");
    }

/*5*/int globalFlags = U1(in);
/*6*/in.skip(1);
    
		List<SVInfo> sl = new ArrayList<>();
		for (int k = 0; k < numCh; k++) { // p=raw->buff+110
/*8*/	int chn = in.read(); 
			System.out.println("\nchn:  " + chn );

			if( chn != k ) {
				// Some problem with parsing has occurred
				in.skip(11);
				continue;
			}
			
			int svid = in.read();
			System.out.println("svid:  " + svid );
			
			/*
			 * svUsed
			 * diffCorr
			 * orbitAvail
			 * orpithEph
			 * unhealthy
			 * orbitAlm
			 * orbitAop
			 * smoothed
			 */
			int flags = U1(in);

			/*
			 * 0: no signal
			 * 1: signal acquired
			 * 3: signal detected but unuseable
			 * 4. code locked and time synchronised
			 * 5, 6, 7: code and carrier locked and time synchronised
			 */
			int quality = U1(in);
			
			int cno = U1(in);
			System.out.println("cNo:  " + cno );
			
			/* elev in integer degrees */
			int elev = I1(in);
			System.out.println("elev:  " + elev );
			
			/* azimuth in integer degrees */
			int azim = I2(in);
			System.out.println("azim:  " + azim );

			/* pseudorange residual in degrees*/
			int prRes = I4(in);
			System.out.println("prRes:  " + prRes );
			
//      char satType = obs.getGnssType(i);			
			SVInfo sp = new SVInfo(time.getMsec(), svid, 'G', azim, elev, cno, prRes, flags, quality);
			sl.add(sp);
		}
		return sl; 
	}

}
