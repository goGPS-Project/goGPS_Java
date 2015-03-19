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

import java.io.BufferedInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.TimeZone;

import org.gogpsproject.EphGps;
import org.gogpsproject.IonoGps;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.UnsignedOperation;


public class Decode4A {

//	private BufferedInputStream in;
	InputStream in;

	public Decode4A(InputStream _in) {
//	public Decode4A(BufferedInputStream _in) {
		in = _in;
	}

	public IonoGps decode() throws IOException,NVSException {
		// parse little Endian data

		IonoGps iono = new IonoGps();
		
		byte bytes[];
			      
//		System.out.println("+----------------  Start of 4A  ------------------+");

        /*  Alpha, 4 bytes each  */		
        bytes = new byte[4];
        float alpha[] = new float[4];
		for(int i=0;i<alpha.length;i++){
			in.read(bytes, 0, bytes.length);
			alpha[i] = Bits.byteToIEEE754Float(bytes);
//			System.out.println("Alpha" +i + ": " + alpha[i]);
		}
		
		
		/*  Beta, 4 bytes each  */		
        bytes = new byte[4];
        float beta[] = new float[4];
		for(int i=0;i<beta.length;i++){
			in.read(bytes, 0, bytes.length);
			beta[i] = Bits.byteToIEEE754Float(bytes);
//			System.out.println("Beta" +i + ": " + beta[i]);
		}
                
        
        int reliable_sign = in.read();
        
        if(reliable_sign == 255){  // 255 - the data is reliable
//          System.out.println("Reliable Sign: "+ reliable_sign); 

        	iono.setAlpha(alpha);
        	iono.setBeta(beta);	
        }
        
        
//        System.out.println("Reliable Sign: "+ reliable_sign); 
//		System.out.println("+-----------------  End of 4A  -------------------+");
						        
//        in.read(); // DLE
//        in.read(); // ETX

		return iono;
	}


}
