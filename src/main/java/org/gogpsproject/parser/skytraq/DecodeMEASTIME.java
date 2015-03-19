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
import java.util.Calendar;
import java.util.TimeZone;

import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;


public class DecodeMEASTIME {
	private InputStream in;

	public DecodeMEASTIME(InputStream in) {
		this.in = in;
	}

	public Observations decode(int len) throws IOException, STQException {

		byte bytes[];
		
		/* IOD, 1 byte */				
		bytes = new byte[1];
		in.read(bytes, 0, bytes.length);
		int IOD = Bits.byteToIntBigEndian(bytes);
		
		/* GPS week, 2 bytes */
		bytes = new byte[2];
		in.read(bytes, 0, bytes.length);
		int week = Bits.byteToIntBigEndian(bytes);
		
		/* GPS time-of-week, 4 bytes */
		bytes = new byte[4];
		in.read(bytes, 0, bytes.length);
		long tow = Bits.byteToLongBigEndian(bytes);
		
		/* Measurement period, 2 bytes */
		bytes = new byte[2];
		in.read(bytes, 0, bytes.length);

		long gmtTS = getGMTTS(tow, week);
		Observations o = new Observations(new Time(gmtTS),0);
		o.setIssueOfData(IOD);

		return o;
	}

	private long getGMTTS(long tow, long week) {
		Calendar c = Calendar.getInstance();
		c.setTimeZone(TimeZone.getTimeZone("GMT Time"));
		c.set(Calendar.YEAR, 1980);
		c.set(Calendar.MONTH, Calendar.JANUARY);
		c.set(Calendar.DAY_OF_MONTH, 6);
		c.set(Calendar.HOUR_OF_DAY, 0);
		c.set(Calendar.MINUTE, 0);
		c.set(Calendar.SECOND, 0);
		c.set(Calendar.MILLISECOND, 0);

		return c.getTimeInMillis() + week*7*24*3600*1000 + tow;
	}
}
