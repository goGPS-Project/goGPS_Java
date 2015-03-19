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

import java.text.SimpleDateFormat;
import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.TimeZone;

import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.Time;
import org.gogpsproject.Constants;
import org.gogpsproject.util.Bits;

public class Decode1004Msg implements Decode {

	private SimpleDateFormat sdf = new SimpleDateFormat("yyyy MM dd HH mm ss.SSS");

	private RTCM3Client client;
	public Decode1004Msg(RTCM3Client client) {
		this.client = client;
	}

	/* (non-Javadoc)
	 * @see com.crysm.gogps.parser.tes#decode()
	 */
	public Observations decode(boolean[] bits, int week) {

		int start = 12;
		//header.setStationID(Bits.bitsToUInt(Bits.subset(bits, start, 12)));
		int DF003 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 12));
		start += 12;
		//header.setEpochTime(Bits.bitsToUInt(Bits.subset(bits, start, 30)));
		double DF004 = (double)Bits.bitsToUInt(Bits.subset(bits, start, 30));
		start += 30;
		//header.setGNSSFlag(Bits.bitsToUInt(Bits.subset(bits, start, 1)));
		boolean DF005 = (Bits.bitsToUInt(Bits.subset(bits, start, 1))==1);
		start += 1;
		//header.setNumberGPS(Bits.bitsToUInt(Bits.subset(bits, start, 5)));
		int DF006 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 5));
		start += 5;
		//header.setDivergenceSmooth(Bits.bitsToUInt(Bits.subset(bits, start, 11)));
		boolean DF007 = (Bits.bitsToUInt(Bits.subset(bits, start, 1))==1);
		start += 1;
		//header.setSmouthInterval(Bits.bitsToUInt(Bits.subset(bits, start, 3)));
		int DF008 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 3));
		start += 3;
		//System.out.println(header);

		Observations o = new Observations(new Time(week,DF004/1000),0);

		for (int i = 0; i < DF006 /*header.getNumberGPS()*/; i++) {
			int DF009 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 6));
			start += 6;
			boolean DF010 = (Bits.bitsToUInt(Bits.subset(bits, start, 1))==1);
			start += 1;
			long DF011 = Bits.bitsToUInt(Bits.subset(bits,start, 24));
			start += 24;
			long DF012 = Bits.bitsTwoComplement(Bits.subset(bits, start, 20));
			start += 20;
			int DF013 = (int)Bits.bitsToUInt(Bits.subset(bits, start,7));
			start += 7;
			int DF014 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
			start += 8;
			int DF015 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
			start += 8;
			int DF016 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 2));
			start += 2;
			int DF017 = (int)Bits.bitsTwoComplement(Bits.subset(bits,start, 14));
			start += 14;
			long DF018 = Bits.bitsTwoComplement(Bits.subset(bits,start, 20));
			start += 20;
			int DF019 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 7));
			start += 7;
			int DF020 = (int)Bits.bitsToUInt(Bits.subset(bits, start, 8));
			start += 8;

			ObservationSet os = new ObservationSet();
			os.setSatID(DF009);
			os.setSatType('G');

			double DF011d=DF011*0.02+DF014*299792.458;
	        if (DF012 != 0x80000) {
	        	if(DF010){
	        		os.setCodeP(ObservationSet.L1, DF011d);
	        	}else{
	        		os.setCodeC(ObservationSet.L1, DF011d);
	        	}
	        	double cp1 = DF012*0.0005/(Constants.SPEED_OF_LIGHT/Constants.FL1);
//	            cp1=adjcp(rtcm,sat,0,DF012*0.0005/lam[0]);
//	            static double adjcp(rtcm_t *rtcm, int sat, int freq, double cp)
//	        	{
//	        	    if (rtcm->cp[sat-1][freq]==0.0) ;
//	        	    else if (cp<rtcm->cp[sat-1][freq]-750.0) cp+=1500.0;
//	        	    else if (cp>rtcm->cp[sat-1][freq]+750.0) cp-=1500.0;
//	        	    rtcm->cp[sat-1][freq]=cp;
//	        	    return cp;
//	        	}
//	        	rtcm->obs.data[index].L[0]=DF011/lam[0]+cp1;

	        	os.setPhaseCycles(ObservationSet.L1, DF011d/(Constants.SPEED_OF_LIGHT/Constants.FL1)+cp1);
	        }

			//os.setCoarseAcquisition((DF011 * 0.02) + (DF014 * 299792.458));
			//os.setPhase(ObservationSet.L1,(os.getCoarseAcquisition() + (DF012*0.0005)) / Constants.LAMBDA_1);
	        double snr = (DF015 * 0.25);
	        snr = (snr<=0.0||255.5<=snr)?0.0:snr+0.5;
			os.setSignalStrength(ObservationSet.L1, (float)snr);

			if (DF017!=0x2000) {
	            os.setCodeP(ObservationSet.L2, DF011d+DF017*0.02);
	        }
	        if (DF018!=0x80000) {
	            double cp2=DF018*0.0005/(Constants.SPEED_OF_LIGHT/Constants.FL2);

	            os.setPhaseCycles(ObservationSet.L2,DF011d/(Constants.SPEED_OF_LIGHT/Constants.FL2)+cp2);

	        }

			//os.setPseudorangeCode(ObservationSet.L2, ((DF011 + DF017) * 0.02) + (DF014 * 299792.458));
			//os.setPhase(ObservationSet.L2,(os.getCoarseAcquisition() + (DF018*0.0005)) / Constants.LAMBDA_2);

	        snr = (DF020 * 0.25);
	        snr = (snr<=0.0||255.5<=snr)?0.0:snr+0.5;
			os.setSignalStrength(ObservationSet.L2, (float)snr);
			o.setGps(i, os);
		}

		//client.addObservation(o);
		return o;
	}


	/**
	 * @return GPS Epoch Time of the beginning of the right GPS week,
	 * which begins at midnight GMT on Saturday
	 * night/Sunday morning, measured in milliseconds.
	 * referenceTS is the reference timestamp to select rigth week near leap saturday/sunday
	 * (ie. if referenceTS report Saturday but TOW is little it must select next Sunday, not previous)
	 */
	private long getWeekTS(long tow, long referenceTS) {
//		Calendar mbCal = new GregorianCalendar(TimeZone.getTimeZone("GMT"));
//		mbCal.setTimeInMillis(referenceTS);

		Calendar cal = Calendar.getInstance();
		cal.setTimeZone(TimeZone.getTimeZone("GMT"));
		cal.setTimeInMillis(referenceTS);
//		cal.set(Calendar.YEAR, mbCal.get(Calendar.YEAR));
//		cal.set(Calendar.MONTH, mbCal.get(Calendar.MONTH));
//		cal.set(Calendar.DAY_OF_MONTH, mbCal.get(Calendar.DAY_OF_MONTH));
		cal.set(Calendar.HOUR_OF_DAY, 0);
		cal.set(Calendar.MINUTE, 0);
		cal.set(Calendar.SECOND, 0);
		cal.set(Calendar.MILLISECOND, 0);

		// search for right Sunday comparing TOW value and reference date to target right week
		if(cal.get(Calendar.DAY_OF_WEEK) >= Calendar.FRIDAY){
			// time ref is friday or saturday, tow should be great

			if( tow < 2*24*3600*1000 ){
				// tow is < than Tuesday so real week passed Sunday, go forward
				while(cal.get(Calendar.DAY_OF_WEEK)!=Calendar.SUNDAY) cal.add(Calendar.DATE, 1);
			}else{
				// tow is > than Tuesday as expected, go backward
				while(cal.get(Calendar.DAY_OF_WEEK)!=Calendar.SUNDAY) cal.add(Calendar.DATE, -1);
			}
		}else{
			// time ref is Sunday to Thusday
			if( tow > 5*24*3600*1000 ){
				// but if tow is still in past week, bring back one week
				cal.add(Calendar.DATE, -7);
			}
			//  ensure Sunday
			while(cal.get(Calendar.DAY_OF_WEEK)!=Calendar.SUNDAY) cal.add(Calendar.DATE, -1);

		}



		while(cal.get(Calendar.DAY_OF_WEEK)!=Calendar.SUNDAY)cal.add(Calendar.DATE, -1);

		return cal.getTimeInMillis();
	}
}
