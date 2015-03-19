/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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
 *
 */
package org.gogpsproject;
import java.io.*;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.*;
import java.text.*;

import org.gogpsproject.parser.rinex.RinexNavigation;
import org.gogpsproject.parser.rinex.RinexNavigationParser;
import org.gogpsproject.parser.rinex.RinexObservationParser;
import org.gogpsproject.parser.rtcm3.RTCM3Client;
import org.gogpsproject.parser.sp3.SP3Navigation;
import org.gogpsproject.parser.ublox.UBXSerialConnection;
import org.gogpsproject.parser.ublox.UBXFileReader;
import org.gogpsproject.producer.KmlProducer;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class TestAssistNow {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		try{
			
			//force dot as decimal separator
			Locale.setDefault(new Locale("en", "US"));
			
			// Get current time
			long start = System.currentTimeMillis();

			RinexNavigation navigationIn = new RinexNavigation(RinexNavigation.IGN_NAVIGATION_HOURLY_ZIM2);


			FileInputStream fis = new FileInputStream(".\\data\\aphemeris.dat");
			//FileInputStream fis = new FileInputStream(".\\data\\assistnow.dat");
			DataInputStream dis = new DataInputStream(fis);
			String msg = null;
			try{
				msg=dis.readUTF();
				while(msg!=null){
					System.out.println("Msg:["+msg+"]");
					if(msg.equalsIgnoreCase(Streamable.MESSAGE_OBSERVATIONS)){
						Observations o = new Observations(dis,false);

					}else
					if(msg.equalsIgnoreCase(Streamable.MESSAGE_EPHEMERIS)){
						EphGps eph1 = new EphGps(dis,false);
						System.out.println("found sat"+eph1.getSatID()+" time:"+eph1.getRefTime().getGpsTime());

						EphGps eph2 =navigationIn.findEph(eph1.getRefTime().getMsec(), eph1.getSatID(), eph1.getSatType());

						//Compare
						if(eph2!=null){
							double ms = eph1.getRefTime().getGpsTime()-eph2.getRefTime().getGpsTime();
							System.out.println(" time dif:"+(ms/1000)+"s "+(ms%1000)+"ms");
							;
							equalDouble("Af0",eph1.getAf0(),eph2.getAf0());
							equalDouble("Af1",eph1.getAf1(),eph2.getAf1());
							equalDouble("Af2",eph1.getAf2(),eph2.getAf2());
							equalDouble("Cic",eph1.getCic(),eph2.getCic());
							equalDouble("Cis",eph1.getCis(),eph2.getCis());
							equalDouble("Crc",eph1.getCrc(),eph2.getCrc());
							equalDouble("Crs",eph1.getCrs(),eph2.getCrs());
							equalDouble("Cuc",eph1.getCuc(),eph2.getCuc());
							equalDouble("Cus",eph1.getCus(),eph2.getCus());
							equalDouble("DeltaN",eph1.getDeltaN(),eph2.getDeltaN());
							equalDouble("E",eph1.getE(),eph2.getE());
							equalDouble("FitInt",eph1.getFitInt(),eph2.getFitInt());
							equalDouble("I0",eph1.getI0(),eph2.getI0());
							equalDouble("iDot",eph1.getiDot(),eph2.getiDot());
							equalDouble("M0",eph1.getM0(),eph2.getM0());
							equalDouble("Omega",eph1.getOmega(),eph2.getOmega());
							equalDouble("Omega0",eph1.getOmega0(),eph2.getOmega0());
							equalDouble("OmegaDot",eph1.getOmegaDot(),eph2.getOmegaDot());
							equalDouble("RootA",eph1.getRootA(),eph2.getRootA());
							equalDouble("Tgd",eph1.getTgd(),eph2.getTgd());
							equalDouble("Toc",eph1.getToc(),eph2.getToc());
							equalDouble("Toe",eph1.getToe(),eph2.getToe());
							equalDouble("Iodc",eph1.getIodc(),eph2.getIodc());
							equalDouble("Iode",eph1.getIode(),eph2.getIode());
							equalDouble("L2Code",eph1.getL2Code(),eph2.getL2Code());
							equalDouble("L2Flag",eph1.getL2Flag(),eph2.getL2Flag());
							equalDouble("SvAccur",eph1.getSvAccur(),eph2.getSvAccur());
							equalDouble("SvHealth",eph1.getSvHealth(),eph2.getSvHealth());



						}else{
							System.out.println("EPH 2 not found for "+eph1.getSatID()+" at "+eph1.getRefTime());
						}

					}else
					if(msg.equalsIgnoreCase(Streamable.MESSAGE_OBSERVATIONS_SET)){
						ObservationSet eps = new ObservationSet(dis,false);
						// nothing to do with ?
					}else
					if(msg.equalsIgnoreCase(Streamable.MESSAGE_IONO)){
						IonoGps iono = new IonoGps(dis,false);
						//addIonospheric(iono);
					}else
					if(msg.equalsIgnoreCase(Streamable.MESSAGE_COORDINATES)){
						Coordinates c = Coordinates.readFromStream(dis,false);
						//setDefinedPosition(c);



					}else{
						System.out.println("Unknow Msg:["+msg+"]");
					}

					msg=dis.readUTF();
				}
			}catch(EOFException eof){
				// ok
			}
			fis.close();


			try{
				navigationIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}

			// Get and display elapsed time
			int elapsedTimeSec = (int) Math.floor((System.currentTimeMillis() - start) / 1000);
			int elapsedTimeMillisec = (int) ((System.currentTimeMillis() - start) - elapsedTimeSec * 1000);
			System.out.println("\nElapsed time (read + proc + display + write): "
					+ elapsedTimeSec + " seconds " + elapsedTimeMillisec
					+ " milliseconds.");
		}catch(Exception e){
			e.printStackTrace();
		}
	}
	private static boolean equalDouble(String param,double d1,double d2){
		if(Double.isNaN(d1) && Double.isNaN(d2)) return true;
		if(Math.abs(d1-d2)>0.001){
			System.out.println("  "+param+" "+d1+" != "+d2+" diff "+(d1-d2));
			return false;
		}
		return true;
	}

}
