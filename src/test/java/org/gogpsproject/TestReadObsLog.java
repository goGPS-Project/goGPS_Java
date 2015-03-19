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

package org.gogpsproject;
import gnu.io.CommPort;
import gnu.io.CommPortIdentifier;
import gnu.io.PortInUseException;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;


import org.gogpsproject.ObservationsBuffer;
import org.gogpsproject.parser.rinex.RinexNavigation;
import org.gogpsproject.producer.KmlProducer;

@SuppressWarnings("restriction")
public class TestReadObsLog {

	/**
	 * @param args
	 */

	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));

		int dynamicModel = GoGPS.DYN_MODEL_CONST_SPEED;
		double goodDopThreshold = 2.5;
		int timeSapleDelaySec = 1;


		try {
			long start = System.currentTimeMillis();

			String master = "P:\\Multimedia\\Dropbox\\GoGPS project\\misurazioni Cryms\\Misurazioni CAB1\\20111006-010011-master.dat";
			String rover = "P:\\Multimedia\\Dropbox\\GoGPS project\\misurazioni Cryms\\Misurazioni CAB1\\20111006-010011-rover.dat";

			System.out.println("MASTER");
			ObservationsBuffer masterIn = new ObservationsBuffer();
			masterIn.readFromLog(master);
//			ObservationsProducer masterIn = new RinexObservationParser(new File("P:\\Multimedia\\Dropbox\\GoGPS project\\misurazioni Cryms\\Misurazioni CAB1\\VirA275W.11o"));

			System.out.println();
			System.out.println("ROVER");
			ObservationsBuffer roverIn = new ObservationsBuffer();
			roverIn.readFromLog(rover);

			System.out.println("NAV");
			NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.GARNER_NAVIGATION_AUTO);
			//NavigationProducer navigationIn = new RinexNavigationParser(new File("data\\VirA275W.11n"));

			roverIn.streamClosed();
			//masterIn.streamClosed();

			navigationIn.init();
			roverIn.init();
			masterIn.init();

			Date date = new Date();
			SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
			String date1 = sdf1.format(date);
			String outPath = "P:\\Multimedia\\Dropbox\\GoGPS project\\misurazioni Cryms\\Misurazioni CAB1\\20111006-010011.kml";
			KmlProducer kml = new KmlProducer(outPath, goodDopThreshold, timeSapleDelaySec);

			GoGPS goGPS = new GoGPS(navigationIn, roverIn, masterIn);
			goGPS.addPositionConsumerListener(kml);
			goGPS.setDynamicModel(dynamicModel);
			//goGPS.runCodeStandalone();
			//goGPS.runCodeDoubleDifferences();
			goGPS.runKalmanFilter();

			try{
				roverIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}
			try{
				masterIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}
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

		} catch (Exception e1) {
			e1.printStackTrace();
		}

//		System.out.println(computeNMEACheckSum("$GPGGA,200530,4600,N,00857,E,4,10,1,200,M,1,M,3,0"));
//
//		File f = new File("./data/rtcm.out");
//		try {
//			FileInputStream fis = new FileInputStream(f);
//			RTCM3Client cl = new RTCM3Client(null);
//			cl.go = true;
//			cl.debug = true;
//			cl.readLoop(fis);
//		} catch (FileNotFoundException e) {
//			e.printStackTrace();
//		} catch (IOException e) {
//			e.printStackTrace();
//		}


	}

}
