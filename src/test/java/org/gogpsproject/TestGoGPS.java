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
import java.text.SimpleDateFormat;
import java.util.*;

import org.gogpsproject.parser.rinex.RinexNavigation;
import org.gogpsproject.parser.rinex.RinexNavigationParser;
import org.gogpsproject.parser.rinex.RinexObservationParser;
import org.gogpsproject.parser.sp3.SP3Navigation;
import org.gogpsproject.parser.ublox.UBXFileReader;
import org.gogpsproject.parser.nvs.NVSFileReader;
import org.gogpsproject.producer.KmlProducer;
import org.gogpsproject.producer.TxtProducer;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class TestGoGPS {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));
		
		int dynamicModel = GoGPS.DYN_MODEL_CONST_SPEED;
//		int dynamicModel = GoGPS.DYN_MODEL_STATIC;

		double goodDopThreshold = 2.5;
		int timeSampleDelaySec = 1;
		
		boolean gpsEnable = true;  // enable GPS data reading
		boolean qzsEnable = false;  // enable QZSS data reading
		boolean gloEnable = false;  // enable GLONASS data reading
		boolean galEnable = false;  // enable Galileo data reading
		boolean bdsEnable = false;  // enable BeiDou data reading

		Boolean[] multiConstellation = {gpsEnable, qzsEnable, gloEnable, galEnable, bdsEnable};
				
		try{
			// Get current time
			long start = System.currentTimeMillis();

			/* for UBX8T development */
//			ObservationsProducer roverIn =  new UBXFileReader(new File("./data/UBX8T20150219s.ubx"), multiConstellation); /* ublox 8T */
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/brdm0500.15p"));
//			NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.IGN_MULTI_NAVIGATION_DAILY);
			
			/*  Jun 15th, 2013, GMSD (Multi-GNSS test) */
//			ObservationsProducer roverIn = new RinexObservationParser(new File("./data/gmsd1660.13o"), multiConstellation);
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/04921660.13o"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/brdm1660.13p"));
//			NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.IGN_MULTI_NAVIGATION_DAILY);
			
			/* Big data in Como, Italy */
//			ObservationsProducer roverIn =  new NVSFileReader(new File("./data/NVS_20140819_A_rover_000c.bin")); /* NVS */
			
			/*  Jan 27th, 2014, OCU (NVS test) */
//			ObservationsProducer roverIn =  new NVSFileReader(new File("./data/140127_SciBLDG_BINR1_rover_000.bin"), multiConstellation); /* NVS */
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/brdm0270.14p"));
			
			/*  Oct 21st, 2013, OCU (NVS test) */
//			ObservationsProducer roverIn =  new NVSFileReader(new File("./data/131021_1430_NVSANT_UBXREC_2NVSREC_KIN_BINR3_rover_00.bin"), multiConstellation); /* NVS Kinematic */
//			ObservationsProducer roverIn =  new NVSFileReader(new File("./data/131021_1300_NVSANT_UBXREC_2NVSREC_BINR3_rover_00.bin")); /* NVS Static*/
//			ObservationsProducer roverIn =  new RinexObservationParser(new File("./data/131021_1300_NVSANT_UBXREC_2NVSREC_BINR2_rover.13o")); /* NVS */
//			ObservationsProducer roverIn =  new RinexObservationParser(new File("./data/131021_1430_NVSANT_UBXREC_2NVSREC_KIN_BINR3_rover.13o_v3")); /* NVS RINEX3*/
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/SciBLDG_VRS3_master.obs"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/Javad_SciBLDG3.13N"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/131021_1430_NVSANT_UBXREC_2NVSREC_KIN_BINR3_rover.13p"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/131021_1300_NVSANT_UBXREC_2NVSREC_BINR2_rover.13n"));
//			NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.GARNER_NAVIGATION_AUTO);

			/*  Oct 21st, 2013, OCU (Javad test) */
//			ObservationsProducer roverIn =  new RinexObservationParser(new File("./data/Javad_SciBLDG3.13o")); /* Javad */
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/SciBLDG_VRS2_master.obs"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/Javad_SciBLDG2.13N"));		
			
			/*  Oct 21st, 2013, OCU (u-blox test) */
//			dynamicModel = GoGPS.DYN_MODEL_STATIC;
//			ObservationsProducer roverIn =  new UBXFileReader(new File("./data/131021_1300_NVSANT_UBXREC_2NVSREC_UBX1_rover_00.bin")); /* ublox */
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/SciBLDG_VRS2_master.obs"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/Javad_SciBLDG2.13N"));
//			NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.GARNER_NAVIGATION_AUTO);
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/131021_1430_NVSANT_UBXREC_2NVSREC_KIN_BINR3_rover.13n"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/131021_1300_NVSANT_UBXREC_2NVSREC_BINR2_rover.13n"));
			
			/* Osaka, Japan (u-blox test) */
			ObservationsProducer roverIn = new RinexObservationParser(new File("./data/yamatogawa_rover.obs"));
			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/yamatogawa_master.obs"));
			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/yamatogawa_rover.nav"));
//			NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.GARNER_NAVIGATION_AUTO);

			/* Osaka, Japan (TOPCON test) */
//			ObservationsProducer roverIn = new RinexObservationParser(new File("./data/log1220b_TOPCON_f.10o")); /* TOPCON front */
//			//ObservationsProducer roverIn = new RinexObservationParser(new File("./data/log1220b_TOPCON_r.10o")); /* TOPCON back */
//			//ObservationsProducer roverIn = new UBXFileReader(new File("./data/log1220b_ublox.ubx")); /* u-blox */
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/Vb05.10o"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/Vb05.10n"));
			//NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.GARNER_NAVIGATION_AUTO);

			/* Como, Italy (static) */
//			dynamicModel = GoGPS.DYN_MODEL_STATIC;
//			ObservationsProducer roverIn = new RinexObservationParser(new File("./data/como_pillar_rover.obs"));
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/como_pillar_master.10o"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/como_pillar_rover.nav"));

			/* Sardinia, Italy */
//			ObservationsProducer roverIn = new RinexObservationParser(new File("./data/goCerchio_rover.obs"));
//			ObservationsProducer masterIn = new RinexObservationParser(new File("./data/sard0880.10o"));
//			NavigationProducer navigationIn = new RinexNavigationParser(new File("./data/sard0880.10n"));
//			//NavigationProducer navigationIn = new RinexNavigation(RinexNavigation.GARNER_NAVIGATION_ZIM2);

			// 1st init
			navigationIn.init();
			roverIn.init();
 			masterIn.init();

			// Name output files name using Timestamp
			Date date = new Date();
			SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
			String date1 = sdf1.format(date);
			String outPathTxt = "./test/" + date1 + ".txt";
			String outPathKml = "./test/" + date1 + ".kml";
			TxtProducer txt = new TxtProducer(outPathTxt);
			KmlProducer kml = new KmlProducer(outPathKml, goodDopThreshold, timeSampleDelaySec);

			GoGPS goGPS = new GoGPS(navigationIn, roverIn, masterIn);
//			GoGPS goGPS = new GoGPS(navigationIn, roverIn);
			goGPS.addPositionConsumerListener(txt);
			goGPS.addPositionConsumerListener(kml);
			goGPS.setDynamicModel(dynamicModel);
//			goGPS.runCodeStandalone();
//			goGPS.runCodeDoubleDifferences();
//			goGPS.runKalmanFilterCodePhaseStandalone();
			goGPS.runKalmanFilterCodePhaseDoubleDifferences();

			try{
				roverIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}
//			try{
//				masterIn.release(true,10000);
//			}catch(InterruptedException ie){
//				ie.printStackTrace();
//			}
			try{
				navigationIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}

			/* To wait for other Thread to be finished */
			System.out.println("waiting for finishing all the processes");
			while (Thread.activeCount() > 1){
			}			
			System.out.println("Finished!");

			
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


}
