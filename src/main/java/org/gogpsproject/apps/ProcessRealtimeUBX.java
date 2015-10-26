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
package org.gogpsproject.apps;
import java.io.*;
import java.text.SimpleDateFormat;
import java.util.*;

import org.gogpsproject.Coordinates;
import org.gogpsproject.GoGPS;
import org.gogpsproject.NavigationProducer;
import org.gogpsproject.ObservationsBuffer;
import org.gogpsproject.parser.rtcm3.RTCM3Client;
import org.gogpsproject.parser.ublox.UBXSerialConnection;
import org.gogpsproject.parser.ublox.UBXAssistNow;
import org.gogpsproject.producer.KmlProducer;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class ProcessRealtimeUBX {


	/**
	 * @param args
	 */
	public static void main(String[] args) {
		int dynamicModel = GoGPS.DYN_MODEL_CONST_SPEED;
		try{
			//force dot as decimal separator
			Locale.setDefault(new Locale("en", "US"));
			
			// Get current time
			long start = System.currentTimeMillis();

			// Realtime
			if(args.length<1){
				System.out.println("ProcessRealtimeUBX <com_port>");
				return;
			}
			
			String comPort = args[0];

			/******************************************
			 * ROVER & NAVIGATION u-blox
			 */
			UBXSerialConnection ubxSerialConn = new UBXSerialConnection(comPort, 115200);
			ubxSerialConn.init();
			ubxSerialConn.enableDebug(false);
			
			Date date = new Date();
			SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
			String date1 = sdf1.format(date);

			ObservationsBuffer roverIn = new ObservationsBuffer(ubxSerialConn,"./out/" + date1 + ".dat");
			NavigationProducer navigationIn = roverIn;
			roverIn.init();

			// wait for some data to buffer
			Thread.sleep(2000);

			GoGPS goGPS = new GoGPS(navigationIn, roverIn, null);
			goGPS.setDynamicModel(dynamicModel);

			// set Output
			String outPath = "./out/" + date1 + ".kml";
			KmlProducer kml = new KmlProducer(outPath, 2.5, 0);
			goGPS.addPositionConsumerListener(kml);

			// run blocking (never exit in live-tracking)
			goGPS.runCodeStandalone();

			// run in background
			goGPS.runThreadMode(GoGPS.RUN_MODE_STANDALONE);

			// wait for 1 minute
			Thread.sleep(120*1000);

			System.out.println();
			System.out.println();

			System.out.println("OK give up ---------------------------------------------");

			/******************************************
			 * END
			 */
			try{
				System.out.println("Stop Rover");
				roverIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}
			try{
				System.out.println("Stop Navigation");
				navigationIn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}
			try{
				System.out.println("Stop UBX");
				ubxSerialConn.release(true,10000);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}

		}catch(Exception e){
			e.printStackTrace();
		}
	}
}
