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

import org.gogpsproject.GoGPS;
import org.gogpsproject.GoGPS.DynamicModel;
import org.gogpsproject.consumer.JakKmlProducer;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.ObservationsBuffer;
import org.gogpsproject.producer.parser.rtcm3.RTCM3Client;
import org.gogpsproject.producer.parser.ublox.UBXAssistNow;
import org.gogpsproject.producer.parser.ublox.UBXSerialConnection;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class ProcessRealtimeUBX {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		DynamicModel dynamicModel = GoGPS.DynamicModel.CONST_SPEED;
		try {
			// force dot as decimal separator
			Locale.setDefault(new Locale("en", "US"));

			// Get current time
			long start = System.currentTimeMillis();

			// Realtime
			if (args.length < 1) {
				System.out.println("ProcessRealtimeUBX <com_port>");
				return;
			}

			String comPort = args[0];

			/******************************************
			 * ROVER & NAVIGATION u-blox
			 */
			UBXSerialConnection ubxSerialConn = new UBXSerialConnection(comPort, 115200);
			ubxSerialConn.enableDebug(false);
			ubxSerialConn.enableEphemeris(1);
			ubxSerialConn.init();

			Date date = new Date();
			SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
			String date1 = sdf1.format(date);

			ObservationsBuffer roverIn = new ObservationsBuffer(ubxSerialConn, "./out/" + date1 + ".dat");
			NavigationProducer navigationIn = roverIn;
			roverIn.setDebug(false);
//			roverIn.setTimeoutNextObsWait(60*1000);
			roverIn.init();

			// wait for some data to buffer
			Thread.sleep(2000);

			// set Output
			String outPath = "./out/" + date1 + ".kml";
			JakKmlProducer kml = new JakKmlProducer(outPath, 0);

			GoGPS goGPS = new GoGPS(navigationIn, roverIn).setDynamicModel(dynamicModel)
					.addPositionConsumerListener(kml);
			
			goGPS.setDebug(true)

					// run (never exit in live-tracking)
//					 .runCodeStandalone(0);
//					 .runKalmanFilterCodePhaseStandalone(0);

					// run in background
			  .runThreadMode(GoGPS.RunMode.CODE_STANDALONE)
//					.runThreadMode(GoGPS.RunMode.KALMAN_FILTER_CODE_PHASE_STANDALONE)

					// wait for 2 minutes
					.runFor(120);

			System.out.println();
			System.out.println();

			System.out.println("OK give up ---------------------------------------------");

			/******************************************
			 * END
			 */
			try {
				System.out.println("Stop Rover");
				roverIn.release(true, 10000);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}
			try {
				System.out.println("Stop Navigation");
				navigationIn.release(true, 10000);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}
			try {
				System.out.println("Stop UBX");
				ubxSerialConn.release(true, 10000);
			} catch (InterruptedException ie) {
				ie.printStackTrace();
			}

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
