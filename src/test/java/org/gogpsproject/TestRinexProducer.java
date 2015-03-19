/*
 * Copyright (c) 2011 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
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

import java.io.IOException;
import java.util.Locale;

import org.gogpsproject.parser.rtcm3.RTCM3Client;
import org.gogpsproject.producer.rinex.RinexV2Producer;

/**
 * @author Lorenzo Patocchi, cryms.com
 *
 */
public class TestRinexProducer {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));
		
		String masterFile = "./test/master.dat";
//		String rover = "./test/20111003-004754-rover.dat";

		try {
			RTCM3Client rtcm = RTCM3Client.getInstance(args[0], Integer.parseInt(args[1]), args[2],args[3], args[4]);
			//RTCM3Client rtcm = RTCM3Client.getInstance("ntrip.jenoba.jp", 80, args[0],args[1], "JVR30");
			rtcm.setDebug(true);
			// Ntrip-GAA: $GPGGA,183836,3435.524,N,13530.231,E,4,10,1,164,M,1,M,3,0*69
			// CH Manno
			//Coordinates coordinates = Coordinates.globalXYZInstance(4382366.510741806,687718.046802147,4568060.791344867);
			// JP Osaka
			//Coordinates coordinates = Coordinates.globalXYZInstance(-3749314.940644724,3684015.867703885,3600798.5084946174);
			// IT Milano
			Coordinates coordinates = Coordinates.globalXYZInstance(4421892.585,718469.9347,4525016.336);
			rtcm.setVirtualReferenceStationPosition(coordinates);
			rtcm.setStreamFileLogger("./test/rtcm-stream.dat");
			rtcm.init();

			ObservationsBuffer ob = new ObservationsBuffer(rtcm, masterFile);
			ob.init();

			Thread.sleep(15*1000);
			rtcm.release(true, 10000);
			ob.release(true, 10000);

		} catch (Exception e1) {
			e1.printStackTrace();
		}

		System.out.println("MASTER");
		System.out.println("MASTER");
		System.out.println("MASTER");
		System.out.println("MASTER");
		System.out.println("MASTER");
		System.out.println("MASTER");
		System.out.println("MASTER");
		System.out.println("MASTER");
		ObservationsBuffer masterIn = new ObservationsBuffer();
		try {
			masterIn.readFromLog(masterFile);
		} catch (IOException e) {
			e.printStackTrace();
		}
//		ObservationsProducer masterIn = new RinexObservationParser(new File("P:\\Multimedia\\Dropbox\\GoGPS project\\misurazioni Cryms\\Misurazioni CAB1\\VirA275W.11o"));

//		System.out.println();
//		System.out.println("ROVER");
//		ObservationsBuffer roverIn = new ObservationsBuffer();
//		try {
//			roverIn.readFromLog(rover);
//		} catch (IOException e) {
//			e.printStackTrace();
//		}

		System.out.println("RINEX");
		System.out.println("RINEX");
		System.out.println("RINEX");
		System.out.println("RINEX");
		System.out.println("RINEX");
		System.out.println("RINEX");
		System.out.println("RINEX");
		RinexV2Producer rp = new RinexV2Producer(true, false);
		rp.setFilename("./test/20111003-004754-master.obs");
		rp.setDefinedPosition(masterIn.getDefinedPosition());

		Observations o = masterIn.getNextObservations();
		while(o!=null){
			rp.addObservations(o);
			o = masterIn.getNextObservations();
		}
		System.out.println("END");

	}

}
