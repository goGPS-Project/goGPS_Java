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
import java.util.Locale;

import org.gogpsproject.ObservationsBuffer;
import org.gogpsproject.parser.rtcm3.RTCM3Client;
import org.gogpsproject.producer.rinex.RinexV2Producer;

public class TestRTCM3 {

	/**
	 * @param args
	 */

	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));

		String NTRIPurl = args[0];
		int NTRIPport = Integer.parseInt(args[1]);
		String NTRIPuser = args[2];
		String NTRIPpass = args[3];
		String NTRIPmountpoint = args[4];

		try {
			RTCM3Client rtcm = RTCM3Client.getInstance(NTRIPurl.trim(), NTRIPport, NTRIPuser.trim(), NTRIPpass.trim(), NTRIPmountpoint.trim());
			// CH Manno
			//Coordinates coordinates = Coordinates.globalXYZInstance(4382366.510741806,687718.046802147,4568060.791344867);
			// JP Osaka
			//Coordinates coordinates = Coordinates.globalXYZInstance(-3749314.940644724,3684015.867703885,3600798.5084946174);
			// IT Milano
			Coordinates coordinates = Coordinates.globalXYZInstance(4421892.585,718469.9347,4525016.336);
			rtcm.setVirtualReferenceStationPosition(coordinates);
			rtcm.setReconnectionPolicy(RTCM3Client.CONNECTION_POLICY_RECONNECT);
			rtcm.setExitPolicy(RTCM3Client.EXIT_ON_LAST_LISTENER_LEAVE);
			rtcm.setDebug(true);
			rtcm.init();

			// log rinex format
			RinexV2Producer rinexOut = new RinexV2Producer(true,false);
			rinexOut.setFilename("./test/test-rinex.obs");
			rtcm.addStreamEventListener(rinexOut);

			ObservationsBuffer ob = new ObservationsBuffer(rtcm,"./test/test-rtcm.dat");
			ob.init();

			Thread.sleep(60*1000);

			rtcm.release(true, 10*1000);

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
