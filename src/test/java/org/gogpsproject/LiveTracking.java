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
import org.gogpsproject.parser.ublox.UBXAssistNow;
import org.gogpsproject.parser.ublox.UBXFileReader;
import org.gogpsproject.producer.KmlProducer;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class LiveTracking {


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
			if(args.length<3){
				System.out.println("GoGPS <com_port> <ntrip_url> <ntrip_user> <ntrip_pass> <ntrip_port> <ntrip_mountpoint> <ubx_user> <ubx_pass>");
				return;
			}
			
			String comPort = args[0];
			String NTRIPurl = args[1];
			String NTRIPuser = args[2];
			String NTRIPpass = args[3];
			int NTRIPport = Integer.parseInt(args[4]);
			String NTRIPmountpoint = args[5];

			/******************************************
			 * ROVER & NAVIGATION uBlox
			 */
			UBXSerialConnection ubxSerialConn = new UBXSerialConnection(comPort, 9600);
			ubxSerialConn.init();

			ObservationsBuffer roverIn = new ObservationsBuffer(ubxSerialConn,"./test/roverOut.dat");
			NavigationProducer navigationIn = roverIn;
			roverIn.init();

			if(args.length>6){
				String assistNowUser = args[6];
				String assistNowPass = args[7];
				String cmd="aid";
//				String lon="135";
//				String lat="35";
				String lon=null;
				String lat=null;
				// UBXAssistNow actually needs broadcast ephemeris from a receiver to produce valid ephemeris
				navigationIn = new UBXAssistNow(assistNowUser, assistNowPass, cmd/*, lon, lat*/);
				try {
					navigationIn.init();
				} catch (Exception e) {
					e.printStackTrace();
				}
				System.out.println("Use UBXAssistNow as Navigation");
			}
			// wait for some data to buffer
			Thread.sleep(5000);

			/******************************************
			 * compute approx position in stand-alone mode
			 */
			GoGPS goGPSstandalone = new GoGPS(navigationIn, roverIn, null);
			goGPSstandalone.setDynamicModel(dynamicModel);
			// retrieve initial position, do not need to be precise
			Coordinates initialPosition = goGPSstandalone.runCodeStandalone(10);

			/******************************************
			 * MASTER RTCM/RINEX
			 */
			RTCM3Client rtcmClient = RTCM3Client.getInstance(NTRIPurl.trim(), NTRIPport, NTRIPuser.trim(), NTRIPpass.trim(), NTRIPmountpoint.trim());
			//navigationIn = new RinexNavigation(RinexNavigation.IGN_NAVIGATION_HOURLY_ZIM2);
			rtcmClient.setVirtualReferenceStationPosition(initialPosition);
			rtcmClient.setReconnectionPolicy(RTCM3Client.CONNECTION_POLICY_RECONNECT);
			rtcmClient.setExitPolicy(RTCM3Client.EXIT_ON_LAST_LISTENER_LEAVE);
			rtcmClient.init();

			ObservationsBuffer masterIn = new ObservationsBuffer(rtcmClient,"./test/masterOut.dat");
			masterIn.init();

			Date date = new Date();
			SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
			String date1 = sdf1.format(date);

			/******************************************
			 * compute precise position in Kalman filter mode
			 */
			GoGPS goGPS = new GoGPS(navigationIn, roverIn, masterIn);
			goGPS.setDynamicModel(dynamicModel);

			// set Output
			String outPath = "test/" + date1 + ".kml";
			KmlProducer kml = new KmlProducer(outPath, 2.5, 0);
			goGPS.addPositionConsumerListener(kml);

			// goGPS.runCodeDoubleDifferences();
			// run blocking (never exit in live-tracking)
			// goGPS.runKalmanFilter();

			// run in background
			goGPS.runThreadMode(GoGPS.RUN_MODE_KALMAN_FILTER);

			ArrayList<String> kmlFiles = new ArrayList<String>();
			kmlFiles.add(date1 + ".kml");
			initLiveKML(kmlFiles, initialPosition);


			// wait for 1 minutes
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
				System.out.println("Stop Master");
				masterIn.release(true,10000);
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

	public static void initLiveKML(ArrayList<String> files, Coordinates initialPosition){
		String line = "";
		line += "<?xml version=\"1.0\" standalone=\"yes\"?>\n";
		line += "<kml creator=\"goGPS-j\" xmlns=\"http://earth.google.com/kml/2.2\">\n";
		line += "  <Document>\n";
		line += "    <open>1</open>\n";

		for(String file:files){
			String url = /*"file:///"+*/file.replaceAll("\\\\", "/");
			line += "    <NetworkLink>\n";
			line += "      <name>goGPS</name>\n";
			line += "      <Link id=\"ID\">\n";
			line += "        <href>"+url+"</href>\n";
			line += "        <refreshMode>onInterval</refreshMode>\n";
			line += "        <refreshInterval>1.0</refreshInterval>\n";
			line += "      </Link>\n";
			line += "    </NetworkLink>\n";
		}

		line += "    <Placemark>\n";
		line += "      <name>Starting location</name>\n";
		line += "      <visibility>0</visibility>\n";
		line += "      <Point>\n";
		line += "        <altitudeMode>clampedToGround</altitudeMode>\n";
		line += "        <coordinates>"+initialPosition.getGeodeticLongitude()+","+initialPosition.getGeodeticLatitude()+","+(initialPosition.getGeodeticHeight())+"</coordinates>\n";
		line += "      </Point>\n";
		line += "      <Snippet></Snippet>\n";
		line += "      <Style>\n";
		line += "        <IconStyle>\n";
		line += "          <Icon>\n";
		line += "            <href>http://maps.google.com/mapfiles/kml/pal2/icon10.png</href>\n";
		line += "          </Icon>\n";
		line += "          <colorMode>normal</colorMode>\n";
		line += "          <scale>0.50</scale>\n";
		line += "        </IconStyle>\n";
		line += "      </Style>\n";
		line += "      <description></description>\n";
		line += "      <LookAt>\n";
		line += "        <longitude>"+initialPosition.getGeodeticLongitude()+"</longitude>\n";
		line += "        <latitude>"+initialPosition.getGeodeticLatitude()+"</latitude>\n";
		line += "        <altitude>0</altitude>\n";
		line += "        <range>120</range>\n";
		line += "        <tilt>30</tilt>\n";
		line += "        <heading>0</heading>\n";
		line += "      </LookAt>\n";
		line += "    </Placemark>\n";
		line += "  </Document>\n";
		line += "</kml>";

		String url = "test/livekml.kml";
		try {
			FileWriter out = new FileWriter(url);

			out.write(line);
			out.flush();
			out.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		Process process;
		try {
			process = Runtime.getRuntime().exec(new String[] { (String) "cmd.exe",
					"/c",
					"start",
					"\"\"",
					'"' + url + '"' });
//			process = Runtime.getRuntime().exec(new String[] { (String) "C:/Program Files (x86)/Google/Google Earth/client/googleearth.exe",
///*					"/c",
//					"start",
//					"\"\"",*/
//					'"' + (new File(url)).getAbsolutePath() + '"' });
			process.waitFor();
			process.exitValue();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}

}
