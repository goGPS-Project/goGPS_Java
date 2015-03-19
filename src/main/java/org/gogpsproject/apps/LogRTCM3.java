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
import java.util.Locale;

import org.gogpsproject.Coordinates;
import org.gogpsproject.parser.rtcm3.RTCM3Client;
import org.gogpsproject.producer.rinex.RinexV2Producer;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.impl.Arguments;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class LogRTCM3 {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));
		
		ArgumentParser parser = ArgumentParsers.newArgumentParser("LogRTCM3")
				.defaultHelp(true)
				.description("Log the binary stream from an NTRIP caster.");
		parser.addArgument("-p", "--port")
				.setDefault(2101)
				.type(Integer.class)
				.help("NTRIP caster port");
		parser.addArgument("-u", "--username")
				.setDefault("")
				.help("NTRIP username");
		parser.addArgument("-w", "--password")
				.setDefault("")
				.help("NTRIP password");
		parser.addArgument("-xo", "--rinexobs")
        		.action(Arguments.storeTrue())
        		.help("write a RINEX observation file while logging");
		parser.addArgument("-c", "--compress")
        		.action(Arguments.storeTrue())
        		.help("if RINEX output is enabled, compress (zip) the RINEX files as they are completed.");
		parser.addArgument("url").nargs(1)
				.help("NTRIP caster URL (e.g. 111.20.31.4)");
		parser.addArgument("mountpoint").nargs(1)
				.help("Mountpoint (e.g. VRS-01)");
		parser.addArgument("latitude").nargs(1)
				.type(Double.class)
				.help("Latitude [decimal degrees] (e.g. 45.80)");
		parser.addArgument("longitude").nargs(1)
				.type(Double.class)
				.help("Longitude [decimal degrees] (e.g. 9.05)");
		parser.addArgument("height").nargs(1)
				.type(Double.class)
				.help("Ellipsoidal height [m]");
		parser.addArgument("marker").nargs(1)
				.help("Marker name [4 characters] (e.g. VRS0)");
		parser.addArgument("-o", "--outdir")
        		.setDefault("./out")
        		.help("specify a directory for the output files.");
		parser.addArgument("-rp", "--policy")
				.choices("WAIT", "RECONNECT", "LEAVE").setDefault("WAIT")
				.help("reconnection policy when data reception stops (WAIT = wait indefinitely for new data; RECONNECT = close current connection and try to reconnect to NTRIP caster after WTIME seconds; LEAVE = stop logging and quit).");
		parser.addArgument("-wt", "--waitingtime")
				.setDefault(300)
				.type(Integer.class)
				.metavar("WTIME")
				.help("waiting time in seconds before attempting to reconnect (used only with the RECONNECT policy).");
		parser.addArgument("-d", "--debug")
        		.action(Arguments.storeTrue())
        		.help("show warning messages for debugging purposes.");
		Namespace ns = null;
		try {
			ns = parser.parseArgs(args);
		} catch (ArgumentParserException e) {
			parser.handleError(e);
			System.exit(1);
		}

		String NTRIPurl = ns.<String> getList("url").get(0);
		int NTRIPport = ns.getInt("port");
		String NTRIPuser = ns.getString("username");
		String NTRIPpass = ns.getString("password");
		String NTRIPmountpoint = ns.<String> getList("mountpoint").get(0);
		String markerName = ns.<String> getList("marker").get(0);
		String reconnectionPolicy = ns.getString("policy");
		int chosenReconnectionPolicy = RTCM3Client.CONNECTION_POLICY_WAIT;
		if (reconnectionPolicy.equals("RECONNECT")) {
			chosenReconnectionPolicy = RTCM3Client.CONNECTION_POLICY_RECONNECT;
		} else if (reconnectionPolicy.equals("LEAVE")) {
			chosenReconnectionPolicy = RTCM3Client.CONNECTION_POLICY_LEAVE;
		}

		try {
			RTCM3Client rtcm = RTCM3Client.getInstance(NTRIPurl.trim(), NTRIPport, NTRIPuser.trim(), NTRIPpass.trim(), NTRIPmountpoint.trim());

			Coordinates coordinates = Coordinates.globalGeodInstance(ns.<Double> getList("latitude").get(0),ns.<Double> getList("longitude").get(0),ns.<Double> getList("height").get(0));
			rtcm.setVirtualReferenceStationPosition(coordinates);
			rtcm.setMarkerName(markerName);
			rtcm.setOutputDir(ns.getString("outdir"));
			
			rtcm.setReconnectionPolicy(chosenReconnectionPolicy);
			rtcm.setExitPolicy(RTCM3Client.EXIT_NEVER);
			rtcm.setReconnectionWaitingTime((Integer) ns.get("waitingtime"));
			rtcm.setDebug(ns.getBoolean("debug"));
			rtcm.init();
			
			if (ns.getBoolean("rinexobs")) {
				boolean singleFreq = false;
				boolean needApproxPos = true;
				RinexV2Producer rp = null;
				rp = new RinexV2Producer(needApproxPos, singleFreq, markerName);
				rp.enableCompression(ns.getBoolean("compress"));
				rp.setOutputDir(ns.getString("outdir"));
				rtcm.addStreamEventListener(rp);
			}

		} catch (Exception e1) {
			e1.printStackTrace();
		}
	}
}
