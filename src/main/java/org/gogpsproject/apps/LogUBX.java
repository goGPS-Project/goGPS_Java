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
import java.util.Vector;

import org.gogpsproject.parser.ublox.UBXSerialConnection;
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
public class LogUBX {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));
		
		ArgumentParser parser = ArgumentParsers.newArgumentParser("LogUBX")
				.defaultHelp(true)
				.description("Log binary streams from one or more u-blox receivers connected to COM ports.");
		parser.addArgument("-s", "--showCOMports")
				.action(Arguments.storeTrue())
				.help("display available COM ports.");
		parser.addArgument("-r", "--rate")
				.choices(1, 2, 5, 10).setDefault(1)
				.type(Integer.class)
				.help("set the measurement rate (in Hz).");
		parser.addArgument("-e", "--ephemeris")
				.setDefault(10)
				.type(Integer.class)
				.metavar("EPH_RATE")
				.help("set the ephemeris (AID-EPH message) poll rate, in seconds. Set to zero to disable ephemeris logging.");
		parser.addArgument("-i", "--ionosphere")
				.setDefault(60)
				.type(Integer.class)
				.metavar("ION_RATE")
				.help("set the ionospheric parameters (AID-HUI message) poll rate, in seconds. Set to zero to disable ionospheric parameters logging.");
		parser.addArgument("-n", "--nmea")
				.choices("GGA", "GSV", "RMC", "GSA", "GLL", "GST", "GRS", "GBS", "DTM", "VTG", "ZDA").setDefault()
				.metavar("NMEA_ID")
				.nargs("+")
				.help("enable and log NMEA sentences. NMEA_ID must be replaced by an existing 3-letter NMEA sentence code (for example: -n GGA GSV RMC).");
		parser.addArgument("-t", "--timetag")
				.action(Arguments.storeTrue())
				.help("log the system time when RXM-RAW messages are received.");
		parser.addArgument("-xo", "--rinexobs")
		        .action(Arguments.storeTrue())
		        .help("write a RINEX observation file while logging.");
		parser.addArgument("-c", "--compress")
                .action(Arguments.storeTrue())
                .help("if RINEX output is enabled, compress (zip) the RINEX files as they are completed.");
		parser.addArgument("-m", "--marker")
		        .setDefault("")
		        .help("specify a marker name for the RINEX file [4 characters] (e.g. UBX0).");
		parser.addArgument("-o", "--outdir")
                .setDefault("./out")
                .help("specify a directory for the output files.");
		parser.addArgument("-d", "--debug")
                .action(Arguments.storeTrue())
                .help("show warning messages for debugging purposes.");
		parser.addArgument("port").nargs("*")
				.help("COM port(s) connected to u-blox receivers (e.g. COM3 COM10).");
		Namespace ns = null;
		try {
			ns = parser.parseArgs(args);
		} catch (ArgumentParserException e) {
			parser.handleError(e);
			System.exit(1);
		}

		try{

			if ((Boolean) ns.get("showCOMports")) {
				UBXSerialConnection.getPortList(true);
				return;
			} else if (ns.<String> getList("port").isEmpty()) {
				parser.printHelp();
				return;
			}
			
			Vector<String> availablePorts = UBXSerialConnection.getPortList(false);
			for (String portId : ns.<String> getList("port")) {
				if (!availablePorts.contains(portId)) {
					System.out.println("Error: port "+portId+" is not available.");
					UBXSerialConnection.getPortList(true);
					return;
				}
			}
			
			int r = 1;
			for (String portId : ns.<String> getList("port")) {
				
				UBXSerialConnection ubxSerialConn = new UBXSerialConnection(portId, 9600);
				
				ubxSerialConn.setMeasurementRate((Integer) ns.get("rate"));
				if (r == 1) {
					ubxSerialConn.enableEphemeris((Integer) ns.get("ephemeris"));
					ubxSerialConn.enableIonoParam((Integer) ns.get("ionosphere"));
				} else {
					ubxSerialConn.enableEphemeris(0);
					ubxSerialConn.enableIonoParam(0);
				}
				ubxSerialConn.enableTimetag(ns.getBoolean("timetag"));
				ubxSerialConn.enableNmeaSentences(ns.<String> getList("nmea"));
				ubxSerialConn.setOutputDir(ns.getString("outdir"));
				ubxSerialConn.enableDebug(ns.getBoolean("debug"));
				ubxSerialConn.init();
				
				if (ns.getBoolean("rinexobs")) {
					boolean singleFreq = true;
					boolean needApproxPos = false;
					RinexV2Producer rp = null;
					String marker = ns.getString("marker");
					if (marker.length() == 0) {
						String portStrMarker = preparePortStringForMarker(portId);
			    		String portStrId = portStrMarker.length() >= 2 ? portStrMarker.substring(portId.length() - 2) : "0" + portStrMarker;
			    		marker = "UB" + portStrId;
					}
					rp = new RinexV2Producer(needApproxPos, singleFreq, marker);
					rp.enableCompression(ns.getBoolean("compress"));
					rp.setOutputDir(ns.getString("outdir"));
					ubxSerialConn.addStreamEventListener(rp);
				}
				
				r++;
			}

		}catch(Exception e){
			e.printStackTrace();
		}
	}
	
	private static String preparePortStringForMarker(String COMPort) {
		if (COMPort.substring(0, 3).equals("COM")) {
			COMPort = COMPort.substring(3, COMPort.length());  //for Windows COM* ports
		}
		return COMPort;
	}
}
