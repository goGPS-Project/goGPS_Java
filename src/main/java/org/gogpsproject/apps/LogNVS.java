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

import org.gogpsproject.parser.nvs.NVSSerialConnection;
import org.gogpsproject.producer.rinex.RinexV2Producer;
import org.gogpsproject.producer.rinex.RinexV3Producer;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.impl.Arguments;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;

/**
 * @author Eugenio Realini, Cryms.com
 *
 */
public class LogNVS {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));
		
		ArgumentParser parser = ArgumentParsers.newArgumentParser("LogNVS")
				.defaultHelp(true)
				.description("Log binary streams from one or more NVS receivers connected to COM ports.");
		parser.addArgument("-s", "--showCOMports")
				.action(Arguments.storeTrue())
				.help("display available COM ports");
		parser.addArgument("-r", "--rate")
				.choices(1, 2, 5, 10).setDefault(1)
				.type(Integer.class)
				.help("set the measurement rate (in Hz)");
		parser.addArgument("-t", "--timetag")
				.action(Arguments.storeTrue())
				.help("log the system time when RXM-RAW messages are received");
		parser.addArgument("-xo", "--rinexobs")
		        .action(Arguments.storeTrue())
		        .help("write a RINEX observation file while logging");
		parser.addArgument("-c", "--compress")
		        .action(Arguments.storeTrue())
		        .help("if RINEX output is enabled, compress (zip) the RINEX files as they are completed.");
		parser.addArgument("-m", "--marker").nargs("+")
                .setDefault()
                .help("specify a marker name for the RINEX file [4 characters] (e.g. NVS0) of each COM port.");
		parser.addArgument("-o", "--outdir")
                .setDefault("./out")
                .help("specify a directory for the output files.");
		parser.addArgument("-d", "--debug")
                .action(Arguments.storeTrue())
                .help("show warning messages for debugging purposes");
		parser.addArgument("port").nargs("*")
				.help("COM port(s) connected to NVS receivers (e.g. COM3 COM10)");
		Namespace ns = null;
		try {
			ns = parser.parseArgs(args);
		} catch (ArgumentParserException e) {
			parser.handleError(e);
			System.exit(1);
		}

		try{

			if ((Boolean) ns.get("showCOMports")) {
				NVSSerialConnection.getPortList(true);
				return;
			} else if (ns.<String> getList("port").isEmpty()) {
				parser.printHelp();
				return;
			} else if (!ns.<String> getList("marker").isEmpty() && ns.<String> getList("marker").get(0) != "" && ns.<String> getList("marker").size() != ns.<String> getList("port").size()) {
				System.out.println("Error: if marker names are specified, their number must match that of the specified COM ports.");
				return;
			}
			
			Vector<String> availablePorts = NVSSerialConnection.getPortList(false);
			for (String portId : ns.<String> getList("port")) {
				if (!availablePorts.contains(portId)) {
					System.out.println("Error: port "+portId+" is not available.");
					NVSSerialConnection.getPortList(true);
					return;
				}
			}
			
			boolean gpsEnable = true;  // enable GPS data writing
			boolean qzsEnable = true;  // enable QZSS data writing
		    boolean gloEnable = true;  // enable GLONASS data writing	
		    boolean galEnable = true;  // enable Galileo data writing
		    boolean bdsEnable = false;  // enable BeiDou data writing

			Boolean[] multiConstellation = {gpsEnable, qzsEnable, gloEnable, galEnable, bdsEnable};
			
			int minDOY = 0;
			
			int r = 0;
			
			for (String portId : ns.<String> getList("port")) {
				
				NVSSerialConnection nvsSerialConn = new NVSSerialConnection(portId, 115200)

				  .setMeasurementRate((Integer) ns.get("rate"))
				  .enableTimetag(ns.getBoolean("timetag"))
				  .setOutputDir(ns.getString("outdir"))
				  .enableDebug(ns.getBoolean("debug"));
				nvsSerialConn.init();
				
				if (ns.getBoolean("rinexobs")) {
					boolean singleFreq = true;
					boolean needApproxPos = false;
//					RinexV2Producer rp = null;
					RinexV3Producer rp = null;
					String marker = "";
					if (ns.<String> getList("marker").isEmpty()) {
						String portStrMarker = preparePortStringForMarker(portId);
			    		String portStrId = portStrMarker.length() >= 2 ? portStrMarker.substring(portStrMarker.length() - 2) : "0" + portStrMarker;
			    		marker = "NV" + portStrId;
					} else {
						marker = ns.<String> getList("marker").get(r);
					}
//					rp = new RinexV2Producer(needApproxPos, singleFreq, marker);
					rp = new RinexV3Producer(needApproxPos, singleFreq, marker, multiConstellation, minDOY);
					rp.enableCompression(ns.getBoolean("compress"));
					rp.setOutputDir(ns.getString("outdir"));
					nvsSerialConn.addStreamEventListener(rp);
				}
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
