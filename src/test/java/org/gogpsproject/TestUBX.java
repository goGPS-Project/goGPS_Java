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

import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Enumeration;
import java.util.HashSet;
import java.util.Locale;
import java.util.Vector;

import org.gogpsproject.parser.ublox.UBXSerialConnection;

@SuppressWarnings("restriction")
public class TestUBX implements StreamEventListener{

	public static int speed = 9600;
	private static UBXSerialConnection ubxSerialConn;
    private String fileNameOutLog = null;
    private FileOutputStream fosOutLog = null;
    private DataOutputStream outLog = null;//new XMLEncoder(os);


    public TestUBX(){
    	this.fileNameOutLog = "./data/aphemeris.dat";
		if(fileNameOutLog!=null){
    		try {
				fosOutLog = new FileOutputStream(fileNameOutLog,true);
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}
    		outLog = new DataOutputStream(fosOutLog);
    	}
    }
	/**
	 * @param args
	 */

	@SuppressWarnings("unchecked")
	public static HashSet<CommPortIdentifier> getAvailableSerialPorts() {
		HashSet<CommPortIdentifier> h = new HashSet<CommPortIdentifier>();
		Enumeration<CommPortIdentifier> thePorts = CommPortIdentifier.getPortIdentifiers();
		while (thePorts.hasMoreElements()) {
			CommPortIdentifier com = (CommPortIdentifier) thePorts.nextElement();
			switch (com.getPortType()) {
			case CommPortIdentifier.PORT_SERIAL:
				try {
					CommPort thePort = com.open("CommUtil", 50);
					thePort.close();
					h.add(com);
				} catch (PortInUseException e) {
					System.out.println("Port, " + com.getName()
							+ ", is in use.");
				} catch (Exception e) {
					System.err.println("Failed to open port " + com.getName());
					e.printStackTrace();
				}
			}
		}
		return h;
	}

	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));

		Vector<String> ports = UBXSerialConnection.getPortList(false);
		if (ports.size() > 0) {
			System.out.println("the following serial ports have been detected:");
		} else {
			System.out.println("sorry, no serial ports were found on your computer\n");
			System.exit(0);
		}
		String port = null;
		for (int i = 0; i < ports.size(); ++i) {
			System.out.println("    " + Integer.toString(i + 1) + ":  "+ ports.elementAt(i));
			if(args.length>0 && args[0].equalsIgnoreCase(ports.elementAt(i))){
				port = ports.elementAt(i);
			}
		}
		if(args.length>2){
			speed = Integer.parseInt(args[1]);
		}


		ubxSerialConn = new UBXSerialConnection(port, speed);
		try {
			ubxSerialConn.init();
//			ObservationsBuffer rover = new ObservationsBuffer();
//			rover.setStreamSource(ubxSerialConn);
	//
//			try {
//				rover.init();
//			} catch (Exception e) {
//				e.printStackTrace();
//			}
			TestUBX test = new TestUBX();
			ubxSerialConn.addStreamEventListener(test);

		} catch (Exception e) {
			e.printStackTrace();
		}




		System.out.println("end");
		// // TODO Auto-generated method stub
		// HashSet<CommPortIdentifier> ha = new HashSet<CommPortIdentifier>();
		// ha = getAvailableSerialPorts();
		// while (ha.iterator().hasNext()) {
		// System.out.println(" port : " + ha.iterator().next().getPortType());
		// }
		//
		// }

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#addEphemeris(org.gogpsproject.EphGps)
	 */
	@Override
	public void addEphemeris(EphGps eph) {
		System.out.println("EPH "+eph);
		if(outLog!=null){
        	try {
        		eph.write(outLog);
        		outLog.flush();
			} catch (IOException e) {
				e.printStackTrace();
			}
        }
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#addIonospheric(org.gogpsproject.IonoGps)
	 */
	@Override
	public void addIonospheric(IonoGps iono) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#addObservations(org.gogpsproject.Observations)
	 */
	@Override
	public void addObservations(Observations o) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#setDefinedPosition(org.gogpsproject.Coordinates)
	 */
	@Override
	public void setDefinedPosition(Coordinates definedPosition) {
		// TODO Auto-generated method stub

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#streamClosed()
	 */
	@Override
	public void streamClosed() {
		// TODO Auto-generated method stub

	}
	@Override
	public Observations getCurrentObservations() {
		// TODO Auto-generated method stub
		return null;
	}
	@Override
	public void pointToNextObservations() {
		// TODO Auto-generated method stub
		
	}

}
