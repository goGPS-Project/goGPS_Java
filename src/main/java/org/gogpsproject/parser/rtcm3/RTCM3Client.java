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

package org.gogpsproject.parser.rtcm3;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.InetSocketAddress;
import java.net.Proxy;
import java.net.Socket;
import java.util.Date;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.NoSuchElementException;
import java.util.StringTokenizer;
import java.util.Vector;

import org.gogpsproject.Coordinates;
import org.gogpsproject.Observations;
import org.gogpsproject.StreamEventListener;
import org.gogpsproject.StreamEventProducer;
import org.gogpsproject.StreamResource;
import org.gogpsproject.Time;
import org.gogpsproject.util.Bits;
import org.gogpsproject.util.InputStreamCounter;

public class RTCM3Client implements Runnable, StreamResource, StreamEventProducer {

	private ConnectionSettings settings;
	private Thread dataThread;

	//private boolean waitForData = true;

	private boolean running = false;
	/**
	 * @return the running
	 */
	public boolean isRunning() {
		return running;
	}

	private boolean askForStop = false;
	private HashMap<Integer, Decode> decodeMap;

	/** Optional message handler for showing error messages. */
	private int messagelength = 0;
	private int[] buffer;
	private boolean[] bits;
	private boolean[] rollbits;

	private InputStream in = null;
	private Socket sck = null;
	private PrintWriter out = null;

	private boolean debug=false;

	private Coordinates virtualReferenceStationPosition = null;
	private Coordinates masterPosition = null;
	private AntennaDescriptor antennaDescriptor = null;

	private Vector<StreamEventListener> streamEventListeners = new Vector<StreamEventListener>();

	//private Vector<Observations> observationsBuffer = new Vector<Observations>();
	//private int obsCursor = 0;

	private String streamFileLogger = null;

	private String ntripGAA = null;
	private long lastNtripGAAsent = 0;
	private long ntripGAAsendDelay = 10*1000; // 10 sec

	public final static int CONNECTION_POLICY_LEAVE = 0;
	public final static int CONNECTION_POLICY_RECONNECT = 1;
	public final static int CONNECTION_POLICY_WAIT = 2;
	private int reconnectionPolicy = CONNECTION_POLICY_RECONNECT;
	private long reconnectionWaitingTime = 300*1000; // 5 minutes

	public final static int EXIT_NEVER = 0;
	public final static int EXIT_ON_LAST_LISTENER_LEAVE = 1;
	private int exitPolicy = EXIT_ON_LAST_LISTENER_LEAVE;
	private boolean online = false;
	private int week;
	private double currentTime;
	private double previousTime = -1;
	private String outputDir = "./test";
	private String markerName = "MMMM";

	/**
	 * @return the exitPolicy
	 */
	public int getExitPolicy() {
		return exitPolicy;
	}

	/**
	 * @param exitPolicy the exitPolicy to set
	 */
	public void setExitPolicy(int exitPolicy) {
		this.exitPolicy = exitPolicy;
	}

	public static RTCM3Client getInstance(String _host, int _port, String _username,
			String _password, String _mountpoint) throws Exception{
		return getInstance(_host, _port, _username, _password, _mountpoint, false);
	}
	public static RTCM3Client getInstance(String _host, int _port, String _username,
			String _password, String _mountpoint, boolean ldebug) throws Exception{

		ArrayList<String> s = new ArrayList<String>();
		ConnectionSettings settings = new ConnectionSettings(_host, _port, _username, _password);
		ArrayList<String> mountpoints = new ArrayList<String>();
		RTCM3Client net = new RTCM3Client(settings);
		try {
			//System.out.println("Get sources");
			s = net.getSources();
			//System.out.println("Got sources");
		} catch (IOException e) {
			e.printStackTrace();
			throw new Exception(e);
		}
		for (int j = 1; j < s.size(); j++) {
			if (j % 2 == 0){
				mountpoints.add(s.get(j));
			}
		}
		if(_mountpoint == null){
			if(ldebug) System.out.println("Available Mountpoints:");
		}
		for (int j = 0; j < mountpoints.size(); j++) {
			if(_mountpoint == null){
				if(ldebug) System.out.println("\t[" + mountpoints.get(j)+"]");
			}else{
				if(ldebug) System.out.print("\t[" + mountpoints.get(j)+"]["+_mountpoint+"]");
				if(_mountpoint.equalsIgnoreCase(mountpoints.get(j))){
					settings.setSource(mountpoints.get(j));
					if(ldebug) System.out.print(" found");
				}
			}
			if(ldebug) System.out.println();
		}
		if(settings.getSource() == null){
			System.out.println("Select a valid mountpoint!");
			return null;
		}
		return net;
	}
	public static RTCM3Client getVRSInstance(String _host, int _port, String _username,
			String _password, String _mountpoint, Coordinates vrsPosition) throws Exception{
		return getVRSInstance(_host, _port, _username, _password, _mountpoint, vrsPosition, false);
	}
	public static RTCM3Client getVRSInstance(String _host, int _port, String _username,
			String _password, String _mountpoint, Coordinates vrsPosition, boolean debug) throws Exception{

		RTCM3Client rtcm = getInstance(_host, _port, _username, _password, _mountpoint, debug);
		rtcm.setVirtualReferenceStationPosition(vrsPosition);
		return rtcm;
	}
	public RTCM3Client(ConnectionSettings settings) {
		super();
		running = false;
		this.settings = settings;

		decodeMap = new HashMap<Integer, Decode>();

		decodeMap.put(new Integer(1004), new Decode1004Msg(this));
		decodeMap.put(new Integer(1005), new Decode1005Msg(this));
		decodeMap.put(new Integer(1006), new Decode1006Msg(this));
		decodeMap.put(new Integer(1007), new Decode1007Msg(this));
		decodeMap.put(new Integer(1008), new Decode1008Msg(this));
		decodeMap.put(new Integer(1012), new Decode1012Msg());
	}
	
	public RTCM3Client(int startWeek) {
		this.week = startWeek;
		running = false;

		decodeMap = new HashMap<Integer, Decode>();

		decodeMap.put(new Integer(1004), new Decode1004Msg(this));
		decodeMap.put(new Integer(1005), new Decode1005Msg(this));
		decodeMap.put(new Integer(1006), new Decode1006Msg(this));
		decodeMap.put(new Integer(1007), new Decode1007Msg(this));
		decodeMap.put(new Integer(1008), new Decode1008Msg(this));
		decodeMap.put(new Integer(1012), new Decode1012Msg());
	}

	public ArrayList<String> getSources() throws IOException {

		//System.out.println("Open Socket "+settings.getHost()+" port "+ settings.getPort());
		@SuppressWarnings("resource")
		Socket sck = new Socket(settings.getHost(), settings.getPort());

		//System.out.println("Open streams");
		// The input and output streams are created
		PrintWriter out = new PrintWriter(sck.getOutputStream(), true);
		InputStream sckIn = sck.getInputStream();
		// A Buffered reader is created so we can read whole lines
		InputStreamReader inRead = new InputStreamReader(sckIn);
		BufferedReader in = new BufferedReader(inRead);

		//System.out.println("Send request");
		// The data request containing the logon and password are send
		out.print("GET / HTTP/1.1\r\n");
		out.print("User-Agent: NTRIP goGPS-project java\r\n");
		out.print("Host: "+settings.getHost()+"\r\n");
		out.print("Connection: close\r\n");
		out.print("Authorization: Basic " + settings.getPass_base64()+"\r\n");
		// out.println("Ntrip-GAA: $GPGGA,200530,4600,N,00857,E,4,10,1,200,M,1,M,3,0*65");
		// out.println("Accept: */*\r\nConnection: close");
		out.print("\r\n");
		out.flush();

		//System.out.println("Get answer");
		boolean going = true;
		boolean first = true;
		Vector<String> lines = new Vector<String>();


		while (going) {
			// The next byte is read and added to the buffer

			String newLine = in.readLine();

			//System.out.println("Read:"+newLine);
			if (newLine == null) {
				going = false;
			} else if (first) {
				// The first line should be "SOURCETABLE 200 OK"
				if (!newLine.equals("SOURCETABLE 200 OK")) {
					going = false;
				}
				first = false;

			} else {
				lines.addElement(newLine);
			}
		}

		// Lines are parsed
		ArrayList<String> sources = new ArrayList<String>();
		for (int i = 0; i < lines.size(); i++) {
			// A new StringTokenizer is created with ";" as delimiter

			StringTokenizer token = new StringTokenizer(lines.elementAt(i), ";");
			try {
				if (token.countTokens() > 1 && token.nextToken().equals("STR")) {

					//System.out.println(lines.elementAt(i));

					// We excpect the correct source to be the first token after
					// "STR" to through the token wich specifies the RTCM
					// version
					// starting with "RTCM "
					// We haven't seen any specification of the sourcetable, but
					// according to what we can see from it it should be correct
					String s = token.nextToken();
					while (!s.startsWith("RTCM 3")) {
						sources.add(s);
						s = token.nextToken();
					}

				}
			} catch (NoSuchElementException ex) {/* The line is ignored */
			}
		}

		in.close();
		inRead.close();
		sckIn.close();
		out.close();

		return sources;
	}

	// public int[] readMessage() throws BufferUnderrunException {
	// return outputBuffer.readMessage();
	// }
	//
	// /**
	// * returns the number of messages ready for reading<B> uses
	// * CRWBuffer.ready()
	// */
	// public int ready() {
	// return outputBuffer.ready();
	// }


	@Override
	public void run() {

		File file = new File(outputDir);
		if(!file.exists() || !file.isDirectory()){
		    boolean wasDirectoryMade = file.mkdirs();
		    if(wasDirectoryMade)System.out.println("Directory "+outputDir+" created");
		    else System.out.println("Could not create directory "+outputDir);
		}
		
		Date date = new Date();
		SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
		String date1 = sdf1.format(date);
		SimpleDateFormat sdfFile = new SimpleDateFormat("yyyy-MM-dd_HHmmss");
		String dateFile = sdfFile.format(date);

		System.out.println(date1 + " - Logging RTCM3 stream in "+outputDir+"/" + markerName + "_" + dateFile + ".rtcm");
		setStreamFileLogger(outputDir+"/" + markerName + "_" + dateFile + ".rtcm");

		try {
			// Socket for receiving data are created

			try {
				Proxy proxy = Proxy.NO_PROXY;
				// proxy = new Proxy(Proxy.Type.SOCKS, new
				// InetSocketAddress("127.0.0.1", 8888));

				sck = new Socket(proxy);
				InetSocketAddress dest = new InetSocketAddress(settings.getHost(), settings.getPort());
				sck.connect(dest);
				// sck = new Socket(settings.getHost(), settings.getPort());
				if (debug)
					System.out.println("Connected to " + settings.getHost() + ":" + settings.getPort());
				running = true;

			} catch (Exception e) {

				if (debug) System.out.println("Connection to " + settings.getHost() + ":" + settings.getPort() + " failed: \n  " + e);
				// if (messages == null) {
				// tester.println("<" + settings.getSource() + ">" + msg);
				// } else {
				// messages.showErrorMessage(settings.getSource(), msg);
				// }
				closeAll();
				if (!askForStop && reconnectionPolicy == CONNECTION_POLICY_RECONNECT) {
					if (debug) System.out.println("Sleep " + reconnectionWaitingTime/1000 + " s before retry");
					Thread.sleep(reconnectionWaitingTime);
					start();
				} else {
					for (StreamEventListener sel : streamEventListeners) {
						sel.streamClosed();
					}
				}
				return;
			}

			// The input and output streams are created
			out = new PrintWriter(sck.getOutputStream(), true);
			in = sck.getInputStream();
			// The data request containing the logon and password is sent
			out.print("GET /" + settings.getSource() + " HTTP/1.1\r\n");
			out.print("Host: " + settings.getHost() + "\r\n");
			// out.print("Ntrip-Version: Ntrip/2.0\r\n");
			out.print("Accept: rtk/rtcm, dgps/rtcm\r\n");
			out.print("User-Agent: NTRIP goGPSprojectJava\r\n");
			if (virtualReferenceStationPosition != null) {
				virtualReferenceStationPosition.computeGeodetic();
				String hhmmss = (new SimpleDateFormat("HHmmss"))
						.format(new Date());

				int h = (int) virtualReferenceStationPosition.getGeodeticHeight();
				double lon = virtualReferenceStationPosition.getGeodeticLongitude();
				double lat = virtualReferenceStationPosition.getGeodeticLatitude();

				int lon_deg = (int) lon;
				double lon_min = (lon - lon_deg) * 60;
				double lon_nmea = lon_deg * 100 + lon_min;
				String lonn = (new DecimalFormat("00000.000")).format(lon_nmea);
				int lat_deg = (int) lat;
				double lat_min = (lat - lat_deg) * 60;
				double lat_nmea = lat_deg * 100 + lat_min;
				String latn = (new DecimalFormat("0000.000")).format(lat_nmea);
				ntripGAA = "$GPGGA," + hhmmss + "," + latn + ","
						+ (lat < 0 ? "S" : "N") + "," + lonn + ","
						+ (lon < 0 ? "W" : "E") + ",1,10,1.00," + (h < 0 ? 0 : h)
						+ ",M,1,M,,,,";
				// String ntripGAA =
				// "$GPGGA,"+hhmmss+".00,"+latn+","+(lat<0?"S":"N")+","+lonn+","+(lon<0?"W":"E")+",1,10,1.00,"+(h<0?0:h)+",M,37.3,M,,";
				// ntripGAA =
				// "$GPGGA,214833.00,3500.40000000,N,13900.10000000,E,1,10,1,-17.3,M,,M,,";

				ntripGAA = /* "Ntrip-GAA: "+ */ntripGAA + "*"
						+ computeNMEACheckSum(ntripGAA);
				if (debug)System.out.println(ntripGAA);

				// out.print(ntripGAA+"\r\n");
			}
			out.print("Connection: close\r\n");
			out.print("Authorization: Basic " + settings.getAuthbase64()
					+ "\r\n");

			// out.println("User-Agent: NTRIP goGps");
			// out.println("Ntrip-GAA: $GPGGA,200530,4600,N,00857,E,4,10,1,200,M,1,M,3,0*65");
			// out.println("User-Agent: NTRIP GoGps");
			// out.println("Accept: */*\r\nConnection: close");
			out.print("\r\n");
			if (ntripGAA != null) {
				out.print(ntripGAA + "\r\n");
				lastNtripGAAsent = System.currentTimeMillis();
			}
			out.flush();
			// System.out.println(" \n %%%%%%%%%%%%%%%%%%%%% \n password >>> "
			// + settings.getAuthbase64());
			// *****************
			// Reading the data

			// /First we read the HTTP header using a small state machine
			// The end of the header is received when a double end line
			// consisting
			// of a "new line" and a "carriage return" character has been received
			int state = 0;
			// First the HTTP header type is read. It should be "ICY 200 OK"
			// But Since we receive integers not characters the correct header is
			// numeric: 73 = 'I', 67 = 'C' and so on.

			int[] header = new int[11];
			int[] correctHeader = { 73, 67, 89, 32, 50, 48, 48, 32, 79, 75, 13 };
			int hindex = 0;
			// when 'running' is changed to false the loop is stopped

			while (running && state == 0) {
				int c = in.read();
				if (debug)
					System.out.print((char) c);
				if (c < 0) {
					break;
				}
				// break;
				// tester.write(c);
				state = transition(state, c);
				if (hindex > 10) {
					// The header should only be 11 characters long
					running = false;
				} else {
					header[hindex] = c;
					hindex++;
				}
			}

			for (int i = 0; i < 11 && running; i++) {
				if (header[i] != correctHeader[i]) {
					running = false;
				}
			}
			if (header[0] == 0) {
				if (debug) System.out.println("Waiting for connection acknowledgment message (\"ICY 200 OK\")...");
				running = true;
			}
			if (!running) {
				for (int i = 0; i < header.length; i++)
					if (debug)
						System.out.print((char) header[i]);
				int c = in.read();
				while (c != -1) {
					if (debug)
						System.out.println(((int) c) + " " + (char) c);

					c = in.read();
				}
				if (debug)
					System.out.println(((int) c) + " " + (char) c);

				//if (debug)
					//System.out.println();
				if (debug)
					System.out.println(settings.getSource() + " invalid header");

				closeAll();
				if (!askForStop && reconnectionPolicy == CONNECTION_POLICY_RECONNECT) {
					System.out.println("Sleep " + reconnectionWaitingTime/1000 + " s before retry");
					Thread.sleep(reconnectionWaitingTime);
					start();
				} else {
					for (StreamEventListener sel : streamEventListeners) {
						sel.streamClosed();
					}
				}
				return;
			}

			while (state != 5) {
				int c = in.read();
				if (debug)
					System.out.println(((int) c) + " " + (char) c);
				if (c < 0) {
					break;
				}
				// tester.write(c);
				state = transition(state, c);
			}
			// When HTTP header is read, the GPS data are recived and parsed:

			// The data is buffered as it is recived. When the buffer has size 6
			// There is a full word + a byte. The extra byte (first in buffer)
			// is
			// used for parity check.
			if (running) {
				// tester.println("<" + settings.getSource() +
				// ">Header least: OK");
				if (debug)
					System.out.println(settings.getSource()
							+ " connected successfully");
			} else {
				// showErrorMessage(settings.getSource(), "Error");
				if (debug)
					System.out.println(settings.getSource() + " not connected");
				closeAll();
				if (!askForStop && reconnectionPolicy == CONNECTION_POLICY_RECONNECT) {
					if (debug) System.out.println("Sleep " + reconnectionWaitingTime/1000 + " s before retry");
					Thread.sleep(reconnectionWaitingTime);
					start();
				} else {
					for (StreamEventListener sel : streamEventListeners) {
						sel.streamClosed();
					}
				}
				return;
			}
			// The read loop is started
			// sck.wait(1000);
			// this.dataThread.sleep(6000);
			// this.notifyAll();

			FileOutputStream fos = null;
			try {
				if (streamFileLogger != null) {
					fos = new FileOutputStream(streamFileLogger);
				}
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}

			InputStream isc = (fos == null ? in : new InputStreamCounter(in,
					fos));

			readLoop(isc, out);
			// System.out.println("1");

		} catch (IOException ex) {
			ex.printStackTrace();
		} catch (Exception ex) {
			ex.printStackTrace();

		} finally {
			// Connection was either terminated or an IOError occurred

			if (running) {
				if (debug)System.out.println(settings.getSource()
						+ " connection error: the data stream stopped");
			} else {
				if (debug)
					System.out.println(settings.getSource()
							+ " connection closed by client");
			}

			running = false;
			
			closeAll();

			// reconnect if needed
			if (!askForStop && reconnectionPolicy == CONNECTION_POLICY_RECONNECT) {
				if (debug) System.out.println("Sleep " + reconnectionWaitingTime/1000 + " s before retry");
				try {
					Thread.sleep(reconnectionWaitingTime);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				start();
			} else {
				for (StreamEventListener sel : streamEventListeners) {
					sel.streamClosed();
				}
			}
		}
	}

	private static String computeNMEACheckSum(String msg){
		// perform NMEA checksum calculation
		int chk = 0;

		for (int i = 1; i < msg.length(); i++){
			chk ^= msg.charAt(i);
		}
		String chk_s = Integer.toHexString(chk).toUpperCase();
		// checksum must be 2 characters!
		while (chk_s.length() < 2){
			chk_s = "0" + chk_s;
		}
		return chk_s;

	}
	public void start() {
		askForStop = false;

		dataThread = new Thread(this);
		dataThread.setName("RTCM3Client "+settings.getHost()+" "+settings.getSource());
		dataThread.start();
	}
	
	private void closeAll() {
		// All connections are closed
		if (out != null) try {out.close();} catch (Exception ex) {}
		if (in  != null) try { in.close();} catch (Exception ex) {}
		if (sck != null) try {sck.close();} catch (Exception ex) {}
	}

	/** stops the execution of this thread
	 * @throws InterruptedException */
	public void stop(boolean waitForThread, long timeoutMs) throws InterruptedException {
		askForStop = true;
		running = false;
		// disable waitForData to avoid wait forever in nextObservations()
		//waitForData = false;

		if(waitForThread && dataThread!=null){
			try{
				dataThread.join(timeoutMs);
			}catch(InterruptedException ie){
				ie.printStackTrace();
			}
			if(dataThread.isAlive()){
				if (debug)System.out.println("Killing thread "+dataThread.getName());
				dataThread.interrupt();
			}
		}

	}

	/** returns true if the data thread still is alive */
	public boolean stopped() {
		// return true;
		return dataThread != null && !dataThread.isAlive();
	}

	public int transition(int state, int input) {
		switch (state) {
			case 0: {
				if (input == 13)
					state = 1;
				break;
			}

			case 1: {
				if (input == 13)
					state = 2;
				break;
			}
			case 2: {
				if (input == 10)
					state = 5;
				else
					state = 1;
				break;
			}
			case 3: {
				if (input == 13)
					state = 4;
				else
					state = 1;
				break;
			}
			case 4: {
				if (input == 10)
					state = 5;
				else
					state = 1;
				break;
			}
		}

		return state;
	}

	/**
	 * reads data from an InputStream while running is true
	 *
	 * @param in
	 *            input stream to read from
	 * @return 
	 */
	
	protected void readLoop(InputStream in,PrintWriter out) throws IOException {
		int c;
		long start = System.currentTimeMillis();
		if(debug) System.out.print("Waiting for header");
		online = true;

		while(running) {
			c = in.read();

			if (c < 0){
				if(reconnectionPolicy != CONNECTION_POLICY_WAIT && System.currentTimeMillis()-start >10*1000) break;
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				if(debug) System.out.print(".");
			}
//			Object o = null;
//			if (header) {
				//if(debug) System.out.println("Header : " + c);
				if (c == 211) { // header
					readMessage(in);
				}
//			}
			
//			if(o instanceof Observations){
//				Observations oo = (Observations)o;
//				if(streamEventListeners!=null && oo!=null){
//					for(StreamEventListener sel:streamEventListeners){
//						//Observations co = sel.getCurrentObservations();
//					    //sel.pointToNextObservations();
//						Observations oc = (Observations) oo.clone();
//					    sel.addObservations(oc);
//					}
//				}
//			}
			
			if(out!=null && System.currentTimeMillis()-lastNtripGAAsent > ntripGAAsendDelay){
				out.print(ntripGAA+"\r\n");
				lastNtripGAAsent = System.currentTimeMillis();
				if(debug) System.out.println("refresh ntripGGA:" + ntripGAA);
			}
		}
	}
	
	public Object readMessage(InputStream in) throws IOException {
		int index;
		Object o = null;
		index = 0;
		buffer = new int[2];
		buffer[0] = in.read();
		buffer[1] = in.read();
		bits = new boolean[buffer.length * 8];
		rollbits = new boolean[8];
		for (int i = 0; i < buffer.length; i++) {
			rollbits = Bits.rollByteToBits(buffer[i]);
			for (int j = 0; j < rollbits.length; j++) {
				bits[index] = rollbits[j];
				index++;
			}
		}
		messagelength = (int)Bits.bitsToUInt(Bits.subset(bits, 6, 10));
		if(debug){
			System.out.println();
			System.out.println("Debug message length : " + messagelength);
		}

		if (messagelength >= 12) {
			setBits(in, messagelength);
			int msgtype = (int)Bits.bitsToUInt(Bits.subset(bits, 0, 12));

			if(debug) System.out.println("message type : " + msgtype);
			messagelength = 0;
			
			Date date = new Date();
			SimpleDateFormat sdf1 = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
			String date1 = sdf1.format(date);

			Decode dec = decodeMap.get(new Integer(msgtype));
			if(dec!=null){
				if (online) {
					Time currentTime = new Time(System.currentTimeMillis());
					o = dec.decode(bits, currentTime.getGpsWeek());
					System.out.println(date1 + " - RTCM message "+msgtype+" received and decoded");
				} else {
					o = dec.decode(bits, week);
					if(o instanceof Observations){
						if (((Observations) o).getRefTime().getDayOfYear() == 11) {
							week++;
							week--;
						}
						currentTime = ((Observations) o).getRefTime().getGpsTime();
						if (currentTime < previousTime) {
							week++;
							((Observations) o).setRefTime(new Time(week, currentTime));
						}
						previousTime = currentTime;
					}
				}
				if(o instanceof Observations){
					addObservation((Observations) o);
				}
			}else{
				//System.err.println("missing RTCM message parser "+msgtype);
				// missing message parser
			}

			// CRC
			setBits(in, 3);

			// setBits(in,1);
			//if(debug) System.out.println(" dati :" + Bits.bitsToStr(bits));
		}
		return o;
	}

	private void setBits(InputStream in, int bufferlength) throws IOException {
		int index = 0;
		buffer = new int[bufferlength];
		bits = new boolean[buffer.length * 8];
		for (int i = 0; i < buffer.length; i++) {
			buffer[i] = in.read();
		}
		// index = 0;
		for (int i = 0; i < buffer.length; i++) {
			rollbits = Bits.rollByteToBits(buffer[i]);
			for (int j = 0; j < 8; j++) {
				bits[index] = rollbits[j];
				index++;
			}
		}
	}

	public void addObservation(Observations o){
		if(streamEventListeners!=null && o!=null){
			for(StreamEventListener sel:streamEventListeners){
				Observations oc = (Observations)o.clone();
				sel.addObservations(oc);
			}
		}
//		if(debug){
//			System.out.println("\t\t\t\tM > obs "+o.getGpsSize()+" time "+new Date(o.getRefTime().getMsec()));
//			for(int i=0;i<o.getGpsSize();i++){
//				ObservationSet os = o.getGpsByIdx(i);
//				System.out.print(" svid:"+os.getSatID());
//				System.out.print(" codeC:"+os.getCodeC(0));
//				System.out.print(" codeP:"+os.getCodeP(0));
//				System.out.print(" doppl:"+os.getDoppler(0));
//				System.out.print(" LLInd:"+os.getLossLockInd(0));
//				System.out.print(" phase:"+os.getPhase(0));
//				System.out.print(" pseud:"+os.getPseudorange(0));
//				System.out.print(" q.ind:"+os.getQualityInd(0));
//				System.out.println(" s.str:"+os.getSignalStrength(0));
//			}
//		}
//		observationsBuffer.add(o);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#getApproxPosition()
	 */
	//@Override
	public Coordinates getMasterPosition() {
		return masterPosition;
	}

//	/* (non-Javadoc)
//	 * @see org.gogpsproject.ObservationsProducer#getCurrentObservations()
//	 */
//	//@Override
//	public Observations getCurrentObservations() {
//		if(obsCursor>=observationsBuffer.size()){
//			if(waitForData){
//				while(obsCursor>=observationsBuffer.size()){
//					if(debug) System.out.print("m");
//					try {
//						Thread.sleep(1000);
//					} catch (InterruptedException e) {}
//				}
//			}else{
//				return null;
//			}
//		}
//		return observationsBuffer.get(obsCursor);
//	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#init()
	 */
	//@Override
	public void init() throws Exception {
		start();
	}

//	/* (non-Javadoc)
//	 * @see org.gogpsproject.ObservationsProducer#nextObservations()
//	 */
//	//@Override
//	public Observations nextObservations() {
//		if(observationsBuffer.size()==0 || (obsCursor+1)>=observationsBuffer.size()){
//			if(waitForData){
//				while(observationsBuffer.size()==0 || (obsCursor+1)>=observationsBuffer.size()){
//					if(debug) System.out.println("\t\t\t\tM cur:"+obsCursor+" pool:"+observationsBuffer.size());
//					try {
//						Thread.sleep(1000);
//					} catch (InterruptedException e) {}
//				}
//			}else{
//				return null;
//			}
//		}
//		Observations o = observationsBuffer.get(++obsCursor);
//		if(debug) System.out.println("\t\t\t\tM < Obs "+o.getRefTime().getMsec());
//        return o;
//	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.ObservationsProducer#release()
	 */
	//@Override
	public void release(boolean waitForThread, long timeoutMs) throws InterruptedException {
		stop(waitForThread, timeoutMs);
	}

	/**
	 * @param initialPosition the initialPosition to set
	 */
	public void setMasterPosition(Coordinates masterPosition) {
		if(debug){
			masterPosition.computeGeodetic();
			System.out.println("Master Position : " + masterPosition);
		}
		this.masterPosition = masterPosition;
		for (StreamEventListener sel : streamEventListeners) {
			sel.setDefinedPosition(masterPosition);
		}
	}

	/**
	 * @return the streamFileLogger
	 */
	public String getStreamFileLogger() {
		return streamFileLogger;
	}

	/**
	 * @param streamFileLogger the streamFileLogger to set
	 */
	public void setStreamFileLogger(String streamFileLogger) {
		this.streamFileLogger = streamFileLogger;
	}

	/**
	 * @return the debug
	 */
	public boolean isDebug() {
		return debug;
	}

	/**
	 * @param debug the debug to set
	 */
	public void setDebug(boolean debug) {
		this.debug = debug;
	}

	/**
	 * @param reconnectionPolicy the reconnectionPolicy to set
	 */
	public void setReconnectionPolicy(int reconnectionPolicy) {
		this.reconnectionPolicy = reconnectionPolicy;
	}

	/**
	 * @return the reconnectionPolicy
	 */
	public int getReconnectionPolicy() {
		return reconnectionPolicy;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#addStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void addStreamEventListener(StreamEventListener streamEventListener) {
		if(streamEventListener==null) return;
		if(!streamEventListeners.contains(streamEventListener))
			this.streamEventListeners.add(streamEventListener);
		// feed defined position
		if(masterPosition!=null)
			streamEventListener.setDefinedPosition(masterPosition);
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#getStreamEventListeners()
	 */
	@SuppressWarnings("unchecked")
	@Override
	public Vector<StreamEventListener> getStreamEventListeners() {
		return (Vector<StreamEventListener>) streamEventListeners.clone();
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventProducer#removeStreamEventListener(org.gogpsproject.StreamEventListener)
	 */
	@Override
	public void removeStreamEventListener(
			StreamEventListener streamEventListener) {
		if(streamEventListener==null) return;
		if(streamEventListeners.contains(streamEventListener))
			this.streamEventListeners.remove(streamEventListener);

		if(exitPolicy == EXIT_ON_LAST_LISTENER_LEAVE && streamEventListeners.size()==0){
			try {
				release(true, 10*1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * @param antennaDescriptor the antennaDescriptor to set
	 */
	public void setAntennaDescriptor(AntennaDescriptor antennaDescriptor) {
		if(debug) System.out.println("Antenna Descriptor : " + antennaDescriptor);
		this.antennaDescriptor = antennaDescriptor;
	}

	/**
	 * @return the antennaDescriptor
	 */
	public AntennaDescriptor getAntennaDescriptor() {
		return antennaDescriptor;
	}

	/**
	 * @param virtualReferenceStationPosition the virtualReferenceStationPosition to set
	 */
	public void setVirtualReferenceStationPosition(
			Coordinates virtualReferenceStationPosition) {
		this.virtualReferenceStationPosition = virtualReferenceStationPosition;
	}

	/**
	 * @return the virtualReferenceStationPosition
	 */
	public Coordinates getVirtualReferenceStationPosition() {
		return virtualReferenceStationPosition;
	}

	public void setMarkerName(String markerName) {
		this.markerName = markerName;
	}
	
	public void setOutputDir(String outDir) {
		this.outputDir = outDir;
	}

	public void setReconnectionWaitingTime(Integer waitingTime) {
		this.reconnectionWaitingTime = waitingTime * 1000;
	}
}
