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
package org.gogpsproject.parser.ublox;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Date;

import org.gogpsproject.EphGps;
import org.gogpsproject.EphemerisSystem;
import org.gogpsproject.IonoGps;
import org.gogpsproject.NavigationProducer;
import org.gogpsproject.Observations;
import org.gogpsproject.SatellitePosition;
import org.gogpsproject.StreamResource;

/**
 * <p>
 * Provide AssistNow service from uBlox as NavigationProducer interface
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com
 */
public class UBXAssistNow extends EphemerisSystem implements NavigationProducer, Runnable{

	public final static String ASSISTNOW_SERVER = "agps.u-blox.com";
	public final static int ASSISTNOW_PORT = 46434;
	public final static String ASSISTNOW_REQUEST = "cmd=${cmd};user=${user};pwd=${pass};lat=${lat};lon=${lon};pacc=${pacc}";

	// delivers Ephemeris and Almanac data and Approximate Time and Position to the client
	public final static String CMD_FULL = "full";
	// identical to "full", but does not deliver Almanac
	public final static String CMD_AID = "aid";
	//  only delivers Ephemeris which is of use to the client at its current location
	public final static String CMD_eph = "eph";
	// delivers Almanac data for the full GPS constellation
	public final static String CMD_ALM = "alm";

	public final static String DEFAULT_PACC = "300000";

	private final static int LAT_MIN = -45;
	private final static int LAT_MAX = +45;
	private final static int LAT_STEP= 90;
	private final static int LON_MIN = -180;
	private final static int LON_MAX =  +90;
	private final static int LON_STEP= 90;


	private String user, pass, cmd, lon=null, lat=null;

	private ArrayList<EphGps> ephs = new ArrayList<EphGps>(); /* GPS broadcast ephemerides */
	private ArrayList<IonoGps> ionos = new ArrayList<IonoGps>(); /* GPS broadcast ionospheric */

	private Thread t = null;

	private long requestSecondDelay = 15*60; // 15 min

	private boolean debug = false;
    private String fileNameOutLog = null;
    private FileOutputStream fosOutLog = null;
    private DataOutputStream outLog = null;//new XMLEncoder(os);


	/**
	 * @param args
	 */
	public static void main(String[] args) {
		UBXAssistNow agps = new UBXAssistNow(args[0], args[1], CMD_AID/*, "8.92", "46.03"*/);

		try {
			//agps.requestSecondDelay = 60;
			agps.setDebug(true);
			agps.setFileNameOutLog("./data/assistnow.dat");

			agps.init();

			Thread.sleep(60*60*1000);
		} catch (Exception e) {
			e.printStackTrace();
		}finally{
			try {
				agps.release(true, 10*1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}


	}


	public UBXAssistNow(String username, String password, String cmd, String lon, String lat){
		this.user = username;
		this.pass = password;
		this.cmd = cmd;
		this.lon = lon;
		this.lat = lat;
	}
	public UBXAssistNow(String username, String password, String cmd){
		this.user = username;
		this.pass = password;
		this.cmd = cmd;
	}

	public ByteArrayOutputStream doRequest(){



		ArrayList<String> lats = new ArrayList<String>();
		ArrayList<String> lons = new ArrayList<String>();

		if(lat!=null&& lon!=null){
			lats.add(lat);
			lons.add(lon);
		}else{
			for(int la=LAT_MIN;la<=LAT_MAX;la+=LAT_STEP){
				for(int lo=LON_MIN;lo<=LON_MAX;lo+=LON_STEP){
					lats.add(""+la);
					lons.add(""+lo);
				}
			}
		}
		ByteArrayOutputStream cache = new ByteArrayOutputStream();

		for(int s=0;s<lats.size();s++){

			String llat = lats.get(s);
			String llon = lons.get(s);

			String request = ASSISTNOW_REQUEST+"\n";
			request = request.replaceAll("\\$\\{cmd\\}", cmd);
			request = request.replaceAll("\\$\\{user\\}", user);
			request = request.replaceAll("\\$\\{pass\\}", pass);
			request = request.replaceAll("\\$\\{lat\\}", llat);
			request = request.replaceAll("\\$\\{lon\\}", llon);
			request = request.replaceAll("\\$\\{pacc\\}", DEFAULT_PACC);


			InputStream is = null;
			OutputStream os = null;
			Socket sck = null;
			try {

				int retry=3;
				while( retry>0 ){


					try {
						/* Open all */
						sck = new Socket(ASSISTNOW_SERVER, ASSISTNOW_PORT);

						os = sck.getOutputStream();
						is = sck.getInputStream();

						if(debug) System.out.println("["+request+"]");
						os.write(request.getBytes("UTF-8"));
			            os.flush();

			            int lenght=-1;
			            String responseLine;
			            boolean start = false;
			            int lines = 0;
			            while ((responseLine = readLine(is)) != null && !start) {
			            	if(debug) System.out.println("["+responseLine+"]");

			            	String key = "Content-Length: ";
			            	if(responseLine.indexOf(key)>-1){

			            		lenght = Integer.parseInt(responseLine.substring(responseLine.indexOf(key)+key.length()));
			            		if(debug) System.out.println("len ["+lenght+"]");

			            	}
			            	key = "Content-Type: application/ubx";
			            	if(responseLine.indexOf(key)>-1){
			            		start = true;
			            	}
			            	key = "error:";
			            	if(responseLine.indexOf(key)>-1){
			            		throw new Exception(responseLine);
			            	}
			            	if(lines++ > 20){
			            		throw new Exception("Read more than 20 lines of header");
			            	}
			            }

			            int init = cache.size();
			            int tot = 0;
			            byte buf[] = new byte[1024];
			            int c = is.read(buf,0,buf.length);
			            while(c>=0){
			            	tot += c;
			            	cache.write(buf,0,c);
			            	if(debug) System.out.println("Read: "+c+" Tot:"+tot);
			                c = is.read(buf,0,buf.length);
			            }
			            if(lenght==(cache.size()-init)){
			            	if(debug) System.out.println("Successfull read");
			            	retry = 0;
			            }else{
			            	if(debug) System.out.println("Read err "+lenght+"!="+(cache.size()-init));
			            	byte tmp[] = cache.toByteArray();
			            	cache.reset();
			            	cache.write(tmp, 0, init);

			            	//cache = null;
			            	Thread.sleep(1000);
			            }
					}catch (IOException e) {
						e.printStackTrace();
				    }catch (Exception e) {
				    	e.printStackTrace();
				    }finally{
		    	    	try{is.close();}catch(Exception ignore){}
		    	    	try{os.close();}catch(Exception ignore){}
		    	    	try{sck.close();}catch(Exception ignore){}
		    	    }
					retry--;
				}
			}catch (Exception e) {
		    	e.printStackTrace();
		    }
		}

//	        if(cache!=null){
//	            // Write to GPS
//
//	            if(coldRestart){
//	            	dialog.setLine(2,"GPS cold start");
//	            	gpsos.write(coldStartMessage);
//	            }
//	            dialog.setLine(2,"Write to GPS");
//	        	gpsos.write(cache.toByteArray());
//	        	dialog.setLine(2,"Write successful");
//	        }else{
//	        	dialog.setLine(2,"Missing A-GPS data");
//	        }
	        // real bytes




		return cache;
	}

	private static String readLine(InputStream is) throws IOException{
		StringBuffer str = new StringBuffer();
		int ch = is.read();
		int chh = ch;
		while (ch != -1 && ch != '\n' && !(chh == '\r' && ch == '\n')) {
			if(ch!='\r') str.append((char) ch);
			chh=ch;
			ch = is.read();
		}
		return str.toString();
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double, double)
	 */
	@Override
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
		long unixTime = obs.getRefTime().getMsec();
		
		EphGps eph = findEph(unixTime, satID);

		if (eph != null) {
			
//			char satType = eph.getSatType();
			
			SatellitePosition sp = computePositionGps(obs, satID, satType, eph, receiverClockError);
			//if(receiverPosition!=null) earthRotationCorrection(receiverPosition, sp);
			return sp;// new SatellitePosition(eph, unixTime, satID, range);
		}
		return null;
	}

	/**
	 * @param unixTime
	 * @param satID
	 * @return Reference ephemeris set for given time and satellite
	 */
	public EphGps findEph(long unixTime, int satID) {

		long dt = 0;
		long dtMin = 0;
		EphGps refEph = null;

		//long gpsTime = (new Time(unixTime)).getGpsTime();

		for (int i = 0; i < ephs.size(); i++) {
			// Find ephemeris sets for given satellite
			if (ephs.get(i).getSatID() == satID) {
				// Compare current time and ephemeris reference time
				dt = Math.abs(ephs.get(i).getRefTime().getMsec() - unixTime );
				// If it's the first round, set the minimum time difference and
				// select the first ephemeris set candidate
				if (refEph == null) {
					dtMin = dt;
					refEph = ephs.get(i);
					// Check if the current ephemeris set is closer in time than
					// the previous candidate; if yes, select new candidate
				} else if (dt < dtMin) {
					dtMin = dt;
					refEph = ephs.get(i);
				}
			}
		}
		return refEph;
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getIono(long)
	 */
	@Override
	public IonoGps getIono(long unixTime) {
		long dt = 0;
		long dtMin = 0;
		IonoGps refIono = null;

		//long gpsTime = (new Time(unixTime)).getGpsTime();

		for (int i = 0; i < ionos.size(); i++) {
			// Find ionospheric sets for given satellite

			// Compare current time and ionospheric reference time
			dt = Math.abs(ionos.get(i).getRefTime().getMsec() - unixTime );
			// If it's the first round, set the minimum time difference and
			// select the first ionospheric set candidate
			if (refIono == null) {
				dtMin = dt;
				refIono = ionos.get(i);
				// Check if the current ionospheric set is closer in time than
				// the previous candidate; if yes, select new candidate
			} else if (dt < dtMin) {
				dtMin = dt;
				refIono = ionos.get(i);
			}

		}
		return refIono;
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#init()
	 */
	@Override
	public void init() throws Exception {
		this.t = new Thread(this);
		this.t.setName("uBlox AssistNow A-GPS");
		t.start();
	}


	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#release(boolean, long)
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs)
			throws InterruptedException {
		Thread tt = t;

		t = null;
		if(waitForThread && tt!=null && tt.isAlive()){
			tt.join(timeoutMs);
		}
	}


	/* (non-Javadoc)
	 * @see java.lang.Runnable#run()
	 */
	@Override
	public void run() {
		long lastRequest = 0;
		while(t!=null && t == Thread.currentThread()){
			long now = System.currentTimeMillis();
			if((now-lastRequest)/1000 > requestSecondDelay){
				lastRequest = now;

				ByteArrayOutputStream os = this.doRequest();
				ByteArrayInputStream is = new ByteArrayInputStream(os.toByteArray());
				UBXReader reader = new UBXReader(is);

				while(is.available()>0){
					int data = is.read();
					if(data == 0xB5){
						try {
							Object msg = reader.readMessage();
							if(msg != null){
								//System.out.println("msg "+msg.getClass().getName());
								if(msg instanceof EphGps){
									if(debug){
										System.out.println("Ephemeris for SatID:"+((EphGps)msg).getSatID()+" time:"+(new Date(((EphGps)msg).getRefTime().getMsec())));
									}
									ephs.add((EphGps)msg);

									if(outLog!=null){
							        	try {
							        		((EphGps)msg).write(outLog);
							        		outLog.flush();
										} catch (IOException e) {
											e.printStackTrace();
										}
							        }
								}
								if(msg instanceof IonoGps){
									if(debug) System.out.println("Iono "+(new Date(((IonoGps)msg).getRefTime().getMsec())));
									ionos.add((IonoGps)msg);
									if(outLog!=null){
							        	try {
							        		((IonoGps)msg).write(outLog);
							        		outLog.flush();
										} catch (IOException e) {
											e.printStackTrace();
										}
							        }
								}
							}else{
								if(debug) System.out.println("msg unknown");
							}
						} catch (IOException e) {
							e.printStackTrace();
						} catch (UBXException e) {
							e.printStackTrace();
						}
					}
				}
			}

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * @param requestSecondDelay the requestSecondDelay to set
	 */
	public void setRequestSecondDelay(long requestSecondDelay) {
		this.requestSecondDelay = requestSecondDelay;
	}


	/**
	 * @return the requestSecondDelay
	 */
	public long getRequestSecondDelay() {
		return requestSecondDelay;
	}


	/**
	 * @param debug the debug to set
	 */
	public void setDebug(boolean debug) {
		this.debug = debug;
	}


	/**
	 * @return the debug
	 */
	public boolean isDebug() {
		return debug;
	}

	/**
	 * @param fileNameOutLog the fileNameOutLog to set
	 * @throws FileNotFoundException
	 */
	public void setFileNameOutLog(String fileNameOutLog) throws FileNotFoundException {
		this.fileNameOutLog = fileNameOutLog;
		if(fileNameOutLog!=null){
    		fosOutLog = new FileOutputStream(fileNameOutLog,true);
    		outLog = new DataOutputStream(fosOutLog);
    	}
	}
	/**
	 * @return the fileNameOutLog
	 */
	public String getFileNameOutLog() {
		return fileNameOutLog;
	}
}
