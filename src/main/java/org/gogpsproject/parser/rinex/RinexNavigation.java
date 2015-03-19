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
package org.gogpsproject.parser.rinex;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.SimpleTimeZone;
import java.util.TimeZone;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;
import org.gogpsproject.Coordinates;
import org.gogpsproject.EphGps;
import org.gogpsproject.IonoGps;
import org.gogpsproject.NavigationProducer;
import org.gogpsproject.Observations;
import org.gogpsproject.StreamResource;
import org.gogpsproject.Time;
import org.gogpsproject.SatellitePosition;
import org.gogpsproject.util.UncompressInputStream;

/**
 * @author Lorenzo Patocchi, cryms.com
 *
 * This class retrieve RINEX file on-demand from known server structures
 *
 */
public class RinexNavigation implements NavigationProducer {

	public final static String GARNER_NAVIGATION_AUTO = "ftp://garner.ucsd.edu/pub/nav/${yyyy}/${ddd}/auto${ddd}0.${yy}n.Z";
	public final static String IGN_MULTI_NAVIGATION_DAILY = "ftp://igs.ign.fr/pub/igs/data/campaign/mgex/daily/rinex3/${yyyy}/${ddd}/brdm${ddd}0.${yy}p.Z";
	public final static String GARNER_NAVIGATION_ZIM2 = "ftp://garner.ucsd.edu/pub/nav/${yyyy}/${ddd}/zim2${ddd}0.${yy}n.Z";
	public final static String IGN_NAVIGATION_HOURLY_ZIM2 = "ftp://igs.ensg.ign.fr/pub/igs/data/hourly/${yyyy}/${ddd}/zim2${ddd}${h}.${yy}n.Z";

	/** cache for negative answers */
	private Hashtable<String,Date> negativeChache = new Hashtable<String, Date>();

	/** Folder containing downloaded files */
	public String RNP_CACHE = "./rnp-cache";

	private boolean waitForData = true;
	/**
	 * @param args
	 */
	public static void main(String[] args) {

		TimeZone.setDefault(TimeZone.getTimeZone("UTC"));

		Calendar c = Calendar.getInstance();
		c.set(Calendar.YEAR, 2011);
		c.set(Calendar.MONTH, 0);
		c.set(Calendar.DAY_OF_MONTH, 9);
		c.set(Calendar.HOUR_OF_DAY, 1);
		c.set(Calendar.MINUTE, 0);
		c.set(Calendar.SECOND, 0);
		c.set(Calendar.MILLISECOND, 0);
		c.setTimeZone(new SimpleTimeZone(0,""));

		Time t = new Time(c.getTimeInMillis());

		System.out.println("ts: "+t.getMsec()+" "+(new Date(t.getMsec())));
		System.out.println("week: "+t.getGpsWeek());
		System.out.println("week sec: "+t.getGpsWeekSec());
		System.out.println("week day: "+t.getGpsWeekDay());
		System.out.println("week hour in day: "+t.getGpsHourInDay());


		System.out.println("ts2: "+(new Time(t.getGpsWeek(),t.getGpsWeekSec())).getMsec());

		RinexNavigation rn = new RinexNavigation(IGN_NAVIGATION_HOURLY_ZIM2);
		rn.init();
//		SatellitePosition sp = rn.getGpsSatPosition(c.getTimeInMillis(), 2, 0, 0);
		Observations obs = new Observations(new Time(c.getTimeInMillis()),0);
		SatellitePosition sp = rn.getGpsSatPosition(obs, 2, 'G', 0);

		if(sp!=null){
			System.out.println("found "+(new Date(sp.getUtcTime()))+" "+(sp.isPredicted()?" predicted":""));
		}else{
			System.out.println("Epoch not found "+(new Date(c.getTimeInMillis())));
		}


	}

	/** Template string where to retrieve files on the net */
	private String urltemplate;
	private HashMap<String,RinexNavigationParser> pool = new HashMap<String,RinexNavigationParser>();

	/**
	 * Instantiates a new RINEX navigation retriever and parser.
	 *
	 * @param urltemplate the template URL where to get the files on the net.
	 */
	public RinexNavigation(String urltemplate){
		this.urltemplate = urltemplate;

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double)
	 */
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {

		long unixTime = obs.getRefTime().getMsec();
		double range = obs.getSatByIDType(satID, satType).getPseudorange(0);
		
		RinexNavigationParser rnp = getRNPByTimestamp(unixTime);
		if(rnp!=null){
			if(rnp.isTimestampInEpocsRange(unixTime)){
				return rnp.getGpsSatPosition(obs, satID, satType, receiverClockError);
			}else{
				return null;
			}
		}

		return null;
	}
	public EphGps findEph(long unixTime, int satID, char satType) {
		long requestedTime = unixTime;
		EphGps eph = null;
		int maxBack = 12;
		while(eph==null && (maxBack--)>0){

			RinexNavigationParser rnp = getRNPByTimestamp(requestedTime);

			if(rnp!=null){
				if(rnp.isTimestampInEpocsRange(unixTime)){
					eph = rnp.findEph(unixTime, satID, satType);
				}
			}
			if(eph==null) requestedTime -= (1L*3600L*1000L);
		}

		return eph;
	}
	private RinexNavigationParser getRNPByTimestamp(long unixTime) {

		RinexNavigationParser rnp = null;
		long reqTime = unixTime;
		boolean retrievable = true;

		do{
			// found none, retrieve from urltemplate
			Time t = new Time(reqTime);
			//System.out.println("request: "+unixTime+" "+(new Date(t.getMsec()))+" week:"+t.getGpsWeek()+" "+t.getGpsWeekDay());

			String url = t.formatTemplate(urltemplate);

			if(url.startsWith("ftp://")){
				try {
					if(pool.containsKey(url)){
						//System.out.println(url+" from memory cache.");
						rnp = pool.get(url);
					}else{
						rnp = getFromFTP(url);
					}
					if(rnp != null){
						pool.put(url, rnp);
						return rnp;
					}
					System.out.println("Try in 10s");
					try {
						Thread.sleep(1000*10);
					} catch (InterruptedException ee) {}
				} catch (FileNotFoundException e) {
					//System.out.println("Try with previous time by 1h");
					reqTime = reqTime - (1L*3600L*1000L);
				}  catch (IOException e) {
					e.printStackTrace();
					System.out.println("Try in 10s");
					try {
						Thread.sleep(1000*10);
					} catch (InterruptedException ee) {}
				}

			}else{
				// no way to get out
				retrievable = false;
			}
		} while(retrievable && waitForData && rnp==null);

		return null;
	}
	private RinexNavigationParser getFromFTP(String url) throws IOException{
		RinexNavigationParser rnp = null;

		String origurl = url;
		if(negativeChache.containsKey(url)){
			if(System.currentTimeMillis()-negativeChache.get(url).getTime() < 60*60*1000){
				throw new FileNotFoundException("cached answer");
			}else{
				negativeChache.remove(url);
			}
		}

		String filename = url.replaceAll("[ ,/:]", "_");
		if(filename.endsWith(".Z")) filename = filename.substring(0, filename.length()-2);
		File rnf = new File(RNP_CACHE,filename);

		if(!rnf.exists()){
			System.out.println(url+" from the net.");
			FTPClient ftp = new FTPClient();

			try {
				int reply;
				System.out.println("URL: "+url);
				url = url.substring("ftp://".length());
				String server = url.substring(0, url.indexOf('/'));
				String remoteFile = url.substring(url.indexOf('/'));
				String remotePath = remoteFile.substring(0,remoteFile.lastIndexOf('/'));
				remoteFile = remoteFile.substring(remoteFile.lastIndexOf('/')+1);


				ftp.connect(server);
				ftp.login("anonymous", "info@eriadne.org");

				System.out.print(ftp.getReplyString());

				// After connection attempt, you should check the reply code to
				// verify
				// success.
				reply = ftp.getReplyCode();

				if (!FTPReply.isPositiveCompletion(reply)) {
					ftp.disconnect();
					System.err.println("FTP server refused connection.");
					return null;
				}

				System.out.println("cwd to "+remotePath+" "+ftp.changeWorkingDirectory(remotePath));
				System.out.println(ftp.getReplyString());
				ftp.setFileType(FTP.BINARY_FILE_TYPE);
				System.out.println(ftp.getReplyString());

				System.out.println("open "+remoteFile);
				InputStream is = ftp.retrieveFileStream(remoteFile);
				InputStream uis = is;
				System.out.println(ftp.getReplyString());
				if(ftp.getReplyString().startsWith("550")){
					negativeChache.put(origurl, new Date());
					throw new FileNotFoundException();
				}

				if(remoteFile.endsWith(".Z")){
					uis = new UncompressInputStream(is);
				}

				rnp = new RinexNavigationParser(uis,rnf);
				rnp.init();
				is.close();


				ftp.completePendingCommand();

				ftp.logout();
			} finally {
				if (ftp.isConnected()) {
					try {
						ftp.disconnect();
					} catch (IOException ioe) {
						// do nothing
					}
				}
			}
		}else{
			System.out.println(url+" from cache file "+rnf);
			rnp = new RinexNavigationParser(rnf);
			rnp.init();
		}
		return rnp;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getIono(int)
	 */
	@Override
	public IonoGps getIono(long unixTime) {
		RinexNavigationParser rnp = getRNPByTimestamp(unixTime);
		if(rnp!=null) return rnp.getIono(unixTime);
		return null;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#init()
	 */
	@Override
	public void init() {

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#release()
	 */
	@Override
	public void release(boolean waitForThread, long timeoutMs) throws InterruptedException {
		waitForData = false;
	}


}
