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
package org.gogpsproject.parser.sp3;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.SimpleTimeZone;
import java.util.TimeZone;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;
import org.gogpsproject.Coordinates;
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
 * Still incomplete
 */
public class SP3Navigation implements NavigationProducer {

	public final static String IGN_FR_ULTRARAPID = "ftp://igs.ensg.ign.fr/pub/igs/products/${wwww}/igu${wwww}${d}_${hh4}.sp3.Z";
	public final static String IGN_FR_RAPID = "ftp://igs.ensg.ign.fr/pub/igs/products/${wwww}/igr${wwww}${d}.sp3.Z";
	public final static String IGN_FR_FINAL = "ftp://igs.ensg.ign.fr/pub/igs/products/${wwww}/igs${wwww}${d}.sp3.Z";

	public String SP3_CACHE = "./sp3-cache";

	private boolean waitForData = true;
	/**
	 * @param args
	 */
	public static void main(String[] args) {

		TimeZone.setDefault(TimeZone.getTimeZone("UTC"));

		Calendar c = Calendar.getInstance();
//		c.set(Calendar.YEAR, 2011);
//		c.set(Calendar.MONTH, 0);
//		c.set(Calendar.DAY_OF_MONTH, 9);
//		c.set(Calendar.HOUR_OF_DAY, 1);
//		c.set(Calendar.MINUTE, 0);
//		c.set(Calendar.SECOND, 0);
//		c.set(Calendar.MILLISECOND, 0);
		c.setTimeZone(new SimpleTimeZone(0,""));

		Time t = new Time(c.getTimeInMillis());

		System.out.println("ts: "+t.getMsec()+" "+(new Date(t.getMsec())));
		System.out.println("week: "+t.getGpsWeek());
		System.out.println("week sec: "+t.getGpsWeekSec());
		System.out.println("week day: "+t.getGpsWeekDay());
		System.out.println("week hour in day: "+t.getGpsHourInDay());
		System.out.println("year: "+t.getYear()+" "+t.getYear2c());
		System.out.println("day of year: "+t.getDayOfYear());


		System.out.println("ts2: "+(new Time(t.getGpsWeek(),t.getGpsWeekSec())).getMsec());

//		SP3Navigation sp3n = new SP3Navigation(IGN_FR_ULTRARAPID);
//		sp3n.init();
//		SatellitePosition sp = sp3n.getGpsSatPosition(c.getTimeInMillis(), 2, 0, null);
//		if(sp!=null){
//			System.out.println("found "+(new Date(sp.getUtcTime()))+" "+(sp.isPredicted()?" predicted":""));
//		}else{
//			System.out.println("Epoch not found "+(new Date(c.getTimeInMillis())));
//		}


	}

	private String urltemplate;
	private HashMap<String,SP3Parser> pool = new HashMap<String,SP3Parser>();

	public SP3Navigation(String urltemplate){
		this.urltemplate = urltemplate;

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getGpsSatPosition(long, int, double)
	 */
	@Override
	public SatellitePosition getGpsSatPosition(Observations obs, int satID, char satType, double receiverClockError) {
		
		long unixTime = obs.getRefTime().getMsec();

		SP3Parser sp3p = null;
		long reqTime = unixTime;

		boolean retrievable = true;

		do{
			// found none, retrieve from urltemplate
			Time t = new Time(reqTime);
			//System.out.print("request: "+unixTime+" "+(new Date(t.getMsec()))+" week:"+t.getGpsWeek()+" "+t.getGpsWeekDay());

			String url = urltemplate.replaceAll("\\$\\{wwww\\}", (new DecimalFormat("0000")).format(t.getGpsWeek()));
			url = url.replaceAll("\\$\\{d\\}", (new DecimalFormat("0")).format(t.getGpsWeekDay()));
			int hh4 = t.getGpsHourInDay();
			if(0<=hh4&&hh4<6) hh4=0;
			if(6<=hh4&&hh4<12) hh4=6;
			if(12<=hh4&&hh4<18) hh4=12;
			if(18<=hh4&&hh4<24) hh4=18;
			url = url.replaceAll("\\$\\{hh4\\}", (new DecimalFormat("00")).format(hh4));



			if(url.startsWith("ftp://")){
				try {
					if(pool.containsKey(url)){
						//System.out.println(url+" from memory cache.");
						sp3p = pool.get(url);
					}else{

						sp3p = getFromFTP(url);
					}
					if(sp3p != null){
						pool.put(url, sp3p);
						// file exist, look for epoch
						if(sp3p.isTimestampInEpocsRange(unixTime)){
							return sp3p.getGpsSatPosition(obs, satID, satType, receiverClockError);
						}else{
							return null;
						}
					}
					System.out.println("Try in 10s");
					try {
						Thread.sleep(1000*10);
					} catch (InterruptedException ee) {}
				} catch (FileNotFoundException e) {
					System.out.println("Try with previous time by 6h");
					reqTime = reqTime - (6L*3600L*1000L);
				}  catch (IOException e) {
					e.printStackTrace();
					System.out.println("Try in 10s");
					try {
						Thread.sleep(1000*10);
					} catch (InterruptedException ee) {}
				}


			}else{
				retrievable = false;
			}
		}while(retrievable && waitForData && sp3p==null);

		return null;
	}

	private SP3Parser getFromFTP(String url) throws IOException{
		SP3Parser sp3p = null;

		String filename = url.replaceAll("[ ,/:]", "_");
		if(filename.endsWith(".Z")) filename = filename.substring(0, filename.length()-2);
		File sp3f = new File(SP3_CACHE,filename);

		if(!sp3f.exists()){
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
					throw new FileNotFoundException();
				}

				if(remoteFile.endsWith(".Z")){
					uis = new UncompressInputStream(is);
				}

				sp3p = new SP3Parser(uis,sp3f);
				sp3p.init();
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
			System.out.println(url+" from cache file "+sp3f);
			sp3p = new SP3Parser(sp3f);
			sp3p.init();
		}
		return sp3p;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.NavigationProducer#getIono(int)
	 */
	@Override
	public IonoGps getIono(long unixTime) {
		return null;//iono[i];
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
