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
package org.gogpsproject.producer.parser.rinex;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.net.CookieHandler;
import java.net.CookieManager;
import java.net.CookiePolicy;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Base64;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.Hashtable;
import java.util.SimpleTimeZone;
import java.util.TimeZone;
import java.util.zip.GZIPInputStream;

import javax.net.ssl.HttpsURLConnection;

import org.apache.commons.net.ftp.FTP;
import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;
import org.gogpsproject.ephemeris.EphGps;
import org.gogpsproject.producer.parser.IonoGps;
import org.gogpsproject.producer.parser.rinex.RinexNavigation;
import org.gogpsproject.producer.parser.rinex.RinexNavigationParser;
import org.gogpsproject.positioning.SatellitePosition;
import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.Observations;
import org.gogpsproject.producer.StreamResource;
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
	public final static String NASA_NAVIGATION_DAILY = "ftp://cddis.gsfc.nasa.gov/pub/gps/data/daily/${yyyy}/${ddd}/${yy}n/brdc${ddd}0.${yy}n.Z";
	public final static String NASA_NAVIGATION_DAILY_HTTP = "https://cddis.nasa.gov/archive/gnss/data/daily/${yyyy}/${ddd}/${yy}n/brdc${ddd}0.${yy}n.gz";
	public final static String NASA_NAVIGATION_HOURLY = "ftp://cddis.gsfc.nasa.gov/pub/gps/data/hourly/${yyyy}/${ddd}/hour${ddd}0.${yy}n.Z";
  public final static String GARNER_NAVIGATION_AUTO_HTTP = "http://garner.ucsd.edu/pub/rinex/${yyyy}/${ddd}/auto${ddd}0.${yy}n.Z"; // ex http://garner.ucsd.edu/pub/rinex/2016/034/auto0340.16n.Z

	/** cache for negative answers */
	private Hashtable<String,Date> negativeChache = new Hashtable<String, Date>();

	/** Folder containing downloaded files */
	public String RNP_CACHE = "./rnp-cache";

	private boolean waitForData = true;
	
	private String username = null;
	private String password = null;
	/**
	 * @param args
	 */
//	public static void main(String[] args) {
//
//		TimeZone.setDefault(TimeZone.getTimeZone("UTC"));
//
//		Calendar c = Calendar.getInstance();
//		c.set(Calendar.YEAR, 2011);
//		c.set(Calendar.MONTH, 0);
//		c.set(Calendar.DAY_OF_MONTH, 9);
//		c.set(Calendar.HOUR_OF_DAY, 1);
//		c.set(Calendar.MINUTE, 0);
//		c.set(Calendar.SECOND, 0);
//		c.set(Calendar.MILLISECOND, 0);
//		c.setTimeZone(new SimpleTimeZone(0,""));
//
//		Time t = new Time(c.getTimeInMillis());
//
//		System.out.println("ts: "+t.getMsec()+" "+(new Date(t.getMsec())));
//		System.out.println("week: "+t.getGpsWeek());
//		System.out.println("week sec: "+t.getGpsWeekSec());
//		System.out.println("week day: "+t.getGpsWeekDay());
//		System.out.println("week hour in day: "+t.getGpsHourInDay());
//
//
//		System.out.println("ts2: "+(new Time(t.getGpsWeek(),t.getGpsWeekSec())).getMsec());
//
//		RinexNavigation rn = new RinexNavigation(IGN_NAVIGATION_HOURLY_ZIM2);
//		rn.init();
////		SatellitePosition sp = rn.getGpsSatPosition(c.getTimeInMillis(), 2, 0, 0);
//		Observations obs = new Observations(new Time(c.getTimeInMillis()),0);
//		SatellitePosition sp = rn.getGpsSatPosition(obs, 2, 'G', 0);
//
//		if(sp!=null){
//			System.out.println("found "+(new Date(sp.getUtcTime()))+" "+(sp.isPredicted()?" predicted":""));
//		}else{
//			System.out.println("Epoch not found "+(new Date(c.getTimeInMillis())));
//		}
//
//
//	}

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
      if(eph==null) 
      	requestedTime -= (1L*3600L*1000L);
		}

		return eph;
	}
	
	/* Convenience method for adding an rnp to memory cache*/
  public void put(long reqTime, RinexNavigationParser rnp) {
    Time t = new Time(reqTime);
     String url = t.formatTemplate(urltemplate);
     if(!pool.containsKey(url))
       pool.put(url, rnp);
   }
   
  public void setUsername( String username ) {
  	this.username = username;
  }
  
  public void setPassword( String password ) {
  	this.password = password;
  }
  
	protected RinexNavigationParser getRNPByTimestamp(long unixTime) {

		RinexNavigationParser rnp = null;
		long reqTime = unixTime;

		do{
			// found none, retrieve from urltemplate
			Time t = new Time(reqTime);
			//System.out.println("request: "+unixTime+" "+(new Date(t.getMsec()))+" week:"+t.getGpsWeek()+" "+t.getGpsWeekDay());

			String url = t.formatTemplate(urltemplate);

      try {
        if(pool.containsKey(url)){
          rnp = pool.get(url);
        }else{
        	if(url.toLowerCase().startsWith("http"))
            rnp = getFromHTTP(url);
          else if(url.toLowerCase().startsWith("ftp"))
            rnp = getFromFTP(url);
          else 
            throw new RuntimeException("Invalid url template " + url);
    
          if(rnp != null){
            pool.put(url, rnp);
          }
        }
        return rnp;
      } catch( IOException e) {
				  System.out.println( e.getClass().getName() + " url: " + url);
				  return null;
      }

		} while( waitForData && rnp==null);
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

		if(rnf.exists()){
      System.out.println(url+" from cache file "+rnf);
      rnp = new RinexNavigationParser(rnf);
      try{
        rnp.init();
        return rnp;
      }
      catch( Exception e ){
        rnf.delete();
      }
		}
		
		// if the file doesn't exist of is invalid
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

			ftp.enterLocalPassiveMode();
			ftp.setRemoteVerificationEnabled(false);

			System.out.println("cwd to "+remotePath+" "+ftp.changeWorkingDirectory(remotePath));
			System.out.println(ftp.getReplyString());
			ftp.setFileType(FTP.BINARY_FILE_TYPE);
			System.out.println(ftp.getReplyString());

			System.out.println("open "+remoteFile);
			InputStream is = ftp.retrieveFileStream(remoteFile);
			System.out.println(ftp.getReplyString());
			if(ftp.getReplyString().startsWith("550")){
				negativeChache.put(origurl, new Date());
				throw new FileNotFoundException();
			}
      InputStream uis = is;

			if(remoteFile.endsWith(".Z")){
				uis = new UncompressInputStream(is);
			}

			rnp = new RinexNavigationParser(uis,rnf);
			rnp.init();
			is.close();


			ftp.completePendingCommand();

			ftp.logout();
		} 
		finally {
			if (ftp.isConnected()) {
				try {
					ftp.disconnect();
				} catch (IOException ioe) {
					// do nothing
				}
			}
		}
		return rnp;
	}

  private RinexNavigationParser getFromHTTP(String tUrl) throws IOException{
    RinexNavigationParser rnp = null;

    if(negativeChache.containsKey(tUrl)){
      if(System.currentTimeMillis()-negativeChache.get(tUrl).getTime() < 60*60*1000){
        throw new FileNotFoundException("cached answer");
      }else{
        negativeChache.remove(tUrl);
      }
    }

    String filename = tUrl.replaceAll("[ ,/:]", "_");
    if(filename.endsWith(".Z")) filename = filename.substring(0, filename.length()-2);
    File rnf = new File(RNP_CACHE,filename);

    if(rnf.exists()){
      System.out.println(tUrl+" from cache file "+rnf);
      rnp = new RinexNavigationParser(rnf);
      rnp.init();
    }
    else {
    	System.out.println(tUrl+" from the net.");
      tUrl = tUrl.substring("http://".length());
      String remoteFile = tUrl.substring(tUrl.indexOf('/'));
      remoteFile = remoteFile.substring(remoteFile.lastIndexOf('/')+1);
      
    	InputStream is = null;
    	if( urltemplate.equals(RinexNavigation.NASA_NAVIGATION_DAILY_HTTP)) {
    		if( username==null || password== null ) {
    			throw new IOException("Username and password needed");
    		}
    	
				try {
		      String resource = "https:/" + tUrl;
					is = getCDDISResource(resource, username, password);
					
				} catch (Exception e1) {
					// TODO Auto-generated catch block
					e1.printStackTrace();
					throw new IOException(e1);
				}
    	}
			else
    	{
	      URL url = new URL("http://" + tUrl);
	      HttpURLConnection con = (HttpURLConnection) url.openConnection();
	      con.setRequestMethod("GET");
	
	//      con.setRequestProperty("Authorization", "Basic "+ new String(Base64.encode(new String("anonymous:info@eriadne.org"))));
	      con.setRequestProperty("Authorization", "Basic "+ new String(Base64.getEncoder().encode((new String("anonymous:info@eriadne.org").getBytes()))));
	
	      int reply = con.getResponseCode();
	
	      if (reply>200) {
	        if( reply == 404 )
	          System.err.println("404 Not Found");
	        else
	          System.err.println("HTTP server refused connection.");
	//        System.out.print(new String(res.getContent()));
	
	        return null;
	      }

	      is  = con.getInputStream();
    	}
    	
    	try {
    		
        if(remoteFile.endsWith(".Z")){
            InputStream uis = new UncompressInputStream(is);
            rnp = new RinexNavigationParser(uis,rnf);
            rnp.init();
            uis.close();
        }
        else if( remoteFile.endsWith(".gz") ) {
//          	FileInputStream fin = new FileInputStream("archive.tar.Z");
//          	BufferedInputStream in = new BufferedInputStream(fin);
//          	FileOutputStream out = new FileOutputStream("archive.tar");
            InputStream uis = new GZIPInputStream(is);
            rnp = new RinexNavigationParser(uis,rnf);
            rnp.init();
  //        Reader decoder = new InputStreamReader(gzipStream, encoding);
  //        BufferedReader buffered = new BufferedReader(decoder);
            uis.close();
          }
        else {
          rnp = new RinexNavigationParser(is,rnf);
          rnp.init();
          is.close();
        }
      }
      catch(IOException e ){
        e.printStackTrace();
        // delete file, maybe it's corrupt
        rnf.delete();
      }
    }
    return rnp;
  }
  
  /* 
   * Return an input stream for a designated resource on a URS-authenticated remote server.
   * See https://cddis.nasa.gov/Data_and_Derived_Products/CDDIS_Archive_Access.html
   */
  public InputStream getCDDISResource ( String resource, String username, String password) throws Exception {
  	
  	/* Set up a cookie handle, required */
  	CookieHandler.setDefault(new CookieManager(null, CookiePolicy.ACCEPT_ALL));
  	
  	/* Set location for Earthdata Login */
  	final String URS = "https://urs.earthdata.nasa.gov";
  	
	  int redirects = 0;
	  /* Place an upper limit on the number of redirects we will follow */
	  while( redirects < 10 ) {
		  ++redirects;
		
		  /* Configure a connection to the resource server and submit the request for our resource. */
		  URL url = new URL(resource);
		  
		  HttpsURLConnection connection = null;
		  if (url.getProtocol().equalsIgnoreCase("https")) {
		  	connection = (HttpsURLConnection) url.openConnection();
		  }
		  /* Handle any redirect that goes to http - set it back to an https request */
		  else {
		  	connection = (HttpsURLConnection)new URL("https", url.getAuthority(), url.getFile()).openConnection();
		  }

			connection.setRequestMethod("GET");
		  connection.setInstanceFollowRedirects(false);
		  connection.setUseCaches(false);
		  connection.setDoInput(true);
		
		  /* If this is the URS server, add in the authentication header. */
		  if( resource.startsWith(URS) ) {
			  connection.setDoOutput(true);
			  connection.setRequestProperty (
			  "Authorization",
			  "Basic " + Base64.getEncoder().encodeToString (
			  		(username + ":" + password).getBytes()));
		  }
		
		  /*Execute the request and get the response status code. A return status code of 200 is good - it means that we have our resource. We can return the input stream so it can be read (may also want to return additional header information such as the mime type or size). */
		  int status = connection.getResponseCode();
		  if( status == 200 ) {
		  	return connection.getInputStream();
		  }
		
		  /* Any returned status code other than 302 (a redirect) will need custom handling. A 401 code means that the credentials aren't valid. A 403 code means that the user hasn't authorized the application. */
		  if( status != 302 ) {
			  throw new Exception(
			  "Invalid response from server - status " + status);
		  }
		
		  /* Get the redirection location and continue. This should really have a null check, just in case. */
		  resource = connection.getHeaderField("Location");
		  
//		  System.out.println("Status: " + status + " Redirect to: " + connection.getHeaderField("Location") );
		  }
		
		/* If we get to this point, we have exceeded our redirect limit. This is most likely a configuration problem somewhere on the remote server. */
		throw new Exception("Redirection limit exceeded");
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
