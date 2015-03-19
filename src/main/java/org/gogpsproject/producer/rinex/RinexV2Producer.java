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
package org.gogpsproject.producer.rinex;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.text.DecimalFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.TimeZone;
import java.util.Vector;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import org.gogpsproject.Coordinates;
import org.gogpsproject.EphGps;
import org.gogpsproject.IonoGps;
import org.gogpsproject.ObservationSet;
import org.gogpsproject.Observations;
import org.gogpsproject.StreamEventListener;
/**
 * <p>
 * Produces Rinex 2 as StreamEventListener
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com
 */
import org.gogpsproject.Time;

/**
 * @author Lorenzo
 *
 */
public class RinexV2Producer implements StreamEventListener {

	private String outFilename;
	private boolean headerWritten;

	private Coordinates approxPosition = null;

	private Vector<Observations> observations = new Vector<Observations>();

	private boolean needApproxPos=false;
	private boolean singleFreq=false;
	private boolean standardFilename=true;

	private FileOutputStream fos = null;
	private PrintStream ps = null;

	private ArrayList<Type> typeConfig = new ArrayList<Type>();

	private SimpleDateFormat sdfHeader = new SimpleDateFormat("dd-MMM-yy HH:mm:ss");
	private DecimalFormat dfX3 = new DecimalFormat("0.000");
	private DecimalFormat dfX7 = new DecimalFormat("0.0000000");
	private DecimalFormat dfX = new DecimalFormat("0");
	private DecimalFormat dfXX = new DecimalFormat("00");
	private DecimalFormat dfX4 = new DecimalFormat("0.0000");
	private String marker;
	private int DOYold = -1;
	private String outputDir = "./test";
	private boolean enableZip = false;

	private final static TimeZone TZ = TimeZone.getTimeZone("GMT");

	public RinexV2Producer(boolean needApproxPos, boolean singleFreq, String marker){
		
		this.needApproxPos = needApproxPos;
		this.singleFreq = singleFreq;
		this.marker = marker;

		// set observation type config
		typeConfig.add(new Type(Type.C,1));
		if (!this.singleFreq) {
			typeConfig.add(new Type(Type.P,1));
		}
		typeConfig.add(new Type(Type.L,1));
		typeConfig.add(new Type(Type.D,1));
		typeConfig.add(new Type(Type.S,1));
		if (!this.singleFreq) {
			typeConfig.add(new Type(Type.P,2));
			typeConfig.add(new Type(Type.L,2));
			typeConfig.add(new Type(Type.D,2));
			typeConfig.add(new Type(Type.S,2));
		}

	}
	
	public RinexV2Producer(boolean needApproxPos, boolean singleFreq){
		this(needApproxPos, singleFreq, null);
		this.standardFilename=false;
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#addEphemeris(org.gogpsproject.EphGps)
	 */
	@Override
	public void addEphemeris(EphGps eph) {

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#addIonospheric(org.gogpsproject.IonoGps)
	 */
	@Override
	public void addIonospheric(IonoGps iono) {

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#addObservations(org.gogpsproject.Observations)
	 */
	@Override
	public void addObservations(Observations o) {
		synchronized (this) {
			Time epoch = o.getRefTime();
			int DOY = epoch.getDayOfYear();
			if (this.standardFilename && (this.outFilename == null || this.DOYold != DOY)) {
					streamClosed();
					
					if (this.enableZip && this.outFilename != null) {
						byte[] buffer = new byte[1024];
						 
				    	try{
				 
				    		String zn = this.outFilename + ".zip";
				    		FileOutputStream fos = new FileOutputStream(zn);
				    		ZipOutputStream zos = new ZipOutputStream(fos);
				    		String [] tokens = this.outFilename.split("/|\\\\");
				    		String fn = "";
				    		if (tokens.length > 0) {
				    			fn = tokens[tokens.length-1].trim();
				    		}
				    		ZipEntry ze= new ZipEntry(fn);
				    		zos.putNextEntry(ze);
				    		FileInputStream in = new FileInputStream(this.outFilename);
				 
				    		int len;
				    		while ((len = in.read(buffer)) > 0) {
				    			zos.write(buffer, 0, len);
				    		}
				 
				    		in.close();
				    		zos.closeEntry();
				    		zos.close();
				    		
				    		File file = new File(this.outFilename);
				    		file.delete();
				    		
				    		System.out.println("--RINEX file compressed as "+zn);
				 
				    	}catch(IOException ex){
				    	   ex.printStackTrace();
				    	}
					}
					
					File file = new File(outputDir);
					if(!file.exists() || !file.isDirectory()){
					    boolean wasDirectoryMade = file.mkdirs();
					    if(wasDirectoryMade)System.out.println("Directory "+outputDir+" created");
					    else System.out.println("Could not create directory "+outputDir);
					}

					char session = '0';
					int year = epoch.getYear2c();
					String outFile = outputDir + "/" + marker + String.format("%03d", DOY) + session + "." + year + "o";
					File f = new File(outFile);
					
					while (f.exists()){
		    			session++;
		    			outFile = outputDir + "/" +  marker + String.format("%03d", DOY) + session + "." + year + "o";
		    			f = new File(outFile);
		    		}

					System.out.println("Started writing RINEX file "+outFile);
					setFilename(outFile);

					DOYold = DOY;
					
					headerWritten = false;
			}
			if(!headerWritten){
				observations.add(o);
				if(needApproxPos && approxPosition==null){
					return;
				}

				try {
					writeHeader(approxPosition, observations.firstElement());
				} catch (IOException e) {
					e.printStackTrace();
				}

				for(Observations obs:observations){
					try {
						writeObservation(obs);
					} catch (IOException e) {
						e.printStackTrace();
					}
				}

				observations.removeAllElements();
				headerWritten = true;
			}else{
				try {
					writeObservation(o);
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}

	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#setDefinedPosition(org.gogpsproject.Coordinates)
	 */
	@Override
	public void setDefinedPosition(Coordinates definedPosition) {
		synchronized (this) {
			this.approxPosition = definedPosition;
		}
	}

	/* (non-Javadoc)
	 * @see org.gogpsproject.StreamEventListener#streamClosed()
	 */
	@Override
	public void streamClosed() {
		try {
			ps.close();
		} catch (Exception e) {

		}
		try {
			fos.close();
		} catch (Exception e) {

		}
	}

	private void writeHeader(Coordinates approxPosition,Observations firstObservation) throws IOException{

//	          2              OBSERVATION DATA    G (GPS)             RINEX VERSION / TYPE
//	     CCRINEXO V2.4.1 LH  Bernese             28-APR-08 17:51     PGM / RUN BY / DATE
//	     TPS2RIN 1.40        GEOMATICA/IREALP    28-APR-08 12:59     COMMENT
//	     BUILD FEB  4 2004 (C) TOPCON POSITIONING SYSTEMS            COMMENT
//	     D:\GPSCOMO\TB01H\2008\04\28\CO6C79~1.JPS                    COMMENT
//	     SE TPS 00000000                                             COMMENT
//	     COMO                                                        MARKER NAME
//	     12761M001                                                   MARKER NUMBER
//	     GEOMATICA/IREALP    MILANO POLYTECHNIC                      OBSERVER / AGENCY
//	     8PRM6AZ2EBK         TPS ODYSSEY_E       3.1 JAN,24,2007 P1  REC # / TYPE / VERS
//	     217-0400            TPSCR3_GGD      CONE                    ANT # / TYPE
//	       4398306.2809   704149.8723  4550154.6777                  APPROX POSITION XYZ
//	             0.2134        0.0000        0.0000                  ANTENNA: DELTA H/E/N
//	          1     1                                                WAVELENGTH FACT L1/2
//	          7    C1    P1    P2    L1    L2    D1    D2            # / TYPES OF OBSERV
//	          1                                                      INTERVAL
//	       2008     4    28    12     0    0.000000                  TIME OF FIRST OBS
//	                                                                 END OF HEADER
		writeLine (sf("",5)+sf("2",15)+sf("OBSERVATION DATA",20)+sf("G (GPS)",20)+se("RINEX VERSION / TYPE",20), false);
		appendLine(sf("goGPS-java",20)+sf("",20)+sf(sdfHeader.format(Calendar.getInstance(TZ).getTime()).toUpperCase(),20)+se("PGM / RUN BY / DATE",20));
		appendLine(sf("",20*3)+se("MARKER NAME",20));
		appendLine(sf("",20*3)+se("MARKER NUMBER",20));
		appendLine(sf("",20*3)+se("OBSERVER / AGENCY",20));
		appendLine(sf("",20*3)+se("REC # / TYPE / VERS",20));
		appendLine(sf("",20*3)+se("ANT # / TYPE",20));
		if(approxPosition != null){
			appendLine(sp(dfX4.format(approxPosition.getX()),14,1)+sp(dfX4.format(approxPosition.getY()),14,1)+sp(dfX4.format(approxPosition.getZ()),14,1)+sf("",18)+se("APPROX POSITION XYZ",20));
		}else{
			appendLine(sp(dfX4.format(0.0),14,1)+sp(dfX4.format(0.0),14,1)+sp(dfX4.format(0.0),14,1)+sf("",18)+se("APPROX POSITION XYZ",20));
		}
		appendLine(sp(dfX4.format(0.0),14,1)+sp(dfX4.format(0.0),14,1)+sp(dfX4.format(0.0),14,1)+sf("",18)+se("ANTENNA: DELTA H/E/N",20));
		boolean found = false;
		for (Type t:typeConfig) {
		    if (t.toString().equals("L2")) {
		        found = true;
		        break;
		    }
		}
		int wf1 = 2; int wf2 = 0; //single frequency (hypothesizing ublox-type low-cost receiver, i.e. with half cycle ambiguities)
		if(found) {
			wf1 = 1; wf2 = 1; //dual frequency (hypothesizing full cycle ambiguities)
		}
		appendLine(sp(dfX.format(wf1),6,1)+sp(dfX.format(wf2),6,1)+sf("",6)+sf("",6)+sf("",6)+sf("",6)+sf("",6)+sf("",6)+sf("",12)+se("WAVELENGTH FACT L1/2",20));

		String line = "";
		int cols=60;
		line += sp(dfX.format(typeConfig.size()),6,1); cols -= 6;
		for(Type t:typeConfig){
			line += sp(t.toString(),6,1);cols -= 6;
		}
		line += se("",cols);
		line += se("# / TYPES OF OBSERV",20);

		appendLine(line);
		//appendLine(sp(dfX.format(1),6,1)+sf("",60-1*6)+se("INTERVAL",20));

		if(firstObservation!=null){
			Calendar c = Calendar.getInstance(TZ);
			c.setTimeInMillis(firstObservation.getRefTime().getMsec());
			appendLine(sp(dfX.format(c.get(Calendar.YEAR)),6,1)
					+sp(dfX.format(c.get(Calendar.MONTH)+1),6,1)
					+sp(dfX.format(c.get(Calendar.DATE)),6,1)
					+sp(dfX.format(c.get(Calendar.HOUR_OF_DAY)),6,1)
					+sp(dfX.format(c.get(Calendar.MINUTE)),6,1)
					+sp(dfX7.format(c.get(Calendar.SECOND)+c.get(Calendar.MILLISECOND)/1000.0),13,1)
					+sp("GPS",8,1)+sf("",9)+se("TIME OF FIRST OBS",20));
		}

		appendLine(sf("",60)+se("END OF HEADER",20));
	}


/**
	 * @return the typeConfig
	 */
	@SuppressWarnings("unchecked")
	public ArrayList<Type> getTypeConfig() {
		return (ArrayList<Type>)typeConfig.clone();
	}

	/**
	 * @param typeConfig the typeConfig to set
	 * @throws Exception if header has been already written in file, is not permitted to change config in middle.
	 */
	public void setTypeConfig(ArrayList<Type> typeConfig) throws Exception {
		if(headerWritten) throw new Exception("Header already written.");
		this.typeConfig = typeConfig;
	}

	//	 10  3  2 13 20  0.0000000  0 12G11G13G32G04G20G17G23R15R20R21R05R04
//	  24501474.376                    24501481.324   128756223.92705 100329408.23106
//	        42.000          21.000
//	  20630307.428                    20630311.680   108413044.43906  84477784.53807
//	        47.000          39.000
//	  23383145.918                    23383151.148   122879286.75106  95750102.77806
//	        43.000          27.000
//	  21517480.617                    21517485.743   113075085.94506  88110522.62107
//	        46.000          37.000
//	  20912191.322                    20912194.474   109894342.71907  85631979.97507
//	        51.000          39.000
//	  23257094.649                    23257100.632   122216783.06806  95233917.17406
//	        43.000          27.000
//	  20161355.093                    20161358.227   105948754.59007  82557486.35607
//	        51.000          40.000
//	  20509222.093    20509221.179    20509227.404   109595309.53707  85240851.57508
//	        49.000          44.000
//	  23031155.571    23031156.184    23031167.285   123157678.29405  95789331.13107
//	        41.000          38.000
//	  21173954.694    21173954.486    21173960.635   113305812.57507  88126783.03008
//	        49.000          45.000
//	  20128038.579    20128037.431    20128044.524   107596015.91107  83685880.30708
//	        52.000          46.000
//	  23792524.319    23792523.288    23792533.414   127408153.57805  99095296.49707
//	        40.000          37.000
	private void writeObservation(Observations o) throws IOException{
		//System.out.println(o);
		Calendar c = Calendar.getInstance(TZ);
		c.setTimeInMillis(o.getRefTime().getMsec());

		String line = "";
		line += sp(dfX.format(c.get(Calendar.YEAR)-2000),3,1);
		line += sp(dfX.format(c.get(Calendar.MONTH)+1),3,1);
		line += sp(dfX.format(c.get(Calendar.DATE)),3,1);
		line += sp(dfX.format(c.get(Calendar.HOUR_OF_DAY)),3,1);
		line += sp(dfX.format(c.get(Calendar.MINUTE)),3,1);
		line += sp(dfX7.format(c.get(Calendar.SECOND)+c.get(Calendar.MILLISECOND)/1000.0+o.getRefTime().getFraction()/1000),11,1);
		line += sp(dfX.format(o.getEventFlag()),3,1);
		int gpsSize = 0;
		for(int i=0;i<o.getNumSat();i++){
			if(o.getSatByIdx(i).getSatID()<=32){
				gpsSize++;
			}
		}
		line += sp(dfX.format(gpsSize),3,1);
		int cnt=0;
		for(int i=0;i<o.getNumSat();i++){
			if(cnt >= 12 && cnt%12 == 0){
				writeLine(line, true);
				line = "                                ";
			}
			line += o.getSatByIdx(i).getSatType()+dfXX.format(o.getSatID(i));
			cnt++;
		}
		writeLine(line, true);

		for(int i=0;i<o.getNumSat();i++){
			if(o.getSatByIdx(i).getSatID()<=32){ // skip non GPS IDs
				ObservationSet os = o.getSatByIdx(i);
				line = "";
				cnt=0;
				for(Type t:typeConfig){
					switch(t.getType()){
					case Type.C:
						if (os.getCodeC(t.getFrequency()-1) == 0) os.setCodeC(t.getFrequency()-1, Double.NaN);
						line += Double.isNaN(os.getCodeC(t.getFrequency()-1))?sf("",16):sp(dfX3.format(os.getCodeC(t.getFrequency()-1)),14,1)+"  ";
						break;
					case Type.P:
						if (os.getCodeP(t.getFrequency()-1) == 0) os.setCodeP(t.getFrequency()-1, Double.NaN);
						line += Double.isNaN(os.getCodeP(t.getFrequency()-1))?sf("",16):sp(dfX3.format(os.getCodeP(t.getFrequency()-1)),14,1)+"  ";
						break;
					case Type.L:
						if (os.getPhaseCycles(t.getFrequency()-1) == 0) os.setPhaseCycles(t.getFrequency()-1, Double.NaN);
						line += Double.isNaN(os.getPhaseCycles(t.getFrequency()-1))?sf("",14):sp(dfX3.format(os.getPhaseCycles(t.getFrequency()-1)),14,1); // L
						line += os.getLossLockInd(t.getFrequency()-1)<0?" ":dfX.format(os.getLossLockInd(t.getFrequency()-1)); // L1 Loss of Lock Indicator
						line += Float.isNaN(os.getSignalStrength(t.getFrequency()-1))?" ":dfX.format(Math.floor(os.getSignalStrength(t.getFrequency()-1)/6)); // L1 Signal Strength Indicator
						break;
					case Type.D:
						if (os.getDoppler(t.getFrequency()-1) == 0) os.setDoppler(t.getFrequency()-1, Float.NaN);
						line += Float.isNaN(os.getDoppler(t.getFrequency()-1))?sf("",16):sp(dfX3.format(os.getDoppler(t.getFrequency()-1)),14,1)+"  ";
						break;
					case Type.S:
						if (os.getSignalStrength(t.getFrequency()-1) == 0) os.setSignalStrength(t.getFrequency()-1, Float.NaN);
						line += Float.isNaN(os.getSignalStrength(t.getFrequency()-1))?sf("",16):sp(dfX3.format(os.getSignalStrength(t.getFrequency()-1)),14,1)+"  ";
						break;
					}
					cnt++;
					if(cnt==typeConfig.size() || cnt==5){
						writeLine(line, true);
						line = "";
						cnt = 0;
					}
				}
				if (typeConfig.size() > 5) {
					writeLine(line, true);
				}
			}
		}

	}

	// space end
	private String se(String in, int max){
		return sf(in,max,0);
	}
	// space fill with 1 space margin
	private String sf(String in, int max){
		return sf(in,max,1);
	}
	// space fill with margin
	private String sf(String in, int max,int margin){
		if(in.length()==max-margin){
			while(in.length()<max) in +=" ";
			return in;
		}
		if(in.length()>max-margin){
			return in.substring(0, max-margin)+" ";
		}
		while(in.length()<max) in +=" ";

		return in;
	}
	// space prepend with margin
	private String sp(String in, int max,int margin){
		if(in.length()==max-margin){
			while(in.length()<max) in =" "+in;
			return in;
		}
		if(in.length()>max-margin){
			return in.substring(0, max);
			//return in.substring(0, max-margin)+" ";
		}
		while(in.length()<max) in =" "+in;

		return in;
	}

	private void appendLine(String line) throws IOException{
		writeLine(line, true);
	}
	private void writeLine(String line, boolean append) throws IOException{

		FileOutputStream fos = this.fos;
		PrintStream ps = this.ps;
		if(this.fos == null){
			fos = new FileOutputStream(outFilename, append);
			ps = new PrintStream(fos);
		}

		ps.println(line);
		//System.out.println(line);

		ps.flush();
		if(this.fos == null){
			ps.close();
			fos.close();
		}


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

	public void setFilename(String outFilename) {
		this.outFilename = outFilename;
		try {
			fos = new FileOutputStream(outFilename, false);
			ps = new PrintStream(fos);
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
	}
	
	public void setOutputDir(String outDir) {
		this.outputDir = outDir;
	}
	
	public void enableCompression(boolean enableZip) {
		this.enableZip = enableZip;
	}
}
