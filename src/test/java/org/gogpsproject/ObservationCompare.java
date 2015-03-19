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
package org.gogpsproject;

import java.io.File;
import java.io.IOException;
import java.util.Calendar;
import java.util.Locale;

import org.gogpsproject.parser.rinex.RinexObservationParser;
import org.gogpsproject.parser.ublox.UBXFileReader;
import org.gogpsproject.producer.rinex.RinexV2Producer;

/**
 * @author Lorenzo Patocchi, cryms.com
 *
 */
public class ObservationCompare {

	/**
	 * @param args
	 */
	public static void main(String[] args) {
		
		//force dot as decimal separator
		Locale.setDefault(new Locale("en", "US"));
		
		Calendar c = Calendar.getInstance();
		int yy = c.get(Calendar.YEAR)-2000;
		int p=0;
		String inFile1 = "P:/eclipse.eriadne/goGPS-tests/data/ubx-partito.out";
		String inFile2 = "P:/eclipse.eriadne/goGPS-tests/data/ubx-partito.out.11o";

		System.out.println("in 1:"+inFile1);
		System.out.println("in 2:"+inFile2);

		ObservationsProducer oIn1 = new UBXFileReader(new File(inFile1));
		try {
			oIn1.init();
		} catch (Exception e) {
			e.printStackTrace();
		}

		ObservationsProducer oIn2 = new RinexObservationParser(new File(inFile2));
		try {
			oIn2.init();
//			oIn2.readFromLog(inFile2,false);
		} catch (Exception e) {
			e.printStackTrace();
		}


		Observations o1 = oIn1.getNextObservations();
		Observations o2 = oIn2.getNextObservations();
		while(o1!=null && o2!=null){
			//r=1 m=2

			double obs1time = o1.getRefTime().getGpsTime();
			//System.out.println("look for M "+o1time);
			while (o1!=null && o2!=null && obs1time > o2.getRefTime().getGpsTime()) {
//				masterIn.skipDataObs();
//				masterIn.parseEpochObs();
				System.out.println("Skip o2 "+o2.getRefTime().getGpsTime());
				o2 = oIn2.getNextObservations();
			}
			//System.out.println("found M "+o1time);

			// Discard rover epochs if correspondent master epochs are
			// not available
			double o2time = o2.getRefTime().getGpsTime();
			//System.out.println("look for R "+o2time);
			while (o2!=null && o1!=null && o1.getRefTime().getGpsTime() < o2time) {
				System.out.println("Skip o1 "+o1.getRefTime().getGpsTime());
				o1 = oIn1.getNextObservations();
			}
			//System.out.println("found R "+o2time);

			if(o2!=null && o1!=null){
				// compare
				System.out.println("Compare: "+o1.getRefTime().getGpsTime()+" "+o2.getRefTime().getGpsTime());

				for(int i=0;i<o1.getNumSat();i++){
					ObservationSet os1 = o1.getSatByIdx(i);
					ObservationSet os2 = o2.getSatByID(os1.getSatID());
					if(os2!=null){
						System.out.println("Sat ID:"+os1.getSatID());
						compareObservationSet(os1.getSatID(),os1, os2);
					}else{
						System.out.println("Missing o2 Sat ID:"+os1.getSatID());
					}
				}
				for(int i=0;i<o2.getNumSat();i++){
					ObservationSet os2 = o2.getSatByIdx(i);
					ObservationSet os1 = o1.getSatByID(os2.getSatID());
					if(os1!=null){

					}else{
						System.out.println("Missing o1 Sat ID:"+os2.getSatID());
					}
				}
			}
			o1 = oIn1.getNextObservations();
			o2 = oIn2.getNextObservations();

		}
		System.out.println("END");

	}

	private static void compareObservationSet(int satID, ObservationSet os1, ObservationSet os2){
		boolean ok=true;
		String line = "";
		if(!equalDouble(os1.getCodeC(0),os2.getCodeC(0))){
			ok=false;
			line += "codeC "+os1.getCodeC(0)+" "+os2.getCodeC(0)+"\n";
		}
		if(!equalDouble(os1.getCodeP(0),os2.getCodeP(0))){
			ok=false;
			System.out.print("codeP ");
		}
		if(!equalDouble(os1.getPhaseCycles(0),os2.getPhaseCycles(0))){
			ok=false;
			System.out.print("phase ");
		}
		if(!equalDouble(os1.getPseudorange(0),os2.getPseudorange(0))){
			ok=false;
			System.out.print("pseudo ");
		}
		if(!equalDouble(os1.getSignalStrength(0),os2.getSignalStrength(0))){
			ok=false;
			System.out.print("snr ");
		}
		if(!equalDouble(os1.getDoppler(0),os2.getDoppler(0))){
			ok=false;
			System.out.print("dop ");
		}
		if(!ok)System.out.println(line);
	}
	private static boolean equalDouble(double d1,double d2){
		if(Double.isNaN(d1) && Double.isNaN(d2)) return true;
		return Math.abs(d1-d2)<0.001;
	}
}
