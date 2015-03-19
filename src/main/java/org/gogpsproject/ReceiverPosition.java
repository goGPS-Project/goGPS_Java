/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland, Daisuke Yoshida. All Rights Reserved.
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
package org.gogpsproject;
import java.util.ArrayList;

import org.ejml.simple.SimpleMatrix;

/**
 * <p>
 * Receiver position class
 * </p>
 *
 * @author Eugenio Realini, Cryms.com, Daisuke Yoshida
 */
public class ReceiverPosition extends Coordinates{

	/* Satellites */
	private int pivot; /* Index of the satellite with highest elevation in satAvail list */
	private ArrayList<Integer> satAvail; /* List of satellites available for processing */
	private ArrayList<Character> satTypeAvail; /* List of satellite Types available for processing */
	private ArrayList<String> gnssAvail;  /* List of satellite Types & Id available for processing */
	
	private ArrayList<Integer> satAvailPhase; /* List of satellites available for processing */
	private ArrayList<Character> satTypeAvailPhase; /* List of satellite Type available for processing */
	private ArrayList<String> gnssAvailPhase;  /* List of satellite Types & Id available for processing */
	
	private SatellitePosition[] pos; /* Absolute position of all visible satellites (ECEF) */

	// Fields related to the receiver position
	private SimpleMatrix positionCovariance; /* Covariance matrix of the position estimation error */
	private double receiverClockError; /* Clock error */
	private double pDop; /* Position dilution of precision (PDOP) */
	private double hDop; /* Horizontal dilution of precision (HDOP) */
	private double vDop; /* Vertical dilution of precision (VDOP) */
	private double kpDop; /* Kalman-derived position dilution of precision (KPDOP) */
	private double khDop; /* Kalman-derived horizontal dilution of precision (KHDOP) */
	private double kvDop; /* Kalman-derived vertical dilution of precision (KVDOP) */

	// Fields for satellite selection
	private TopocentricCoordinates[] roverTopo;
	private TopocentricCoordinates[] masterTopo;

	// Fields related to receiver-satellite geometry
	private SimpleMatrix[] diffRoverSat; /* Rover-satellite vector */
	private SimpleMatrix[] diffMasterSat; /* Master-satellite vector */
	private double[] roverSatAppRange; /* Rover-satellite approximate range */
	private double[] masterSatAppRange; /* Master-satellite approximate range */
	private double[] roverSatTropoCorr; /* Rover-satellite troposphere correction */
	private double[] masterSatTropoCorr; /* Master-satellite troposphere correction */
	private double[] roverSatIonoCorr; /* Rover-satellite ionosphere correction */
	private double[] masterSatIonoCorr; /* Master-satellite ionosphere correction */

	// Fields for storing values from previous epoch
	private double[] roverDopplerPredPhase; /* rover L Carrier Phase predicted from previous epoch (based on Doppler) [cycle] */
	private double[] masterDopplerPredPhase; /* master L Carrier Phase predicted from previous epoch (based on Doppler) [cycle] */

	// Fields for Kalman filter
	int o1, o2, o3;
	int i1, i2, i3;
	int nN;
	private SimpleMatrix T;
	private SimpleMatrix H;
	private SimpleMatrix y0;
	private SimpleMatrix Cvv;
	private SimpleMatrix Cee;
	private SimpleMatrix Cnn;
	private SimpleMatrix KFstate;
	private SimpleMatrix KFprediction;

	// Fields for keeping track of satellite configuration changes
	private ArrayList<Integer> satOld;
	private ArrayList<Character> satTypeOld;
	private int oldPivotId;
	private char oldPivotType;

	private boolean debug = false;

	char satType;
	
	private GoGPS goGPS;

	public ReceiverPosition(GoGPS goGPS){
		super();
		this.goGPS = goGPS;
		this.setXYZ(0.0, 0.0, 0.0);
		this.receiverClockError = 0.0;
	}

	/**
	 * @param obs
	 */
	public void bancroft(Observations obs) {

		//this.coord = null;
		//this.coord = Coordinates.globalXYZInstance(0.0, 0.0, 0.0);

		double travelTime = 0;
		double angle;
		double a, b, c;
		double root;
		double[] r, omc;
		double cdt, calc;
		double rho;

		// Define matrices
		SimpleMatrix Binit;
		SimpleMatrix B;
		SimpleMatrix BBB;
		SimpleMatrix BBBe;
		SimpleMatrix BBBalpha;
		SimpleMatrix e;
		SimpleMatrix alpha;
		SimpleMatrix possiblePosA;
		SimpleMatrix possiblePosB;

		// Allocate vectors
		r = new double[2];
		omc = new double[2];

		// Number of GPS observations
		int nObs = obs.getNumSat();

		// Allocate an array to store GPS satellite positions
		pos = new SatellitePosition[nObs];

		// Allocate a 2D array to store Bancroft matrix data
		double[][] dataB = new double[nObs][4];

		int p=0;
		int id = 0;
		
		
		for (int i = 0; i < nObs; i++) {

			id = obs.getSatID(i);
			satType = obs.getGnssType(i);
			
					// Create new satellite position object
					//pos[i] = new SatellitePosition(obs.getRefTime().getGpsTime(), obs.getGpsSatID(i), obs.getGpsByID(id).getPseudorange(goGPS.getFreq()));
		
					// Compute clock-corrected satellite position
					//pos[i].computePositionGps(goGPS.getNavigation());
		
					double obsPseudorange = obs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
					pos[i] = goGPS.getNavigation().getGpsSatPosition(obs, id, satType, this.getReceiverClockError());
		
					try {
//						System.out.println("SatPos "+obs.getGpsSatID(i)+" x:"+pos[i].getX()+" y:"+pos[i].getY()+" z:"+pos[i].getZ());
						// Store Bancroft matrix data (X, Y, Z and clock-corrected
						// range)
						if(pos[i]!=null){
							dataB[p][0] = pos[i].getX();
							dataB[p][1] = pos[i].getY();
							dataB[p][2] = pos[i].getZ();
							dataB[p][3] = obsPseudorange + Constants.SPEED_OF_LIGHT * pos[i].getSatelliteClockError();
							p++;
						}else{
							if(debug) System.out.println("Error: satellite positions not computed for satID:"+obs.getSatID(i));
						}
					} catch (NullPointerException u) {
						u.printStackTrace();
						if(debug) System.out.println("Error: satellite positions not computed for satID:"+obs.getSatID(i));
						//return; // don't break eggs so quickly :-)
					}
			
//			}else{
//				
//				p++;
//			}
		}
		
		if(p<4) return;
		if(dataB.length != p){
			double[][] dataB1 = new double[p][4];
			for(int i=0;i<p;i++){
				dataB1[i]=dataB[i];
			}
			dataB = dataB1;
		}
		// Allocate matrices
		BBB = new SimpleMatrix(4, dataB.length);
		BBBe = new SimpleMatrix(4, 1);
		BBBalpha = new SimpleMatrix(4, 1);
		e = new SimpleMatrix(dataB.length, 1);
		alpha = new SimpleMatrix(dataB.length, 1);
		possiblePosA = new SimpleMatrix(4, 1);
		possiblePosB = new SimpleMatrix(4, 1);

		// Allocate initial B matrix
		Binit = new SimpleMatrix(dataB);

		// Make two iterations
		for (int iter = 0; iter < 2; iter++) {

			// Allocate B matrix
			B = new SimpleMatrix(Binit);

			for (int i = 0; i < dataB.length; i++) {

				double x = B.get(i, 0);
				double y = B.get(i, 1);

				if (iter == 0) {
					travelTime = Constants.GPS_APPROX_TRAVEL_TIME;
				} else {
					double z = B.get(i, 2);
					rho = Math.pow((x - this.getX()), 2)
							+ Math.pow((y - this.getY()), 2)
							+ Math.pow((z - this.getZ()), 2);
					travelTime = Math.sqrt(rho) / Constants.SPEED_OF_LIGHT;
				}
				angle = travelTime * Constants.EARTH_ANGULAR_VELOCITY;
				B.set(i, 0, Math.cos(angle) * x + Math.sin(angle) * y);
				B.set(i, 1, -Math.sin(angle) * x + Math.cos(angle) * y);
			}

			if (dataB.length > 4) {
				BBB = B.transpose().mult(B).solve(B.transpose());
			} else {
				BBB = B.invert();
			}

			e.set(1);
			for (int i = 0; i < dataB.length; i++) {

				alpha.set(i, 0, lorentzInnerProduct(B.extractMatrix(i, i+1, 0, 4), B
						.extractMatrix(i, i+1, 0, 4)) / 2);
			}

			BBBe = BBB.mult(e);
			BBBalpha = BBB.mult(alpha);
			a = lorentzInnerProduct(BBBe, BBBe);
			b = lorentzInnerProduct(BBBe, BBBalpha) - 1;
			c = lorentzInnerProduct(BBBalpha, BBBalpha);
			root = Math.sqrt(b * b - a * c);
			r[0] = (-b - root) / a;
			r[1] = (-b + root) / a;
			possiblePosA = BBBalpha.plus(r[0], BBBe);
			possiblePosB = BBBalpha.plus(r[1], BBBe);
			possiblePosA.set(3, 0, -possiblePosA.get(3, 0));
			possiblePosB.set(3, 0, -possiblePosB.get(3, 0));
			for (int i = 0; i < dataB.length; i++) {
				cdt = possiblePosA.get(3, 0);
				calc = B.extractMatrix(i, i+1, 0, 3).transpose().minus(
						possiblePosA.extractMatrix(0, 3, 0, 1)).normF()
						+ cdt;
				omc[0] = B.get(i, 3) - calc;
				cdt = possiblePosB.get(3, 0);
				calc = B.extractMatrix(i, i+1, 0, 3).transpose().minus(
						possiblePosB.extractMatrix(0, 3, 0, 1)).normF()
						+ cdt;
				omc[1] = B.get(i, 3) - calc;
			}

			// Discrimination between roots (choose one of the possible
			// positions)
			if (Math.abs(omc[0]) > Math.abs(omc[1])) {
				//this.coord.ecef = possiblePosB.extractMatrix(0, 3, 0, 1); // new SimpleMatrix(
				SimpleMatrix sm = possiblePosB.extractMatrix(0, 3, 0, 1);
				this.setXYZ(sm.get(0),sm.get(1),sm.get(2));
				// Clock offset
				this.receiverClockError = possiblePosB.get(3, 0) / Constants.SPEED_OF_LIGHT;
			} else {
				//this.coord.ecef = possiblePosA.extractMatrix(0, 3, 0, 1); // new SimpleMatrix(
				SimpleMatrix sm = possiblePosA.extractMatrix(0, 3, 0, 1);
				this.setXYZ(sm.get(0),sm.get(1),sm.get(2));
				// Clock offset
				this.receiverClockError = possiblePosA.get(3, 0) / Constants.SPEED_OF_LIGHT;
			}
		}
//		System.out.println("## x: " + this.getX() );
//		System.out.println("## y: " + this.getY() );
//		System.out.println("## z: " + this.getZ() );
		
		
		// Compute Bancroft's positioning in geodetic coordinates
		this.computeGeodetic();
		
	}

	/**
	 * @param roverObs
	 */
	public void codeStandalone(Observations roverObs, boolean estimateOnlyClock) {

		// Number of GNSS observations without cutoff
		int nObs = roverObs.getNumSat();

		// Number of unknown parameters
		int nUnknowns = 4;
		
		// Add one unknown for each constellation in addition to the first (to estimate Inter-System Biases - ISBs)
		String sys = getAvailGnssSystems().substring(1);
		nUnknowns = nUnknowns + sys.length();

		// Define least squares matrices
		SimpleMatrix A;
		SimpleMatrix b;
		SimpleMatrix y0;
		SimpleMatrix Q;
		SimpleMatrix x;
		SimpleMatrix vEstim;
		SimpleMatrix tropoCorr;
		SimpleMatrix ionoCorr;

		// Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
		SimpleMatrix covXYZ;
		covXYZ = new SimpleMatrix(3, 3);

		// Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
		SimpleMatrix covENU;
		covENU = new SimpleMatrix(3, 3);

		// Number of available satellites (i.e. observations)
		int nObsAvail = satAvail.size();

		// Least squares design matrix
		A = new SimpleMatrix(nObsAvail, nUnknowns);

		// Vector for approximate pseudoranges
		b = new SimpleMatrix(nObsAvail, 1);

		// Vector for observed pseudoranges
		y0 = new SimpleMatrix(nObsAvail, 1);

		// Cofactor matrix
		Q = new SimpleMatrix(nObsAvail, nObsAvail);

		// Solution vector
		x = new SimpleMatrix(nUnknowns, 1);

		// Vector for observation error
		vEstim = new SimpleMatrix(nObsAvail, 1);

		// Vectors for troposphere and ionosphere corrections
		tropoCorr = new SimpleMatrix(nObsAvail, 1);

		ionoCorr = new SimpleMatrix(nObsAvail, 1);
		
		// Counter for available satellites
		int k = 0;

		// Initialize the cofactor matrix
		Q.set(1);

		// Satellite ID
		int id = 0;

		// Set up the least squares matrices
		for (int i = 0; i < nObs; i++) {

			id = roverObs.getSatID(i);
			char satType = roverObs.getGnssType(i);		
			String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
			
			if (pos[i]!=null && gnssAvail.contains(checkAvailGnss)) {
//			if (pos[i]!=null && satAvail.contains(id)  && satTypeAvail.contains(satType)) {
//				System.out.println("####" + checkAvailGnss  + "####");

				// Fill in one row in the design matrix
				A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i]); /* X */
				A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i]); /* Y */
				A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i]); /* Z */
				A.set(k, 3, 1); /* clock error */
				for (int c = 0; c < sys.length(); c++) {
					A.set(k, 4+c, sys.indexOf(satType)==c?1:0); /* inter-system bias */
				}

				// Add the approximate pseudorange value to b
				b.set(k, 0, roverSatAppRange[i] - pos[i].getSatelliteClockError() * Constants.SPEED_OF_LIGHT);

				// Add the clock-corrected observed pseudorange value to y0
				y0.set(k, 0, roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()));

				// Fill in troposphere and ionosphere double differenced
				// corrections
				tropoCorr.set(k, 0, roverSatTropoCorr[i]);
				ionoCorr.set(k, 0, roverSatIonoCorr[i]);

				// Fill in the cofactor matrix
				double weight = Q.get(k, k)
						+ computeWeight(roverTopo[i].getElevation(),
								roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
				Q.set(k, k, weight);

				// Increment available satellites counter
				k++;
			}
			
		}

		// Apply troposphere and ionosphere correction
		b = b.plus(tropoCorr);
		b = b.plus(ionoCorr);

		// Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
		x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose())
				.mult(Q.invert()).mult(y0.minus(b));

		// Receiver clock error
		this.receiverClockError = x.get(3) / Constants.SPEED_OF_LIGHT;

		if(estimateOnlyClock)
			return;

		// Receiver position
		//this.coord.ecef.set(this.coord.ecef.plus(x.extractMatrix(0, 3, 0, 1)));
		this.setPlusXYZ(x.extractMatrix(0, 3, 0, 1));

		// Estimation of the variance of the observation error
		vEstim = y0.minus(A.mult(x).plus(b));
		double varianceEstim = (vEstim.transpose().mult(Q.invert())
				.mult(vEstim)).get(0)
				/ (nObsAvail - nUnknowns);

		// Covariance matrix of the estimation error
		if (nObsAvail > nUnknowns) {
			SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
			.scale(varianceEstim);
			this.positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
		}else{
			this.positionCovariance = null;
		}

		// Compute covariance matrix from A matrix [ECEF reference system]
		covXYZ = A.extractMatrix(0, nObsAvail, 0, 3).transpose().mult(A.extractMatrix(0, nObsAvail, 0, 3)).invert();

		// Allocate and build rotation matrix
		SimpleMatrix R = new SimpleMatrix(3, 3);
		R = Coordinates.rotationMatrix(this);

		// Propagate covariance from global system to local system
		covENU = R.mult(covXYZ).mult(R.transpose());

		//Compute DOP values
		this.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
		this.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
		this.vDop = Math.sqrt(covENU.get(2, 2));

		// Compute positioning in geodetic coordinates
		this.computeGeodetic();

	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 */
	public void codeDoubleDifferences(Observations roverObs,Observations masterObs, Coordinates masterPos) {

		// Number of GPS observations
		int nObs = roverObs.getNumSat();

		// Number of unknown parameters
		int nUnknowns = 3;

		// Define least squares matrices
		SimpleMatrix A;
		SimpleMatrix Adop;
		SimpleMatrix b;
		SimpleMatrix y0;
		SimpleMatrix Q;
		SimpleMatrix x;
		SimpleMatrix vEstim;
		SimpleMatrix tropoCorr;
		SimpleMatrix ionoCorr;

		// Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
		SimpleMatrix covXYZ;
		covXYZ = new SimpleMatrix(3, 3);

		// Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
		SimpleMatrix covENU;
		covENU = new SimpleMatrix(3, 3);

		// Number of available satellites (i.e. observations)
		int nObsAvail = satAvail.size();

		// Full design matrix for DOP computation
		Adop = new SimpleMatrix(nObsAvail, 3);

		// Double differences with respect to pivot satellite reduce
		// observations by 1
		nObsAvail--;

		// Least squares design matrix
		A = new SimpleMatrix(nObsAvail, nUnknowns);

		// Vector for approximate pseudoranges
		b = new SimpleMatrix(nObsAvail, 1);

		// Vector for observed pseudoranges
		y0 = new SimpleMatrix(nObsAvail, 1);

		// Cofactor matrix
		Q = new SimpleMatrix(nObsAvail, nObsAvail);

		// Solution vector
		x = new SimpleMatrix(nUnknowns, 1);

		// Vector for observation error
		vEstim = new SimpleMatrix(nObsAvail, 1);

		// Vectors for troposphere and ionosphere corrections
		tropoCorr = new SimpleMatrix(nObsAvail, 1);
		ionoCorr = new SimpleMatrix(nObsAvail, 1);

		// Counter for available satellites (without pivot)
		int k = 0;

		// Counter for available satellites (with pivot)
		int d = 0;

		// Pivot satellite index
		int pivotId = roverObs.getSatID(pivot);
		satType = roverObs.getGnssType(pivot);			
		
		// Store rover-pivot and master-pivot observed pseudoranges
		double roverPivotObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
		double masterPivotObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

		// Rover-pivot approximate pseudoranges
		SimpleMatrix diffRoverPivot = diffRoverSat[pivot];
		double roverPivotAppRange = roverSatAppRange[pivot];

		// Master-pivot approximate pseudoranges
		double masterPivotAppRange = masterSatAppRange[pivot];

		// Computation of rover-pivot troposphere correction
		double roverPivotTropoCorr = roverSatTropoCorr[pivot];

		// Computation of master-pivot troposphere correction
		double masterPivotTropoCorr = masterSatTropoCorr[pivot];;

		// Computation of rover-pivot ionosphere correction
		double roverPivotIonoCorr = roverSatIonoCorr[pivot];

		// Computation of master-pivot ionosphere correction
		double masterPivotIonoCorr = masterSatIonoCorr[pivot];

		// Compute rover-pivot and master-pivot weights
		double roverPivotWeight = computeWeight(roverTopo[pivot].getElevation(),
				roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
		double masterPivotWeight = computeWeight(masterTopo[pivot].getElevation(),
				masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
		Q.set(roverPivotWeight + masterPivotWeight);

		// Satellite ID
		int id = 0;

		// Set up the least squares matrices
		for (int i = 0; i < nObs; i++) {

			id = roverObs.getSatID(i);
			char satType = roverObs.getGnssType(i);
			String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

			if (pos[i] !=null && gnssAvail.contains(checkAvailGnss) && i != pivot) {
//			if (pos[i] !=null && satAvail.contains(id) && satTypeAvail.contains(satType) && i != pivot) {

				// Fill in one row in the design matrix
				A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i]
						- diffRoverPivot.get(0) / roverPivotAppRange); /* X */

				A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i]
						- diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

				A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i]
						- diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

				// Add the differenced approximate pseudorange value to b
				b.set(k, 0, (roverSatAppRange[i] - masterSatAppRange[i])
						- (roverPivotAppRange - masterPivotAppRange));

				// Add the differenced observed pseudorange value to y0
				y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
						- (roverPivotObs - masterPivotObs));

				// Fill in troposphere and ionosphere double differenced
				// corrections
				tropoCorr.set(k, 0, (roverSatTropoCorr[i] - masterSatTropoCorr[i])
						- (roverPivotTropoCorr - masterPivotTropoCorr));
				ionoCorr.set(k, 0, (roverSatIonoCorr[i] - masterSatIonoCorr[i])
						- (roverPivotIonoCorr - masterPivotIonoCorr));

				// Fill in the cofactor matrix
				double roverSatWeight = computeWeight(roverTopo[i].getElevation(),
						roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
				double masterSatWeight = computeWeight(masterTopo[i].getElevation(),
						masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
				Q.set(k, k, Q.get(k, k) + roverSatWeight + masterSatWeight);

				// Increment available satellites counter
				k++;
			}

			// Design matrix for DOP computation
			if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)) {
//			if (pos[i] != null && satAvail.contains(id) && satTypeAvail.contains(satType)) {
				// Fill in one row in the design matrix (complete one, for DOP)
				Adop.set(d, 0, diffRoverSat[i].get(0) / roverSatAppRange[i]); /* X */
				Adop.set(d, 1, diffRoverSat[i].get(1) / roverSatAppRange[i]); /* Y */
				Adop.set(d, 2, diffRoverSat[i].get(2) / roverSatAppRange[i]); /* Z */
				d++;
			}
		}

		// Apply troposphere and ionosphere correction
		b = b.plus(tropoCorr);
		b = b.plus(ionoCorr);

		// Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
		x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose())
				.mult(Q.invert()).mult(y0.minus(b));

		// Receiver position
		//this.coord.ecef.set(this.coord.ecef.plus(x));
		this.setPlusXYZ(x);

		// Estimation of the variance of the observation error
		vEstim = y0.minus(A.mult(x).plus(b));
		double varianceEstim = (vEstim.transpose().mult(Q.invert())
				.mult(vEstim)).get(0)
				/ (nObsAvail - nUnknowns);

		// Covariance matrix of the estimation error
		if (nObsAvail > nUnknowns){
			SimpleMatrix covariance = A.transpose().mult(Q.invert()).mult(A).invert()
					.scale(varianceEstim);
			this.positionCovariance = covariance.extractMatrix(0, 3, 0, 3);
		}else{
			this.positionCovariance = null;
		}

		// Compute covariance matrix from A matrix [ECEF reference system]
		covXYZ = Adop.transpose().mult(Adop).invert();

		// Allocate and build rotation matrix
		SimpleMatrix R = new SimpleMatrix(3, 3);
		R = Coordinates.rotationMatrix(this);

		// Propagate covariance from global system to local system
		covENU = R.mult(covXYZ).mult(R.transpose());

		//Compute DOP values
		this.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
		this.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
		this.vDop = Math.sqrt(covENU.get(2, 2));

		// Compute positioning in geodetic coordinates
		this.computeGeodetic();
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 *
	 */
	public void kalmanFilterInit(Observations roverObs, Observations masterObs,
			Coordinates masterPos) {

		// Order-related quantities
		o1 = goGPS.getOrder();
		o2 = goGPS.getOrder() * 2;
		o3 = goGPS.getOrder() * 3;

		// Order-related indices
		i1 = o1 - 1;
		i2 = o2 - 1;
		i3 = o3 - 1;

		// Set number of ambiguities
		if (goGPS.isDualFreq())
			nN = 64;
		else
			nN = 32;

		// Allocate matrices
		T = SimpleMatrix.identity(o3 + nN);
		KFstate = new SimpleMatrix(o3 + nN, 1);
		KFprediction = new SimpleMatrix(o3 + nN, 1);
		Cvv = new SimpleMatrix(o3 + nN, o3 + nN);
		Cee = new SimpleMatrix(o3 + nN, o3 + nN);

		// System dynamics
		int j = 0;
		for (int i = 0; i < o3; i++) {
			if (j < (o1 - 1)) {
				T.set(i, i + 1, 1);
				j++;
			} else {
				j = 0;
			}
		}

		// Model error covariance matrix
		Cvv.zero();
		Cvv.set(i1, i1, Math.pow(goGPS.getStDevE(), 2));
		Cvv.set(i2, i2, Math.pow(goGPS.getStDevN(), 2));
		Cvv.set(i3, i3, Math.pow(goGPS.getStDevU(), 2));

		// Improve approximate position accuracy by applying twice code double differences
		for (int i = 0; i < 2; i++) {
			// Select satellites available for double differences
			// (twice because the rover position must be updated)
			selectSatellitesDoubleDiff(roverObs, masterObs, masterPos);

			if (satAvail.size() >= 4) {
				codeDoubleDifferences(roverObs, masterObs, masterPos);
			} else {
				this.setXYZ(0, 0, 0);
				return;
			}
		}

		// Estimate phase ambiguities
		ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
		newSatellites.addAll(satAvailPhase);
		estimateAmbiguities(roverObs, masterObs, masterPos, newSatellites, pivot, true);

		// Compute predicted phase ranges based on Doppler observations
		computeDopplerPredictedPhase(roverObs, masterObs);

		// Initial state
		KFstate.set(0, 0, this.getX());
		KFstate.set(i1 + 1, 0, this.getY());
		KFstate.set(i2 + 1, 0, this.getZ());

		// Prediction
		KFprediction = T.mult(KFstate);

		// Covariance matrix of the initial state
		if (this.positionCovariance != null) {
			Cee.set(0, 0, this.positionCovariance.get(0, 0));
			Cee.set(i1 + 1, i1 + 1, this.positionCovariance.get(1, 1));
			Cee.set(i2 + 1, i2 + 1, this.positionCovariance.get(2, 2));
		} else {
			this.positionCovariance = new SimpleMatrix(3, 3);
			Cee.set(0, 0, Math.pow(goGPS.getStDevInit(), 2));
			Cee.set(i1 + 1, i1 + 1, Math.pow(goGPS.getStDevInit(), 2));
			Cee.set(i2 + 1, i2 + 1, Math.pow(goGPS.getStDevInit(), 2));
		}
		for (int i = 1; i < o1; i++) {
			Cee.set(i, i, Math.pow(goGPS.getStDevInit(), 2));
			Cee.set(i + i1 + 1, i + i1 + 1, Math.pow(goGPS.getStDevInit(), 2));
			Cee.set(i + i2 + 1, i + i2 + 1, Math.pow(goGPS.getStDevInit(), 2));
		}
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 *
	 */
	public void kalmanFilterLoop(Observations roverObs, Observations masterObs, Coordinates masterPos) {

		// Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
		SimpleMatrix covENU;
		covENU = new SimpleMatrix(3, 3);

		// Set linearization point (approximate coordinates by KF
		// prediction at previous step)
		this.setXYZ(KFprediction.get(0), KFprediction.get(i1 + 1), KFprediction.get(i2 + 1));
//		this.coord.ecef.set(0, 0, KFprediction.get(0));
//		this.coord.ecef.set(1, 0, KFprediction.get(i1 + 1));
//		this.coord.ecef.set(2, 0, KFprediction.get(i2 + 1));

		// Save previous list of available satellites with phase
		satOld = satAvailPhase;
		satTypeOld = satTypeAvailPhase;

		// Save the ID and index of the previous pivot satellite
		try {
			oldPivotId = pos[pivot].getSatID();
			oldPivotType = pos[pivot].getSatType();
		} catch(ArrayIndexOutOfBoundsException e) {
			oldPivotId = 0;
		}

		// Select satellites for standalone
		selectSatellitesStandalone(roverObs);

		if (satAvail.size() >= 4)
			// Estimate receiver clock error by code stand-alone
			codeStandalone(roverObs, true);

		// Select satellites for double differences
		selectSatellitesDoubleDiff(roverObs, masterObs, masterPos);

		// Number of observations (code and phase)
		int nObs = satAvail.size();

		// Double differences with respect to pivot satellite reduce
		// number of observations by 1
		nObs = nObs - 1;

		if (satAvailPhase.size() != 0) {
			// Add number of satellites with phase (minus 1 for double diff)
			nObs = nObs + satAvailPhase.size() - 1;
		}

		if (satAvail.size() >= goGPS.getMinNumSat()) {
			// Allocate transformation matrix
			H = new SimpleMatrix(nObs, o3 + nN);

			// Allocate observation vector
			y0 = new SimpleMatrix(nObs, 1);

			// Allocate observation error covariance matrix
			Cnn = new SimpleMatrix(nObs, nObs);

			// Allocate K and G matrices
			SimpleMatrix K = new SimpleMatrix(o3 + nN, o3 + nN);
			SimpleMatrix G = new SimpleMatrix(o3 + nN, nObs);

			// Re-initialization of the model error covariance matrix
			Cvv.zero();

			// Set variances only if dynamic model is not static
			if (o1 != 1) {
				// Allocate and build rotation matrix
				SimpleMatrix R = new SimpleMatrix(3, 3);
				R = Coordinates.rotationMatrix(this);

				// Build 3x3 diagonal matrix with variances
				SimpleMatrix diagonal = new SimpleMatrix(3, 3);
				diagonal.zero();
				diagonal.set(0, 0, Math.pow(goGPS.getStDevE(), 2));
				diagonal.set(1, 1, Math.pow(goGPS.getStDevN(), 2));
				diagonal.set(2, 2, Math.pow(goGPS.getStDevU(), 2));

				// Propagate local variances to global variances
				diagonal = R.transpose().mult(diagonal).mult(R);

				// Set global variances in the model error covariance matrix
				Cvv.set(i1, i1, diagonal.get(0, 0));
				Cvv.set(i1, i2, diagonal.get(0, 1));
				Cvv.set(i1, i3, diagonal.get(0, 2));
				Cvv.set(i2, i1, diagonal.get(1, 0));
				Cvv.set(i2, i2, diagonal.get(1, 1));
				Cvv.set(i2, i3, diagonal.get(1, 2));
				Cvv.set(i3, i1, diagonal.get(2, 0));
				Cvv.set(i3, i2, diagonal.get(2, 1));
				Cvv.set(i3, i3, diagonal.get(2, 2));
			}

			// Fill in Kalman filter transformation matrix, observation
			// vector and observation error covariance matrix
			setupKalmanFilterInput(roverObs, masterObs, masterPos);

			// Check if satellite configuration changed since the previous epoch
			checkSatelliteConfiguration(roverObs, masterObs, masterPos);

			// Identity matrix
			SimpleMatrix I = SimpleMatrix.identity(o3 + nN);

			// Kalman filter equations
			K = T.mult(Cee).mult(T.transpose()).plus(Cvv);
			G = K.mult(H.transpose()).mult(
					H.mult(K).mult(H.transpose()).plus(Cnn).invert());
			KFstate = I.minus(G.mult(H)).mult(KFprediction).plus(G.mult(y0));
			KFprediction = T.mult(KFstate);
			Cee = I.minus(G.mult(H)).mult(K);

		} else {

			// Positioning only by system dynamics
			KFstate = KFprediction;
			KFprediction = T.mult(KFstate);
			Cee = T.mult(Cee).mult(T.transpose());
		}

		// Compute predicted phase ranges based on Doppler observations
		computeDopplerPredictedPhase(roverObs, masterObs);

		// Set receiver position
		this.setXYZ(KFstate.get(0), KFstate.get(i1 + 1), KFstate.get(i2 + 1));
//		this.coord.ecef.set(0, 0, KFstate.get(0));
//		this.coord.ecef.set(1, 0, KFstate.get(i1 + 1));
//		this.coord.ecef.set(2, 0, KFstate.get(i2 + 1));

		// Set receiver position error covariance
//		SimpleMatrix rows = Cee.extractMatrix(0, 1, 0, i2 + 2);
//		rows = rows.combine(1, 0, Cee.extractMatrix(i1 + 1, i1 + 2, 0, i2 + 2));
//		rows = rows.combine(2, 0, Cee.extractMatrix(i2 + 1, i2 + 2, 0, i2 + 2));
//		this.positionCovariance = rows.extractMatrix(0, 3, 0, 1);
//		this.positionCovariance = this.positionCovariance.combine(0, 1, rows.extractMatrix(0, 3, i1 + 1, i1 + 2));
//		this.positionCovariance = this.positionCovariance.combine(0, 2, rows.extractMatrix(0, 3, i2 + 1, i2 + 2));

		this.positionCovariance.set(0, 0, Cee.get(0, 0));
		this.positionCovariance.set(1, 1, Cee.get(i1 + 1, i1 + 1));
		this.positionCovariance.set(2, 2, Cee.get(i2 + 1, i2 + 1));
		this.positionCovariance.set(0, 1, Cee.get(0, i1 + 1));
		this.positionCovariance.set(0, 2, Cee.get(0, i2 + 1));
		this.positionCovariance.set(1, 0, Cee.get(i1 + 1, 0));
		this.positionCovariance.set(1, 2, Cee.get(i1 + 1, i2 + 1));
		this.positionCovariance.set(2, 0, Cee.get(i2 + 1, 0));
		this.positionCovariance.set(2, 1, Cee.get(i2 + 1, i1 + 1));

		// Allocate and build rotation matrix
		SimpleMatrix R = new SimpleMatrix(3, 3);
		R = Coordinates.rotationMatrix(this);

		// Propagate covariance from global system to local system
		covENU = R.mult(this.positionCovariance).mult(R.transpose());

		// Kalman filter DOP computation
		this.kpDop = Math.sqrt(this.positionCovariance.get(0, 0) + this.positionCovariance.get(1, 1) + this.positionCovariance.get(2, 2));
		this.khDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
		this.kvDop = Math.sqrt(covENU.get(2, 2));

		// Compute positioning in geodetic coordinates
		this.computeGeodetic();
	}

	/**
	 * @param roverObs
	 */
	public void selectSatellitesStandalone(Observations roverObs) {

		NavigationProducer navigation = goGPS.getNavigation();

		// Retrieve options from goGPS class
		double cutoff = goGPS.getCutoff();

		// Number of GPS observations
		int nObs = roverObs.getNumSat();

		// Allocate an array to store GPS satellite positions
		pos = new SatellitePosition[nObs];

		// Allocate an array to store receiver-satellite vectors
		diffRoverSat = new SimpleMatrix[nObs];

		// Allocate an array to store receiver-satellite approximate range
		roverSatAppRange = new double[nObs];

		// Allocate arrays to store receiver-satellite atmospheric corrections
		roverSatTropoCorr = new double[nObs];
		roverSatIonoCorr = new double[nObs];

		// Create a list for available satellites after cutoff
		satAvail = new ArrayList<Integer>(0);
		satTypeAvail = new ArrayList<Character>(0);
		gnssAvail = new ArrayList<String>(0);

		// Create a list for available satellites with phase
		satAvailPhase = new ArrayList<Integer>(0);
		satTypeAvailPhase = new ArrayList<Character>(0);
		gnssAvailPhase = new ArrayList<String>(0);
		
		// Allocate array of topocentric coordinates
		roverTopo = new TopocentricCoordinates[nObs];

		// Satellite ID
		int id = 0;

		// Compute topocentric coordinates and
		// select satellites above the cutoff level
		for (int i = 0; i < nObs; i++) {

			id = roverObs.getSatID(i);
			satType = roverObs.getGnssType(i);

			// Compute GPS satellite positions getGpsByIdx(idx).getSatType()
			pos[i] = navigation.getGpsSatPosition(roverObs, id, satType, this.getReceiverClockError());

			
			if(pos[i]!=null){

				// Compute rover-satellite approximate pseudorange
				diffRoverSat[i] = this.minusXYZ(pos[i]);
				roverSatAppRange[i] = Math.sqrt(Math.pow(diffRoverSat[i].get(0), 2)
						+ Math.pow(diffRoverSat[i].get(1), 2)
						+ Math.pow(diffRoverSat[i].get(2), 2));

				// Compute azimuth, elevation and distance for each satellite
				roverTopo[i] = new TopocentricCoordinates();
				roverTopo[i].computeTopocentric(this, pos[i]);

				// Correct approximate pseudorange for troposphere
				roverSatTropoCorr[i] = computeTroposphereCorrection(roverTopo[i].getElevation(), this.getGeodeticHeight());

				// Correct approximate pseudorange for ionosphere
				roverSatIonoCorr[i] = computeIonosphereCorrection(navigation, this, roverTopo[i].getAzimuth(), roverTopo[i].getElevation(), roverObs.getRefTime());

				
//				System.out.println("getElevation: " + id + "::" + roverTopo[i].getElevation() ); 
				// Check if satellite elevation is higher than cutoff
				if (roverTopo[i].getElevation() > cutoff) {
					
					satAvail.add(id);
					satTypeAvail.add(satType);
					gnssAvail.add(String.valueOf(satType) + String.valueOf(id));

					// Check if also phase is available
					if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {
						satAvailPhase.add(id);
						satTypeAvailPhase.add(satType);
						gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));				
						
					}
				}else{
					if(debug) System.out.println("Not useful sat "+roverObs.getSatID(i)+" for too low elevation "+roverTopo[i].getElevation()+" < "+cutoff);
				}
			}
			
		}
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 */
	public void selectSatellitesDoubleDiff(Observations roverObs,
			Observations masterObs, Coordinates masterPos) {

		NavigationProducer navigation = goGPS.getNavigation();

		// Retrieve options from goGPS class
		double cutoff = goGPS.getCutoff();

		// Number of GPS observations
		int nObs = roverObs.getNumSat();

		// Allocate an array to store GPS satellite positions
		pos = new SatellitePosition[nObs];

		// Allocate arrays to store receiver-satellite vectors
		diffRoverSat = new SimpleMatrix[nObs];
		diffMasterSat = new SimpleMatrix[nObs];

		// Allocate arrays to store receiver-satellite approximate range
		roverSatAppRange = new double[nObs];
		masterSatAppRange = new double[nObs];

		// Allocate arrays to store receiver-satellite atmospheric corrections
		roverSatTropoCorr = new double[nObs];
		roverSatIonoCorr = new double[nObs];
		masterSatTropoCorr = new double[nObs];
		masterSatIonoCorr = new double[nObs];

		// Create a list for available satellites
		satAvail = new ArrayList<Integer>(0);
		satTypeAvail = new ArrayList<Character>(0);
		gnssAvail = new ArrayList<String>(0);

		// Create a list for available satellites with phase
		satAvailPhase = new ArrayList<Integer>(0);
		satTypeAvailPhase = new ArrayList<Character>(0);
		gnssAvailPhase = new ArrayList<String>(0);

		// Allocate arrays of topocentric coordinates
		roverTopo = new TopocentricCoordinates[nObs];
		masterTopo = new TopocentricCoordinates[nObs];

		// Variables to store highest elevation
		double maxElevCode = 0;
		double maxElevPhase = 0;

		// Variables for code pivot and phase pivot
		int pivotCode = -1;
		int pivotPhase = -1;

		// Satellite ID
		int id = 0;

		// Compute topocentric coordinates and
		// select satellites above the cutoff level
		for (int i = 0; i < nObs; i++) {

			id = roverObs.getSatID(i);
			satType = roverObs.getGnssType(i);

			// Compute GPS satellite positions
			pos[i] = navigation.getGpsSatPosition(roverObs, id, satType, this.getReceiverClockError());

			if(pos[i]!=null){

				// Compute rover-satellite approximate pseudorange
				diffRoverSat[i] = this.minusXYZ(pos[i]);
				roverSatAppRange[i] = Math.sqrt(Math.pow(diffRoverSat[i].get(0), 2)
						+ Math.pow(diffRoverSat[i].get(1), 2)
						+ Math.pow(diffRoverSat[i].get(2), 2));

				// Compute master-satellite approximate pseudorange
				diffMasterSat[i] = masterPos.minusXYZ(pos[i]);
				masterSatAppRange[i] = Math.sqrt(Math.pow(diffMasterSat[i].get(0), 2)
						+ Math.pow(diffMasterSat[i].get(1), 2)
						+ Math.pow(diffMasterSat[i].get(2), 2));

				// Compute azimuth, elevation and distance for each satellite from
				// rover
				roverTopo[i] = new TopocentricCoordinates();
				roverTopo[i].computeTopocentric(this, pos[i]);

				// Compute azimuth, elevation and distance for each satellite from
				// master
				masterTopo[i] = new TopocentricCoordinates();
				masterTopo[i].computeTopocentric(masterPos, pos[i]);

				// Computation of rover-satellite troposphere correction
				roverSatTropoCorr[i] = computeTroposphereCorrection(roverTopo[i].getElevation(), this.getGeodeticHeight());

				// Computation of master-satellite troposphere correction
				masterSatTropoCorr[i] = computeTroposphereCorrection(masterTopo[i].getElevation(), masterPos.getGeodeticHeight());

				// Computation of rover-satellite ionosphere correction
				roverSatIonoCorr[i] = computeIonosphereCorrection(navigation,
								this, roverTopo[i].getAzimuth(), roverTopo[i].getElevation(), roverObs.getRefTime());

				// Computation of master-satellite ionosphere correction
				masterSatIonoCorr[i] = computeIonosphereCorrection(navigation,
								masterPos, masterTopo[i].getAzimuth(), masterTopo[i].getElevation(), roverObs.getRefTime());

				// Check if satellite is available for double differences, after
				// cutoff
				if (masterObs.containsSatIDType(roverObs.getSatID(i), roverObs.getGnssType(i)) // gpsSat.get( // masterObs.gpsSat.contains(roverObs.getGpsSatID(i)
						&& roverTopo[i].getElevation() > cutoff) {

					// Find code pivot satellite (with highest elevation)
					if (roverTopo[i].getElevation() > maxElevCode) {
						pivotCode = i;
						maxElevCode = roverTopo[i].getElevation();
					}

					satAvail.add(id);
					satTypeAvail.add(satType);
					gnssAvail.add(String.valueOf(satType) + String.valueOf(id));	

					// Check if also phase is available for both rover and master
					if (!Double.isNaN(roverObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq())) &&
							!Double.isNaN(masterObs.getSatByIDType(id, satType).getPhaseCycles(goGPS.getFreq()))) {

						// Find code pivot satellite (with highest elevation)
						if (roverTopo[i].getElevation() > maxElevPhase) {
							pivotPhase = i;
							maxElevPhase = roverTopo[i].getElevation();
						}

						satAvailPhase.add(id);
						satTypeAvailPhase.add(satType);
						gnssAvailPhase.add(String.valueOf(satType) + String.valueOf(id));
						
					}
				}
			}
		}

		// Select best pivot satellite
		if (pivotPhase != -1){
			pivot = pivotPhase;
		}else{
			pivot = pivotCode;
		}
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 */
	private void setupKalmanFilterInput(Observations roverObs, Observations masterObs, Coordinates masterPos) {

		// Definition of matrices
		SimpleMatrix A;
		SimpleMatrix covXYZ;
		SimpleMatrix covENU;

		// Number of GPS observations
		int nObs = roverObs.getNumSat();

		// Number of available satellites (i.e. observations)
		int nObsAvail = satAvail.size();

		// Double differences with respect to pivot satellite reduce
		// observations by 1
		nObsAvail--;

		// Matrix containing parameters obtained from the linearization of the observation equations
		A = new SimpleMatrix(nObsAvail, 3);

		// Covariance matrix obtained from matrix A (satellite geometry) [ECEF coordinates]
		covXYZ = new SimpleMatrix(3, 3);

		// Covariance matrix obtained from matrix A (satellite geometry) [local coordinates]
		covENU = new SimpleMatrix(3, 3);

		// Counter for available satellites
		int k = 0;

		// Counter for satellites with phase available
		int p = 0;

		// Pivot satellite ID
		int pivotId = roverObs.getSatID(pivot);
		char satType = roverObs.getGnssType(pivot);

		// Store rover-pivot and master-pivot observed pseudoranges
		double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
		double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

		// Compute and store rover-pivot and master-pivot observed phase ranges
		double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
		double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());

		// Rover-pivot approximate pseudoranges
		SimpleMatrix diffRoverPivot = diffRoverSat[pivot];
		double roverPivotAppRange = roverSatAppRange[pivot];

		// Master-pivot approximate pseudoranges
		double masterPivotAppRange = masterSatAppRange[pivot];

		// Rover-pivot and master-pivot troposphere correction
		double roverPivotTropoCorr = roverSatTropoCorr[pivot];
		double masterPivotTropoCorr = masterSatTropoCorr[pivot];;

		// Rover-pivot and master-pivot ionosphere correction
		double roverPivotIonoCorr = roverSatIonoCorr[pivot];
		double masterPivotIonoCorr = masterSatIonoCorr[pivot];

		// Compute rover-pivot and master-pivot weights
		double roverElevation = roverTopo[pivot].getElevation();
		double masterElevation = masterTopo[pivot].getElevation();
		double roverPivotWeight = computeWeight(roverElevation,
				roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
		double masterPivotWeight = computeWeight(masterElevation,
				masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));

		// Start filling in the observation error covariance matrix
		Cnn.zero();
		int nSatAvail = satAvail.size() - 1;
		int nSatAvailPhase = satAvailPhase.size() - 1;
		for (int i = 0; i < nSatAvail + nSatAvailPhase; i++) {
			for (int j = 0; j < nSatAvail + nSatAvailPhase; j++) {

				if (i < nSatAvail && j < nSatAvail)
					Cnn.set(i, j, goGPS.getStDevCode(roverObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
							* goGPS.getStDevCode(masterObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
							* (roverPivotWeight + masterPivotWeight));
				else if (i >= nSatAvail && j >= nSatAvail)
					Cnn.set(i, j, Math.pow(goGPS.getStDevPhase(), 2)
							* (roverPivotWeight + masterPivotWeight));
			}
		}

		// Satellite ID
		int id = 0;

		for (int i = 0; i < nObs; i++) {

			id = roverObs.getSatID(i);
			satType = roverObs.getGnssType(i);
			String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

			if (pos[i]!=null && gnssAvail.contains(checkAvailGnss)
					&& i != pivot) {

				// Compute parameters obtained from linearization of observation equations
				double alphaX = diffRoverSat[i].get(0) / roverSatAppRange[i]
						- diffRoverPivot.get(0) / roverPivotAppRange;
				double alphaY = diffRoverSat[i].get(1) / roverSatAppRange[i]
						- diffRoverPivot.get(1) / roverPivotAppRange;
				double alphaZ = diffRoverSat[i].get(2) / roverSatAppRange[i]
						- diffRoverPivot.get(2) / roverPivotAppRange;

				// Fill in the A matrix
				A.set(k, 0, alphaX); /* X */
				A.set(k, 1, alphaY); /* Y */
				A.set(k, 2, alphaZ); /* Z */

				// Approximate code double difference
				double ddcApp = (roverSatAppRange[i] - masterSatAppRange[i])
						- (roverPivotAppRange - masterPivotAppRange);

				// Observed code double difference
				double ddcObs = (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
						- (roverPivotCodeObs - masterPivotCodeObs);

				// Observed phase double difference
				double ddpObs = (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
						- (roverPivotPhaseObs - masterPivotPhaseObs);

				// Compute troposphere and ionosphere residuals
				double tropoResiduals = (roverSatTropoCorr[i] - masterSatTropoCorr[i])
						- (roverPivotTropoCorr - masterPivotTropoCorr);
				double ionoResiduals = (roverSatIonoCorr[i] - masterSatIonoCorr[i])
						- (roverPivotIonoCorr - masterPivotIonoCorr);

				// Compute approximate ranges
				double appRangeCode;
				double appRangePhase;
				if (goGPS.getFreq() == 0) {
					appRangeCode = ddcApp + tropoResiduals + ionoResiduals;
					appRangePhase = ddcApp + tropoResiduals - ionoResiduals;
				} else {
					appRangeCode = ddcApp + tropoResiduals + ionoResiduals * Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(1)/roverObs.getSatByIDType(id, satType).getWavelength(0), 2);
					appRangePhase = ddcApp + tropoResiduals - ionoResiduals * Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(1)/roverObs.getSatByIDType(id, satType).getWavelength(0), 2);
				}

				// Fill in one row in the design matrix (for code)
				H.set(k, 0, alphaX);
				H.set(k, i1 + 1, alphaY);
				H.set(k, i2 + 1, alphaZ);

				// Fill in one element of the observation vector (for code)
				y0.set(k, 0, ddcObs - appRangeCode + alphaX * this.getX()
						+ alphaY * this.getY() + alphaZ
						* this.getZ());

				// Fill in the observation error covariance matrix (for code)
				double roverSatWeight = computeWeight(roverElevation,
						roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
				double masterSatWeight = computeWeight(masterElevation,
						masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
				double CnnBase = Cnn.get(k, k);
				Cnn.set(k, k, CnnBase + goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
						* goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq())
						* (roverSatWeight + masterSatWeight));

				if (gnssAvail.contains(checkAvailGnss)){
//				if (satAvailPhase.contains(id) && satTypeAvailPhase.contains(satType)) {

					// Fill in one row in the design matrix (for phase)
					H.set(nObsAvail + p, 0, alphaX);
					H.set(nObsAvail + p, i1 + 1, alphaY);
					H.set(nObsAvail + p, i2 + 1, alphaZ);
					H.set(nObsAvail + p, i3 + id, -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()));

					// Fill in one element of the observation vector (for phase)
					y0.set(nObsAvail + p, 0, ddpObs - appRangePhase + alphaX
							* this.getX() + alphaY
							* this.getY() + alphaZ
							* this.getZ());

					// Fill in the observation error covariance matrix (for
					// phase)
					CnnBase = Cnn.get(nObsAvail + p, nObsAvail + p);
					Cnn.set(nObsAvail + p, nObsAvail + p, CnnBase
							+ Math.pow(goGPS.getStDevPhase(), 2)
							* (roverSatWeight + masterSatWeight));

					// Increment satellites with phase counter
					p++;
				}

				// Increment available satellites counter
				k++;
			}
		}

		// Compute covariance matrix from A matrix [ECEF reference system]
		covXYZ = A.transpose().mult(A).invert();

		// Allocate and build rotation matrix
		SimpleMatrix R = new SimpleMatrix(3, 3);
		R = Coordinates.rotationMatrix(this);

		// Propagate covariance from global system to local system
		covENU = R.mult(covXYZ).mult(R.transpose());

		//Compute DOP values
		this.pDop = Math.sqrt(covXYZ.get(0, 0) + covXYZ.get(1, 1) + covXYZ.get(2, 2));
		this.hDop = Math.sqrt(covENU.get(0, 0) + covENU.get(1, 1));
		this.vDop = Math.sqrt(covENU.get(2, 2));
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 */
	private void checkSatelliteConfiguration(Observations roverObs,
			Observations masterObs, Coordinates masterPos) {

		// Lists for keeping track of satellites that need ambiguity (re-)estimation
		ArrayList<Integer> newSatellites = new ArrayList<Integer>(0);
		ArrayList<Integer> slippedSatellites = new ArrayList<Integer>(0);

		// Check if satellites were lost since the previous epoch
		for (int i = 0; i < satOld.size(); i++) {

			// Set ambiguity of lost satellites to zero
//			if (!gnssAvailPhase.contains(satOld.get(i))) {
			if (!satAvailPhase.contains(satOld.get(i)) && satTypeAvailPhase.contains(satOld.get(i))) {

				if(debug) System.out.println("Lost satellite "+satOld.get(i));

				KFprediction.set(i3 + satOld.get(i), 0, 0);
			}
		}

		// Check if new satellites are available since the previous epoch
		int temporaryPivot = 0;
		boolean newPivot = false;
		for (int i = 0; i < pos.length; i++) {

			if (pos[i] != null && satAvailPhase.contains(pos[i].getSatID()) && satTypeAvailPhase.contains(pos[i].getSatType())
					&& !satOld.contains(pos[i].getSatID()) && satTypeOld.contains(pos[i].getSatType())) {

				//TODO: need to check below 
				newSatellites.add(pos[i].getSatID());

				if (pos[i].getSatID() != pos[pivot].getSatID() && pos[i].getSatType() == pos[pivot].getSatType()) {
					if(debug) System.out.println("New satellite "+pos[i].getSatID());
				} else {
					newPivot = true;
					if(debug) System.out.println("New satellite "+pos[i].getSatID()+" (new pivot)");
				}
			}
		}

		// If a new satellite is going to be the pivot, its ambiguity needs to be estimated before switching pivot
		if (newPivot) {
			// If it is not the only satellite with phase
			if (satAvailPhase.size() > 1) {
				// If the former pivot is still among satellites with phase
				if (satAvailPhase.contains(oldPivotId) && satTypeAvailPhase.contains(oldPivotType)) {
					// Find the index of the old pivot
					for (int j = 0; j < pos.length; j ++) {
						if (pos[j] != null && pos[j].getSatID() == oldPivotId && pos[j].getSatType() == oldPivotType) {
							temporaryPivot = j;
						}
					}
				} else {
					double maxEl = 0;
					// Find a temporary pivot with phase
					for (int j = 0; j < pos.length; j ++) {
						if (pos[j] != null && satAvailPhase.contains(pos[j].getSatID()) && satTypeAvailPhase.contains(pos[j].getSatType())
								&& j != pivot
								&& roverTopo[j].getElevation() > maxEl) {
							temporaryPivot = j;
							maxEl = roverTopo[j].getElevation();
						}
					}
					// Reset the ambiguities of other satellites according to the temporary pivot
					newSatellites.clear();
					newSatellites.addAll(satAvailPhase);
					oldPivotId = pos[temporaryPivot].getSatID();
					oldPivotType = pos[temporaryPivot].getSatType();
					
				}
				// Estimate the ambiguity of the new pivot and other (new) satellites, using the temporary pivot
				estimateAmbiguities(roverObs, masterObs, masterPos, newSatellites, temporaryPivot, false);
				newSatellites.clear();
			}
		}

		// Check if pivot satellite changed since the previous epoch
		if (oldPivotId != pos[pivot].getSatID() && oldPivotType == pos[pivot].getSatType()  && satAvailPhase.size() > 1) {

			if(debug) System.out.println("Pivot change from satellite "+oldPivotId+" to satellite "+pos[pivot].getSatID());

			// Matrix construction to manage the change of pivot satellite
			SimpleMatrix A = new SimpleMatrix(o3 + nN, o3 + nN);

			//TODO: need to check below
			int pivotIndex = i3 + pos[pivot].getSatID();
			int pivotOldIndex = i3 + oldPivotId;
			for (int i = 0; i < o3; i++) {
				for (int j = 0; j < o3; j++) {
					if (i == j)
						A.set(i, j, 1);
				}
			}
			for (int i = 0; i < satAvailPhase.size(); i++) {
				for (int j = 0; j < satAvailPhase.size(); j++) {
					int satIndex = i3 + satAvailPhase.get(i);
					if (i == j) {
						A.set(satIndex, satIndex, 1);
					}
					A.set(satIndex, pivotIndex, -1);
				}
			}
			A.set(pivotOldIndex, pivotOldIndex, 0);
			A.set(pivotIndex, pivotIndex, 0);

			// Update predicted state
			KFprediction = A.mult(KFprediction);

			// Re-computation of the Cee covariance matrix at the previous epoch
			Cee = A.mult(Cee).mult(A.transpose());
		}

		// Cycle-slip detection
		boolean lossOfLockCycleSlipRover;
		boolean lossOfLockCycleSlipMaster;
		boolean dopplerCycleSlipRover;
		boolean dopplerCycleSlipMaster;
		boolean cycleSlip;
		//boolean slippedPivot = false;
		for (int i = 0; i < satAvailPhase.size(); i++) {

			int satID = satAvailPhase.get(i);
			char satType = satTypeAvailPhase.get(i);					
			
			// cycle slip detected by loss of lock indicator (temporarily disabled)
			lossOfLockCycleSlipRover = roverObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
			lossOfLockCycleSlipMaster = masterObs.getSatByIDType(satID, satType).isPossibleCycleSlip(goGPS.getFreq());
			lossOfLockCycleSlipRover = false;
			lossOfLockCycleSlipMaster = false;

			// cycle slip detected by Doppler predicted phase range
			dopplerCycleSlipRover = this.getRoverDopplerPredictedPhase(satID) != 0.0 && (Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
					- this.getRoverDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());
			dopplerCycleSlipMaster = this.getMasterDopplerPredictedPhase(satID) != 0.0 && (Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
					- this.getMasterDopplerPredictedPhase(satID)) > goGPS.getCycleSlipThreshold());

			cycleSlip = (lossOfLockCycleSlipRover || lossOfLockCycleSlipMaster || dopplerCycleSlipRover || dopplerCycleSlipMaster);

			if (satID != pos[pivot].getSatID() && !newSatellites.contains(satID) && cycleSlip) {

				slippedSatellites.add(satID);

//				if (satID != pos[pivot].getSatID()) {
					if (dopplerCycleSlipRover)
						if(debug) System.out.println("[ROVER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
								- this.getRoverDopplerPredictedPhase(satID))+")");
					if (dopplerCycleSlipMaster)
						if(debug) System.out.println("[MASTER] Cycle slip on satellite "+satID+" (range diff = "+Math.abs(masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq())
								- this.getMasterDopplerPredictedPhase(satID))+")");
//				} else {
//					boolean slippedPivot = true;
//					if (dopplerCycleSlipRover)
//						System.out.println("[ROVER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(roverObs.getGpsByID(satID).getPhase(goGPS.getFreq())
//								- this.getRoverDopplerPredictedPhase(satID))+")");
//					if (dopplerCycleSlipMaster)
//						System.out.println("[MASTER] Cycle slip on pivot satellite "+satID+" (range diff = "+Math.abs(masterObs.getGpsByID(satID).getPhase(goGPS.getFreq())
//								- this.getMasterDopplerPredictedPhase(satID))+")");
//				}
			}
		}

//		// If the pivot satellites slipped, the ambiguities of all the other satellites must be re-estimated
//		if (slippedPivot) {
//			// If it is not the only satellite with phase
//			if (satAvailPhase.size() > 1) {
//				// Reset the ambiguities of other satellites
//				newSatellites.clear();
//				slippedSatellites.clear();
//				slippedSatellites.addAll(satAvailPhase);
//			}
//		}

		// Ambiguity estimation
		if (newSatellites.size() != 0 || slippedSatellites.size() != 0) {
			// List of satellites that need ambiguity estimation
			ArrayList<Integer> satAmb = newSatellites;
			satAmb.addAll(slippedSatellites);
			estimateAmbiguities(roverObs, masterObs, masterPos, satAmb, pivot, false);
		}
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 * @param masterPos
	 */
	private void estimateAmbiguities(Observations roverObs,
			Observations masterObs, Coordinates masterPos, ArrayList<Integer> satAmb, int pivotIndex, boolean init) {

		// Check if pivot is in satAmb, in case remove it
		if (satAmb.contains(pos[pivotIndex].getSatID()))
			satAmb.remove(satAmb.indexOf(pos[pivotIndex].getSatID()));

		// Number of GPS observations
		int nObs = roverObs.getNumSat();

		// Number of available satellites (i.e. observations)
		int nObsAvail = satAvail.size();

		// Number of available satellites (i.e. observations) with phase
		int nObsAvailPhase = satAvailPhase.size();

		// Double differences with respect to pivot satellite reduce
		// observations by 1
		nObsAvail--;
		nObsAvailPhase--;

		// Number of unknown parameters
		int nUnknowns = 3 + satAmb.size();

		// Pivot satellite ID
		int pivotId = roverObs.getSatID(pivotIndex);
		char satType = roverObs.getGnssType(pivotIndex);

		// Rover-pivot and master-pivot observed pseudorange
		double roverPivotCodeObs = roverObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());
		double masterPivotCodeObs = masterObs.getSatByIDType(pivotId, satType).getPseudorange(goGPS.getFreq());

		// Rover-pivot and master-pivot observed phase
		double roverPivotPhaseObs = roverObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());
		double masterPivotPhaseObs = masterObs.getSatByIDType(pivotId, satType).getPhaserange(goGPS.getFreq());

		// Rover-pivot approximate pseudoranges
		SimpleMatrix diffRoverPivot = diffRoverSat[pivotIndex];
		double roverPivotAppRange = roverSatAppRange[pivotIndex];

		// Master-pivot approximate pseudoranges
		double masterPivotAppRange = masterSatAppRange[pivotIndex];

		// Estimated ambiguity combinations (double differences)
		double[] estimatedAmbiguityComb;
		estimatedAmbiguityComb = new double[satAmb.size()];

		// Covariance of estimated ambiguity combinations
		double[] estimatedAmbiguityCombCovariance;
		estimatedAmbiguityCombCovariance = new double[satAmb.size()];

		// Variables to store rover-satellite and master-satellite observed code
		double roverSatCodeObs;
		double masterSatCodeObs;

		// Variables to store rover-satellite and master-satellite observed phase
		double roverSatPhaseObs;
		double masterSatPhaseObs;

		// Variables to store rover-satellite and master-satellite observed
        // code
        double roverSatCodeAppRange;
        double masterSatCodeAppRange;

		// Variables to store code and phase double differences
		double codeDoubleDiffObserv;
		double codeDoubleDiffApprox;
		double phaseDoubleDiffObserv;

		// Satellite ID
		int id = 0;

		if (goGPS.getAmbiguityStrategy() == GoGPS.AMBIGUITY_OBSERV) {

			for (int i = 0; i < nObs; i++) {

				id = roverObs.getSatID(i);
				satType = roverObs.getGnssType(i);

				if (pos[i]!=null && satAmb.contains(id) && id != pivotId) {

					// Rover-satellite and master-satellite observed code
					roverSatCodeObs = roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());
					masterSatCodeObs = masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq());

					// Rover-satellite and master-satellite observed phase
					roverSatPhaseObs = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());
					masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

					// Observed code double difference
					codeDoubleDiffObserv = (roverSatCodeObs - masterSatCodeObs)
					- (roverPivotCodeObs - masterPivotCodeObs);

					// Observed phase double difference
					phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs)
					- (roverPivotPhaseObs - masterPivotPhaseObs);

					// Store estimated ambiguity combinations and their covariance
					estimatedAmbiguityComb[satAmb.indexOf(id)] = (codeDoubleDiffObserv - phaseDoubleDiffObserv) / roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());
					estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
					* goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
					* goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
				}
			}
		} else if(goGPS.getAmbiguityStrategy() == GoGPS.AMBIGUITY_APPROX | (nObsAvail + nObsAvailPhase <= nUnknowns)) {

			for (int i = 0; i < nObs; i++) {

				id = roverObs.getSatID(i);
				satType = roverObs.getGnssType(i);

				if (pos[i]!=null && satAmb.contains(id) && id != pivotId) {

					// Rover-satellite and master-satellite approximate pseudorange
	                roverSatCodeAppRange = roverSatAppRange[i];
	                masterSatCodeAppRange = masterSatAppRange[i];

	                // Rover-satellite and master-satellite observed phase
					roverSatPhaseObs = roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());
					masterSatPhaseObs = masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq());

	                // Estimated code pseudorange double differences
	                codeDoubleDiffApprox = (roverSatCodeAppRange - masterSatCodeAppRange)
	                                - (roverPivotAppRange - masterPivotAppRange);

	                // Observed phase double differences
	                phaseDoubleDiffObserv = (roverSatPhaseObs - masterSatPhaseObs)
	                                - (roverPivotPhaseObs - masterPivotPhaseObs);

					// Store estimated ambiguity combinations and their covariance
					estimatedAmbiguityComb[satAmb.indexOf(id)] = (codeDoubleDiffApprox - phaseDoubleDiffObserv) / roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq());
					estimatedAmbiguityCombCovariance[satAmb.indexOf(id)] = 4
					* goGPS.getStDevCode(roverObs.getSatByIDType(id, satType), goGPS.getFreq())
					* goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq()) / Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2);
				}
			}
		} else if (goGPS.getAmbiguityStrategy() == GoGPS.AMBIGUITY_LS) {

			// Define least squares matrices
			SimpleMatrix A;
			SimpleMatrix b;
			SimpleMatrix y0;
			SimpleMatrix Qcode;
			SimpleMatrix Qphase;
			SimpleMatrix Q;
			SimpleMatrix x;
			SimpleMatrix vEstim;
			SimpleMatrix covariance;
			SimpleMatrix tropoCorr;
			SimpleMatrix ionoCorr;

			// Least squares design matrix
			A = new SimpleMatrix(nObsAvail+nObsAvailPhase, nUnknowns);
			A.zero();

			// Vector for approximate pseudoranges
			b = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

			// Vector for observed pseudoranges
			y0 = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

			// Cofactor matrices
			Qcode = new SimpleMatrix(nObsAvail, nObsAvail);
			Qphase = new SimpleMatrix(nObsAvailPhase, nObsAvailPhase);
			Q = new SimpleMatrix(nObsAvail+nObsAvailPhase, nObsAvail+nObsAvailPhase);
			Q.zero();

			// Solution vector
			x = new SimpleMatrix(nUnknowns, 1);

			// Vector for observation error
			vEstim = new SimpleMatrix(nObsAvail, 1);

			// Error covariance matrix
			covariance = new SimpleMatrix(nUnknowns, nUnknowns);

			// Vectors for troposphere and ionosphere corrections
			tropoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);
			ionoCorr = new SimpleMatrix(nObsAvail+nObsAvailPhase, 1);

			// Counters for available satellites
			int k = 0;
			int p = 0;

			// Rover-pivot and master-pivot troposphere correction
			double roverPivotTropoCorr = roverSatTropoCorr[pivotIndex];
			double masterPivotTropoCorr = masterSatTropoCorr[pivotIndex];;

			// Rover-pivot and master-pivot ionosphere correction
			double roverPivotIonoCorr = roverSatIonoCorr[pivotIndex];
			double masterPivotIonoCorr = masterSatIonoCorr[pivotIndex];

			// Compute rover-pivot and master-pivot weights
			double roverPivotWeight = computeWeight(roverTopo[pivotIndex].getElevation(),
					roverObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
			double masterPivotWeight = computeWeight(masterTopo[pivotIndex].getElevation(),
					masterObs.getSatByIDType(pivotId, satType).getSignalStrength(goGPS.getFreq()));
			Qcode.set(goGPS.getStDevCode(roverObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
					* goGPS.getStDevCode(masterObs.getSatByIDType(pivotId, satType), goGPS.getFreq())
					* (roverPivotWeight + masterPivotWeight));
			Qphase.set(Math.pow(goGPS.getStDevPhase(), 2) * (roverPivotWeight + masterPivotWeight));

			// Set up the least squares matrices...
			// ... for code ...
			for (int i = 0; i < nObs; i++) {

				id = roverObs.getSatID(i);
				satType = roverObs.getGnssType(i);
				String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);
				
				if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)
						&& i != pivotIndex) {

					// Fill in one row in the design matrix
					A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

					A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

					A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

					// Add the differenced approximate pseudorange value to b
					b.set(k, 0, (roverSatAppRange[i] - masterSatAppRange[i])
							- (roverPivotAppRange - masterPivotAppRange));

					// Add the differenced observed pseudorange value to y0
					y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPseudorange(goGPS.getFreq()))
							- (roverPivotCodeObs - masterPivotCodeObs));

					// Fill in troposphere and ionosphere double differenced
					// corrections
					tropoCorr.set(k, 0, (roverSatTropoCorr[i] - masterSatTropoCorr[i])
							- (roverPivotTropoCorr - masterPivotTropoCorr));
					ionoCorr.set(k, 0, (roverSatIonoCorr[i] - masterSatIonoCorr[i])
							- (roverPivotIonoCorr - masterPivotIonoCorr));

					// Fill in the cofactor matrix
					double roverSatWeight = computeWeight(roverTopo[i].getElevation(),
							roverObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
					double masterSatWeight = computeWeight(masterTopo[i].getElevation(),
							masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
					Qcode.set(k, k, Qcode.get(k, k) + goGPS.getStDevCode(roverObs.getSatByID(id), goGPS.getFreq())
							* goGPS.getStDevCode(masterObs.getSatByIDType(id, satType), goGPS.getFreq())
							* (roverSatWeight + masterSatWeight));

					// Increment available satellites counter
					k++;
				}
			}

			// ... and phase
			for (int i = 0; i < nObs; i++) {

				id = roverObs.getSatID(i);
				satType = roverObs.getGnssType(i);
				String checkAvailGnss = String.valueOf(satType) + String.valueOf(id);

				if (pos[i] !=null && gnssAvail.contains(checkAvailGnss)
						&& i != pivotIndex) {

					// Fill in one row in the design matrix
					A.set(k, 0, diffRoverSat[i].get(0) / roverSatAppRange[i] - diffRoverPivot.get(0) / roverPivotAppRange); /* X */

					A.set(k, 1, diffRoverSat[i].get(1) / roverSatAppRange[i] - diffRoverPivot.get(1) / roverPivotAppRange); /* Y */

					A.set(k, 2, diffRoverSat[i].get(2) / roverSatAppRange[i] - diffRoverPivot.get(2) / roverPivotAppRange); /* Z */

					if (satAmb.contains(id)) {
						A.set(k, 3 + satAmb.indexOf(id), -roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq())); /* N */

						// Add the differenced observed pseudorange value to y0
						y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
								- (roverPivotPhaseObs - masterPivotPhaseObs));
					} else {
						// Add the differenced observed pseudorange value + known N to y0
						y0.set(k, 0, (roverObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()) - masterObs.getSatByIDType(id, satType).getPhaserange(goGPS.getFreq()))
								- (roverPivotPhaseObs - masterPivotPhaseObs) + KFprediction.get(i3 + id));
					}

					// Add the differenced approximate pseudorange value to b
					b.set(k, 0, (roverSatAppRange[i] - masterSatAppRange[i])
							- (roverPivotAppRange - masterPivotAppRange));

					// Fill in troposphere and ionosphere double differenced corrections
					tropoCorr.set(k, 0, (roverSatTropoCorr[i] - masterSatTropoCorr[i])
							- (roverPivotTropoCorr - masterPivotTropoCorr));
					ionoCorr.set(k, 0, -((roverSatIonoCorr[i] - masterSatIonoCorr[i])
							- (roverPivotIonoCorr - masterPivotIonoCorr)));

					// Fill in the cofactor matrix
					double roverSatWeight = computeWeight(
							roverTopo[i].getElevation(), roverObs.getSatByIDType(id, satType)
							.getSignalStrength(goGPS.getFreq()));
					double masterSatWeight = computeWeight(
							masterTopo[i].getElevation(),
							masterObs.getSatByIDType(id, satType).getSignalStrength(goGPS.getFreq()));
					Qphase.set(p, p, Qphase.get(p, p)
							+ (Math.pow(goGPS.getStDevPhase(), 2) + Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2) * Cee.get(i3 + id, i3 + id))
							* (roverPivotWeight + masterPivotWeight)
							+ (Math.pow(goGPS.getStDevPhase(), 2) + Math.pow(roverObs.getSatByIDType(id, satType).getWavelength(goGPS.getFreq()), 2) * Cee.get(i3 + id, i3 + id))
							* (roverSatWeight + masterSatWeight));
					int r = 1;
					for (int m = i+1; m < nObs; m++) {
						if (pos[m] !=null && satAvailPhase.contains(pos[m].getSatID()) && m != pivotIndex) {
							Qphase.set(p, p+r, 0);
							Qphase.set(p+r, p, 0);
							r++;
						}
					}
					//					int r = 1;
					//					for (int j = i+1; j < nObs; j++) {
					//						if (pos[j] !=null && satAvailPhase.contains(pos[j].getSatID()) && j != pivotIndex) {
					//							Qphase.set(p, p+r, Qphase.get(p, p+r)
					//									+ (Math.pow(lambda, 2) * Cee.get(i3 + pos[i].getSatID(), i3 + pos[j].getSatID()))
					//									* (roverPivotWeight + masterPivotWeight));
					//							Qphase.set(p+r, p, Qphase.get(p, p+r));
					//							r++;
					//						}
					//					}

					// Increment available satellite counters
					k++;
					p++;
				}
			}

			// Apply troposphere and ionosphere correction
			b = b.plus(tropoCorr);
			b = b.plus(ionoCorr);

			//Build complete cofactor matrix (code and phase)
			Q.insertIntoThis(0, 0, Qcode);
			Q.insertIntoThis(nObsAvail, nObsAvail, Qphase);

			// Least squares solution x = ((A'*Q^-1*A)^-1)*A'*Q^-1*(y0-b);
			x = A.transpose().mult(Q.invert()).mult(A).invert().mult(A.transpose())
			.mult(Q.invert()).mult(y0.minus(b));

			// Estimation of the variance of the observation error
			vEstim = y0.minus(A.mult(x).plus(b));
			double varianceEstim = (vEstim.transpose().mult(Q.invert())
					.mult(vEstim)).get(0)
					/ (nObsAvail + nObsAvailPhase - nUnknowns);

			// Covariance matrix of the estimation error
			covariance = A.transpose().mult(Q.invert()).mult(A).invert()
			.scale(varianceEstim);

			// Store estimated ambiguity combinations and their covariance
			for (int m = 0; m < satAmb.size(); m++) {
				estimatedAmbiguityComb[m] = x.get(3 + m);
				estimatedAmbiguityCombCovariance[m] = covariance.get(3 + m, 3 + m);
			}
		}

		if (init) {
			for (int i = 0; i < satAmb.size(); i++) {
				// Estimated ambiguity
				KFstate.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

				// Store the variance of the estimated ambiguity
				Cee.set(i3 + satAmb.get(i), i3 + satAmb.get(i),
						estimatedAmbiguityCombCovariance[i]);
			}
		} else {
			for (int i = 0; i < satAmb.size(); i++) {
				// Estimated ambiguity
				KFprediction.set(i3 + satAmb.get(i), 0, estimatedAmbiguityComb[i]);

				// Store the variance of the estimated ambiguity
				Cvv.set(i3 + satAmb.get(i), i3 + satAmb.get(i),
						Math.pow(goGPS.getStDevAmbiguity(), 2));
			}
		}
	}

	/**
	 * @param elevation
	 * @param snr
	 * @return weight computed according to the variable "goGPS.weights"
	 */
	private double computeWeight(double elevation, float snr) {

		double weight = 1;
		float Sa = Constants.SNR_a;
		float SA = Constants.SNR_A;
		float S0 = Constants.SNR_0;
		float S1 = Constants.SNR_1;

		if (Float.isNaN(snr) && (goGPS.getWeights() == GoGPS.WEIGHT_SIGNAL_TO_NOISE_RATIO ||
				goGPS.getWeights() == GoGPS.WEIGHT_COMBINED_ELEVATION_SNR)) {
			if(debug) System.out.println("SNR not available: forcing satellite elevation-based weights...");
			goGPS.setWeights(GoGPS.WEIGHT_SAT_ELEVATION);
		}

		switch (goGPS.getWeights()) {

			// Weight based on satellite elevation
			case GoGPS.WEIGHT_SAT_ELEVATION:
				weight = 1 / Math.pow(Math.sin(elevation * Math.PI / 180), 2);
				break;

			// Weight based on signal-to-noise ratio
			case GoGPS.WEIGHT_SIGNAL_TO_NOISE_RATIO:
				if (snr >= S1) {
					weight = 1;
				} else {
					weight = Math.pow(10, -(snr - S1) / Sa)
							* ((SA / Math.pow(10, -(S0 - S1) / Sa) - 1) / (S0 - S1)
									* (snr - S1) + 1);
				}
				break;

			// Weight based on combined elevation and signal-to-noise ratio
			case GoGPS.WEIGHT_COMBINED_ELEVATION_SNR:
				if (snr >= S1) {
					weight = 1;
				} else {
					double weightEl = 1 / Math.pow(Math.sin(elevation * Math.PI / 180), 2);
					double weightSnr = Math.pow(10, -(snr - S1) / Sa)
							* ((SA / Math.pow(10, -(S0 - S1) / Sa) - 1) / (S0 - S1) * (snr - S1) + 1);
					weight = weightEl * weightSnr;
				}
				break;

			// Same weight for all observations or default
			case GoGPS.WEIGHT_EQUAL:
			default:
				weight = 1;
		}

		return weight;
	}

	/**
	 * @param x
	 * @param y
	 * @return Lorentz inner product
	 */
	private static double lorentzInnerProduct(SimpleMatrix x, SimpleMatrix y) {

		double prod = x.get(0) * y.get(0) + x.get(1) * y.get(1) + x.get(2) * y.get(2) - x.get(3) * y.get(3);

		return prod;
	}

	/**
	 * @param elevation
	 * @param height
	 * @return troposphere correction value by Saastamoinen model
	 */
	private double computeTroposphereCorrection(double elevation, double height) {

		double tropoCorr = 0;

		if (height < 5000) {

			elevation = Math.toRadians(Math.abs(elevation));
			if (elevation == 0){
				elevation = elevation + 0.01;
			}

			// Numerical constants and tables for Saastamoinen algorithm
			// (troposphere correction)
			double hr = 50.0;
			int[] ha = new int[9];
			double[] ba = new double[9];

			ha[0] = 0;
			ha[1] = 500;
			ha[2] = 1000;
			ha[3] = 1500;
			ha[4] = 2000;
			ha[5] = 2500;
			ha[6] = 3000;
			ha[7] = 4000;
			ha[8] = 5000;

			ba[0] = 1.156;
			ba[1] = 1.079;
			ba[2] = 1.006;
			ba[3] = 0.938;
			ba[4] = 0.874;
			ba[5] = 0.813;
			ba[6] = 0.757;
			ba[7] = 0.654;
			ba[8] = 0.563;

			// Saastamoinen algorithm
			double P = Constants.STANDARD_PRESSURE * Math.pow((1 - 0.0000226 * height), 5.225);
			double T = Constants.STANDARD_TEMPERATURE - 0.0065 * height;
			double H = hr * Math.exp(-0.0006396 * height);

			// If height is below zero, keep the maximum correction value
			double B = ba[0];
			// Otherwise, interpolate the tables
			if (height >= 0) {
				int i = 1;
				while (height > ha[i]) {
					i++;
				}
				double m = (ba[i] - ba[i - 1]) / (ha[i] - ha[i - 1]);
				B = ba[i - 1] + m * (height - ha[i - 1]);
			}

			double e = 0.01
					* H
					* Math.exp(-37.2465 + 0.213166 * T - 0.000256908
							* Math.pow(T, 2));

			tropoCorr = ((0.002277 / Math.sin(elevation))
					* (P - (B / Math.pow(Math.tan(elevation), 2))) + (0.002277 / Math.sin(elevation))
					* (1255 / T + 0.05) * e);
		}

		return tropoCorr;
	}

	/**
	 * @param ionoParams
	 * @param coord
	 * @param time
	 * @return ionosphere correction value by Klobuchar model
	 */
	private double computeIonosphereCorrection(NavigationProducer navigation,
			Coordinates coord, double azimuth, double elevation, Time time) {

		double ionoCorr = 0;

		IonoGps iono = navigation.getIono(time.getMsec());
		if(iono==null) return 0.0;
//		double a0 = navigation.getIono(time.getMsec(),0);
//		double a1 = navigation.getIono(time.getMsec(),1);
//		double a2 = navigation.getIono(time.getMsec(),2);
//		double a3 = navigation.getIono(time.getMsec(),3);
//		double b0 = navigation.getIono(time.getMsec(),4);
//		double b1 = navigation.getIono(time.getMsec(),5);
//		double b2 = navigation.getIono(time.getMsec(),6);
//		double b3 = navigation.getIono(time.getMsec(),7);

		elevation = Math.abs(elevation);

		// Parameter conversion to semicircles
		double lon = coord.getGeodeticLongitude() / 180; // geod.get(0)
		double lat = coord.getGeodeticLatitude() / 180; //geod.get(1)
		azimuth = azimuth / 180;
		elevation = elevation / 180;

		// Klobuchar algorithm
		double f = 1 + 16 * Math.pow((0.53 - elevation), 3);
		double psi = 0.0137 / (elevation + 0.11) - 0.022;
		double phi = lat + psi * Math.cos(azimuth * Math.PI);
		if (phi > 0.416){
			phi = 0.416;
		}
		if (phi < -0.416){
			phi = -0.416;
		}
		double lambda = lon + (psi * Math.sin(azimuth * Math.PI))
				/ Math.cos(phi * Math.PI);
		double ro = phi + 0.064 * Math.cos((lambda - 1.617) * Math.PI);
		double t = lambda * 43200 + time.getGpsTime();
		while (t >= 86400)
			t = t - 86400;
		while (t < 0)
			t = t + 86400;
		double p = iono.getBeta(0) + iono.getBeta(1) * ro + iono.getBeta(2) * Math.pow(ro, 2) + iono.getBeta(3) * Math.pow(ro, 3);

		if (p < 72000)
			p = 72000;
		double a = iono.getAlpha(0) + iono.getAlpha(1) * ro + iono.getAlpha(2) * Math.pow(ro, 2) + iono.getAlpha(3) * Math.pow(ro, 3);
		if (a < 0)
			a = 0;
		double x = (2 * Math.PI * (t - 50400)) / p;
		if (Math.abs(x) < 1.57){
			ionoCorr = Constants.SPEED_OF_LIGHT
					* f
					* (5e-9 + a
							* (1 - (Math.pow(x, 2)) / 2 + (Math.pow(x, 4)) / 24));
		}else{
			ionoCorr = Constants.SPEED_OF_LIGHT * f * 5e-9;
		}
		return ionoCorr;
	}

	/**
	 * @param roverObs
	 * @param masterObs
	 */
	private void computeDopplerPredictedPhase(Observations roverObs, Observations masterObs) {

		this.roverDopplerPredPhase = new double[32];
		this.masterDopplerPredPhase = new double[32];

		for (int i = 0; i < satAvailPhase.size(); i++) {

			int satID = satAvailPhase.get(i);
			char satType = satTypeAvailPhase.get(i);
			
			double roverPhase = roverObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
			double masterPhase = masterObs.getSatByIDType(satID, satType).getPhaseCycles(goGPS.getFreq());
			float roverDoppler = roverObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());
			float masterDoppler = masterObs.getSatByIDType(satID, satType).getDoppler(goGPS.getFreq());

			if (!Double.isNaN(roverPhase) && !Float.isNaN(roverDoppler))
				this.setRoverDopplerPredictedPhase(satAvailPhase.get(i), roverPhase - roverDoppler);
			if (!Double.isNaN(masterPhase) && !Float.isNaN(masterDoppler))
				this.setMasterDopplerPredictedPhase(satAvailPhase.get(i), masterPhase - masterDoppler);
		}
	}

	/**
	 * @return the number of available satellites
	 */
	public int getSatAvailNumber() {
		return satAvail.size();
	}

	/**
	 * @return the number of available satellites (with phase)
	 */
	public int getSatAvailPhaseNumber() {
		return satAvailPhase.size();
	}
	
	public String getAvailGnssSystems(){
		if(satTypeAvail.isEmpty()) return "";
		String GnssSys = "";
		for(int i=0;i<satTypeAvail.size();i++) {
			if (GnssSys.indexOf((satTypeAvail.get(i))) < 0)
				GnssSys = GnssSys + satTypeAvail.get(i);
		}
		return GnssSys;
	}

	/**
	 * @return the positionCovariance
	 */
	public SimpleMatrix getPositionCovariance() {
		return positionCovariance;
	}

	/**
	 * @param positionCovariance the positionCovariance to set
	 */
	public void setPositionCovariance(SimpleMatrix positionCovariance) {
		this.positionCovariance = positionCovariance;
	}

	/**
	 * @return the receiver clock error
	 */
	public double getReceiverClockError() {
		return receiverClockError;
	}

	/**
	 * @param receiverClockError the receiver clock error to set
	 */
	public void setReceiverClockError(double receiverClockError) {
		this.receiverClockError = receiverClockError;
	}

	/**
	 * @return the rover Doppler predicted phase
	 */
	public double getRoverDopplerPredictedPhase(int satID) {
		return roverDopplerPredPhase[satID - 1];
	}

	/**
	 * @param roverDopplerPredictedPhase the Doppler predicted phase to set
	 */
	public void setRoverDopplerPredictedPhase(int satID, double roverDopplerPredictedPhase) {
		this.roverDopplerPredPhase[satID - 1] = roverDopplerPredictedPhase;
	}

	/**
	 * @return the master Doppler predicted phase
	 */
	public double getMasterDopplerPredictedPhase(int satID) {
		return masterDopplerPredPhase[satID - 1];
	}

	/**
	 * @param masterDopplerPredictedPhase the Doppler predicted phase to set
	 */
	public void setMasterDopplerPredictedPhase(int satID, double masterDopplerPredictedPhase) {
		this.masterDopplerPredPhase[satID - 1] = masterDopplerPredictedPhase;
	}

	/**
	 * @return the pDop
	 */
	public double getpDop() {
		return pDop;
	}

	/**
	 * @param pDop the pDop to set
	 */
	public void setpDop(double pDop) {
		this.pDop = pDop;
	}

	/**
	 * @return the hDop
	 */
	public double gethDop() {
		return hDop;
	}

	/**
	 * @param hDop the hDop to set
	 */
	public void sethDop(double hDop) {
		this.hDop = hDop;
	}

	/**
	 * @return the vDop
	 */
	public double getvDop() {
		return vDop;
	}

	/**
	 * @param vDop the vDop to set
	 */
	public void setvDop(double vDop) {
		this.vDop = vDop;
	}

	/**
	 * @return the kpDop
	 */
	public double getKpDop() {
		return kpDop;
	}

	/**
	 * @param kpDop the kpDop to set
	 */
	public void setKpDop(double kpDop) {
		this.kpDop = kpDop;
	}

	/**
	 * @return the khDop
	 */
	public double getKhDop() {
		return khDop;
	}

	/**
	 * @param khDop the khDop to set
	 */
	public void setKhDop(double khDop) {
		this.khDop = khDop;
	}

	/**
	 * @return the kvDop
	 */
	public double getKvDop() {
		return kvDop;
	}

	/**
	 * @param kvDop the kvDop to set
	 */
	public void setKvDop(double kvDop) {
		this.kvDop = kvDop;
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
}
