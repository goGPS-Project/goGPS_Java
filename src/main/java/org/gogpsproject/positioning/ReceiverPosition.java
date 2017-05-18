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
package org.gogpsproject.positioning;
import org.ejml.simple.SimpleMatrix;
import org.gogpsproject.Coordinates;
import org.gogpsproject.SatellitePosition;
import org.gogpsproject.Status;

/**
 * <p>
 * Receiver position class
 * </p>
 *
 * @author Eugenio Realini, Cryms.com, Daisuke Yoshida, Emanuele Ziglioli (Sirtrack Ltd)
 */
public class ReceiverPosition extends Position {

  /** Clock error */
	double receiverClockError; 

  /** Clock error rate */
	double receiverClockErrorRate; 
	
  /** Position dilution of precision (PDOP) */
	double pDop; 

	/** Horizontal dilution of precision (HDOP) */
	double hDop; 

  /** Vertical dilution of precision (VDOP) */
	double vDop; 
	
	/** Kalman-derived position dilution of precision (KPDOP) */
	double kpDop; 
	
	/** Kalman-derived horizontal dilution of precision (KHDOP) */
	double khDop; 
	
	/** Kalman-derived vertical dilution of precision (KVDOP) */
	double kvDop; 
	
  /** Sats in use from an observation set */
	public long satsInUse = 0;
  
	/** Average residual error for least-squares computation */
  public double eRes;

  public Status status = Status.None;

	public ReceiverPosition(){
		super();
		this.setXYZ(0.0, 0.0, 0.0);
		this.receiverClockError = 0.0;
	}

	public double getReceiverClockError() {
		return receiverClockError;
	}

	public void setReceiverClockError(double receiverClockError) {
		this.receiverClockError = receiverClockError;
	}

  public double getReceiverClockErrorRate() {
    return receiverClockErrorRate;
  }

  public void setReceiverClockErrorRate(double receiverClockErrorRate) {
    this.receiverClockErrorRate = receiverClockErrorRate;
  }

	public double getpDop() {
		return pDop;
	}

	public void setpDop(double pDop) {
		this.pDop = pDop;
	}

	public double gethDop() {
		return hDop;
	}

	public void sethDop(double hDop) {
		this.hDop = hDop;
	}

	public double getvDop() {
		return vDop;
	}

	public void setvDop(double vDop) {
		this.vDop = vDop;
	}

	public double getKpDop() {
		return kpDop;
	}

	public void setKpDop(double kpDop) {
		this.kpDop = kpDop;
	}

	public double getKhDop() {
		return khDop;
	}

	public void setKhDop(double khDop) {
		this.khDop = khDop;
	}

	public double getKvDop() {
		return kvDop;
	}

	public void setKvDop(double kvDop) {
		this.kvDop = kvDop;
	}
}
