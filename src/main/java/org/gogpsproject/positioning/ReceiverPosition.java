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
import org.gogpsproject.Coordinates;
import org.gogpsproject.Status;

/**
 * <p>
 * Receiver position class
 * </p>
 *
 * @author Eugenio Realini, Cryms.com, Daisuke Yoshida, Emanuele Ziglioli (Sirtrack Ltd)
 */
public class ReceiverPosition extends Coordinates{

	double receiverClockError; /* Clock error */
	double receiverClockErrorRate; /* Clock error rate */
	double pDop; /* Position dilution of precision (PDOP) */
	double hDop; /* Horizontal dilution of precision (HDOP) */
	double vDop; /* Vertical dilution of precision (VDOP) */
	double kpDop; /* Kalman-derived position dilution of precision (KPDOP) */
	double khDop; /* Kalman-derived horizontal dilution of precision (KHDOP) */
	double kvDop; /* Kalman-derived vertical dilution of precision (KVDOP) */

  public Status status = Status.None;

  public long satsInUse = 0;
  public double eRes;

	public ReceiverPosition(){
		super();
		this.setXYZ(0.0, 0.0, 0.0);
		this.receiverClockError = 0.0;
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
   * @return the receiver clock error rate
   */
  public double getReceiverClockErrorRate() {
    return receiverClockErrorRate;
  }

  public void setReceiverClockErrorRate(double receiverClockErrorRate) {
    this.receiverClockErrorRate = receiverClockErrorRate;
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
}
