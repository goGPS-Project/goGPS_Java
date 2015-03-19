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
public class GlonassSatellite {

	// GLONASS Satellite ID (Satellite Slot Number) DF038 uint6 6
	// GLONASS L1 Code Indicator DF039 bit(1) 1
	// GLONASS Satellite Frequency Channel Number DF040 uint5 5
	// GLONASS L1 Pseudorange DF041 uint25 25
	// GLONASS L1 PhaseRange – L1 Pseudorange DF042 int20 20
	// GLONASS L1 Lock time Indicator DF043 uint7 7
	// GLONASS Integer L1 Pseudorange Modulus
	// Ambiguity
	// DF044 uint7 7
	// GLONASS L1 CNR DF045 uint8 8
	// GLONASS L2 Code Indicator DF046 bit(2) 2
	// GLONASS L2-L1 Pseudorange Difference DF047 uint14 14
	// GLONASS L2 PhaseRange – L1 Pseudorange DF048 int20 20
	// GLONASS L2 Lock time Indicator DF049 uint7 7
	// GLONASS L2 CNR DF050 uint8 8

	private int satID;
	private int l1code;
	private int satFrequency;
	private long l1pseudorange;
	private double l1phaserange;
	private int l1locktime;
	private int l1psedorangemod;
	private int l1CNR;
	private int l2code;
	private double l2l1psedorangeDif;
	private double l2l1phaserangeDif;
	private int l2locktime;
	private int l2CNR;

	public int getL1CNR() {
		return l1CNR;
	}

	public int getL1code() {
		return l1code;
	}

	public int getL1locktime() {
		return l1locktime;
	}

	public double getL1phaserange() {
		return l1phaserange;
	}

	public int getL1psedorangemod() {
		return l1psedorangemod;
	}

	public long getL1pseudorange() {
		return l1pseudorange;
	}

	public int getL2CNR() {
		return l2CNR;
	}

	public int getL2code() {
		return l2code;
	}

	public double getL2l1phaserangeDif() {
		return l2l1phaserangeDif;
	}

	public double getL2l1psedorangeDif() {
		return l2l1psedorangeDif;
	}

	public int getL2locktime() {
		return l2locktime;
	}

	public int getSatFrequency() {
		return satFrequency;
	}

	public int getSatID() {
		return satID;
	}

	public void setL1CNR(int l1CNR) {
		this.l1CNR = l1CNR;
	}

	public void setL1code(int l1code) {
		this.l1code = l1code;
	}

	public void setL1locktime(int l1locktime) {
		this.l1locktime = l1locktime;
	}

	public void setL1phaserange(double l1phaserange) {
		this.l1phaserange = l1phaserange;
	}

	public void setL1psedorangemod(int l1psedorangemod) {
		this.l1psedorangemod = l1psedorangemod;
	}

	public void setL1pseudorange(long l1pseudorange) {
		this.l1pseudorange = l1pseudorange;
	}

	public void setL2CNR(int l2CNR) {
		this.l2CNR = l2CNR;
	}

	public void setL2code(int l2code) {
		this.l2code = l2code;
	}

	public void setL2l1phaserangeDif(double l2l1phaserangeDif) {
		this.l2l1phaserangeDif = l2l1phaserangeDif;
	}

	public void setL2l1psedorangeDif(double l2l1psedorangeDif) {
		this.l2l1psedorangeDif = l2l1psedorangeDif;
	}

	public void setL2locktime(int l2locktime) {
		this.l2locktime = l2locktime;
	}

	public void setSatFrequency(int satFrequency) {
		this.satFrequency = satFrequency;
	}

	public void setSatID(int satID) {
		this.satID = satID;
	}

	@Override
	public String toString() {
		return "GlonassSatellite [l1CNR=" + l1CNR + ", l1code=" + l1code
				+ ", l1locktime=" + l1locktime + ", l1phaserange="
				+ l1phaserange + ", l1psedorangemod=" + l1psedorangemod
				+ ", l1pseudorange=" + l1pseudorange + ", l2CNR=" + l2CNR
				+ ", l2code=" + l2code + ", l2l1phaserangeDif="
				+ l2l1phaserangeDif + ", l2l1psedorangeDif="
				+ l2l1psedorangeDif + ", l2locktime=" + l2locktime
				+ ", satFrequency=" + satFrequency + ", satID=" + satID + "]";
	}

}
