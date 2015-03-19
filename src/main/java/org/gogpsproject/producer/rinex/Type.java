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
/**
 * <p>
 * This class holds the data Type config to output in Rinex
 * </p>
 *
 * @author Lorenzo Patocchi, cryms.com Patocchi cryms.com
 */

/**
 * @author Lorenzo Patocchi, cryms.com
 *
 */
class Type{
	public final static int C = 1;
	public final static int P = 2;
	public final static int L = 3;
	public final static int D = 4;
	public final static int S = 5;
	public final static int T = 6;

	public final static String TYPE_NOTATIONS[] = {"","C","P","L","D","S","T"};

	private int type=0;
	private int frequency=0;

	public Type(int type,int freq){
		this.type = type;
		this.frequency = freq;
	}

	public String toString(){
		if(type<0||type>=TYPE_NOTATIONS.length) return null;
		return TYPE_NOTATIONS[type]+frequency;
	}

	/**
	 * @return the type
	 */
	public int getType() {
		return type;
	}

	/**
	 * @param type the type to set
	 */
	public void setType(int type) {
		this.type = type;
	}

	/**
	 * @return the frequency
	 */
	public int getFrequency() {
		return frequency;
	}

	/**
	 * @param frequency the frequency to set
	 */
	public void setFrequency(int frequency) {
		this.frequency = frequency;
	}
}
