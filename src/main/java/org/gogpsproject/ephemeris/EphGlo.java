/*
 * Copyright (c) 2010, Eugenio Realini, Mirko Reguzzoni, Cryms sagl, Daisuke Yoshida. All Rights Reserved.
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
package org.gogpsproject.ephemeris;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;

import org.gogpsproject.positioning.Time;
import org.gogpsproject.producer.Streamable;

/**
 * <p>
 * GPS broadcast ephemerides
 * </p>
 *
 * @author Eugenio Realini, Cryms.com, Daisuke Yoshida 
 */

public class EphGlo implements Streamable {
	private final static int STREAM_V = 1;

	private Time refTime; /* Reference time of the dataset */
	private char satType; /* Satellite Type */
	private int satID; /* Satellite ID number */
	private int week; /* GPS week number */
	private double toc; /* clock data reference time */
	private double tow;
	private double toe; /* ephemeris reference time */
	
	private float tauN;
	private float gammaN;
	private double tk;

	private double X;
	private double Xv;
	private double Xa;
	private double Bn;
	
	private double Y;
	private double Yv;
	private double Ya;
	private double freq_num;
	private double tb;

	private double Z;
	private double Zv;
	private double Za;
	private double En;


	public EphGlo(){

	}
	public EphGlo(DataInputStream dai, boolean oldVersion) throws IOException{
		read(dai,oldVersion);
	}

	/**
	 * @return the refTime
	 */
	public Time getRefTime() {
		return refTime;
	}
	/**
	 * @param refTime the refTime to set
	 */
	public void setRefTime(Time refTime) {
		this.refTime = refTime;
	}
	/**
	 * @return the satType
	 */
	public char getSatType() {
		return satType;
	}
	/**
	 * @param satType the satType to set
	 */
	public void setSatType(char satType) {
		this.satType = satType;
	}
	/**
	 * @return the satID
	 */
	public int getSatID() {
		return satID;
	}
	/**
	 * @param satID the satID to set
	 */
	public void setSatID(int satID) {
		this.satID = satID;
	}
	/**
	 * @return the week
	 */
	public int getWeek() {
		return week;
	}
	/**
	 * @param week the week to set
	 */
	public void setWeek(int week) {
		this.week = week;
	}
	/**
	 * @return the toc
	 */
	public double getToc() {
		return toc;
	}
	/**
	 * @param toc the toc to set
	 */
	public void setToc(double toc) {
		this.toc = toc;
	}
	/**
	 * @return the tow
	 */
	public double getTow() {
		return tow;
	}
	/**
	 * @param tow the tow to set
	 */
	public void setTow(double tow) {
		this.tow = tow;
	}
	/**
	 * @return the toe
	 */
	public double getToe() {
		return toe;
	}
	/**
	 * @param toe the toe to set
	 */
	public void setToe(double toe) {
		this.toe = toe;
	}
	
	
	public float getTauN() {
		return tauN;
	}
	public void setTauN(float tauN) {
		this.tauN = tauN;
	}
	
	public float getGammaN() {
		return gammaN;
	}
	public void setGammaN(float gammaN) {
		this.gammaN = gammaN;
	}
	
	public double gettk() {
		return tk;
	}
	public void settk(double tk) {
		this.tk = tk;
	}
	
	public double getX() {
		return X;
	}
	public void setX(double X) {
		this.X = X;
	}
	
	public double getXv() {
		return Xv;
	}
	public void setXv(double Xv) {
		this.Xv = Xv;
	}
	
	public double getXa() {
		return Xa;
	}
	public void setXa(double Xa) {
		this.Xa = Xa;
	}
	
	public double getBn() {
		return Bn;
	}
	public void setBn(double Bn) {
		this.Bn = Bn;
	}
	
	public double getY() {
		return Y;
	}
	public void setY(double Y) {
		this.Y = Y;
	}
	
	public double getYv() {
		return Yv;
	}
	public void setYv(double Yv) {
		this.Yv = Yv;
	}
	
	public double getYa() {
		return Ya;
	}
	public void setYa(double Ya) {
		this.Ya = Ya;
	}
	
	public double getfreq_num() {
		return freq_num;
	}
	public void setfreq_num(double freq_num) {
		this.freq_num = freq_num;
	}
	
	public double gettb() {
		return tb;
	}
	public void settb(double tb) {
		this.tb = tb;
	}
	
	public double getZ() {
		return Z;
	}
	public void setZ(double Z) {
		this.Z = Z;
	}
	
	public double getZv() {
		return Zv;
	}
	public void setZv(double Zv) {
		this.Zv = Zv;
	}
	
	public double getZa() {
		return Za;
	}
	public void setZa(double Za) {
		this.Za = Za;
	}
	
	public double getEn() {
		return En;
	}
	public void setEn(double En) {
		this.En = En;
	}
	
//	public long getEn() {
//		return En;
//	}
//	public void setEn(long En) {
//		this.En = En;
//	}
	
	
	/* (non-Javadoc)
	 * @see org.gogpsproject.Streamable#write(java.io.DataOutputStream)
	 */
	@Override
	public int write(DataOutputStream dos) throws IOException {
		int size=5;
		dos.writeUTF(MESSAGE_EPHEMERIS); // 5
		dos.writeInt(STREAM_V); size+=4; // 4

		dos.writeLong(refTime==null?-1:refTime.getMsec());  size +=8;
		dos.write(satID);  size +=1;
		dos.writeInt(week); size +=4;

//		dos.writeInt(L2Code); size +=4;
//		dos.writeInt(L2Flag); size +=4;
//
//		dos.writeInt(svAccur); size +=4;
//		dos.writeInt(svHealth); size +=4;
//
//		dos.writeInt(iode); size +=4;
//		dos.writeInt(iodc); size +=4;
//
//		dos.writeDouble(toc); size +=8;
//		dos.writeDouble(toe); size +=8;
//
//		dos.writeDouble(af0); size +=8;
//		dos.writeDouble(af1); size +=8;
//		dos.writeDouble(af2); size +=8;
//		dos.writeDouble(tgd); size +=8;
//
//
//		dos.writeDouble(rootA); size +=8;
//		dos.writeDouble(e); size +=8;
//		dos.writeDouble(i0); size +=8;
//		dos.writeDouble(iDot); size +=8;
//		dos.writeDouble(omega); size +=8;
//		dos.writeDouble(omega0); size +=8;
//
//		dos.writeDouble(omegaDot); size +=8;
//		dos.writeDouble(M0); size +=8;
//		dos.writeDouble(deltaN); size +=8;
//		dos.writeDouble(crc); size +=8;
//		dos.writeDouble(crs); size +=8;
//		dos.writeDouble(cuc); size +=8;
//		dos.writeDouble(cus); size +=8;
//		dos.writeDouble(cic); size +=8;
//		dos.writeDouble(cis); size +=8;
//
//		dos.writeDouble(fitInt); size +=8;

		return size;
	}
	/* (non-Javadoc)
	 * @see org.gogpsproject.Streamable#read(java.io.DataInputStream)
	 */
	@Override
	public void read(DataInputStream dai, boolean oldVersion) throws IOException {
		int v=1;
		if(!oldVersion) v=dai.readInt();

		if(v==1){
			long l = dai.readLong();
			refTime = new Time(l>0?l:System.currentTimeMillis());
			satID = dai.read();
			week = dai.readInt();
//			L2Code = dai.readInt();
//			L2Flag = dai.readInt();
//			svAccur = dai.readInt();
//			svHealth = dai.readInt();
//			iode = dai.readInt();
//			iodc = dai.readInt();
//			toc = dai.readDouble();
//			toe = dai.readDouble();
//			af0 = dai.readDouble();
//			af1 = dai.readDouble();
//			af2 = dai.readDouble();
//			tgd = dai.readDouble();
//			rootA = dai.readDouble();
//			e = dai.readDouble();
//			i0 = dai.readDouble();
//			iDot = dai.readDouble();
//			omega = dai.readDouble();
//			omega0 = dai.readDouble();
//			omegaDot = dai.readDouble();
//			M0 = dai.readDouble();
//			deltaN = dai.readDouble();
//			crc = dai.readDouble();
//			crs = dai.readDouble();
//			cuc = dai.readDouble();
//			cus = dai.readDouble();
//			cic = dai.readDouble();
//			cis = dai.readDouble();
//			fitInt = dai.readDouble();
		}else{
			throw new IOException("Unknown format version:"+v);
		}
	}

}
