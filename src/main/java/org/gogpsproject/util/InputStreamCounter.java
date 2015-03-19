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
package org.gogpsproject.util;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
/**
 * <p>
 * This class wrap around InputStream and counts the read bytes
 * </p>
 *
 * @author Lorenzo Patocchi cryms.com
 */
public class InputStreamCounter extends InputStream{

	private InputStream is;
	private OutputStream os;

	private long counter = 0;

	private long markCount;
	private long markTS;


	/**
	 *
	 */
	public InputStreamCounter(InputStream is,OutputStream os) {
		this.is = is;
		this.os = os;
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#read()
	 */
	@Override
	public int read() throws IOException {
		int c = is.read();
		//if(c>=0){
			//System.out.println("*");
			counter++;
		//}
		if(os!=null && c>=0) os.write(c);
		return c;
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#available()
	 */
	@Override
	public int available() throws IOException {
		return is.available();
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#close()
	 */
	@Override
	public void close() throws IOException {
		is.close();
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#mark(int)
	 */
	@Override
	public synchronized void mark(int readlimit) {
		is.mark(readlimit);
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#markSupported()
	 */
	@Override
	public boolean markSupported() {
		return is.markSupported();
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#read(byte[], int, int)
	 */
	@Override
	public int read(byte[] b, int off, int len) throws IOException {
		int c = is.read(b, off, len);
		if(c>0)counter += c;
		if(os!=null && c>0) os.write(b, off, c);
		//System.out.println(""+c);

		return c;
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#read(byte[])
	 */
	@Override
	public int read(byte[] b) throws IOException {
		int c = is.read(b);
		if(c>0) counter += c;
		if(os!=null && c>0) os.write(b, 0, c);
		//System.out.println(""+c);

		return c;
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#reset()
	 */
	@Override
	public synchronized void reset() throws IOException {
		is.reset();
	}

	/* (non-Javadoc)
	 * @see java.io.InputStream#skip(long)
	 */
	@Override
	public long skip(long n) throws IOException {
		//counter += n;
		//System.out.println(""+n);

		return is.skip(n);
	}

	/**
	 * @return the counter
	 */
	public long getCounter() {
		return counter;
	}

	/**
	 * @param counter the counter to set
	 */
	public void setCounter(long counter) {
		this.counter = counter;
	}

	public long start(){
		markCount = counter;
		markTS = System.currentTimeMillis();
		return markCount;
	}

	public int getCurrentBps(){
		try {
		return (int) ((counter-markCount) / ((System.currentTimeMillis()-markTS)/1000L));
		} catch (ArithmeticException e) {
			return 0;
		}
	}

	public long stop(){
		markCount = counter;
		markTS = System.currentTimeMillis();
		return markCount;
	}

}
