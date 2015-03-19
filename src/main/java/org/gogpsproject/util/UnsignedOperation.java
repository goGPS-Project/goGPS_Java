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

package org.gogpsproject.util;

public class UnsignedOperation {
	public static final int SIZEOF_LONG = Long.SIZE / Byte.SIZE;
	public static final int SIZEOF_INT = Integer.SIZE / Byte.SIZE;

	private static IllegalArgumentException explainWrongLengthOrOffset(
			final byte[] bytes, final int offset, final int length,
			final int expectedLength) {
		String reason;
		if (length != expectedLength) {
			reason = "Wrong length: " + length + ", expected " + expectedLength;
		} else {
			reason = "offset (" + offset + ") + length (" + length
					+ ") exceed the" + " capacity of the array: "
					+ bytes.length;
		}
		return new IllegalArgumentException(reason);
	}

	public static void main(String[] args) {
		int[] val = { 2, 2 };
		System.out.println(unsignedByteToIntto(val));
	}

	public static int putDouble(byte[] bytes, int offset, double d) {
		return putLong(bytes, offset, Double.doubleToLongBits(d));
	}

	public static int putFloat(byte[] bytes, int offset, float f) {
		return putInt(bytes, offset, Float.floatToRawIntBits(f));
	}

	public static int putInt(byte[] bytes, int offset, int val) {
		if (bytes.length - offset < SIZEOF_INT) {
			throw new IllegalArgumentException(
					"Not enough room to put an int at" + " offset " + offset
							+ " in a " + bytes.length + " byte array");
		}
		for (int i = offset + 3; i > offset; i--) {
			bytes[i] = (byte) val;
			val >>>= 8;
		}
		bytes[offset] = (byte) val;
		return offset + SIZEOF_INT;
	}

	public static int putLong(byte[] bytes, int offset, long val) {
		if (bytes.length - offset < SIZEOF_LONG) {
			throw new IllegalArgumentException(
					"Not enough room to put a long at" + " offset " + offset
							+ " in a " + bytes.length + " byte array");
		}
		for (int i = offset + 7; i > offset; i--) {
			bytes[i] = (byte) val;
			val >>>= 8;
		}
		bytes[offset] = (byte) val;
		return offset + SIZEOF_LONG;
	}

	public static byte[] toBytes(int val) {
		byte[] b = new byte[4];
		for (int i = 3; i > 0; i--) {
			b[i] = (byte) val;
			val >>>= 8;
		}
		b[0] = (byte) val;
		return b;

	}

	public static double toDouble(final byte[] bytes) {
		return toDouble(bytes, 0);
	}

	public static double toDouble(final byte[] bytes, final int offset) {
		return Double.longBitsToDouble(toLong(bytes, offset, SIZEOF_LONG));
	}

	public static float toFloat(byte[] bytes) {
		return toFloat(bytes, 0);
	}

	public static float toFloat(byte[] bytes, int offset) {
		return Float.intBitsToFloat(toInt(bytes, offset, SIZEOF_INT));
	}

	public static int toInt(byte[] bytes) {
		return toInt(bytes, 0, SIZEOF_INT);
	}

	public static int toInt(byte[] bytes, int offset) {
		return toInt(bytes, offset, SIZEOF_INT);
	}

	public static int toInt(byte[] bytes, int offset, final int length) {
		if (length != SIZEOF_INT || offset + length > bytes.length) {
			throw explainWrongLengthOrOffset(bytes, offset, length, SIZEOF_INT);
		}
		int n = 0;
		for (int i = offset; i < (offset + length); i++) {
			n <<= 8;
			n ^= bytes[i] & 0xFF;
		}
		return n;
	}

	public static long toLong(byte[] bytes) {
		return toLong(bytes, 0, SIZEOF_LONG);
	}

	public static long toLong(byte[] bytes, int offset) {
		return toLong(bytes, offset, SIZEOF_LONG);
	}

	public static long toLong(byte[] bytes, int offset, final int length) {
		if (length != SIZEOF_LONG || offset + length > bytes.length) {
			throw explainWrongLengthOrOffset(bytes, offset, length, SIZEOF_LONG);
		}
		long l = 0;
		for (int i = offset; i < offset + length; i++) {
			l <<= 8;
			l ^= bytes[i] & 0xFF;
		}
		return l;
	}

	public static int unsignedByteToInt(byte b) {
		return b & 0xFF;
	}

	public static int[] unsignedByteToInt(byte b[]) {
		int i[] = new int[b.length];
		for (int x = 0; x < b.length; x++)
			i[x] = b[x] & 0xFF;
		return i;
	}

	public static long unsignedByteToIntto(int i[]) {
		long val = 0L;
		int x = i.length - 1;
		for (int j = 0; j < i.length; j++) {
			val += i[j];
			val = val << x * 8;
			System.out.println((val));
			x--;
		}
		return val;
	}

	public static byte unsignedIntToByte(int b) {
		return (byte) (b & 0xFF);
	}

	public static byte[] unsignedIntToByte(int i[]) {
		byte b[] = new byte[i.length];
		for (int x = 0; x < i.length; x++)
			b[x] = (byte) (i[x] & 0xFF);
		return b;
	}

}