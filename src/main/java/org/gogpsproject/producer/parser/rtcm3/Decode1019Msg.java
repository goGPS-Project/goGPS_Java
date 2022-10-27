///=================================================================================================
// Class Decode1019Msg
//      Author :  Antoine GRENIER
//        Date :  2019/09/06
///=================================================================================================
// This code is based on the GoGPS library - Geomatics Laboratory of Politecnico di Milano, Italy.
// The GoGPS library is licenced under the LGPL.
///=================================================================================================
/*
 * Copyright 2018(c) IFSTTAR - TeamGEOLOC
 *
 * This file is part of the GeolocPVT application.
 *
 * GeolocPVT is distributed as a free software in order to build a community of users, contributors,
 * developers who will contribute to the project and ensure the necessary means for its evolution.
 *
 * GeolocPVT is a free software; you can redistribute it and/or modify it under the terms of the
 * GNU Lesser General Public License as published by the Free Software Foundation; either version 3
 * of the License, or (at your option) any later version. Any modification of source code in this
 * LGPL software must also be published under the LGPL license.
 *
 * GeolocPVT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU Lesser General Public License along with GeolocPVT.
 * If not, see <https://www.gnu.org/licenses/lgpl.txt/>.
 */
///=================================================================================================

package org.gogpsproject.producer.parser.rtcm3;

import android.location.GnssStatus;
import android.util.Log;

import org.gogpsproject.util.Bits;
import org.gogpsproject.ephemeris.GNSSEphemeris;
import org.gogpsproject.Constants;

public class Decode1019Msg implements Decode {
    public Decode1019Msg() {

    }

    public Object decode(boolean[] bits, int referenceTS) {
        GNSSEphemeris mEph = new GNSSEphemeris();
        int i = 0;

        //i = 24; //Found in RTKLIB, to be verified ...

        i += 12; //Bits of message number

        mEph.setGnssSystem(GnssStatus.CONSTELLATION_GPS);

        //Start decoding
        try {
            mEph.setPrn((int) Bits.bitsToUInt(Bits.subset(bits, i, 6)));
            i += 6;
            mEph.setWeek((int) Bits.bitsToUInt(Bits.subset(bits, i, 10)));
            i += 10;
            i += 4; //SV ACCURACY word
            i += 2; //CODE ON L2 word
            mEph.setIdot((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 14)) * Constants.P2_43 * Constants.PI_ORBIT); //BON

            i += 14;
            mEph.setIode((int) Bits.bitsToUInt(Bits.subset(bits, i, 8)));
            i += 8;
            mEph.setToc((double) Bits.bitsToUInt(Bits.subset(bits, i, 16)) * 16.0);
            i += 16;
            mEph.setAf2((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 8)) * Constants.P2_55);
            i += 8;
            mEph.setAf1((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_43);
            i += 16;
            mEph.setAf0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 22)) * Constants.P2_31);
            i += 22;
            mEph.setIodc((int) Bits.bitsToUInt(Bits.subset(bits, i, 10)));
            i += 10;
            mEph.setCrs((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_5);
            i += 16;
            mEph.setDeltaN((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_43 * Constants.PI_ORBIT);
            i += 16;
            mEph.setM0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setCuc((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_29);
            i += 16;
            mEph.setEc((double) Bits.bitsToUInt(Bits.subset(bits, i, 32)) * Constants.P2_33);
            i += 32;
            mEph.setCus((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_29);
            i += 16;
            mEph.setSquareA((double) Bits.bitsToUInt(Bits.subset(bits, i, 32)) * Constants.P2_19);
            i += 32;
            mEph.setToe((double) Bits.bitsToUInt(Bits.subset(bits, i, 16)) * 16.0);
            i += 16;
            mEph.setCic((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_29);
            i += 16;
            mEph.setOmega0(((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31) * Constants.PI_ORBIT);
            i += 32;
            mEph.setCis((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_29);
            i += 16;
            mEph.setI0((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setCrc((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 16)) * Constants.P2_5);
            i += 16;
            mEph.setOmega((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 32)) * Constants.P2_31 * Constants.PI_ORBIT);
            i += 32;
            mEph.setOmegaDot(((double) Bits.bitsTwoComplement(Bits.subset(bits, i, 24)) * Constants.P2_43)* Constants.PI_ORBIT);
            i += 24;
            i += 8;  //tgd word
            mEph.setSvh((int) Bits.bitsToUInt(Bits.subset(bits, i, 6)));
            i += 6;
        }
        catch (Exception e)
        {
            Log.e("STREAMS", "RTCM3 1019 error length" + e);
            return -1;
        }

        //Log.d("EPHEMERIS", "G PRN: " + mEph.getPrn() + ", " + mEph.getIode());

        return mEph;
    }
}