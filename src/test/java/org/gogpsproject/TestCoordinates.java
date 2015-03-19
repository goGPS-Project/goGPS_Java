package org.gogpsproject;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class TestCoordinates {

  /**
  *  Test converting from lat/lon to ECEF and back
  */
 //@Test
  public void computeGeodTest(){
   final double lat0 = -39.664914;
   final double lon0 = 176.881899;
   final double alt0 = 300.0;

   Coordinates c1 = Coordinates.globalGeodInstance( lat0, lon0, alt0 );
   
   double x = c1.getX();
   double y = c1.getY();
   double z = c1.getZ();
   assertEquals( -4909490.918604111, x, 0.1 );
   assertEquals( 267444.1161770544, y, 0.1 );
   assertEquals( -4049606.5536576197, z, 0.1 );
   
   Coordinates c2 = Coordinates.globalXYZInstance(x, y, z);
   c2.computeGeodetic();
   
   double lat = c2.getGeodeticLatitude();
   double lon = c2.getGeodeticLongitude();
   double alt = c2.getGeodeticHeight();
   
   assertEquals( lat0, lat, 0.1 );
   assertEquals( lon0, lon, 0.1 );
   assertEquals( alt0, alt, 0.1 );
  }

}
