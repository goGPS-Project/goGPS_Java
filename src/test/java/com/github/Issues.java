package com.github;

import org.junit.Test;

import org.gogpsproject.GoGPS;
import org.gogpsproject.GoGPS.DynamicModel;
import org.gogpsproject.GoGPS.RunMode;
import org.gogpsproject.consumer.KmlProducer;
import org.gogpsproject.producer.NavigationProducer;
import org.gogpsproject.producer.ObservationsProducer;
import org.gogpsproject.producer.parser.rinex.RinexNavigation;
import org.gogpsproject.producer.parser.rinex.RinexObservationParser;
import org.gogpsproject.producer.parser.ublox.UBXFileReader;

import static org.junit.Assert.*;

import java.io.*;

public class Issues {

  /**
   * Code by @MartaBanach
   * See https://github.com/goGPS-Project/goGPS_Java/issues/27#issuecomment-290527675
   */
  @Test
  public void i27(){
    ObservationsProducer roverIn = new UBXFileReader(new File("./src/test/resources/i27/ublox.ubx"));
    ObservationsProducer masterIn = new RinexObservationParser(new File("./src/test/resources/i27/vrs.17o"));
//    NavigationProducer navigationIn = new RinexNavigationParser(new File("./src/test/resources/i27/vrs.17n"));
    NavigationProducer navigationIn = new RinexNavigation( RinexNavigation.NASA_NAVIGATION_DAILY ); 

    double goodDopThreshold = 3.0; // Threshold - pr�g graniczny
    int TimeSampleDelaySec = 30;
    String outPath = "./src/test/resources/i27/out.kml";
    // should be tuned according to the dataset;
    // use '0' to disable timestamps in the KML String outPath = "./test/out.kml";
    try {
        KmlProducer kml = new KmlProducer(outPath, goodDopThreshold, TimeSampleDelaySec);

        navigationIn.init();
        roverIn.init();
        masterIn.init();

        GoGPS goGPS = new GoGPS(navigationIn, roverIn, masterIn)
                         .addPositionConsumerListener(kml)
                         .setDynamicModel(DynamicModel.STATIC)
                         .setCutoff(0)
        
        .run( RunMode.CODE_STANDALONE );
//        .run( RunMode.KALMAN_FILTER_CODE_PHASE_STANDALONE );
//                         .run( RunMode.KALMAN_FILTER_DOUBLE_DIFF); // -> Missing M or R obs
        
        roverIn.release(true, 10000);
        masterIn.release(true, 10000);
        navigationIn.release(true, 10000);
        
        goGPS.runUntilFinished();
    } catch (Exception e) {
        e.printStackTrace();
    }

  }

  /** https://github.com/goGPS-Project/goGPS_Java/issues/32 */
  @Test
  public void i32(){
    String name = "ublox2";
    ObservationsProducer roverIn = new UBXFileReader(new File("./src/test/resources/i32/Fail/" + name + ".ubx"));
//    NavigationProducer navigationIn = new RinexNavigation( RinexNavigation.NASA_NAVIGATION_DAILY ); 
    NavigationProducer navigationIn = new RinexNavigation( RinexNavigation.GARNER_NAVIGATION_AUTO_HTTP ); 

    double goodDopThreshold = 3.0; 
    int TimeSampleDelaySec = 30;
    String outPath = "./src/test/resources/i32/Fail/"+ name + ".kml";
    try {
        KmlProducer kml = new KmlProducer(outPath, goodDopThreshold, TimeSampleDelaySec);

        navigationIn.init();
        roverIn.init();

        GoGPS goGPS = new GoGPS(navigationIn, roverIn )
                                     .addPositionConsumerListener(kml)
                                     .setDynamicModel(DynamicModel.STATIC)
                                     .setCutoff(0)
                                     .run( RunMode.CODE_STANDALONE_SNAPSHOT )
                                     .runUntilFinished();

        assertTrue( goGPS.getRoverPos().isValidXYZ() );
        
    } catch (Exception e) {
        e.printStackTrace();
    }

  }

}
