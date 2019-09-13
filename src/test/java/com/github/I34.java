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

public class I34 {

  /** https://github.com/goGPS-Project/goGPS_Java/issues/34 */
  @Test
  public void i34(){
    ObservationsProducer roverIn = new RinexObservationParser(new File("./src/test/resources/i34/axpv241z.18o"));
//    NavigationProducer navigationIn = new RinexNavigation( RinexNavigation.NASA_NAVIGATION_DAILY ); 
    NavigationProducer navigationIn = new RinexNavigation( RinexNavigation.GARNER_NAVIGATION_AUTO_HTTP ); 

    double goodDopThreshold = 3.0; 
    int TimeSampleDelaySec = 30;
    String outPath = "./src/test/resources/i32/axpv241z.kml";
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
