//<<<<<<<<<<<<<MOON GAZE>>>>>>>>>>>>>>>>>>>v0.0.1
//Yiyi Shao and Finlay Braithwaite - 2017
//Created for Experiment 4 for DIGF6037, Creation & Computation, OCAD University.
//PubNub & BlinkLED code modified from course example provided by Nicholas Puckett and Kate Hartman.

//Rhumb line bearing calculations sourced from 'Movable Type Scripts - Calculate distance, bearing and more between Latitude/Longitude points', © 2002-2017 Chris Veness, https://www.movable-type.co.uk/scripts/latlong.html ,  Used with Permission
//Format conversion math sourced from 'Converting UTM to Latitude and Longitude (Or Vice Versa)', Steven Dutch, Created 12 September 2003, Last Update 21 January 2016, https://www.uwgb.edu/dutchs/UsefulData/UTMFormulas.HTM


int ledPin1 = 13; //LED for local match status
int ledPin2 = 12; //LED for remote match status

int blinkRate;  //the time between blinks in milliseconds, set by local match status
bool ledState1 = false;
long lastTimeYouBlinked;  //this stores the time of the last change

#include <Adafruit_GPS.h>
#define GPSSerial Serial1 //names the GPS Serial Port
Adafruit_GPS GPS(&GPSSerial); // Connect to the GPS on the hardware port
#define GPSECHO false //mutes GPS echo on Serial

//vvvvvvvvvvvv//MagentometerInput//vvvvvvvvvvvvvvvv
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//vvvvvvvvvvvv//Wifi&PubNub//vvvvvvvvvvvvvvvv

#include <ArduinoJson.h>
#include <SPI.h>
#include <WiFi101.h>
#define PubNub_BASE_CLIENT WiFiClient
#include <PubNub.h>

static char ssid[] = "ocadu-embedded";  //SSID of the wireless network
static char pass[] = "internetofthings";  //password of that network
int status = WL_IDLE_STATUS;  // the Wifi radio's status

const static char pubkey[] = "pub-c-7170e22e-aa33-4214-bd8c-f3d2eef56e8b";
const static char subkey[] = "sub-c-961c17ee-ce76-11e7-bf34-3236001d850a";

const static char pubChannel[] = "FromDeviceA"; //publish channel
const static char subChannel[] = "FromDeviceB"; //read channel
const static char subChannelSelfRead[] = "FromDeviceA"; //self read channel for initialization on boot, same as publish channel

//vvvvvvvvvvvv//Timers//vvvvvvvvvvvvvvvv


unsigned long lastRefreshA = 0; //timer A counter
uint32_t refreshRateA = 4000;

unsigned long lastRefreshB = 0; //timer B counter
uint32_t refreshRateB = 10000;

//vvvvvvvvvvvv//Position & Match Status//vvvvvvvvvvvvvvvv

double localLatitude; //local postional data for bearing calculation and publish. Sourced from GPS if GPS is fixed.
double localLongitude;
bool localMatchStatus;  //true if facing partner, published to PubNub

double remoteLatitude;  //remote postional data for bearing calculation.
double remoteLongitude;
bool remoteMatchStatus; //true if partner is facing you.

double bearingToPartner;  //rhumb line bearing from local to remote GPS coordinates
double localHeadingDegrees; //current heading from magnetomer

int matchState = 0; //local and remote match state

void setup()
{
  pinMode(ledPin1, OUTPUT); //Set the pin to output mode
  pinMode(ledPin2, OUTPUT);
  Serial.begin(115200); //begin Serial connection
  connectToServer();  //connect to WiFi
  checkMagnetometer();  //initialize compass
  GPSsetup(); //initialize GPS
  SelfReadFromPubNub(); //fetch most recent publish to initialize bearing calculations.
}

void connectToServer()  //connect to WiFi
{
  WiFi.setPins(8, 7, 4, 2); //This is specific to the feather M0

  status = WiFi.begin(ssid, pass);                    //attempt to connect to the network
  Serial.println("***Connecting to WiFi Network***");


  for (int trys = 1; trys <= 10; trys++)                 //use a loop to attempt the connection more than once
  {
    if ( status == WL_CONNECTED)                        //check to see if the connection was successful
    {
      Serial.print("Connected to ");
      Serial.println(ssid);

      PubNub.begin(pubkey, subkey);                      //connect to the PubNub Servers
      Serial.println("PubNub Connected");
      break;                                             //exit the connection loop
    }

    else
    {
      Serial.print("Could Not Connect - Attempt:");
      Serial.println(trys);
    }

    if (trys == 10)
    {
      Serial.println("I don't this this is going to work");
    }
    delay(1000);
  }

}

void checkMagnetometer()  //initialize compass
{
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  /* Display some basic information on this sensor */
  //displaySensorDetails();
}

void GPSsetup() //initialize GPS
{


  //Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since the parser doesn't care about other sentences at this time

  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  Serial.println("GPS Connected!");

}

void SelfReadFromPubNub() //fetch most recent publish to initialize bearing calculations
{
  StaticJsonBuffer<1200> inBuffer;                    //create a memory buffer to hold a JSON Object
  WiFiClient *sClient = PubNub.history(subChannelSelfRead, 1);

  if (!sClient)
  {
    Serial.println("message read error");
    //delay(1000); //Another delay......
    return;
  }

  while (sClient->connected())
  {
    while (sClient->connected() && !sClient->available()) ; // wait
    char c = sClient->read();
    JsonObject& sMessage = inBuffer.parse(*sClient);

    if (sMessage.success())
    {
      //sMessage.prettyPrintTo(Serial); //uncomment to see the JSON message in the serial monitor
      localLatitude = sMessage["latitude"];  //
      Serial.print("SELFlatitude ");
      Serial.println(localLatitude);
      localLongitude = sMessage["longitude"];
      Serial.print("SELFlongitude ");
      Serial.println(localLongitude);
      // remoteMatchStatus = sMessage["MatchStatus"];
      // Serial.print("MatchStatus ");
      // Serial.println(remoteMatchStatus);

    }

  }

  sClient->stop();

}

void loop()
{
  masterTimerA(); //faster timer that triggers functions
  masterTimerB(); //slower timer that triggers functions
  GPSread();  //reads GPS data
  MagnetometerRead(); //reads magnetometer
  BearingCalculation(); //calculates rhumb line bearing between local and remote position
  bearingMatch(); //compares local heading against calculated bearing
  blinkRateCALC();  //calculates LED blink rate for local heading to bearing match
  blinkLED1();  //blinks LED1 based on blinkRateCALC
  RemoteMatchLED(); //turns on LED2 based on remote match status

}

void masterTimerA() //faster timer that triggers functions
{
  if (millis() - lastRefreshA >= refreshRateA)
  {
    GPSpublish(); //publishes GPS data if GPS is fixed.

    lastRefreshA = millis();
  }
}

void masterTimerB() //slower timer that triggers functions
{
  if (millis() - lastRefreshB >= refreshRateB)
  {
    readFromPubNub(); //fetches remote postinal data and match status

    lastRefreshB = millis();
  }
}

void GPSread()
{
  // read data from the GPS in the 'main loop'
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived())
  {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  //if (publishrefreshRateA > millis()) publishrefreshRateA = millis();

  // approximately every 2 seconds or so, print out the current stats
  //if (millis() - publishrefreshRateA > 2000)


}

void MagnetometerRead()
{

  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  //  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  double heading = atan2(event.magnetic.y, event.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  double declinationAngle = -0.1815; //adjusted for toronto, let's pull this from GPS?
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * M_PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * M_PI)
    heading -= 2 * M_PI;

  // Convert radians to degrees for readability.
  localHeadingDegrees = heading * 180 / M_PI;
  //float localHeadingDegrees = heading * 180/M_PI;


  Serial.print("Heading (degrees): "); Serial.println(localHeadingDegrees);

}

void BearingCalculation()
{
  //format conversions

  //decimal degree calculations not necessary
  //calculate to decimal degrees (position 1)
  //ddLAT1 = dLAT1 + mLAT1/60 + sLAT1/3600;
  //ddLONG1 = dLONG1 + mLONG1/60 + sLONG1/3600;

  //calculate to decimal degrees (position 2)
  //ddLAT2 = dLAT2 + mLAT2/60 + sLAT2/3600;
  //ddLONG2 = dLONG2 + mLONG2/60 + sLONG2/3600;

  //float ddLAT1 = 43.38; //dummy value
  // double ddLAT1 = localLatitude;

  // double ddLONG1 = localLongitude;
  //float ddLONG1 = 79.23;

  double ddLAT1 = localLatitude;  //local GPS position, sourced from SelfReadFromPubNub on boot, source from GPS if fixed.
  double ddLONG1 = localLongitude;

  double ddLAT2 = remoteLatitude; //remote GPS position, sourced from readFromPubNub
  double ddLONG2 = remoteLongitude;

  Serial.print("ddLAT1 = "); Serial.println(ddLAT1);
  Serial.print("ddLONG1 = "); Serial.println(ddLONG1);
  Serial.print("ddLAT2 = "); Serial.println(ddLAT2);
  Serial.print("ddLONG2 = "); Serial.println(ddLONG2);

  //convert from degrees to radians

  double rLAT1 = ddLAT1 * M_PI / 180.0;
  double rLAT2 = ddLAT2 * M_PI / 180.0;

  double rLONG1 = ddLONG1 * M_PI / 180.0;
  double rLONG2 = ddLONG2 * M_PI / 180.0;

  //rhumb line bearing calculations. rhumb line is a single bearing from one position to another. Crosses all meridians at the same angle.

  double deltaLONG = rLONG2 - rLONG1; //"the ‘projected’ latitude difference"
  // "if deltaLong over 180° take shorter rhumb line across the anti-meridian:"
  if (deltaLONG >  M_PI)
  {
    deltaLONG -= 2 * PI;
  }
  if (deltaLONG < -M_PI)
  {
    deltaLONG += 2 * PI;
  }
  double rhumbDIFF = log(abs(tan(rLAT2 / 2 + M_PI / 4) / tan(rLAT1 / 2 + M_PI / 4)));
  double rhumbBRNGrad = atan2(deltaLONG, rhumbDIFF);
  double rhumbBRNGdeg = rhumbBRNGrad * 180.0 / M_PI;
  bearingToPartner = fmod((rhumbBRNGdeg + 360), 360);
  Serial.print("bearingToPartner = ");
  Serial.println(bearingToPartner);

}

void bearingMatch() //compares local heading against calculated bearing
{

  if (abs(localHeadingDegrees - bearingToPartner) >= 20 && remoteMatchStatus == false && matchState != 1)
  {
    Serial.println("No one is facing each other");
    localMatchStatus = false; //local match status for publish to remote partner
    matchState = 1;
  }
  if (abs(localHeadingDegrees - bearingToPartner) >= 20 && remoteMatchStatus == true && matchState != 2)
  {
    Serial.println("Your partner is facing you");
    localMatchStatus = false;
    matchState = 2;
  }
  if (abs(localHeadingDegrees - bearingToPartner) < 20 && remoteMatchStatus == false && matchState != 3)
  {
    Serial.println("You are facing your partner");
    localMatchStatus = true;
    matchState = 3;
  }
  if (abs(localHeadingDegrees - bearingToPartner) < 20 && remoteMatchStatus == true && matchState != 4)
  {
    Serial.println("You are facing each other!");
    localMatchStatus = true;
    matchState = 4;
  }

}

void blinkRateCALC()  //calculates LED blink rate for local heading to bearing match
{
  double blinkCalc = abs(bearingToPartner - localHeadingDegrees);
  // Correct for when signs are reversed.
  if (blinkCalc > 180)
  {
    blinkCalc = -blinkCalc + 360; //this is one of my proudest moments in this project. If the absolute difference between local heading and partner bearing is greater than 180(long way around the cirlce), this corrects to make it the short way around the circle.
    Serial.println("......offset");
  }
  Serial.print("blink RATE: ");
  Serial.println(blinkCalc);
  blinkRate = blinkCalc;

}

void blinkLED1()  //blinks LED1 based on blinkRateCALC
{
  if (millis() - lastTimeYouBlinked >= blinkRate) //this very simple statement is the timer,
  { //it subtracts the value of the moment in time the last blink happened, and sees if that number is larger than your set blinking value
    if (ledState1 == false) //This statement is used to see if it should be turning the LED on or off by looking at its current state
    {
      digitalWrite(ledPin1, HIGH);  //This statement addresses the LED itself and turns it on. because it was previously off
      ledState1 = true; //It then flips then changes its current value to true
    }
    else  //Since there are only 2 options, we can use an else
    {
      digitalWrite(ledPin1, LOW); //this event happens when the LED is on so we write the pin to be LOW , which means off
      ledState1 = false;  //save that value
    }

    lastTimeYouBlinked = millis();  //save the value in time that this switch occured, so we can use it again.

  }

}

void RemoteMatchLED()  //turns on LED2 based on remote match status
{
  if (remoteMatchStatus == true)
  {
    digitalWrite(ledPin2, HIGH);

  }
  if (remoteMatchStatus == false)
  {
    digitalWrite(ledPin2, LOW);

  }
}

void GPSpublish() //publishes GPS data if GPS is fixed.
{
  if (GPS.fix)
  {
    //Serial.print("Location: ");
    Serial.println("GPS FIXED**************************************");
    localLatitude = GPS.latitudeDegrees;
    localLongitude = GPS.longitudeDegrees;
    //Serial.print(localLatitude); Serial.print(GPS.lat);
    //Serial.print(", ");
    //Serial.print(localLongitude); Serial.println(GPS.lon);
    //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    //Serial.print("Angle: "); Serial.println(GPS.angle);
    //Serial.print("Altitude: "); Serial.println(GPS.altitude);
    //Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

    publishToPubNub();
  }

}

void publishToPubNub() //publishes GPS data if GPS is fixed - PubNub component.
{
  WiFiClient *client;
  StaticJsonBuffer<800> messageBuffer;                    //create a memory buffer to hold a JSON Object
  JsonObject& pMessage = messageBuffer.createObject();    //create a new JSON object in that buffer

  ///the imporant bit where you feed in values
  pMessage["latitude"] = localLatitude;                      //add a new property and give it a value
  pMessage["longitude"] = localLongitude;
  pMessage["MatchStatus"] = localMatchStatus;


  ///                                                       //you can add/remove parameter as you like

  pMessage.prettyPrintTo(Serial);   //uncomment this to see the messages in the serial monitor


  int mSize = pMessage.measureLength() + 1;                   //determine the size of the JSON Message
  char msg[mSize];                                            //create a char array to hold the message
  pMessage.printTo(msg, mSize);                              //convert the JSON object into simple text (needed for the PN Arduino client)

  client = PubNub.publish(pubChannel, msg);                      //publish the message to PubNub

  if (!client)                                                //error check the connection
  {
    Serial.println("client error");
    //delay(1000);
    return;
  }

  if (PubNub.get_last_http_status_code_class() != PubNub::http_scc_success)  //check that it worked
  {
    Serial.print("Got HTTP status code error from PubNub, class: ");
    Serial.print(PubNub.get_last_http_status_code_class(), DEC);
  }

  while (client->available())
  {
    Serial.write(client->read());
  }
  client->stop();
  Serial.println("Successful Publish");
}

void readFromPubNub() //fetches remote postinal data and match status
{
  Serial.println("---------------------------------------------");
  StaticJsonBuffer<1200> inBuffer;                    //create a memory buffer to hold a JSON Object
  Serial.println("Setting client...");
  PubNub_BASE_CLIENT *sClient = PubNub.history(subChannel, 1);
  Serial.println("Client set.");

  if (!sClient)
  {
    Serial.println("message read error");
    //delay(1000); //Another delay......
    return;
  }

  while (sClient->connected())
  {
    Serial.println("Connected to client.");
    while (sClient->connected() && !sClient->available()) ; // wait
    char c = sClient->read();
    JsonObject& sMessage = inBuffer.parse(*sClient);

    if (sMessage.success())
    {
      Serial.println("Message received");
      //sMessage.prettyPrintTo(Serial); //uncomment to see the JSON message in the serial monitor
      remoteLatitude = sMessage["latitude"];  //
      Serial.print("latitude ");
      Serial.println(remoteLatitude);
      remoteLongitude = sMessage["longitude"];
      Serial.print("longitude ");
      Serial.println(remoteLongitude);
      remoteMatchStatus = sMessage["MatchStatus"];
      Serial.print("MatchStatus ");
      Serial.println(remoteMatchStatus);
      break;
    }

  }

  sClient->stop();
  Serial.println("--------------------------------------");
}
