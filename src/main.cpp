// Signal K application template file.
//
// This application demonstrates core SensESP concepts in a very
// concise manner. You can build and upload the application as is
// and observe the value changes on the serial port monitor.
//
// You can use this source file as a basis for your own projects.
// Remove the parts that are not relevant to you, and add your own code
// for external hardware libraries.

// Comment out this line to disable NMEA 2000 output.
#define ENABLE_NMEA2000_OUTPUT

#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <Wire.h>

#ifdef ENABLE_NMEA2000_OUTPUT
#include <NMEA2000_esp32.h>
#endif

#include "halmet_analog.h"
#include "halmet_const.h"
#include "halmet_digital.h"
#include "halmet_display.h"
#include "halmet_serial.h"
#include "n2k_senders.h"
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp_app_builder.h"

#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp_onewire/onewire_temperature.h"

#include "time_counter.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/curveinterpolator.h"


using namespace sensesp;

/////////////////////////////////////////////////////////////////////
//deze variabelen gaan naar SetN2kPNG
double N2kfrequency = N2kDoubleNA;
double N2kengine_hours = N2kDoubleNA;
double N2kfuel_rate = N2kDoubleNA;
double fuel_rate;
float avgFuelRate = 0.0000041667;

/////////////////////////////////////////////////////////////////////
//I2C Variabelen
double outside_pressure = N2kDoubleNA;        
double inside_temperature = N2kDoubleNA;
double outside_humidity = N2kDoubleNA;

double oil_temperature = N2kDoubleNA;         // 1-Wire drie maal thermometer
double coolant_temperature = N2kDoubleNA;
double outside_temperature = N2kDoubleNA;  


double frequency = N2kDoubleNA;
double RPM = N2kDoubleNA;
double engine_hours = N2kDoubleNA;


bool alarm_2_input  = false;
bool alarm_3_input  = false;
bool alarm_4_input  = false;
bool bilge  = false;


//////////
//voor de urenteller
unsigned long cycle_start_time = 0;
unsigned long freq_start_time = 0;
int freq = 0;



/////////////////////////////////////////////////////////////////////
// Declare some global variables required for the firmware operation.

#ifdef ENABLE_NMEA2000_OUTPUT
tNMEA2000* nmea2000;
#endif

TwoWire* i2c;
Adafruit_SSD1306* display;


//********DEZE LATER NAAR N2K_SENDERS.H VERPLAATSEN!!!
//De toerentalparameters 
void SendEngineData() {
  tN2kMsg N2kMsg;
  SetN2kEngineDynamicParam	(N2kMsg,
                             0,                   // unsigned char 	EngineInstance,
                             N2kDoubleNA,         // double 	EngineOilPress,
                             oil_temperature,     // double 	EngineOilTemp,
                             coolant_temperature, // double 	EngineCoolantTemp,
                             N2kDoubleNA,         // double 	AltenatorVoltage,
                             N2kfuel_rate,        // double 	FuelRate,
                             N2kengine_hours,        // double 	EngineHours,
                             N2kDoubleNA,         // double 	EngineCoolantPress = N2kDoubleNA,
                             N2kDoubleNA,         // double 	EngineFuelPress = N2kDoubleNA,
                             N2kInt8NA,           // int8_t 	EngineLoad = N2kInt8NA,
                             N2kInt8NA,           // int8_t 	EngineTorque = N2kInt8NA,
                             0,                   // bool 	flagCheckEngine = false,
                             0,                   // bool 	flagOverTemp = false,
                             0,                   // bool 	flagLowOilPress = false,
                             0,                   // bool 	flagLowOilLevel = false,
                             0,                   // bool 	flagLowFuelPress = false,
                             0,                   // bool 	flagLowSystemVoltage = false,
                             0,             // bool 	flagLowCoolantLevel = false,
                             0,             // bool 	flagWaterFlow = false,
                             0,             // bool 	flagWaterInFuel = false,
                             0,             // bool 	flagChargeIndicator = false,
                             0,             // bool 	flagPreheatIndicator = false,
                             0,             // bool 	flagHighBoostPress = false,
                             0,             // bool 	flagRevLimitExceeded = false,
                             0,             // bool 	flagEgrSystem = false,
                             0,             // bool 	flagTPS = false,
                             0,             // bool 	flagEmergencyStopMode = false,
                             0,             // bool 	flagWarning1 = false,
                             0,             // bool 	flagWarning2 = false,
                             0,             // bool 	flagPowerReduction = false,
                             0,             // bool 	flagMaintenanceNeeded = false,
                             0,             // bool 	flagEngineCommError = false,
                             0,             // bool 	flagSubThrottle = false,
                             0,             // bool 	flagNeutralStartProtect = false,
                             0);             // bool 	flagEngineShuttingDown = false 
  nmea2000->SendMsg(N2kMsg);
}

//De omgevingsparameters   OK
void SendingEnvironment() {
  tN2kMsg N2kMsg;
  SetN2kEnvironmentalParameters(N2kMsg,
                                      0,
                                      N2kts_InsideTemperature,
                                      inside_temperature,         //WaterTemperature,
                                      N2khs_OutsideHumidity,
                                      outside_humidity,
                                      outside_pressure);
  nmea2000->SendMsg(N2kMsg);
}

//De omgevingsparameters   OK
void SendingN2kPGN130310() {
  tN2kMsg N2kMsg;
  SetN2kPGN130310( N2kMsg,
                     0,                     //unsigned char 	SID,
                     N2kDoubleNA,           // double 	WaterTemperature,
                     outside_temperature,   // double 	OutsideAmbientAirTemperature = N2kDoubleNA,
                     N2kDoubleNA);          // double 	AtmosphericPressure = N2kDoubleNA 
  nmea2000->SendMsg(N2kMsg);
}

// //De toerentalparameters 
void SendingN2kPGN127488() {
  tN2kMsg N2kMsg;
  SetN2kPGN127488(N2kMsg,
                  0,
                  N2kfrequency,        //double 	EngineSpeed,
                  N2kDoubleNA,      //double 	EngineBoostPressure = N2kDoubleNA,
                  N2kInt8NA);       //int8_t 	EngineTiltTrim = N2kInt8NA 
  nmea2000->SendMsg(N2kMsg);
}


//EINDE VERSTUREN NMEA2000 naar het netwerk
//********DEZE LATER NAAR N2K_SENDERS.H VERPLAATSEN!!!




reactesp::ReactESP app;

// Store alarm states in an array for local display output
bool alarm_states[4] = {false, false, false, false};

/////////////////////////////////////////////////////////////////////
// Test output pin configuration. If ENABLE_TEST_OUTPUT_PIN is defined,
// GPIO 33 will output a pulse wave at 380 Hz with a 50% duty cycle.
// If this output and GND are connected to one of the digital inputs, it can
// be used to test that the frequency counter functionality is working.
#define ENABLE_TEST_OUTPUT_PIN
#ifdef ENABLE_TEST_OUTPUT_PIN
const int kTestOutputPin = GPIO_NUM_33;
// With the default pulse rate of 100 pulses per revolution (configured in
// halmet_digital.cpp), this frequency corresponds to 3.8 r/s or about 228 rpm.
const int kTestOutputFrequency = 380;
#endif

/////////////////////////////////////////////////////////////////////
//Lookup fuel per rpm voor de M3.10 van VETUS
class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to M^3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, M^3/s))   1,5 liter=0,002M^3/uur 0.0000041667 M^3/sec;
    add_sample(CurveInterpolator::Sample(500, 0.00000882));
    add_sample(CurveInterpolator::Sample(1000, 0.00000882));
    add_sample(CurveInterpolator::Sample(1400, 0.00000573));
    add_sample(CurveInterpolator::Sample(1500, 0.00000512));
    add_sample(CurveInterpolator::Sample(1700, 0.00000480));
    add_sample(CurveInterpolator::Sample(2000, 0.00000441));
    add_sample(CurveInterpolator::Sample(2300, 0.00000480));
    add_sample(CurveInterpolator::Sample(2600, 0.00000513));
    add_sample(CurveInterpolator::Sample(2800, 0.00000573));
    add_sample(CurveInterpolator::Sample(3000, 0.00000633)); 
  }
};

///////////////////////////////////////////////////////////me
// Convenience function to found the addresses the I2C bus
void ScanI2C(TwoWire* i2c) {                                  
  uint8_t error, address;
  Serial.println("Scanning...");
  for (address = 1; address < 127; address++) {
    i2c->beginTransmission(address);
    error = i2c->endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  Serial.println("done");
}

///////////////////////////////////////////////////////////////////////me
//I2C BME280_ADDRESS
Adafruit_BME280 bme280;                                                       
  float read_temp_callback() { return (bme280.readTemperature() + 273.15);}   //Uitlezen en aanpassen temp in Kelvin voor SK
  float read_pressure_callback() { return (bme280.readPressure());}
  float read_humidity_callback() { return (bme280.readHumidity());} 

/////////////////////////////////////////////////////////////////////
// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_DISABLED
  SetupSerialDebug(115200);
#endif

  // initialize the I2C bus
  i2c = new TwoWire(0);
  i2c->begin(kSDAPin, kSCLPin);

  // Initialize ADS1115
  auto ads1115 = new Adafruit_ADS1115();
  ads1115->setGain(GAIN_ONE);
  bool ads_initialized = ads1115->begin(kADS1115Address, i2c);
  debugD("ADS1115 initialized: %d", ads_initialized);

#ifdef ENABLE_TEST_OUTPUT_PIN
  pinMode(kTestOutputPin, OUTPUT);
  // Set the LEDC peripheral to a 13-bit resolution
  ledcSetup(0, kTestOutputFrequency, 13);
  // Attach the channel to the GPIO pin to be controlled
  ledcAttachPin(kTestOutputPin, 0);
  // Set the duty cycle to 50%
  // Duty cycle value is calculated based on the resolution
  // For 13-bit resolution, max value is 8191, so 50% is 4096
  ledcWrite(0, 4096);
#endif

#ifdef ENABLE_NMEA2000_OUTPUT
  // Initialize NMEA 2000

  nmea2000 = new tNMEA2000_esp32(kCANTxPin, kCANRxPin);

  // Reserve enough buffer for sending all messages.
  nmea2000->SetN2kCANSendFrameBufSize(250);
  nmea2000->SetN2kCANReceiveFrameBufSize(250);

  // Set Product information
  // EDIT: Change the values below to match your device.
  nmea2000->SetProductInformation(
      "13233",  // Manufacturer's Model serial code (max 32 chars)
      104,         // Manufacturer's product code
      "MBS-Engine for Vetus M3.10",    // Manufacturer's Model ID (max 33 chars)
      "2.0.0",     // Manufacturer's Software version code (max 40 chars)
      "2.0.0"      // Manufacturer's Model version (max 24 chars))
  );

  // For device class/function information, see:
  // http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf

  // For mfg registration list, see:
  // https://actisense.com/nmea-certified-product-providers/
  // The format is inconvenient, but the manufacturer code below should be
  // one not already on the list.

  // EDIT: Change the class and function values below to match your device.
  nmea2000->SetDeviceInformation(
      GetBoardSerialNumber(),  // Unique number. Use e.g. Serial number.
      140,                     // Device function: Engine
      50,                      // Device class: Propulsion
      2046);                   // Manufacturer code

  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly,
                    71  // Default N2k node address
  );
  nmea2000->EnableForward(false);
  nmea2000->Open();

  // No need to parse the messages at every single loop iteration; 1 ms will do
  app.onRepeat(1, []() { nmea2000->ParseMessages(); });
#endif  // ENABLE_NMEA2000_OUTPUT

  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("MBS-Engine 2.0")          // De produktnaam en build
                    // Optionally, hard-code the WiFi and Signal K server
                    ->set_wifi("MERLYN", "Merlyn26")
                    ->set_sk_server("192.168.1.112", 3000)       //  Deze aanpassen voor laatste install!!!!!!!
                    // settings. This is normally not needed.
                    // SensESP has several builtin sensors, e.g. freemem, uptime, IP address
                    //  Optionally enable them here to output their values in SK reports.
                    ->enable_uptime_sensor()
                    ->enable_ip_address_sensor()
                    ->enable_free_mem_sensor()
                    ->enable_system_hz_sensor()
                    ->enable_wifi_signal_sensor()
                   // ->set_button_pin(-1) //uitzetten van de BOOT knop op de esp32=(-1) strandaard (0)
                    ->enable_ota("Merlyn120") //Activating Over The Air (OTA) updates
                    ->get_app();

  // Initialize the OLED display
  bool display_present = InitializeSSD1306(&app, sensesp_app, &display, i2c);

//////////////////////////////////////////////////////////////////////////////////
// 1-Wire Temp Sensors -  

DallasTemperatureSensors* dts = new DallasTemperatureSensors(ONEWIRE_PIN);          
                                                                              
  auto main_engine_oil_temperature =                                                // define three 1-Wire temperature sensor that update every 1000 ms
      new OneWireTemperature(dts, 1000, "/mainEngineOilTemp/oneWire");
  auto main_engine_coolant_temperature =                                            // and have specific web UI configuration paths
      new OneWireTemperature(dts, 1000, "/mainEngineCoolantTemp/oneWire");
  auto environment_outside_temperature =
      new OneWireTemperature(dts, 1000, "/environmentOutsideTemperature/oneWire");  // Outside temperature!!!!!! was mainEngineWetExhaustTemp

  auto main_engine_oil_temperature_metadata =   // define SK metadata for sensors
      new SKMetadata("K",                       // units
                     "Engine Oil Temperature",  // display name
                     "Engine Oil Temperature",  // description
                     "Oil Temperature",         // short name
                     10.                        // timeout, in seconds
  );

  auto main_engine_coolant_temperature_metadata =
      new SKMetadata("K",                           // units
                     "Engine Coolant Temperature",  // display name
                     "Engine Coolant Temperature",  // description
                     "Coolant Temperature",         // short name
                     10.                            // timeout, in seconds
   );

auto environment_outside_temperature_metadata =
      new SKMetadata("K",                           // units
                     "Outside Temperature",  // display name
                     "Outside Temperature",  // description
                     "Outside Temperature",         // short name
                     10.                            // timeout, in seconds
   );

  main_engine_oil_temperature->connect_to(new SKOutput<float>(                    // Connect the sensors to Signal K output paths
      "propulsion.main.oilTemperature", "/mainEngineOilTemp/skPath",
      main_engine_oil_temperature_metadata));

  main_engine_coolant_temperature->connect_to(new SKOutput<float>(
      "propulsion.main.coolantTemperature", "/mainEngineCoolantTemp/skPath",
      main_engine_coolant_temperature_metadata));

  environment_outside_temperature->connect_to(new SKOutputFloat(
      "environment.outside.temperature", "/outsideTemperatureTemp/skPath",
      environment_outside_temperature_metadata));  // environment_outside_temperature  

//N2K
  main_engine_oil_temperature->connect_to(                    // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float temperature) {
        oil_temperature = temperature;
        SendEngineData();
      }));

  main_engine_coolant_temperature->connect_to(                // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float temperature) {
        coolant_temperature = temperature;
        SendEngineData();
      }));

  environment_outside_temperature->connect_to(                // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float temperature) {
        inside_temperature = temperature;
        SendingN2kPGN130310();
      }));





  ///////////////////////////////////////////////////////////////////
  // Analog inputs

  // Connect the tank senders.
  // EDIT: To enable more tanks, uncomment the lines below.
  auto tank_a1_volume = ConnectTankSender(ads1115, 0, "fuel");
  // auto tank_a2_volume = ConnectTankSender(ads1115, 1, "A2");
  // auto tank_a3_volume = ConnectTankSender(ads1115, 2, "A3");
  // auto tank_a4_volume = ConnectTankSender(ads1115, 3, "A4");

#ifdef ENABLE_NMEA2000_OUTPUT
  // Tank 1, instance 0. Capacity 60 liters.
  // EDIT: Make sure this matches your tank configuration above.
  N2kFluidLevelSender* tank_a1_sender = new N2kFluidLevelSender(
      "/NMEA 2000/Tank 1", 0, N2kft_Fuel, 60, nmea2000);
  tank_a1_volume->connect_to(&(tank_a1_sender->tank_level_consumer_));
#endif  // ENABLE_NMEA2000_OUTPUT

  if (display_present) {
    // EDIT: Duplicate the lines below to make the display show all your tanks.
    tank_a1_volume->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 2, "Tank A1", 100 * value); }));
  }

  ///////////////////////////////////////////////////////////////////
  // Digital alarm inputs

  // EDIT: More alarm inputs can be defined by duplicating the lines below.
  // Make sure to not define a pin for both a tacho and an alarm.
  auto alarm_d2_input = ConnectAlarmSender(kDigitalInputPin2, "D2");
  auto alarm_d3_input = ConnectAlarmSender(kDigitalInputPin3, "D3");
  auto alarm_d4_input = ConnectAlarmSender(kDigitalInputPin4, "D4");

   // Update the alarm states based on the input value changes.
  // EDIT: If you added more alarm inputs, uncomment the respective lines below.

  // propulsion.Engine.flagLowOilLevel
  alarm_d2_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[1] = value; }));

  // propulsion.Engine.flagLowOilPress   
  alarm_d3_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[2] = value; }));
  
  //propulsion.Engine.flagOverTemp
  alarm_d4_input->connect_to(
      new LambdaConsumer<bool>([](bool value) { alarm_states[3] = value; }));

//Alarms to SK here!



#ifdef ENABLE_NMEA2000_OUTPUT
  // EDIT: This example connects the D2 alarm input to the low oil pressure
  // warning. Modify according to your needs.
  N2kEngineParameterDynamicSender* engine_dynamic_sender =
      new N2kEngineParameterDynamicSender("/NMEA 2000/Engine 1 Dynamic", 0,
                                          nmea2000);
  alarm_d2_input->connect_to(
      &(engine_dynamic_sender->low_oil_level_consumer_));  
  
  alarm_d3_input->connect_to(
      &(engine_dynamic_sender->low_oil_pressure_consumer_));
  // This is just an example -- normally temperature alarms would not be
  // active-low (inverted).
  alarm_d4_input->connect_to(
      &(engine_dynamic_sender->over_temperature_consumer_));
      
#endif  // ENABLE_NMEA2000_OUTPUT

  
  
  
  
  // FIXME: Transmit the alarms over SK as well.


///////////////////////////////////////////////////////////////////
// 0x77 is the default address. Some chips use 0x76, which is shown here.
 bme280.begin(0x76);   

  auto* environment_inside_temp =      // Create a RepeatSensor with float output that reads the temperature using the function defined above.
      new RepeatSensor<float>(5000, read_temp_callback);
  auto* environment_outside_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);
  auto* environment_outside_humidity = 
      new RepeatSensor<float>(60000, read_humidity_callback); 

  // environment_inside_temp->connect_to(new SKOutputFloat("environment.inside.temperatureSK"));       // Send  to the Signal K 
  // environment_outside_pressure->connect_to(new SKOutputFloat("environment.outside.pressureSK"));
  // environment_outside_humidity->connect_to(new SKOutputFloat("environment.outside.humiditySK"));

  environment_outside_pressure->connect_to(                   // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float pressure) {          // PGN130311
        outside_pressure = pressure;
        SendingEnvironment();     
      }));

  environment_inside_temp->connect_to(                       // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float temperature) {      // PGN130311
        inside_temperature = temperature;
        SendingEnvironment();      
      }));

  environment_outside_humidity->connect_to(                   // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float humidity) {          // PGN130311
        outside_humidity = humidity;
        SendingEnvironment();      
      }));

////////////////////////////////////////////////////////////////
// Vessel Bilge Alarm  Hier de juiste PIN opgeven 35 !
// auto* bilge = new DigitalInputState(35, INPUT_PULLUP, 5000);   
// auto int_to_bool_function = [](int input) ->bool {
//      if (input == 1) {
//        return true;
//      } 
//      else { // input == 0
//        return false;
//      }
// };

//// Bilge Monitor /////

auto* bilge = new DigitalInputState(35, INPUT_PULLUP, 5000);

auto int_to_string_function = [](int input) ->String {
     if (input == 1) {
       return "Water present!";
     } 
     else { // input == 0
       return "bilge clear";
     }
};

auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

bilge->connect_to(int_to_string_transform)
      ->connect_to(new SKOutputString("propulsion.engine.bilge"));

bilge->connect_to(new SKOutputString("propulsion.engine.bilge.raw"));

// auto int_to_bool_transform = new LambdaTransform<int, bool>(int_to_bool_function);
// bilge->connect_to(int_to_bool_transform)
//       ->connect_to(new SKOutputBool("notifications.engine.bilge"));


  ///////////////////////////////////////////////////////////////////
  // Digital tacho inputs
// set GPIO 15  was 23!!! to output mode
  pinMode(15, OUTPUT);
  app.onRepeat(10, []() {
    if (freq == 0) {
      if (millis() - freq_start_time >= 10000) {
        freq = 10;
        freq_start_time = millis();
      } else {
        return;
      }
    } else {
      if (millis() - freq_start_time >= 1000) {
        freq += 10;
        freq_start_time = millis();
      }
      if (freq > 100) {
        freq = 0;
        return;
      }

      if ((millis() - cycle_start_time) >= 1000 / freq) {
        cycle_start_time = millis();
      } else {
        return;
      }
    }
    digitalWrite(15, !digitalRead(15));
  });


  //////////////////////////////////////////////////////////////////
  // Connect the tacho senders. Engine name is "main".
  // EDIT: More tacho inputs can be defined by duplicating the line below.
  auto tacho_d1_frequency = ConnectTachoSender(kDigitalInputPin1, "main");

#ifdef ENABLE_NMEA2000_OUTPUT
  // Connect outputs to the N2k senders.
  // EDIT: Make sure this matches your tacho configuration above.
  //       Duplicate the lines below to connect more tachos, but be sure to
  //       use different engine instances.
  N2kEngineParameterRapidSender* engine_rapid_sender =
      new N2kEngineParameterRapidSender("/NMEA 2000/Engine 0 Rapid Update", 0,
                                        nmea2000);  // Engine 0, instance 0
  tacho_d1_frequency->connect_to(&(engine_rapid_sender->engine_speed_consumer_));
#endif  // ENABLE_NMEA2000_OUTPUT

//////SL en Uren
  // create a propulsion state lambda transform
  auto* propulsion_state = new LambdaTransform<float, String>(
      [](bool freq) {
        if (freq > 0) {
          return "started";
        } else {
          return "stopped";
        }
      },
      "/Transforms/Propulsion State");

  tacho_d1_frequency->connect_to(propulsion_state);

  // create engine hours counter using PersistentDuration
  auto* engine_hours =
      new TimeCounter<float>("/Transforms/Engine Hours");

  tacho_d1_frequency->connect_to(engine_hours);

// create and connect the propulsion state output object
  propulsion_state->connect_to(
      new SKOutput<String>("propulsion.main.state", "",
                           new SKMetadata("", "Main Engine State")));

// create and connect the engine hours output object
  engine_hours->connect_to(
      new SKOutput<float>("propulsion.main.runTime", "",
                          new SKMetadata("s", "Main Engine running time")));

  engine_hours->connect_to(                   // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float engine_hours) {
        N2kengine_hours = engine_hours ;
        SendEngineData();        
      }));

  // create fuel rate counter using PersistentDuration
  auto* fuel_rate =
      new FuelInterpreter("/Engine Fuel/curve");
          fuel_rate->connect_to(new SKOutputFloat("propulsion.main.fuel.rate", "/Engine Fuel/sk_path")); 

  fuel_rate->connect_to(                    // Send data to NMEA SEND Function  N2K PGN
      new LambdaConsumer<float>([](float fuel_rate) {
        N2kfuel_rate = (fuel_rate*3600000); //NMEA2000 is in liters per uur! dus *3600000!!! 
        SendEngineData();
      }));

  tacho_d1_frequency->connect_to(fuel_rate);







//DISPLAY
  if (display_present) {
        tacho_d1_frequency->connect_to(new LambdaConsumer<float>(
        [](float value) { PrintValue(display, 3, "RPM D1", 60 * value); }));
  }

  //////////////////////////////////////////////////////////////////////////
  // Implement the N2K PGN sending. Engine (oil) temperature and coolant ///
  // temperature are a bit more complex because they're sent together    ///
  // as part of a Engine Dynamic Parameter PGN.                          ///
  //////////////////////////////////////////////////////////////////////////

  





// //////////////////////////////////////////////////////////////////
// // BME naar N2K
//   environment_outside_pressure->connect_to(                   // Send data to NMEA SEND Function  N2K PGN
//       new LambdaConsumer<float>([](float pressure) {          // PGN130311
//         outside_pressure = pressure;
//         SendingEnvironment();     
//       }));

//   environment_inside_temp->connect_to(                       // Send data to NMEA SEND Function  N2K PGN
//       new LambdaConsumer<float>([](float temperature) {      // PGN130311
//         inside_temperature = temperature;
//         SendingEnvironment();      
//       }));

//   environment_outside_humidity->connect_to(                   // Send data to NMEA SEND Function  N2K PGN
//       new LambdaConsumer<float>([](float humidity) {          // PGN130311
//         outside_humidity = humidity;
//         SendingEnvironment();      
//       }));
// //Temp pressure and humidity NAAR N2K




  ///////////////////////////////////////////////////////////////////
  // Display setup

  // Connect the outputs to the display
  if (display_present) {
    app.onRepeat(1000, []() {
      PrintValue(display, 1, "IP:", WiFi.localIP().toString());
    });

    // Create a poor man's "christmas tree" display for the alarms
    app.onRepeat(1000, []() {
      char state_string[5] = {};
      for (int i = 0; i < 4; i++) {
        state_string[i] = alarm_states[i] ? '*' : '_';
      }
      PrintValue(display, 4, "Alarm", state_string);
    });
  }

  ///////////////////////////////////////////////////////////////////
  // Start the application

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();
}

void loop() { app.tick(); }
