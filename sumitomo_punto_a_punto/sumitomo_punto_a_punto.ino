/**
 * File: punto_a_punto.h
 * Author: pepemanboy
 * 
 * Communication protocol: (ADDRESS,MESSAGE)
 */

#include "SoftwareSerial.h"

#define ADDRESS (4)

// #define DEBUG

/* Pin defines */
#define HC12_RX 3
#define HC12_TX 2
#define HC12_SET 7
#define RSB_AUX 6
#define LED 9
#define LED_NEGATIVE 8

/* HC12 configuration */
#define HC12_BAUDRATE 9600
#define HC12_CHANNEL  (60 + ADDRESS * 5)

/* Software serial object */
SoftwareSerial HC12(HC12_TX, HC12_RX);

/* Serial communication config */
#define BUFFER_SIZE 64
#define TIMEOUT_MS (100)

/* Error codes */
typedef enum
{
  Ok = 0,
  Error_timeout,
  Error_config,
  Error_parsing,
  Error_address,
}error_codes;

/* Global variables */
String read_buf;
String write_buf;

/* Function declarations */
String formatChannel(uint8_t channel);
uint8_t HC12_configure_parameter(String query);
uint8_t HC12_setup();
void error();
uint8_t decodeMessage();

void setup() 
{
  // Begin serial
  Serial.begin(9600);
  Serial.setTimeout(TIMEOUT_MS);
  HC12.begin(9600);
  HC12.setTimeout(TIMEOUT_MS);

  // Pin modes
  pinMode(RSB_AUX, OUTPUT);
  pinMode(HC12_SET, OUTPUT);
  pinMode(LED_NEGATIVE, OUTPUT);
  pinMode(LED, OUTPUT);

  // Reserve space for buffers
  read_buf.reserve(BUFFER_SIZE);
  write_buf.reserve(BUFFER_SIZE);

  // Drive low DTR and CTS of RS232
  digitalWrite(RSB_AUX, LOW); // RS232 has inverted logic

  // Init leds
  digitalWrite(LED_NEGATIVE, LOW);
  digitalWrite(LED, HIGH);
  
  // Setup HC12
  uint8_t r = HC12_setup();
  if (r != Ok)
    error();
}

void loop() 
{
  // Read from usb / rsb
  if (Serial.available())
  {
    read_buf = Serial.readString();
    write_buf = "(" + String(ADDRESS) + "," + read_buf + ")";
    HC12.print(write_buf); // Send to HC12    
  }
  
  // Read from HC12
  if (HC12.available())
  {
    read_buf = HC12.readString();
    uint8_t r = decodeMessage();
    if (r != Ok)
    {
      #ifdef DEBUG
      Serial.print("Error + " + String(r));
      #endif
    }
    else
    {
      Serial.print(write_buf); // Send to usb/rsb  
    }    
  }
}

/** format channel */
String formatChannel(uint8_t channel)
{
  String s = "";
  if (channel < 10) s = "00" + String(channel);
  else if (channel < 100) s = "0" + String(channel);
  else if (channel < 1000) s = String(channel);
  return s;
}

/** Configure HC 12 AT parameter */
uint8_t HC12_configure_parameter(String query)
{
  query = "AT+" + query;
  HC12.println(query);
  String response = HC12.readString();
  if (response.indexOf("OK") < 0)
    return Error_config;
  return Ok;
}

/** Setup HC12 module */
uint8_t HC12_setup()
{
  digitalWrite(HC12_SET, LOW); // Enter setup mode
  delay(200);
  
  // Set baud rate
  uint8_t r = HC12_configure_parameter("B" + String(HC12_BAUDRATE));
  if (r != Ok)
    return r;

  // Set channel
  r = HC12_configure_parameter("C" + formatChannel(HC12_CHANNEL));
  if (r != Ok)
    return r;

  digitalWrite(HC12_SET, HIGH); // Exit setup mode
  delay(200);
  
  return Ok;
}

/** Error function */
void error()
{
  while(1)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
  }
}

/** Decode message */
uint8_t decodeMessage()
{  
    // Locate comma
    int16_t comma = read_buf.indexOf(",");
    if (comma < 0)
      return Error_parsing;

    // Locate address
    uint8_t address = read_buf.substring(1,comma).toInt();

    // Not for this node
    if (address != ADDRESS)
      return Error_address;

    // Locate closed parenthesis
    int16_t closed_parenthesis = read_buf.indexOf(")");
    if (closed_parenthesis < 0)
      return Error_parsing;

    // Locate message
    write_buf = read_buf.substring(comma+1,closed_parenthesis);

    return Ok;
}
