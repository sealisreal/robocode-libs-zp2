/***************************************************
  This is a library for our optical Fingerprint sensor
 ****************************************************/

#include "RoboCodeFingerprint.h"

//#define FINGERPRINT_DEBUG

#if ARDUINO >= 100
  #define SERIAL_WRITE(...) mySerial->write(__VA_ARGS__)
#else
  #define SERIAL_WRITE(...) mySerial->write(__VA_ARGS__, BYTE)
#endif

#define SERIAL_WRITE_U16(v) SERIAL_WRITE((uint8_t)(v>>8)); SERIAL_WRITE((uint8_t)(v & 0xFF));

#define GET_CMD_PACKET(...) \
  uint8_t data[] = {__VA_ARGS__}; \
  Fingerprint_Packet packet(FINGERPRINT_COMMANDPACKET, sizeof(data), data); \
  writeStructuredPacket(packet); \
  if (getStructuredPacket(&packet) != FINGERPRINT_OK) return FINGERPRINT_PACKETRECIEVEERR; \
  if (packet.type != FINGERPRINT_ACKPACKET) return FINGERPRINT_PACKETRECIEVEERR;

#define SEND_CMD_PACKET(...) GET_CMD_PACKET(__VA_ARGS__); return packet.data[0];

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/


#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
/**************************************************************************/
/*!
    @brief  Instantiates sensor with Software Serial
    @param  ss Pointer to SoftwareSerial object
    @param  password 32-bit integer password (default is 0)
*/
/**************************************************************************/

RoboCodeFingerprint::RoboCodeFingerprint(uint8_t rx, uint8_t tx, uint32_t password) {
  thePassword = password;
  theAddress = 0xFFFFFFFF;
  hwSerial = NULL;
  swSerial = new SoftwareSerial(rx, tx);
  mySerial = swSerial;
}
#endif

/**************************************************************************/
/*!
    @brief  Instantiates sensor with Hardware Serial
    @param  hs Pointer to HardwareSerial object
    @param  password 32-bit integer password (default is 0)

*/
/**************************************************************************/
RoboCodeFingerprint::RoboCodeFingerprint(HardwareSerial *hs, uint32_t password) {
  thePassword = password;
  theAddress = 0xFFFFFFFF;

#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
  swSerial = NULL;
#endif
  hwSerial = hs;
  mySerial = hwSerial;
}

/**************************************************************************/
/*!
    @brief  Initializes serial interface and baud rate
    @param  baudrate Sensor's UART baud rate (usually 57600, 9600 or 115200)
*/
/**************************************************************************/

void RoboCodeFingerprint::begin(uint32_t baudrate) {
  delay(1000);  // one second delay to let the sensor 'boot up'

  if (hwSerial) hwSerial->begin(baudrate);
#if defined(__AVR__) || defined(ESP8266) || defined(FREEDOM_E300_HIFIVE1)
  if (swSerial) swSerial->begin(baudrate);
#endif
  if (verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
    while (1) { delay(1); }
  }
}

/**************************************************************************/
/*!
    @brief  Verifies the sensors' access password (default password is 0x0000000). A good way to also check if the sensors is active and responding
    @returns True if password is correct
*/
/**************************************************************************/
boolean RoboCodeFingerprint::verifyPassword(void) {
  return checkPassword() == FINGERPRINT_OK;
}

uint8_t RoboCodeFingerprint::checkPassword(void) {
  GET_CMD_PACKET(FINGERPRINT_VERIFYPASSWORD,
                  (uint8_t)(thePassword >> 24), (uint8_t)(thePassword >> 16),
                  (uint8_t)(thePassword >> 8), (uint8_t)(thePassword & 0xFF));
  if (packet.data[0] == FINGERPRINT_OK)
    return FINGERPRINT_OK;
  else
    return FINGERPRINT_PACKETRECIEVEERR;
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take an image of the finger pressed on surface
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_NOFINGER</code> if no finger detected
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_IMAGEFAIL</code> on imaging error
*/
/**************************************************************************/
uint8_t RoboCodeFingerprint::getImage(void) {
  SEND_CMD_PACKET(FINGERPRINT_GETIMAGE);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to convert image to feature template
    @param slot Location to place feature template (put one in 1 and another in 2 for verification to create model)
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_IMAGEMESS</code> if image is too messy
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_FEATUREFAIL</code> on failure to identify fingerprint features
    @returns <code>FINGERPRINT_INVALIDIMAGE</code> on failure to identify fingerprint features
*/
uint8_t RoboCodeFingerprint::image2Tz(uint8_t slot) {
  SEND_CMD_PACKET(FINGERPRINT_IMAGE2TZ,slot);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to take two print feature template and create a model
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
    @returns <code>FINGERPRINT_ENROLLMISMATCH</code> on mismatch of fingerprints
*/
uint8_t RoboCodeFingerprint::createModel(void) {
  SEND_CMD_PACKET(FINGERPRINT_REGMODEL);
}


/**************************************************************************/
/*!
    @brief   Ask the sensor to store the calculated model for later matching
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t RoboCodeFingerprint::storeModel(uint16_t location) {
  SEND_CMD_PACKET(FINGERPRINT_STORE, 0x01, (uint8_t)(location >> 8), (uint8_t)(location & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to load a fingerprint model from flash into buffer 1
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t RoboCodeFingerprint::loadModel(uint16_t location) {
  SEND_CMD_PACKET(FINGERPRINT_LOAD, 0x01, (uint8_t)(location >> 8), (uint8_t)(location & 0xFF));
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to transfer 256-byte fingerprint template from the buffer to the UART
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t RoboCodeFingerprint::getModel(void) {
  SEND_CMD_PACKET(FINGERPRINT_UPLOAD, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete a model in memory
    @param   location The model location #
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t RoboCodeFingerprint::deleteModel(uint16_t location) {
  SEND_CMD_PACKET(FINGERPRINT_DELETE, (uint8_t)(location >> 8), (uint8_t)(location & 0xFF), 0x00, 0x01);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to delete ALL models in memory
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_BADLOCATION</code> if the location is invalid
    @returns <code>FINGERPRINT_FLASHERR</code> if the model couldn't be written to flash memory
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
uint8_t RoboCodeFingerprint::emptyDatabase(void) {
  SEND_CMD_PACKET(FINGERPRINT_EMPTY);
}

/**************************************************************************/
/*!
    @brief   Ask the sensor to search the current slot 1 fingerprint features to match saved templates. The matching location is stored in <b>fingerID</b> and the matching confidence in <b>confidence</b>
    @returns <code>FINGERPRINT_OK</code> on fingerprint match success
    @returns <code>FINGERPRINT_NOTFOUND</code> no match made
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t RoboCodeFingerprint::fingerFastSearch(void) {
  // high speed search of slot #1 starting at page 0x0000 and page #0x00A3
  GET_CMD_PACKET(FINGERPRINT_HISPEEDSEARCH, 0x01, 0x00, 0x00, 0x00, 0xA3);
  fingerID = 0xFFFF;
  confidence = 0xFFFF;

  fingerID = packet.data[1];
  fingerID <<= 8;
  fingerID |= packet.data[2];

  confidence = packet.data[3];
  confidence <<= 8;
  confidence |= packet.data[4];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Ask the sensor for the number of templates stored in memory. The number is stored in <b>templateCount</b> on success.
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t RoboCodeFingerprint::getTemplateCount(void) {
  GET_CMD_PACKET(FINGERPRINT_TEMPLATECOUNT);

  templateCount = packet.data[1];
  templateCount <<= 8;
  templateCount |= packet.data[2];

  return packet.data[0];
}

/**************************************************************************/
/*!
    @brief   Set the password on the sensor (future communication will require password verification so don't forget it!!!)
    @param   password 32-bit password code
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_PACKETRECIEVEERR</code> on communication error
*/
/**************************************************************************/
uint8_t RoboCodeFingerprint::setPassword(uint32_t password) {
  SEND_CMD_PACKET(FINGERPRINT_SETPASSWORD, (password >> 24), (password >> 16), (password >> 8), password);
}

/**************************************************************************/
/*!
    @brief   Helper function to process a packet and send it over UART to the sensor
    @param   packet A structure containing the bytes to transmit
*/
/**************************************************************************/

void RoboCodeFingerprint::writeStructuredPacket(const Fingerprint_Packet & packet) {
  SERIAL_WRITE_U16(packet.start_code);
  SERIAL_WRITE(packet.address[0]);
  SERIAL_WRITE(packet.address[1]);
  SERIAL_WRITE(packet.address[2]);
  SERIAL_WRITE(packet.address[3]);
  SERIAL_WRITE(packet.type);

  uint16_t wire_length = packet.length + 2;
  SERIAL_WRITE_U16(wire_length);

  uint16_t sum = ((wire_length)>>8) + ((wire_length)&0xFF) + packet.type;
  for (uint8_t i=0; i< packet.length; i++) {
    SERIAL_WRITE(packet.data[i]);
    sum += packet.data[i];
  }

  SERIAL_WRITE_U16(sum);
  return;
}

/**************************************************************************/
/*!
    @brief   Helper function to receive data over UART from the sensor and process it into a packet
    @param   packet A structure containing the bytes received
    @param   timeout how many milliseconds we're willing to wait
    @returns <code>FINGERPRINT_OK</code> on success
    @returns <code>FINGERPRINT_TIMEOUT</code> or <code>FINGERPRINT_BADPACKET</code> on failure
*/
/**************************************************************************/
int RoboCodeFingerprint::getFingerID() {
	int p = getImage();
  	if (p != FINGERPRINT_OK)  return -1;

  	p = image2Tz();
  	if (p != FINGERPRINT_OK)  return -1;

  	p = fingerFastSearch();
  	if (p != FINGERPRINT_OK){
    	return 0;
  	}
  	return fingerID; 
}

int RoboCodeFingerprint::writeNewFinger(int id) {
	int p = -1;
	Serial.print("Waiting for valid finger to enroll as #"); Serial.println(id);
	while (p != FINGERPRINT_OK) {
	    p = getImage();
	    switch (p) {
	      	case FINGERPRINT_OK:
	        	Serial.println("Image taken");
	        break;
          case FINGERPRINT_NOFINGER:
          //Serial.println(".");
          break;
	      	case FINGERPRINT_PACKETRECIEVEERR:
	        	Serial.println("Communication error");
	        break;
	      	case FINGERPRINT_IMAGEFAIL:
	        	Serial.println("Imaging error");
	        break;
	      	default:
	        	Serial.println("Unknown error");
	        break;
	    }
	}

	p = image2Tz(1);
	switch (p) {
	    case FINGERPRINT_OK:
	      	Serial.println("Image converted");
	      	break;
	    case FINGERPRINT_IMAGEMESS:
	      	Serial.println("Image too messy");
	      	return p;
	    case FINGERPRINT_PACKETRECIEVEERR:
	      	Serial.println("Communication error");
	      	return p;
	    case FINGERPRINT_FEATUREFAIL:
	      	Serial.println("Could not find fingerprint features");
	      	return p;
	    case FINGERPRINT_INVALIDIMAGE:
	      	Serial.println("Could not find fingerprint features");
	      	return p;
	    default:
	      	Serial.println("Unknown error");
	      	return p;
	  	}

	Serial.println("Remove finger");
	delay(2000);
	p = 0;
	while (p != FINGERPRINT_NOFINGER) {
	    p = getImage();
	}

	Serial.print("ID "); Serial.println(id);

	m1:
	p = -1;
	Serial.println("Place same finger again");
	while (p != FINGERPRINT_OK) {
	    p = getImage();
	    switch (p) {
	      	case FINGERPRINT_OK:
	        	Serial.println("Image taken");
	        	break;
          case FINGERPRINT_NOFINGER:
            //Serial.println(".");
            break;
	      	case FINGERPRINT_PACKETRECIEVEERR:
	        	Serial.println("Communication error");
	        	break;
	      	case FINGERPRINT_IMAGEFAIL:
	        	Serial.println("Imaging error");
	        	break;
	      	default:
	        	Serial.println("Unknown error");
	        	break;
	    }
	}

	  // OK success!

	p = image2Tz(2);
	  	switch (p) {
	    	case FINGERPRINT_OK:
	      		Serial.println("Image converted");
	      		break;
	    	case FINGERPRINT_IMAGEMESS:
	      		Serial.println("Image too messy");
	      		return p;
	    	case FINGERPRINT_PACKETRECIEVEERR:
	      		Serial.println("Communication error");
	      		return p;
	    	case FINGERPRINT_FEATUREFAIL:
	      		Serial.println("Could not find fingerprint features");
	      		return p;
	    	case FINGERPRINT_INVALIDIMAGE:
	      		Serial.println("Could not find fingerprint features");
	      		return p;
	    	default:
	      		Serial.println("Unknown error");
	      		return p;
	  	}
	  // OK converted!

	Serial.print("Creating model for #");  Serial.println(id);

	p = createModel();
	if (p == FINGERPRINT_OK) {
		Serial.println("Prints matched!");
	} else if (p == FINGERPRINT_PACKETRECIEVEERR) {
	    Serial.println("Communication error");
	    return p;
	} else if (p == FINGERPRINT_ENROLLMISMATCH) {
	    Serial.println("Fingerprints did not match");
	    goto m1;
	} else {
	    Serial.println("Unknown error");
	    return p;
	}

	Serial.print("ID "); Serial.println(id);
	p = storeModel(id);
	if (p == FINGERPRINT_OK) {
	    Serial.println("Stored!");
	    return 1;
	} else if (p == FINGERPRINT_PACKETRECIEVEERR) {
	    Serial.println("Communication error");
	    return p;
	} else if (p == FINGERPRINT_BADLOCATION) {
	    Serial.println("Could not store in that location");
	    return p;
	} else if (p == FINGERPRINT_FLASHERR) {
	    Serial.println("Error writing to flash");
	    return p;
	} else {
	    Serial.println("Unknown error");
	    return p;
  	}
}

int RoboCodeFingerprint::deleteFinger(int id) {
  uint8_t p = -1;
  p = deleteModel(id);
  if (p == FINGERPRINT_OK) {
    Serial.println("Deleted!");
    return p;
  } else if (p == FINGERPRINT_PACKETRECIEVEERR) {
    Serial.println("Communication error");
    return p;
  } else if (p == FINGERPRINT_BADLOCATION) {
    Serial.println("Could not delete in that location");
    return p;
  } else if (p == FINGERPRINT_FLASHERR) {
    Serial.println("Error writing to flash");
    return p;
  } else {
    Serial.print("Unknown error: 0x"); Serial.println(p, HEX);
    return p;
  }   
}

bool RoboCodeFingerprint::searchFinger(int id) {
  if (loadModel(id) == FINGERPRINT_OK) { 
      return true;                                  
  } 
  return false;
}


uint8_t RoboCodeFingerprint::getStructuredPacket(Fingerprint_Packet * packet, uint16_t timeout) {
  uint8_t byte;
  uint16_t idx=0, timer=0;

  while(true) {
    while(!mySerial->available()) {
      delay(1);
      timer++; 
      if( timer >= timeout) {
#ifdef FINGERPRINT_DEBUG
	Serial.println("Timed out");
#endif
	return FINGERPRINT_TIMEOUT;
      }
    }
    byte = mySerial->read();
#ifdef FINGERPRINT_DEBUG
    Serial.print("<- 0x"); Serial.println(byte, HEX);
#endif
    switch (idx) {
      case 0:
        if (byte != (FINGERPRINT_STARTCODE >> 8)) 
	  continue;
        packet->start_code = (uint16_t)byte << 8;
        break;
      case 1:
        packet->start_code |= byte;
        if (packet->start_code != FINGERPRINT_STARTCODE) 
	  return FINGERPRINT_BADPACKET;
        break;
      case 2:
      case 3:
      case 4:
      case 5:
        packet->address[idx-2] = byte;
        break;
      case 6: 
	packet->type = byte; 
	break;
      case 7: 
	packet->length = (uint16_t)byte << 8; 
	break;
      case 8: 
	packet->length |= byte; 
	break;
      default:
        packet->data[idx-9] = byte;
        if((idx-8) == packet->length)
          return FINGERPRINT_OK;
        break;
    }
    idx++;
  }
  // Shouldn't get here so...
  return FINGERPRINT_BADPACKET;
}
