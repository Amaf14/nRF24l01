#include "nRF24L01.h"
#include "RF24_config.h"
#include "RF24.h"

static const uint8_t child_pipe_enable[] PROGMEM =
{
  ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5
};
static const uint8_t child_pipe[] PROGMEM =
{
  RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5
};
static const uint8_t child_payload_size[] PROGMEM =
{
  RX_PW_P0, RX_PW_P1, RX_PW_P2, RX_PW_P3, RX_PW_P4, RX_PW_P5
};

RF24::RF24(uint16_t _cepin, uint16_t _cspin):
  ce_pin(_cepin), csn_pin(_cspin), p_variant(false),
  payload_size(32), dynamic_payloads_enabled(false), addr_width(5), csDelay(5) //,pipe0_reading_address(0)
{
  pipe0_reading_address[0] = 0;
}

/****************************************************************************/
uint8_t RF24::spiTrans(uint8_t cmd) {

  uint8_t status;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);
  status = _SPI.transfer( cmd );
  //csn(HIGH);
  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();
  return status;
}
/****************************************************************************/
uint8_t RF24::read_register(uint8_t reg, uint8_t* buf, uint8_t len)
{
  uint8_t status;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);

  status = _SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- ) {
    *buf++ = _SPI.transfer(0xff);
  }
  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();

  return status;
}

/****************************************************************************/

uint8_t RF24::read_register(uint8_t reg)
{
  uint8_t result;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);

  _SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  result = _SPI.transfer(0xff);

  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();

  return result;
}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, const uint8_t* buf, uint8_t len)
{
  uint8_t status;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);

  status = _SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  while ( len-- )
    _SPI.transfer(*buf++);

  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();

  return status;
}
/****************************************************************************/
void RF24::startFastWrite( const void* buf, uint8_t len, const bool multicast, bool startTx) { //TMRh20

  //write_payload( buf,len);
  write_payload( buf, len, multicast ? W_TX_PAYLOAD_NO_ACK : W_TX_PAYLOAD ) ;
  if (startTx) {
    digitalWrite(ce_pin, HIGH);
  }

}

/****************************************************************************/

uint8_t RF24::write_register(uint8_t reg, uint8_t value)
{
  uint8_t status;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);

  status = _SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
  _SPI.transfer(value);

  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();

  return status;
}
/****************************************************************************/
bool RF24::write( const void* buf, uint8_t len, const bool multicast )
{
  //Start Writing
  startFastWrite(buf, len, multicast);

  //Wait until complete or failed
  while ( ! ( spiTrans(RF24_NOP) & ( 1 << TX_DS | 1 << MAX_RT )))

    digitalWrite(ce_pin, LOW);

  uint8_t status = write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT );

  //Max retries exceeded
  if ( status & 1 << MAX_RT) {
    //Only going to be 1 packet int the FIFO at a time using this method, so just flush
    spiTrans( FLUSH_TX );
    return 0;
  }
  //TX OK 1 or 0
  return 1;
}

void RF24::read( void* buf, uint8_t len ) {

  // Fetch the payload
  read_payload( buf, len );
  //Clear the two possible interrupt flags with one command
  write_register(NRF_STATUS, 1 << RX_DR | 1 << MAX_RT | 1 << TX_DS );

}
/****************************************************************************/


uint8_t RF24::read_payload(void* buf, uint8_t data_len)
{
  uint8_t status;
  uint8_t* current = reinterpret_cast<uint8_t*>(buf);

  if (data_len > payload_size) data_len = payload_size;
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);

  status = _SPI.transfer( R_RX_PAYLOAD );
  while ( data_len-- ) {
    *current++ = _SPI.transfer(0xFF);
  }
  while ( blank_len-- ) {
    _SPI.transfer(0xff);
  }

  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();

  return status;
}
/****************************************************************************/
uint8_t RF24::write_payload(const void* buf, uint8_t data_len, const uint8_t writeType)
{
  uint8_t status;
  const uint8_t* current = reinterpret_cast<const uint8_t*>(buf);

  data_len = rf24_min(data_len, payload_size);
  uint8_t blank_len = dynamic_payloads_enabled ? 0 : payload_size - data_len;

  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  //csn(LOW);
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);

  status = _SPI.transfer( writeType );
  while ( data_len-- ) {
    _SPI.transfer(*current++);
  }
  while ( blank_len-- ) {
    _SPI.transfer(0);
  }

  //csn(HIGH);
  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();

  return status;
}
/****************************************************************************/

bool RF24::begin(void)
{

  uint8_t setup = 0;
  pinMode(ce_pin, OUTPUT);
  pinMode(csn_pin, OUTPUT);

  _SPI.begin();
  digitalWrite(ce_pin, LOW);
  //csn(HIGH);
  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);

  // Must allow the radio time to settle else configuration bits will not necessarily stick.
  // This is actually only required following power up but some settling time also appears to
  // be required after resets too. For full coverage, we'll always assume the worst.
  // Enabling 16b CRC is by far the most obvious case if the wrong timing is used - or skipped.
  // Technically we require 4.5ms + 14us as a worst case. We'll just call it 5ms for good measure.
  // WARNING: Delay is based on P-variant whereby non-P *may* require different timing.
  delay( 5 ) ;

  // Reset NRF_CONFIG and enable 16-bit CRC.
  write_register( NRF_CONFIG, 0x0C ) ;

  // Set 1500uS (minimum for 32B payload in ESB@250KBPS) timeouts, to make testing a little easier
  // WARNING: If this is ever lowered, either 250KBS mode with AA is broken or maximum packet
  // sizes must never be used. See documentation for a more complete explanation.
  //setRetries(5,15);
  write_register(SETUP_RETR, (5 & 0xf) << ARD | (15 & 0xf) << ARC);

  // Reset value is MAX
  //setPALevel( RF24_PA_MAX ) ;

  // check for connected module and if this is a p nRF24l01 variant
  //
  if ( setDataRate( RF24_250KBPS ) )
  {
    p_variant = true ;
  }
  setup = read_register(RF_SETUP);
  /*if( setup == 0b00001110 )     // register default for nRF24L01P
    {
    p_variant = true ;
    }*/

  // Then set the data rate to the slowest (and most reliable) speed supported by all
  // hardware.
  setDataRate( RF24_1MBPS ) ;

  // Initialize CRC and request 2-byte (16bit) CRC
  //setCRCLength( RF24_CRC_16 ) ;

  // Disable dynamic payloads, to match dynamic_payloads_enabled setting - Reset value is 0
  toggle_features();
  write_register(FEATURE, 0 );
  write_register(DYNPD, 0);
  dynamic_payloads_enabled = false;

  // Reset current status
  // Notice reset and flush is the last thing we do
  write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT );

  // Set up default configuration.  Callers can always change it later.
  // This channel should be universally safe and not bleed over into adjacent
  // spectrum.
  //setChannel(76);
  const uint8_t max_channel = 125;
  write_register(RF_CH, rf24_min(76, max_channel));

  spiTrans( FLUSH_RX );
  spiTrans( FLUSH_TX );

  powerUp(); //Power up by default when begin() is called

  // Enable PTX, do not write CE high so radio will remain in standby I mode ( 130us max to transition to RX or TX instead of 1500us from powerUp )
  // PTX should use only 22uA of power
  write_register(NRF_CONFIG, ( read_register(NRF_CONFIG) ) & ~1 << PRIM_RX );

  // if setup is 0 or ff then there was no response from module
  return ( setup != 0 && setup != 0xff );
}
/****************************************************************************/
void RF24::startListening(void)
{
  powerUp();
  write_register(NRF_CONFIG, read_register(NRF_CONFIG) | 1 << PRIM_RX);
  write_register(NRF_STATUS, 1 << RX_DR | 1 << TX_DS | 1 << MAX_RT );
  digitalWrite(ce_pin, HIGH);
  // Restore the pipe0 adddress, if exists
  if (pipe0_reading_address[0] > 0) {
    write_register(RX_ADDR_P0, pipe0_reading_address, addr_width);
  } else {
    closeReadingPipe(0);
  }

  // Flush buffers
  //flush_rx();
  if (read_register(FEATURE) & 1 << EN_ACK_PAY)
    spiTrans( FLUSH_TX );

  // Go!
  //delayMicroseconds(100);
}


void RF24::stopListening(void)
{
  digitalWrite(ce_pin, LOW);

  delayMicroseconds(txDelay);

  if (read_register(FEATURE) & 1 << EN_ACK_PAY) {
    delayMicroseconds(txDelay); //200
    spiTrans( FLUSH_TX );
  }
  //flush_rx();
  write_register(NRF_CONFIG, ( read_register(NRF_CONFIG) ) & ~1 << PRIM_RX );
  write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << pgm_read_byte(&child_pipe_enable[0])); // Enable RX on pipe0

  //delayMicroseconds(100);
}
/****************************************************************************/
void RF24::powerUp(void)
{
  uint8_t cfg = read_register(NRF_CONFIG);

  // if not powered up then power up and wait for the radio to initialize
  if (!(cfg & 1 << PWR_UP)) {
    write_register(NRF_CONFIG, cfg | 1 << PWR_UP);

    // For nRF24L01+ to go from power down mode to TX or RX mode it must first pass through stand-by mode.
    // There must be a delay of Tpd2stby (see Table 16.) after the nRF24L01+ leaves power down mode before
    // the CEis set high. - Tpd2stby can be up to 5ms per the 1.0 datasheet
    delay(5);
  }
}

void RF24::powerDown(void)
{
  digitalWrite(ce_pin, LOW); // Guarantee CE is low on powerDown
  write_register(NRF_CONFIG, read_register(NRF_CONFIG) & ~1 << PWR_UP);
}
/****************************************************************************/
void RF24::toggle_features(void)
{
  //beginTransaction();
  _SPI.beginTransaction(SPISettings(RF24_SPI_SPEED, MSBFIRST, SPI_MODE0));
  //csn(LOW);
  digitalWrite(csn_pin, LOW);
  delayMicroseconds(csDelay);
  _SPI.transfer( ACTIVATE );
  _SPI.transfer( 0x73 );
  //csn(HIGH);
  digitalWrite(csn_pin, HIGH);
  delayMicroseconds(csDelay);
  _SPI.endTransaction();
}
/****************************************************************************/
bool RF24::setDataRate(rf24_datarate_e speed)
{
  bool result = false;
  uint8_t setup = read_register(RF_SETUP) ;

  // HIGH and LOW '00' is 1Mbs - our default
  setup &= ~(1 << RF_DR_LOW | 1 << RF_DR_HIGH) ;

  txDelay = 85;
  if ( speed == RF24_250KBPS )
{
  // Must set the RF_DR_LOW to 1; RF_DR_HIGH (used to be RF_DR) is already 0
  // Making it '10'.
  setup |= 1 << RF_DR_LOW;
  txDelay = 155;
}
else
{
  // Set 2Mbs, RF_DR (RF_DR_HIGH) is set 1
  // Making it '01'
  if ( speed == RF24_2MBPS )
    {
      setup |= 1 << RF_DR_HIGH;
      txDelay = 65;
    }
  }
  write_register(RF_SETUP, setup);

  // Verify our result
  if ( read_register(RF_SETUP) == setup )
  {
    result = true;
  }
  return result;
}
/****************************************************************************/

void RF24::openWritingPipe(uint64_t value)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, reinterpret_cast<uint8_t*>(&value), addr_width);
  write_register(TX_ADDR, reinterpret_cast<uint8_t*>(&value), addr_width);


  //const uint8_t max_payload_size = 32;
  //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
  write_register(RX_PW_P0, payload_size);
}

/****************************************************************************/
void RF24::openWritingPipe(const uint8_t *address)
{
  // Note that AVR 8-bit uC's store this LSB first, and the NRF24L01(+)
  // expects it LSB first too, so we're good.

  write_register(RX_ADDR_P0, address, addr_width);
  write_register(TX_ADDR, address, addr_width);

  //const uint8_t max_payload_size = 32;
  //write_register(RX_PW_P0,rf24_min(payload_size,max_payload_size));
  write_register(RX_PW_P0, payload_size);
}

/****************************************************************************/
void RF24::openReadingPipe(uint8_t child, uint64_t address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0) {
    memcpy(pipe0_reading_address, &address, addr_width);
  }

  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 )
      write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), addr_width);
    else
      write_register(pgm_read_byte(&child_pipe[child]), reinterpret_cast<const uint8_t*>(&address), 1);

    write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << pgm_read_byte(&child_pipe_enable[child]));
  }
}

/****************************************************************************/
void RF24::setAddressWidth(uint8_t a_width) {

  if (a_width -= 2) {
    write_register(SETUP_AW, a_width % 4);
    addr_width = (a_width % 4) + 2;
  } else {
    write_register(SETUP_AW, 0);
    addr_width = 2;
  }

}
/****************************************************************************/

void RF24::openReadingPipe(uint8_t child, const uint8_t *address)
{
  // If this is pipe 0, cache the address.  This is needed because
  // openWritingPipe() will overwrite the pipe 0 address, so
  // startListening() will have to restore it.
  if (child == 0) {
    memcpy(pipe0_reading_address, address, addr_width);
  }
  if (child <= 6)
  {
    // For pipes 2-5, only write the LSB
    if ( child < 2 ) {
      write_register(pgm_read_byte(&child_pipe[child]), address, addr_width);
    } else {
      write_register(pgm_read_byte(&child_pipe[child]), address, 1);
    }
    write_register(pgm_read_byte(&child_payload_size[child]), payload_size);

    // Note it would be more efficient to set all of the bits for all open
    // pipes at once.  However, I thought it would make the calling code
    // more simple to do it this way.
    write_register(EN_RXADDR, read_register(EN_RXADDR) | 1 << pgm_read_byte(&child_pipe_enable[child]));
  }
}

void RF24::closeReadingPipe( uint8_t pipe )
{
  write_register(EN_RXADDR, read_register(EN_RXADDR) & ~1 << pgm_read_byte(&child_pipe_enable[pipe]));
}
/****************************************************************************/
void RF24::setPALevel(uint8_t level)
{

  uint8_t setup = read_register(RF_SETUP) & 0xF8;

  if (level > 3) {            // If invalid level, go to max PA
    level = (RF24_PA_MAX << 1) + 1;   // +1 to support the SI24R1 chip extra bit
  } else {
    level = (level << 1) + 1;     // Else set level as requested
  }


  write_register( RF_SETUP, setup |= level ) ;  // Write it to the chip
}
/****************************************************************************/

bool RF24::available(uint8_t* pipe_num)
{
  if (!( read_register(FIFO_STATUS) & 1 << RX_EMPTY )) {

    // If the caller wants the pipe number, include that
    if ( pipe_num ) {
      uint8_t status = get_status();
      *pipe_num = ( status >> RX_P_NO ) & 0x07;
    }
    return 1;
  }
  return 0;
}
