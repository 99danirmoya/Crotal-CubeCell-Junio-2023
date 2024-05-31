// OTAA parameters, all in "big endian" (msb)
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t devEui[] = { 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x05, 0xF0, 0x1A };
uint8_t appKey[] = { 0xBC, 0x2C, 0x13, 0x35, 0x19, 0xF2, 0xF3, 0xFA, 
                      0x61, 0x58, 0xF0, 0x6B, 0xD6, 0x81, 0x8A, 0x58 };
// ABP parameters, all in "big endian" (msb)
uint8_t nwkSKey[] = { 0x64, 0xBE, 0x0C, 0x29, 0x9C, 0x83, 0xC7, 0x3C, 0xCA, 0xEA, 0x21, 0xE9, 0x95, 0x33, 0x89, 0x3D };
uint8_t appSKey[] = { 0x6D, 0x4D, 0x30, 0x21, 0x09, 0xEE, 0x31, 0x38, 0x2A, 0x28, 0x32, 0x87, 0x57, 0xC6, 0x48, 0x6E };
uint32_t devAddr =  ( uint32_t )0x260BAACA1;

uint16_t userChannelsMask[6]={ 0x00FF,0x0000,0x0000,0x0000,0x0000,0x0000 }; //LoraWan channelsmask, default channels 0-7*/ 

LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;  // LoraWan region, defined in platformio.ini
DeviceClass_t  loraWanClass = LORAWAN_CLASS;    // LoraWan Class, Class A and Class C are supported
uint32_t appTxDutyCycle = 120000;                // the application data transmission duty cycle.  value in [ms].
bool overTheAirActivation = LORAWAN_NETMODE;    // OTAA or ABP, defined in platformio.ini
bool loraWanAdr = LORAWAN_ADR;                  // ADR enable
bool keepNet = LORAWAN_NET_RESERVE;             // set LORAWAN_Net_Reserve ON, the node could save the network info to flash, when node reset not need to join again
bool isTxConfirmed = LORAWAN_UPLINKMODE;        // Indicates if the node is sending confirmed or unconfirmed messages 
uint8_t appPort = 1;                            // Application port
uint8_t confirmedNbTrials = 4;