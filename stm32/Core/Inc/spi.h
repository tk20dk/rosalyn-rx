#ifndef SPI_H__
#define SPI_H__

#include <main.h>


struct TDriverSpi
{
  static void Setup();
  static void Write( uint8_t const Data );
  static void Write( void const* const TxData, uint32_t const Length );
  static void Read( void* const RxData, uint32_t const Length );

  static void WriteRead( void const* const TxData, void* const RxData, uint32_t const Length );
  static uint8_t WriteRead( uint8_t const Data = 0x00 );

  static void WaitTXE()
  {
    while( !LL_SPI_IsActiveFlag_TXE( SPI1 ))
    {
    }
  }

  static void WaitRXNE()
  {
    while( !LL_SPI_IsActiveFlag_RXNE( SPI1 ))
    {
    }
  }

  static void WaitNBSY()
  {
    while( LL_SPI_IsActiveFlag_BSY( SPI1 ))
    {
    }
  }

  static void ClearOVR()
  {
    LL_SPI_ClearFlag_OVR( SPI1 );
  }

  static void TransmitBSY( uint8_t const Data )
  {
    LL_SPI_TransmitData8( SPI1, Data );
    WaitTXE();
  }
};

extern TDriverSpi Spi;

#endif // SPI_H__
