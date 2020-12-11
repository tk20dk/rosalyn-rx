#ifndef SPI_H__
#define SPI_H__

#include <main.h>


class TSpi
{
public:
  TSpi( SPI_TypeDef *const SPIx );

  void Setup()
  {
    LL_SPI_SetRxFIFOThreshold( SPI, LL_SPI_RX_FIFO_TH_QUARTER );
    LL_SPI_Enable( SPI );
  }

  void Read( void* const RxData, uint32_t const Length ) const;
  void Write( void const* const TxData, uint32_t const Length ) const;
  void WriteRead( void const* const TxData, void* const RxData, uint32_t const Length ) const;

  uint8_t Read() const
  {
    return WriteRead();
  }

  void Write( uint8_t const Data ) const
  {
    (void)WriteRead( Data );
  }

  uint8_t WriteRead( uint8_t const Data = 0x00 ) const;

  void WaitTXE() const
  {
    while( !LL_SPI_IsActiveFlag_TXE( SPI ))
    {
    }
  }

  void WaitRXNE() const
  {
    while( !LL_SPI_IsActiveFlag_RXNE( SPI ))
    {
    }
  }

  void WaitNBSY() const
  {
    while( LL_SPI_IsActiveFlag_BSY( SPI ))
    {
    }
  }

  void ClearOVR() const
  {
    LL_SPI_ClearFlag_OVR( SPI );
  }

  void TransmitBSY( uint8_t const Data ) const
  {
    LL_SPI_TransmitData8( SPI, Data );
    WaitTXE();
  }

private:
  SPI_TypeDef *const SPI;
};

#endif // SPI_H__
