#include "spi.h"


TSpi::TSpi( SPI_TypeDef *const SPIx ) :
  SPI( SPIx )
{
}

uint8_t TSpi::WriteRead( uint8_t const Data ) const
{
  ClearOVR();
  LL_SPI_TransmitData8( SPI, Data );
  WaitRXNE();
  return LL_SPI_ReceiveData8( SPI );
}

void TSpi::Write( void const* const TxData, uint32_t const Length ) const
{
  auto TxPtr = reinterpret_cast< uint8_t const*>( TxData );
  auto const TxEnd = &TxPtr[ Length ];

  do
  {
    (void)WriteRead( *TxPtr++ );
  }
  while( TxPtr < TxEnd );
}

void TSpi::Read( void* const RxData, uint32_t const Length ) const
{
  auto RxPtr = reinterpret_cast< uint8_t*>( RxData );
  auto const RxEnd = &RxPtr[ Length ];

  do
  {
    *RxPtr++ = WriteRead();
  }
  while( RxPtr < RxEnd );
}

void TSpi::WriteRead( void const* const TxData, void* const RxData, uint32_t const Length ) const
{
  auto RxPtr = reinterpret_cast< uint8_t*>( RxData );
  auto TxPtr = reinterpret_cast< uint8_t const*>( TxData );
  auto const TxEnd = &TxPtr[ Length ];

  do
  {
    *RxPtr++ = WriteRead( *TxPtr++ );
  }
  while( TxPtr < TxEnd );
}
