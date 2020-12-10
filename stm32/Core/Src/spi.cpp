#include <spi.h>


void TDriverSpi::Setup()
{
  LL_SPI_SetRxFIFOThreshold( SPI, LL_SPI_RX_FIFO_TH_QUARTER );
  LL_SPI_Enable( SPI );
}

uint8_t TDriverSpi::WriteRead( uint8_t const Data )
{
  ClearOVR();
  LL_SPI_TransmitData8( SPI, Data );
//  WaitTXE();
  WaitRXNE();
  return LL_SPI_ReceiveData8( SPI );
}

void TDriverSpi::Write( uint8_t const Data )
{
  (void)WriteRead( Data );
//  TransmitBSY( Data );
//  WaitNBSY();
}

void TDriverSpi::Write( void const* const TxData, uint32_t const Length )
{
  auto TxPtr = reinterpret_cast< uint8_t const*>( TxData );
  auto const TxEnd = &TxPtr[ Length ];

  do
  {
    (void)WriteRead( *TxPtr++ );
//    LL_SPI_TransmitData8( SPI, *TxPtr++ );
//    WaitTXE();
  }
  while( TxPtr < TxEnd );

//  WaitNBSY();
}

void TDriverSpi::Read( void* const RxData, uint32_t const Length )
{
  auto RxPtr = reinterpret_cast< uint8_t*>( RxData );
  auto const RxEnd = &RxPtr[ Length ];

  ClearOVR();

  do
  {
    *RxPtr++ = WriteRead();
//    LL_SPI_TransmitData8( SPI, 0x00 );
//    WaitTXE();
//    WaitRXNE();
//    *RxPtr++ = LL_SPI_ReceiveData8( SPI );
  }
  while( RxPtr < RxEnd );
}

void TDriverSpi::WriteRead( void const* const TxData, void* const RxData, uint32_t const Length )
{
  auto RxPtr = reinterpret_cast< uint8_t*>( RxData );
  auto TxPtr = reinterpret_cast< uint8_t const*>( TxData );
  auto const TxEnd = &TxPtr[ Length ];

  ClearOVR();

  do
  {
    *RxPtr++ = WriteRead( *TxPtr++ );
//    LL_SPI_TransmitData8( SPI, *TxPtr++ );
//    WaitTXE();
//    WaitRXNE();
//    *RxPtr++ = LL_SPI_ReceiveData8( SPI );
  }
  while( TxPtr < TxEnd );
}
