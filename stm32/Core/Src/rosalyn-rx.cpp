#include "rosalyn-rx.h"


TSpi Spi( hspi1 );
TRosalynRx RosalynRx;

void TRosalynRx::Loop()
{
  if( RadioFlag )
  {
    RadioFlag = false;
    Radio.Interrupt();
  }
}

void TRosalynRx::RadioEvent( TRadioEvent const Event )
{
  uint8_t Buffer[ 256 ];

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );

    Radio.Receive();
  }

  if( Event == TRadioEvent::TxDone )
  {
    Radio.Receive();
  }

  if( Event == TRadioEvent::Timeout )
  {
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TRosalynRx::Setup()
{
  NvData.Setup();
  UartPrintf( "RosalynTX\n" );
  HmiStatus( true );

  if( Radio.Setup( NvData.Modulation[ 2 ], NvData.TxPower, NvData.Channel ))
  {
    Radio.Receive();
  }
}

void TRosalynRx::HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin )
{
  switch( GPIO_Pin )
  {
    case RADIO_DIO1_Pin:
    {
      RadioFlag = true;
    }
    break;

    case RADIO_DIO2_Pin:
    {
    }
    break;

    case RADIO_BUSY_Pin:
    {
    }
    break;

    default:
    {
      HmiError( true );
    }
    break;
  }
}

TRosalynRx::TRosalynRx() :
  Failsafe( true ),
  RadioFlag( false ),
  Radio(
    433050000,
	RADIO_NSS_GPIO_Port,
	RADIO_NSS_Pin,
	RADIO_NRST_GPIO_Port,
	RADIO_NRST_Pin,
	RADIO_BUSY_GPIO_Port,
	RADIO_BUSY_Pin,
	RADIO_RXEN_GPIO_Port,
	RADIO_RXEN_Pin,
	RADIO_TXEN_GPIO_Port,
	RADIO_TXEN_Pin,
    std::bind( &TRosalynRx::RadioEvent, this, std::placeholders::_1 )),
  NvData()
{
}

extern "C" void RosalynRxLoop()
{
  RosalynRx.Loop();
}

extern "C" void RosalynRxSetup()
{
  RosalynRx.Setup();
}

extern "C" void HAL_GPIO_EXTI_Callback( uint16_t const GPIO_Pin )
{
  RosalynRx.HAL_GPIO_EXTI_Callback( GPIO_Pin );
}
