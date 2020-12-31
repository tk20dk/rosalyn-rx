#include "rosalyn-rx.h"


TRosalynRx RosalynRx;

uint32_t volatile TickSys;
extern "C" uint32_t HAL_GetTick()
{
  return TickSys;
}

static uint32_t Map( uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max )
{
  if( x < in_min ) x = in_min;
  if( x > in_max ) x = in_max;
  return ((( x - in_min ) * ( out_max - out_min )) / ( in_max - in_min )) + out_min;
}

uint32_t tMax = 0;
uint32_t tMin = 0xffffffffu;
uint32_t tCount;

void TRosalynRx::UpdatePWM( TSbusData const &SbusData )
{
  if( tCount < 500 )
  {
    tCount++;
  }
  else
  {
    auto const Count = LL_TIM_GetCounter( TIM7 );

    if( Count > tMax )
      tMax = Count;
    if( Count < tMin )
      tMin = Count;
  }

  LL_TIM_SetCounter( TIM7, 0 );

  LL_TIM_OC_SetCompareCH1( TIM2, Map( SbusData.Ch1, 0, 2047, 2000, 4000));
  LL_TIM_OC_SetCompareCH2( TIM2, Map( SbusData.Ch2, 0, 2047, 2000, 4000));
  LL_TIM_OC_SetCompareCH1( TIM3, Map( SbusData.Ch3, 0, 2047, 2000, 4000));
  LL_TIM_OC_SetCompareCH2( TIM3, Map( SbusData.Ch4, 0, 2047, 2000, 4000));
//  LL_TIM_OC_SetCompareCH1( TIM16, Map( SbusData.Ch5, 0, 2047, 2000, 4000));
//  LL_TIM_OC_SetCompareCH1( TIM17, Map( SbusData.Ch6, 0, 2047, 2000, 4000));
  LL_TIM_OC_SetCompareCH1( TIM16, Map( SbusData.Ch7, 0, 2047, 2000, 4000));
  LL_TIM_OC_SetCompareCH1( TIM17, Map( SbusData.Ch8, 0, 2047, 2000, 4000));
}

void TRosalynRx::Loop()
{
  HmiLoop();

  if( IcmFlag )
  {
    IcmFlag = false;
  }

  if( RadioFlag )
  {
    RadioFlag = false;
    Radio.Interrupt();
  }

  if( SerialFlag )
  {
    SerialFlag = false;
    SbusDataUpstream = SbusSerial.Receive();
  }
}

void TRosalynRx::RadioEvent( TRadioEvent const Event )
{
  uint8_t Buffer[ 64 ];

  if( Event == TRadioEvent::RxDone )
  {
ResetPin( HMI_ERROR_GPIO_Port, HMI_ERROR_Pin );
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const LenRx = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, LenRx );

    if( LenRx == 25 )
    {
      int32_t LenOut;
      TSbusFrame SbusFrameRx;

      auto const Status0 = AesCrypto.DecryptCFB( Buffer, LenRx, SbusFrameRx.Buffer, LenOut );
      if(( Status0 == AES_SUCCESS ) && ( LenOut == 25 ))
      {
        TSbusData SbusData( SbusFrameRx );
        UpdatePWM( SbusData );
        SbusSerial.Transmit( SbusData );

        auto const SbusFrameTx = SbusData.Encode();
        auto const Status1 = AesCrypto.EncryptCFB( SbusFrameTx.Buffer, LenRx, Buffer, LenOut );
        if(( Status1 == AES_SUCCESS ) && ( LenOut == 25 ))
        {
          Radio.Transmit( Buffer, LenOut );
        }
        else
        {
          Radio.Receive();
          HmiError( 1000 );
        }
      }
      else
      {
        Radio.Receive();
        HmiError( 1000 );
      }
    }
    else
    {
      Radio.Receive();
      HmiError( 1000 );
    }
  }

  if( Event == TRadioEvent::TxDone )
  {
    Radio.Receive();
SetPin( HMI_ERROR_GPIO_Port, HMI_ERROR_Pin );
  }

  if( Event == TRadioEvent::Timeout )
  {
    HmiError( 1000 );
    Radio.Receive();
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
    HmiError( 1000 );
    Radio.Receive();
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
    HmiError( 1000 );
    Radio.Receive();
  }
}

void TRosalynRx::Setup()
{
//  NvData.Load();
  UartPrintf( "RosalynTX\n" );
  HmiStatus();

  // Enable SysTick IRQ
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

  RadioSpi.Setup();
  SbusSerial.Setup();

  if( Radio.Setup( NvData.Modulation[ 2 ], NvData.TxPower, NvData.Channel ))
  {
    Radio.Receive();
  }

  LL_TIM_EnableIT_UPDATE( TIM7 );
  LL_TIM_EnableCounter( TIM7 );

  LL_TIM_CC_EnableChannel( TIM2, LL_TIM_CHANNEL_CH1 );
  LL_TIM_CC_EnableChannel( TIM2, LL_TIM_CHANNEL_CH2 );
  LL_TIM_EnableCounter( TIM2 );

  LL_TIM_CC_EnableChannel( TIM3, LL_TIM_CHANNEL_CH1 );
  LL_TIM_CC_EnableChannel( TIM3, LL_TIM_CHANNEL_CH2 );
  LL_TIM_EnableCounter( TIM3 );

  LL_TIM_CC_EnableChannel( TIM16, LL_TIM_CHANNEL_CH1 );
  LL_TIM_EnableCounter( TIM16 );

  LL_TIM_CC_EnableChannel( TIM17, LL_TIM_CHANNEL_CH1 );
  LL_TIM_EnableCounter( TIM17 );
}

void TRosalynRx::HmiLoop()
{
  if( TimeoutHmiError && ( HAL_GetTick() >= TimeoutHmiError ))
  {
    TimeoutHmiError = 0;
    SetPin( HMI_ERROR_GPIO_Port, HMI_ERROR_Pin );
  }

  if( TimeoutHmiStatus && ( HAL_GetTick() >= TimeoutHmiStatus ))
  {
    TimeoutHmiStatus = 0;
    SetPin( HMI_STATUS_GPIO_Port, HMI_STATUS_Pin );
  }
}

void TRosalynRx::HmiError( uint32_t const Interval )
{
  if( Interval )
  {
    TimeoutHmiError = HAL_GetTick() + Interval;
  }
  ResetPin( HMI_ERROR_GPIO_Port, HMI_ERROR_Pin );
}

void TRosalynRx::HmiStatus( uint32_t const Interval )
{
  if( Interval )
  {
    TimeoutHmiStatus = HAL_GetTick() + Interval;
  }
  ResetPin( HMI_STATUS_GPIO_Port, HMI_STATUS_Pin );
}

void TRosalynRx::SysTick_Handler()
{
  TickSys++;
}

void TRosalynRx::TIM7_IRQHandler()
{
  if( LL_TIM_IsActiveFlag_UPDATE( TIM7 ) == 1 )
  {
    LL_TIM_ClearFlag_UPDATE( TIM7 );
  }
}

void TRosalynRx::EXTI2_3_IRQHandler()
{
  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_2 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_2 );
    IcmFlag = true;
  }
}

void TRosalynRx::EXTI4_15_IRQHandler()
{
  // Radio DIO1 interrupt
  if( LL_EXTI_IsActiveFlag_0_31( LL_EXTI_LINE_10 ) != RESET )
  {
    LL_EXTI_ClearFlag_0_31( LL_EXTI_LINE_10 );
    RadioFlag = true;
  }
}

void TRosalynRx::USART2_IRQHandler()
{
}

void TRosalynRx::USART3_4_IRQHandler()
{
  SbusSerial.USART_IRQHandler();
}

TRosalynRx::TRosalynRx() :
  NvData(),
  IcmFlag( false ),
  TimerFlag( false ),
  RadioFlag( false ),
  SerialFlag( false ),
  RadioSpi( SPI2 ),
  Radio(
    RadioSpi,
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
  TimeoutHmiError( 0 ),
  TimeoutHmiStatus( 0 ),
  SbusDataUpstream(),
  SbusDataDownstream(),
  AesCrypto( NvData.AesIV, NvData.AesKey ),
  SbusSerial( USART3, SerialFlag )
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

extern "C" void SysTick_Handler()
{
  RosalynRx.SysTick_Handler();
}

extern "C" void TIM7_IRQHandler()
{
  RosalynRx.TIM7_IRQHandler();
}

extern "C" void EXTI2_3_IRQHandler()
{
  RosalynRx.EXTI2_3_IRQHandler();
}

extern "C" void EXTI4_15_IRQHandler()
{
  RosalynRx.EXTI4_15_IRQHandler();
}

extern "C" void USART2_IRQHandler()
{
  RosalynRx.USART2_IRQHandler();
}

extern "C" void USART3_4_IRQHandler()
{
  RosalynRx.USART3_4_IRQHandler();
}
