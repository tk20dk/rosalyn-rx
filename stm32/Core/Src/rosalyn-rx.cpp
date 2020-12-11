#include "rosalyn-rx.h"


TDriverSpi Spi;
TRosalynRx RosalynRx;

uint32_t volatile TickSys;
extern "C" uint32_t HAL_GetTick()
{
  return TickSys;
}

void TRosalynRx::TestPWM()
{
}


void TRosalynRx::Loop()
{
  TestPWM();
  HmiLoop();

  static uint32_t LastTick;
  auto const Tick = HAL_GetTick();
  if(( Tick > LastTick ) && ( Tick % 14 ) == 0 )
  {
    LastTick = Tick;
    SbusSerial.Transmit( SbusDataDownstream );
  }

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
    HmiError( 5 );
  }
}

void TRosalynRx::RadioEvent( TRadioEvent const Event )
{
  uint8_t Buffer[ 256 ];

  if( Event == TRadioEvent::RxDone )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const LenRx = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    HmiStatus( 10 );
    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u Length error\n", Rssi, Snr / 10, abs(Snr) % 10, LenRx );

    if( LenRx == 25 )
    {
      int32_t LenOut;
      TSbusFrame SbusFrameRx;

      AesCrypto.DecryptCFB( Buffer, LenRx, SbusFrameRx.Buffer, LenOut );
      TSbusData SbusData( SbusFrameRx );

      auto const SbusFrameTx = SbusData.Encode();
      AesCrypto.EncryptCFB( SbusFrameTx.Buffer, LenRx, Buffer, LenOut );

      Radio.Transmit( Buffer, LenRx );
    }
    else
    {
      Radio.Receive();
    }
  }

  if( Event == TRadioEvent::TxDone )
  {
    HmiStatus( 10 );
    Radio.Receive();
  }

  if( Event == TRadioEvent::Timeout )
  {
    HmiError( 1000 );
  }

  if( Event == TRadioEvent::CrcError )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    HmiError( 1000 );
    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u CRC Error\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }

  if( Event == TRadioEvent::NoCrc )
  {
    auto const Snr = Radio.GetSnr();
    auto const Rssi = Radio.GetRssi();
    auto const Length = Radio.ReadPacket( Buffer, sizeof( Buffer ));

    HmiError( 1000 );
    UartPrintf( "Rssi:%4d Snr:%3d.%u Len:%u No CRC\n", Rssi, Snr / 10, abs(Snr) % 10, Length );
  }
}

void TRosalynRx::Setup()
{
//  NvData.Load();
  UartPrintf( "RosalynTX\n" );
  HmiStatus();

  // Enable SysTick IRQ
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

  Spi.Setup();
  SbusSerial.Setup();

  if( Radio.Setup( NvData.Modulation[ 2 ], NvData.TxPower, NvData.Channel ))
  {
    Radio.Receive();
  }

  LL_TIM_CC_EnableChannel( TIM2, LL_TIM_CHANNEL_CH1 );
  LL_TIM_CC_EnableChannel( TIM2, LL_TIM_CHANNEL_CH2 );
  LL_TIM_EnableCounter( TIM2 );
  LL_TIM_OC_SetCompareCH1( TIM2, 2000 );
  LL_TIM_OC_SetCompareCH2( TIM2, 2000 );

  LL_TIM_CC_EnableChannel( TIM3, LL_TIM_CHANNEL_CH1 );
  LL_TIM_CC_EnableChannel( TIM3, LL_TIM_CHANNEL_CH2 );
  LL_TIM_EnableCounter( TIM3 );
  LL_TIM_OC_SetCompareCH1( TIM3, 2000 );
  LL_TIM_OC_SetCompareCH2( TIM3, 2000 );

  LL_TIM_CC_EnableChannel( TIM16, LL_TIM_CHANNEL_CH1 );
  LL_TIM_EnableCounter( TIM16 );
  LL_TIM_OC_SetCompareCH1( TIM16, 2000 );

  LL_TIM_CC_EnableChannel( TIM17, LL_TIM_CHANNEL_CH1 );
  LL_TIM_EnableCounter( TIM17 );
  LL_TIM_OC_SetCompareCH1( TIM17, 2000 );
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
}

TRosalynRx::TRosalynRx() :
  NvData(),
  IcmFlag( false ),
  TimerFlag( false ),
  RadioFlag( false ),
  SerialFlag( false ),
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
  TimeoutHmiError( 0 ),
  TimeoutHmiStatus( 0 ),
  SbusDataUpstream(),
  SbusDataDownstream(),
  AesCrypto( NvData.AesIV, NvData.AesKey ),
  SbusSerial( USART1, SerialFlag )
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
