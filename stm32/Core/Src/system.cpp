#include <cstdio>
#include <cstdarg>
#include "system.h"


extern "C" UART_HandleTypeDef huart2;

int putchar( char const ch )
{
  if( ch == '\n' )
  {
    char cr = '\r';
    auto const Status = HAL_UART_Transmit( &huart2, (uint8_t*)&cr, 1, 50U );
    assert_param( Status == HAL_OK );
  }

  auto const Status = HAL_UART_Transmit( &huart2, (uint8_t*)&ch, 1, 50U );
  assert_param( Status == HAL_OK );

  return 0;
}

void UartPrintf( char const *const Format, ... )
{
  char Buffer[ 256 ];
  va_list Args;
  va_start( Args, Format );
  int const Result = vsnprintf( Buffer, sizeof( Buffer ), Format, Args );
  va_end( Args );

  for( auto Index = 0; Index < Result; Index ++ )
  {
    putchar( Buffer[ Index ] );
  }
}
