#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "main.h"


void UartPrintf( char const *const Format, ... );

inline void SetPin( GPIO_TypeDef *const Gpio, uint32_t const PinMask )
{
  LL_GPIO_SetOutputPin( Gpio, PinMask );
}
inline void ResetPin( GPIO_TypeDef *const Gpio, uint32_t const PinMask )
{
  LL_GPIO_ResetOutputPin( Gpio, PinMask );
}
inline bool ReadPin( GPIO_TypeDef *const Gpio, uint32_t const PinMask )
{
  return LL_GPIO_IsInputPinSet( Gpio, PinMask ) != 0;
}
inline void TogglePin( GPIO_TypeDef *const Gpio, uint32_t const PinMask )
{
	LL_GPIO_TogglePin( Gpio, PinMask );
}

#endif // SYSTEM_H_
