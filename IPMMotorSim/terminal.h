#ifndef TERMINAL_TESTSTUBS_H
#define TERMINAL_TESTSTUBS_H
#include <stdint.h>
#include "printf.h"

class Terminal : public IPutChar
{
public:
   Terminal();
   void PutChar(char c) override;
   void SendBinary(uint8_t* data, uint32_t length);
   static Terminal* defaultTerminal;
   void BinaryLogging(char *arg);
   bool BinLoggingEnabled();

private:
   bool binLoggingEnabled = false;
};

#endif // CPP_TESTSTUBS_H
