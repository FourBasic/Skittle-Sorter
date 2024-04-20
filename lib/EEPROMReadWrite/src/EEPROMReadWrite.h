#ifndef EEPROMREADWRITE_H
#define EEPROMREADWRITE_H

class EEPROMReadWrite {
  public:		
		EEPROMReadWrite();
		void read();	
  private:
		bool raw;
};
#endif
