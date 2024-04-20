#ifndef ENCODER_H
#define ENCODER_H

class Encoder {
  public:		
		Encoder(unsigned int pulsePerRotation, unsigned int indexCount, bool moduloPositioning);
		int update(long rollingCount);
		unsigned int correctOverflow(long rollingCounter);
		int CWDistanceTo(unsigned int actuatorPos, unsigned int destPos, unsigned int zeroWindow);
		int getCurrentIndexPos(unsigned int index);
		int getStaticIndexPos(unsigned int index);
		int getPosition();
		unsigned int getPulsePerRotation();
		unsigned int getPulsePerIndex();
		unsigned int getIndexCount();
		void setPosition(int pos);
  private:
		int cPos;			// Current Position
		unsigned int ppr; 	// Pulse Per Rotation
		unsigned int ppi; 	// Pulse Per Index
		unsigned int ic;	// Index Count
		bool modeModulo;	// Resets to 0 on ppr overflow
};
#endif
