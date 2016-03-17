#ifndef TEXT_TO_SPEECH_H
#define TEXT_TO_SPEECH_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <pthread.h>
#include "ftd2xx.h"

class TextToSpeech{
	public:
		TextToSpeech();
		~TextToSpeech();
		void setDefaultConfiguration();
        void setString(std::string text);
        void setVolume(unsigned int vol = 66);
        void setVoiceType(unsigned int type = 8);
        void setSpeakingRate(unsigned int wordsPerMinute = 200);
        void setLanguage(unsigned int type = 2);
        void playDemo(unsigned int demo);
        void setParser(int parser = 1);
        void increasePitch();
        void decreasePitch();
        bool isEndOfSpeaking();
    private:
    	int sendCommand(std::string text);
    	FT_HANDLE ftHandle;
};
#endif