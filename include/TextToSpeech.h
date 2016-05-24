#ifndef TEXT_TO_SPEECH_H
#define TEXT_TO_SPEECH_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <clocale>
#include <pthread.h>

#include "ftd2xx.h"
#define SPECIAL_CHARACTER -61

#define SMALL_LETTER_ENIE -79
#define SMALL_LETTER_A_ACUTE -95
#define SMALL_LETTER_E_ACUTE -87
#define SMALL_LETTER_I_ACUTE -83
#define SMALL_LETTER_O_ACUTE -77
#define SMALL_LETTER_U_ACUTE -70
#define SMALL_LETTER_U_UMLAUT -68

#define CAPITAL_LETTER_ENIE -111
#define CAPITAL_LETTER_A_ACUTE -127
#define CAPITAL_LETTER_E_ACUTE -119
#define CAPITAL_LETTER_I_ACUTE -115
#define CAPITAL_LETTER_O_ACUTE -109
#define CAPITAL_LETTER_U_ACUTE -102
#define CAPITAL_LETTER_U_UMLAUT -100


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
        std::string normalizeString(std::string str);
    	FT_HANDLE ftHandle;
};
#endif