#ifndef DORISLIPSYNC_H
#define DORISLIPSYNC_H

#define MAX_SYLLABLES 1000
#define MAX_WORD_LENGTH 1000

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>

#include <iostream>
#include <string>
#include <fstream>
#include <vector>
#include <algorithm>

#include "SerialPort.h"
#include "xmldefs.h"
#include "TextToSpeech.h"
#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"

#define XML_FILE_LIP_SYNC_PATH "/conf/BeaLipSync.xml"
#define XML_FILE_VISEMES_CODES_PATH "/conf/BeaVisemeCodes.xml"

class DorisLipSync 
{
private:
	int  wordLength;
	int  numSyl;
	int  stressed;
	bool stressedFound;
	int  letterAccent;
	std::vector<int> positions;
	std::string lastWord;
	//int  positions [MAX_SYLLABLES + 1];
	//char lastWord [MAX_WORD_LENGTH + 1];

	std::string xmlLipSyncFullPath;
	std::string xmlVisemesCodesFullPath;

	SerialPort* mc;
	TextToSpeech* tts;

	void onSet   (std::string, int &);
	void nucleus (std::string, int &);
	void coda    (std::string, int &);

	bool hiatus ();

	void process (std::string);

	bool openVowel (char);
	bool isConsonant (char);

	void syllablePositions ();

public:
	DorisLipSync(SerialPort* mc, std::string packagePath);
    virtual ~DorisLipSync(){}

	int numberOfSyllables (const char *); /* Returns the number of syllables in a word */
	std::vector<int> syllablePositions (const char *); /* Returns an array with the start positions of every syllables */
    int stressedSyllable (const char *);
    char* cutData (char *, int, int); /* Cut the caracter chains in the Start/Stop range */
    char* textNorm(const char *, char); /* normalizes the text by removing the symbols of the string */
    char* syllableToViseme(char *); /* Convert the syllable to Viseme */
    void textToViseme(const char *); /* Convert all Syllables in actual Viseme */
    float timeSync(int, int); /* Calculates the time for sync */
    char* speakingRate(char *); /* Calculates the Speaking Rate*/
    char* amplitudeWave(char *); /* Calculates the Amplitude of the wave of sound*/
    int configureEmicTwo(char *, char *); /* Configure Device TTS Emic 2 Parallax */
    void setViseme(std::string id); /* Set the Viseme configuration to the mouth of Doris */    
    void selectMotion(char *, float); /* Set motion configuration to the mouth of Doris */

};

#endif
