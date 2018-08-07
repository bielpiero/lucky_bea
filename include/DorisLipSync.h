#ifndef DORISLIPSYNC_H
#define DORISLIPSYNC_H

#define MAX_SYLLABLES 1000
#define MAX_WORD_LENGTH 1000

#include "TextToSpeech.h"
#include "SerialPort.h"
#include "xmldefs.h"
#include "FaceGesture.h"

#include "xml/rapidxml_print.hpp"
#include "xml/rapidxml.hpp"

#define XML_FILE_LIP_SYNC_PATH "conf/BeaLipSync.xml"
#define XML_FILE_VISEMES_CODES_PATH "conf/BeaVisemeCodes.xml"

#define SPANISH_LANG_STR "es"
#define ENGLISH_LANG_STR "en"

using namespace rapidxml;

class DorisLipSync 
{
private:
	std::vector<FaceGesture*> *gestures;

	std::string xmlLipSyncFullPath;
	std::string xmlVisemesCodesFullPath;

	SerialPort* mc;
	TextToSpeech* tts;
	int faceId;

private:
	void onSet   (std::string word, int start, int* end);
	bool nucleus (std::string word, int start, int* end);
	void coda    (std::string word, int start, int* end);

	bool hiatus (std::string vowels, int* size);
	bool triphthong (std::string vowels, int* size);

	bool openVowel (char vowel);
	bool openAcutedVowel(char vowel);
	bool closedVowel (char vowel);
	bool closedAcutedVowel(char vowel);
	bool isConsonant (char vowel);
	std::string fixSyllable(std::string syllable);
	int getSyllables (std::string word, std::vector<std::string>* syllables);

public:
	DorisLipSync(SerialPort* mc, int faceId);
    virtual ~DorisLipSync(){}

    std::string textNorm(const std::string str); /* normalizes the text by removing the symbols of the string */
    std::string removeExtraChars(const std::string str, char symbol); /* normalizes the text by removing the symbols of the string */
    std::string syllableToViseme(const std::string syllable); /* Convert the syllable to Viseme */
    void textToViseme(const std::string text); /* Convert all Syllables in actual Viseme */
    float timeSync(int, int); /* Calculates the time for sync */
    int speakingRate(const double& emotion); /* Calculates the Speaking Rate*/
    int amplitudeWave(const double& emotion); /* Calculates the Amplitude of the wave of sound*/
    int configureEmicTwo(const double& emotion, const std::string lang = SPANISH_LANG_STR); /* Configure Device TTS Emic 2 Parallax */
    void setViseme(std::string id); /* Set the Viseme configuration to the mouth of Doris */    
    void selectMotion(int motion, float a); /* Set motion configuration to the mouth of Doris */

};

#endif
