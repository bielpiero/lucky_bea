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
	void onSet   (std::wstring word, int start, int* end);
	bool nucleus (std::wstring word, int start, int* end);
	void coda    (std::wstring word, int start, int* end);

	bool hiatus (std::wstring vowels, int* size);
	bool triphthong (std::wstring vowels, int* size);

	bool openVowel (wchar_t vowel);
	bool openAcutedVowel(wchar_t vowel);
	bool closedVowel (wchar_t vowel);
	bool closedAcutedVowel(wchar_t vowel);
	bool isConsonant (wchar_t vowel);
	bool isAlveolarConsonant(wchar_t letter);
	std::string fixSyllable(std::wstring syllable);
	int getSyllables (std::wstring word, std::vector<std::wstring>* syllables);

public:
	DorisLipSync(SerialPort* mc, int faceId);
    virtual ~DorisLipSync(){}

    std::wstring textNorm(const std::wstring, wchar_t symbol); /* normalizes the text by removing the symbols of the string */
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
