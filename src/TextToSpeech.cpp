#include "TextToSpeech.h"
#include <stdio.h>
#include <string.h>


TextToSpeech::TextToSpeech(){
    FT_STATUS ftStatus;
    ftStatus = FT_Open(0, &ftHandle);
    if(ftStatus == FT_OK){
        std::cout << "FTDI connected..." << std::endl;
        FT_SetBaudRate(ftHandle, 9600);
    }
}

TextToSpeech::~TextToSpeech(){
    FT_Close(ftHandle);
}

void TextToSpeech::setString(std::string text){
    std::string nText = normalizeString(text);
    std::string finalString = "S" + nText + "\n";
    std::cout << "Saying: " << text << "... ";

    sendCommand(finalString);
}

void TextToSpeech::setVolume(unsigned int vol){
    int realVol = vol - 48;
    std::string result;
    std::ostringstream convert;
    convert << realVol;

    result = "V" + convert.str() + "\n";
    std::cout << "Setting volume at: " << vol << "... ";
    sendCommand(result);
}

void TextToSpeech::setVoiceType(unsigned int type){
    std::string result;
    std::ostringstream convert;
    convert << type;

    result = "N" + convert.str() + "\n";
    std::cout << "Setting voice type: " << type << "... ";
    sendCommand(result);
}

void TextToSpeech::setSpeakingRate(unsigned int wordsPerMinute){
    std::string result;
    std::ostringstream convert;
    convert << wordsPerMinute;

    result = "W" + convert.str() + "\n";
    std::cout << "Setting words per minute: " << wordsPerMinute << "... ";
    sendCommand(result);
}

void TextToSpeech::setLanguage(unsigned int type){
    std::string result;
    std::ostringstream convert;
    convert << type;

    result = "L" + convert.str() + "\n";
    std::cout << "Setting language type: " << type << "... ";
    sendCommand(result);
}

void TextToSpeech::playDemo(unsigned int demo){
    std::string result;
    std::ostringstream convert;
    convert << demo;

    result = "D" + convert.str() + "\n";
    std::cout << "Playing demo: " << demo << "... ";
    sendCommand(result);
}

void TextToSpeech::setParser(int parser){
    std::string result;
    std::ostringstream convert;
    convert << parser;

    result = "P" + convert.str() + "\n";
    std::cout << "Setting Parser: " << parser << "... ";
    sendCommand(result);
}

void TextToSpeech::setDefaultConfiguration(){
    setVolume();
    setVoiceType();
    setSpeakingRate();
    setLanguage();
    setParser();
}

void TextToSpeech::increasePitch(){
    sendCommand("/\\\n");
}

void TextToSpeech::decreasePitch(){
    sendCommand("\\/\n");
}

int TextToSpeech::sendCommand(std::string text) {
    int result = 0;
    FT_STATUS ftStatus;
    DWORD bytesWritten;
    ftStatus = FT_Write(this->ftHandle, (char*)text.c_str(), text.length(), &bytesWritten);
    if(ftStatus == FT_OK){
        std::cout << "OK..." << std::endl;
    } else {
        std::cout << "Failed..." << std::endl;
        result = 1;
    }
    return result;
}

bool TextToSpeech::isEndOfSpeaking(){
    bool result = false;
    FT_STATUS ftStatus;
    DWORD rxBytes = 10;
    DWORD bytesReceived; 
    char rxBuffer[10];

    FT_SetTimeouts(this->ftHandle, 1000, 0);

    ftStatus = FT_Read(this->ftHandle, rxBuffer, rxBytes, &bytesReceived);

    if(ftStatus == FT_OK){
        if(bytesReceived > 0){
            if(rxBuffer[1] == ':'){
                //std::cout << "Done receiving :!!!..." << std::endl;
                result = true;
            } else {
                result = false;
            }
        }
    } else {
        //std::cout << "received failed..." << std::endl;
    }
    return result;
}

 std::string TextToSpeech::normalizeString(std::string str){
    std::string result;
    unsigned char* str_c = (unsigned char*)str.c_str();

    for(int i = 0; i < str.length(); i++){
        if(((int)str.at(i)) == SPECIAL_CHARACTER){
            i++;
            if(((int)str.at(i)) == SMALL_LETTER_ENIE){
                result += "\\xF1";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_ENIE){
                result += "\\xD1";
            } else if(((int)str.at(i)) == SMALL_LETTER_A_ACUTE){
                result += "\\xE1";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_A_ACUTE){
                result += "\\xC1";
            } else if(((int)str.at(i)) == SMALL_LETTER_E_ACUTE){
                result += "\\xE9";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_E_ACUTE){
                result += "\\xC9";
            } else if(((int)str.at(i)) == SMALL_LETTER_I_ACUTE){
                result += "\\xED";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_I_ACUTE){
                result += "\\xCD";
            } else if(((int)str.at(i)) == SMALL_LETTER_O_ACUTE){
                result += "\\xF3";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_O_ACUTE){
                result += "\\xD3";
            } else if(((int)str.at(i)) == SMALL_LETTER_U_ACUTE){
                result += "\\xFA";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_U_ACUTE){
                result += "\\xDA";
            } else if(((int)str.at(i)) == SMALL_LETTER_U_UMLAUT){
                result += "\\xFC";
            } else if(((int)str.at(i)) == CAPITAL_LETTER_U_UMLAUT){
                result += "\\xDC";
            }
        }else{
            result.push_back(str.at(i));
        }
    }
    return result;
}