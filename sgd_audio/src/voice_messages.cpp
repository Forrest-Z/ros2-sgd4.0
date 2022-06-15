#include "sgd_audio/voice_messages.hpp"


VoiceMessages::VoiceMessages()
{

}
VoiceMessages::~VoiceMessages()
{

}

void VoiceMessages::say(num_messages message)
{
    std::string shell_command = bashfile + messages[message];
    char * play_command = new char [shell_command.length() + 1];
    std::strcpy(play_command, shell_command.c_str());
    system(play_command);
}




