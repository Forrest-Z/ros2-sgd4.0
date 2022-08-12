#ifndef VOICE_MESSAGES_HPP
#define VOICE_MESSAGES_HPP

#include <cstring>
#include <string>

class VoiceMessages
{
    private:    
        std::string messages[7] = {"achtung", "buergersteige", "links", "rechts", "start", "stop", "ziel_erreicht"};
        std::string bashfile = "~/dev_ws/src/ros2-sgd4.0/sgd_audio/src/play_message.sh ";

    public:
        VoiceMessages();
        ~VoiceMessages();
        enum num_messages {achtung = 0, buergersteige, links, rechts, start, stop, ziel_erreicht};

        /**
         * @brief Plays the audio file indicated by the enum.
         * 
         * @param message enumeration of voice messages.
         */
        void say(num_messages message);


};

#endif