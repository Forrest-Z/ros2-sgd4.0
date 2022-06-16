# Voice messages

This program works by calling a bash file which plays the sound.

The pre-recorded messages are: *Achtung , Bürgersteige, Links, Rechts, Start, Stop und Ziel erreicht*.

## Usage

Simply create a **VoiceMessages** object and use the method **say**. Then type the object name and choose the voice message from an `enum`.

###### Example code

```c++
#include <iostream>
#include "voice_messages.hpp"

int main() {
    VoiceMessages robot1;
    robot1.say(robot1.start);
    robot1.say(robot1.stop);
    return 0;
}
```

