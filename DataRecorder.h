#define SD_CS 7

#include <WString.h>

class DataRecorder
{
  public:
    void init();
    void writeData(long currentTime, String string);
};
