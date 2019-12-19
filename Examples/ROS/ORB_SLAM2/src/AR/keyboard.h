#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <string>

#include <termio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>

namespace ORB_SLAM2
{
class keyboard
{
public:
  keyboard();
  
  void Run();
  int scanKeyboard();
  int scanKeyboard1();

  int mna;

};
}

#endif // KEYBOARD_H