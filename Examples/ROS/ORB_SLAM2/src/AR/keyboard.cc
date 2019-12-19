#include"keyboard.h"


using namespace std;


namespace ORB_SLAM2
{
keyboard::keyboard()
{

}

void keyboard::Run()
{
    while(1)
    {
	scanKeyboard();
    }
}

int keyboard::scanKeyboard()
{
    int in;
    struct termios new_settings;
    struct termios stored_settings;
    tcgetattr(0,&stored_settings);
    new_settings = stored_settings;
    new_settings.c_lflag &= (~ICANON);
    new_settings.c_cc[VTIME] = 0;
    tcgetattr(0,&stored_settings);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0,TCSANOW,&new_settings);    
    in = getchar();
    mna = in;
    tcsetattr(0,TCSANOW,&stored_settings);
    return in;
}

int keyboard::scanKeyboard1()
{
    int in;
    int keys_fd;
    //char ret[2];
    struct input_event t;
    keys_fd=open("/dev/input/event0",O_RDONLY);
    if(keys_fd<=0)
    {   
        printf("error\n");
        return -1; 
    }   
   
       in = read(keys_fd,&t,sizeof(struct input_event));
        if(t.type==1)
            printf("key %i state %i \n",t.code,t.value);
  
    //close(keys_fd);
    return in;

}

}