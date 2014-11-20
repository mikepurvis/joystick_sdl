#include <SDL.h>

int main(int argc, char *argv[])
{
  if (SDL_Init(SDL_INIT_JOYSTICK))
  {
    return 1;
  }
  int num_joy, i;
  num_joy=SDL_NumJoysticks();
  printf("%d joysticks found\n", num_joy);
  for(i=0;i<num_joy;i++)
      printf("%s\n", SDL_JoystickName(i));

  SDL_Quit();
  return 0;
}
