#include "SDL_gamecontroller.h"
#include "SDL.h"

int main(int argc, char *argv[])
{
  if (SDL_Init(SDL_INIT_GAMECONTROLLER | SDL_INIT_JOYSTICK))
  {
    printf("SDL_Init failed.\r\n");
    return 1;
  }
  int ret = SDL_GameControllerAddMappingsFromFile("SDL_GameControllerDB/gamecontrollerdb.txt");
  printf("ret %d\r\n", ret);
  SDL_JoystickEventState(SDL_DISABLE);

  int num_joy=SDL_NumJoysticks();
  printf("%d joysticks found\n", num_joy);

  SDL_Joystick* joystick = SDL_JoystickOpen(0);
  if (joystick == NULL)
  {
    printf("Joystick not opened.\r\n");
    return 1;
  }

  while(true)
  {
    SDL_JoystickUpdate();
    int16_t x = SDL_JoystickGetAxis(joystick, 0);

    printf("x: %d\r\n", x);
  }

  SDL_Quit();
  return 0;
}
