#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Camera {
  vec4 position;
};

struct Intersection {
  vec4 position;
  float distance;
  int triangleIndex;
};

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 320
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH SCREEN_HEIGHT/2
#define SENSITIVITY 0.1

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(Camera &camera);
void Draw(screen* screen,Camera &camera, std::vector<Triangle> &triangles,Intersection &closestIntersection);
bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection );


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  std::vector<Triangle> triangles;
  Camera camera;
  Intersection closestIntersection;
  //Initialize triangles
  LoadTestModel(triangles);
  camera.position = vec4(0,0,-2,1);

  while( NoQuitMessageSDL() ){
      Update(camera);
      Draw(screen,camera,triangles,closestIntersection);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen,Camera &camera, std::vector<Triangle> &triangles,Intersection &closestIntersection){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vec3 black(0.0,0.0,0.0);
  for(int x = 0; x < SCREEN_WIDTH; x++){
    for(int y = 0; y < SCREEN_HEIGHT; y++){
    vec4 d(x - (SCREEN_WIDTH/2), y - (SCREEN_HEIGHT/2), FOCAL_LENGTH,1);
      bool intersection = ClosestIntersection(camera.position,d,triangles,closestIntersection);
      //printf("intersection is %s",intersection);
      if (intersection==true) {
        //printf("Intersection is true I'm in");
        PutPixelSDL(screen, x, y, triangles[closestIntersection.triangleIndex].color);
      }
      else PutPixelSDL(screen, x, y, black);
    }
  }
}




/*Place updates of parameters here*/
void Update(Camera &camera)
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
  const uint8_t* keystate = SDL_GetKeyboardState(NULL);
  if(keystate[SDL_SCANCODE_UP]){
    camera.position.z += SENSITIVITY;
  }
  if(keystate[SDL_SCANCODE_DOWN]){
    camera.position.z -= SENSITIVITY;
  }
  if(keystate[SDL_SCANCODE_LEFT]){
    camera.position.x -= SENSITIVITY;
  }
  if(keystate[SDL_SCANCODE_RIGHT]){
    camera.position.x += SENSITIVITY;
  }

}

/*
start: start position of ray
dir :  direction of ray
*/

bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection ){
  bool flag = false;
  mat3 A;
  //mat3 M;
  for (size_t i = 0; i < triangles.size(); i++){

    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3(v1-v0);
    vec3 e2 = vec3(v2-v0);
    vec3 b = vec3(start-v0);

    A = mat3( -vec3(dir), e1, e2 );
    //M[0] = b;
    //vec3 x = (1/glm::det(A)*M)*b;
    vec3 x = glm::inverse( A ) * b;
    //x = [t u v]
    float t = x.x;
    float u = x.y;
    float v = x.z;
    //printf("t : %f, u : %f, v : %f",v);

    //Otan valume = fevgun oi mavres grammes
    if(u >= 0 && v >= 0 && (u + v) < 1 && t >= 0){
      if (flag == false){
        flag = true;
        closestIntersection.triangleIndex = i;
        closestIntersection.distance = t;
        closestIntersection.position = vec4(t,u,v,1);
      }
      else {
        if (t < closestIntersection.distance){
          closestIntersection.triangleIndex = i;
          closestIntersection.distance = t;
          closestIntersection.position = vec4(t,u,v,1);
        }
      }
    }
  }
  return flag;
}
