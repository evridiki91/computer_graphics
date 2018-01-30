#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Intersection {
  vec4 position;
  float distance;
  int triangleIndex;
};

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH 1

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen, std::vector<Triangle> &triangles,Intersection &closestIntersection);
bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection );


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  std::vector<Triangle> triangles;
  Intersection closestIntersection;
  //Initialize triangles
  LoadTestModel(triangles);

  while( NoQuitMessageSDL() ){
      Update();
      Draw(screen,triangles,closestIntersection);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen, std::vector<Triangle> &triangles,Intersection &closestIntersection){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  vec4 cameraPos( 1.0, 1.0, 1.0, 1.0);
  vec3 black(0.0,0.0,0.0);
  for(int x = 0; x < SCREEN_WIDTH; x++){
    for(int y = 0; y < SCREEN_HEIGHT; y++){
      vec4 d(x - SCREEN_WIDTH/2, y - SCREEN_HEIGHT/2, FOCAL_LENGTH,1);
      bool intersection = ClosestIntersection(cameraPos,d,triangles,closestIntersection);
      if (intersection) PutPixelSDL(screen, x, y, triangles[closestIntersection.triangleIndex].color);
      else PutPixelSDL(screen, x, y, black);
    }
  }
}

/*Place updates of parameters here*/
void Update()
{
  static int t = SDL_GetTicks();
  /* Compute frame time */
  int t2 = SDL_GetTicks();
  float dt = float(t2-t);
  t = t2;
  /*Good idea to remove this*/
  std::cout << "Render time: " << dt << " ms." << std::endl;
  /* Update variables*/
}

/*
start: start position of ray
dir :  direction of ray
*/

bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection ){
  bool flag = false;
  for (size_t i = 0; i < triangles.size(); i++){

    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
    vec3 e2 = vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
    vec3 b = vec3(start.x-v0.x,start.y-v0.y,start.z-v0.z);

    mat3 A( -vec3(dir), e1, e2 );
    vec3 x = glm::inverse( A ) * b;
    //x = [t u v]
    float t = x.x;
    float u = x.y;
    float v = x.z;

    if(u > 0 && v > 0 && (u + v) < 1 && t >= 0){
      if (flag == false){
        flag = true;
        closestIntersection.triangleIndex = i;
        closestIntersection.distance = t;
        closestIntersection.position = start + t*dir;
      }
      else {
        if (t < closestIntersection.distance){
          closestIntersection.triangleIndex = i;
          closestIntersection.distance = t;
          closestIntersection.position = start + t*dir;
        }
      }
    }
  }
  return flag;
}
