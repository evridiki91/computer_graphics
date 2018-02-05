#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <math.h>
#include <limits>


using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Camera {
  vec4 position;
  mat4 R; //rotation
  float yaw; //angle for rotating around y-axis
};

struct Intersection {
  vec4 position;
  float distance;
  int triangleIndex;
};

vec4 lightPos( 0, -0.5, -0.7, 1.0 );
vec3 lightColor = 14.f * vec3( 1, 1, 1 );
vector<Triangle> triangles;

#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 320
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH SCREEN_HEIGHT/2
#define SENSITIVITY 0.1f
#define ROTATION_SENSITIVITY 1f
#define LIGHT_SENSITIVITY 0.2f
#define PI_F 3.14159265358979f




/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */

void Update(Camera &camera);
void Draw(screen* screen,Camera &camera, std::vector<Triangle> &triangles,Intersection &closestIntersection);
bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection );
vec3 DirectLight( const Intersection& i );
void initialize_camera(Camera &camera);

int main( int argc, char* argv[] )
{
  // R = [cost 0 sint, 0 1 0, -sint 0 cost ]
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  Camera camera;
  initialize_camera(camera);
  Intersection closestIntersection;
  //Initialize triangles
  LoadTestModel(triangles);

  while( NoQuitMessageSDL() ){
      Update(camera);
      Draw(screen,camera,triangles,closestIntersection);
      SDL_Renderframe(screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

void initialize_camera(Camera &camera){
  camera.position = vec4(0,0,-2,1);
  camera.yaw = 0;
}

vec3 DirectLight( const Intersection& i){
  Triangle triangle = triangles[i.triangleIndex];
  float r = glm::distance(lightPos, i.position);
  vec4 n_hat = (triangle.normal);
  vec4 r_hat = glm::normalize(lightPos - i.position);
  float dotProduct = glm::dot(r_hat,n_hat);

  vec3 D = (triangle.color*lightColor * max(dotProduct,(0.0f)))/(4*PI_F*r*r);
  return D;
}


/*Place your drawing here*/
void Draw(screen* screen,Camera &camera, std::vector<Triangle> &triangles,Intersection &closestIntersection){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vec3 black(0.0,0.0,0.0);
  for(int x = 0; x < SCREEN_WIDTH; x++){
    for(int y = 0; y < SCREEN_HEIGHT; y++){
    vec4 d(x - (SCREEN_WIDTH/2), y - (SCREEN_HEIGHT/2), FOCAL_LENGTH,1);
    d = camera.R * d;
      bool intersection = ClosestIntersection(camera.position,d,triangles,closestIntersection);
      //printf("intersection is %s",intersection);
      if (intersection==true) {
        //printf("Intersection is true I'm in");
        vec3 light = DirectLight(closestIntersection);
        PutPixelSDL(screen, x, y, light);
        // PutPixelSDL(screen, x, y, triangles[closestIntersection.triangleIndex].color);
      }
      else PutPixelSDL(screen, x, y, black);
    }
  }
}

void rotation_aroundY(Camera& camera, int dir){
  camera.yaw += dir*SENSITIVITY;
  vec4 v1(cos(camera.yaw), 0, sin(camera.yaw),0);
  vec4 v2(0,1,0,0);
  vec4 v3(-sin(camera.yaw), 0, cos(camera.yaw),0);
  vec4 v4(0,0,0,1);
  camera.R = mat4(v1,v2,v3,v4);
}


/*Place updates of parameters here*/
void Update(Camera &camera)
{
  vec4 right(camera.R[0][0], camera.R[0][1], camera.R[0][2], 1 );
  vec4 down(camera.R[1][0], camera.R[1][1], camera.R[1][2], 1 );
  vec4 forward(camera.R[2][0], camera.R[2][1], camera.R[2][2], 1 );
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
  else if(keystate[SDL_SCANCODE_DOWN]){
    camera.position.z -= SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_LEFT]){
    // camera.position.x -= SENSITIVITY; //Camera moves to the left
    rotation_aroundY(camera,1);
  }
  else if(keystate[SDL_SCANCODE_RIGHT]){
    // camera.position.x += SENSITIVITY; //Camera moves to the right
    rotation_aroundY(camera,-1);
  }
  else if(keystate[SDL_SCANCODE_W]){
    lightPos += forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_S]){
      lightPos -= forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_A]){
      lightPos -= right*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_D]){
      lightPos += right*LIGHT_SENSITIVITY;
  }


}


bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection ){
  bool flag = false;
  mat3 A;
  // mat3 M;
  for (size_t i = 0; i < triangles.size(); i++){

    vec4 v0 = triangles[i].v0;
    vec4 v1 = triangles[i].v1;
    vec4 v2 = triangles[i].v2;

    vec3 e1 = vec3(v1-v0);
    vec3 e2 = vec3(v2-v0);
    vec3 b = vec3(start-v0);

    A = mat3( -vec3(dir), e1, e2 );
    // M[0] = b;
    // vec3 x = ((1/glm::determinant(A))*M)*b;
    vec3 x = glm::inverse( A ) * b;
    //x = [t u v]
    float t = x.x;
    float u = x.y;
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
