#include <iostream>
#include <glm/glm.hpp>
#include <SDL.h>
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>

using namespace std;
using namespace glm;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;


#define SCREEN_WIDTH 320
#define FOCAL_LENGTH SCREEN_WIDTH/2
#define SCREEN_HEIGHT 256
#define FULLSCREEN_MODE false

vec3 white(1,1,1);
vector<Triangle> triangles;

struct Camera {
  vec4 position;
  mat4 R; //rotation
  float yaw; //angle for rotating around y-axis
  float pitch;
  float roll;
};

/* -------------------------------------------------
---------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void TransformationMatrix(glm::mat4x4 M, vec4 translation, vec4 rotation);
void VertexShader( const vec4& v, ivec2& p, camera &camera );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices,screen* screen );
void DrawTriangles(vector<Triangle> triangles, screen* screen);
void TransformationMatrix(glm:mat4x4 M, vec3 translation, vec3 rotation);

void TransformationMatrix(glm:mat4 &M, vec4 translation, mat4 yaw, mat4 roll, mat4 pitch){
  M = yaw*roll*pitch;
  M[3] = translation;
}

//yaw
void yaw_rotation(Camera& camera, mat4 &R){
  vec4 v1(cos(camera.yaw), 0, sin(camera.yaw),0);
  vec4 v2(0,1,0,0);
  vec4 v3(-sin(camera.yaw), 0, cos(camera.yaw),0);
  vec4 v4(0,0,0,1);
  R = mat4(v1,v2,v3,v4);
}

//roll
void roll_rotation(Camera& camera, mat4 &R){
  vec4 v1(cos(camera.roll), -sin(camera.roll), 0 ,0);
  vec4 v2(sin(camera.roll),cos(camera.roll),0,0);
  vec4 v3(0, 0, 1,0);
  vec4 v4(0,0,0,1);
  R = mat4(v1,v2,v3,v4);
}

//pitch
void pitch_rotation(Camera& camera, mat4 &R){
  vec4 v1(1, 0, 0,0);
  vec4 v2(0,cos(camera.pitch),-sin(camera.pitch),0);
  vec4 v3(0, sin(camera.pitch), cos(camera.pitch),0);
  vec4 v4(0,0,0,1);
  R = mat4(v1,v2,v3,v4);
}

void initialize_camera(Camera &camera){
  camera.position = vec4( 0, 0, -3.001,1 );
  camera.yaw = 0;
  camera.pitch = 0;
  camera.roll = 0;
  vec4 translation = vec4(0,0,0,0);
  TransformationMatrix(camera.R, translation, rotation,camera.yaw,camera.pitch,camera.roll );
}

int main( int argc, char* argv[] )
{
  Camera camera;
  initialize_camera(camera);
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  LoadTestModel(triangles);
  int i = 1;
  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
      DrawTriangles(triangles, screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}
void DrawTriangles(vector<Triangle> triangles, screen* screen){
  vector<vec4> vertices;
  for (uint32_t i = 0; i < triangles.size(); i ++){
    vertices.push_back(triangles[i].v0);
    vertices.push_back(triangles[i].v1);
    vertices.push_back(triangles[i].v2);
  }
  DrawPolygonEdges(vertices, screen);
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  vec3 colour(1.0,0.0,0.0);
  for( uint32_t i=0; i<triangles.size(); ++i ){
    vector<vec4> vertices(3);
    vertices[0] = triangles[i].v0;
    vertices[1] = triangles[i].v1;
    vertices[2] = triangles[i].v2;
    for(int v=0; v<3; ++v){
      ivec2 projPos;
      VertexShader( vertices[v], projPos );
      vec3 color(1,1,1);
      PutPixelSDL( screen, projPos.x, projPos.y, color );
    }
  }
}

void VertexShader( const vec4& v, ivec2& p, Camera &camera, glm:mat4x4 M ){
  vec4 p_origin = v - camera.position;
  p.x = FOCAL_LENGTH*p_origin.x/p_origin.z + SCREEN_WIDTH/2;
  p.y = FOCAL_LENGTH*p_origin.y/p_origin.z + SCREEN_HEIGHT/2;
}

void Interpolate( ivec2 a, ivec2 b, vector<ivec2>& result ){
  int N = result.size();
  vec2 step = vec2(b-a) / float(std::max(N-1,1));
  vec2 current( a );
  for( int i=0; i<N; ++i ) {
    result[i] = current;
    current += step;
  }
}

//a - start b - end
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color ){
  ivec2 delta = glm::abs( a - b );
  uint32_t pixels = glm::max( delta.x, delta.y ) + 1;
  vector<ivec2> line( pixels ); //get the pixel positions of the line
  Interpolate( a, b, line );
  for (uint32_t i = 0; i < pixels; i++)
    PutPixelSDL(screen,line[i].x,line[i].y,color);
}



void DrawPolygonEdges( const vector<vec4>& vertices,screen* screen, Camera &camera ) {
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i ) {
    VertexShader( vertices[i], projectedVertices[i],camera );
  }
  // Loop over all vertices and draw the edge from it to the next vertex:
  for( int i=0; i<V; ++i ){
    int j = (i+1)%V; // The next vertex
    vec3 color( 1, 1, 1 );
    DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
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
  int dx;
  int dy;
  SDL_GetRelativeMouseState( &dx, &dy );

  // const uint8_t* keystate = SDL_GetKeyboardState(NULL);
  // if(keystate[SDL_SCANCODE_UP]){
  //   camera.position.z += SENSITIVITY;
  // }
  // else if(keystate[SDL_SCANCODE_DOWN]){
  //   camera.position.z -= SENSITIVITY;
  // }
  // else if(keystate[SDL_SCANCODE_LEFT]){
  //   rotation_aroundY(camera,1);
  // }
  // else if(keystate[SDL_SCANCODE_RIGHT]){
  //   rotation_aroundY(camera,-1);
  // }
  // else if(keystate[SDL_SCANCODE_W]){
  //   lights[light_selection].pos += forward*LIGHT_SENSITIVITY;
  // }
  // else if(keystate[SDL_SCANCODE_S]){
  //   lights[light_selection].pos -= forward*LIGHT_SENSITIVITY;
  // }
  // else if(keystate[SDL_SCANCODE_A]){
  //     lights[light_selection].pos -= right*LIGHT_SENSITIVITY;
  // }
  // else if(keystate[SDL_SCANCODE_D]){
  //     lights[light_selection].pos += right*LIGHT_SENSITIVITY;
  // }
  // else if (keystate[SDL_SCANCODE_I]){
  //   rotation_aroundX(camera,-1);
  // }
  // else if (keystate[SDL_SCANCODE_O]){
  //   initLights();
  //   light_selection ++;
  // }

}