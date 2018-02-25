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
/* -------------------------------------------------
---------------------------*/
/* FUNCTIONS                                                                   */

void Update();
void Draw(screen* screen);
void TransformationMatrix(glm::mat4x4 M, vec4 translation, vec4 rotation);
vec4 cameraPos( 0, 0, -3.001,1 );
void VertexShader( const vec4& v, ivec2& p );
void DrawLineSDL( screen* screen, ivec2 a, ivec2 b, vec3 color );
void DrawPolygonEdges( const vector<vec4>& vertices,screen* screen );
// void TransformationMatrix(glm:mat4x4 M, vec4 translation, vec4 rotation){
//   return
// }


int main( int argc, char* argv[] )
{

  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  LoadTestModel(triangles);
  while( NoQuitMessageSDL() )
    {
      Update();
      Draw(screen);
      SDL_Renderframe(screen);
      DrawPolygonEdges(triangles, screen);
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/*Place your drawing here*/
void Draw(screen* screen)
{
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  vec3 colour(1.0,0.0,0.0);

  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
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

void VertexShader( const vec4& v, ivec2& p ){
  vec4 p_origin = v - cameraPos;
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
  PutPixelSDL(screen, a.x, a.y, white);
  PutPixelSDL(screen, b.x, b.y, white);

  ivec2 delta = glm::abs( a - b );
  int pixels = glm::max( delta.x, delta.y ) + 1;
  vector<ivec2> line( pixels ); //get the pixel positions of the line
  Interpolate( a, b, line );
  for (uint32_t i = 0; i < pixels; i++)
    PutPixelSDL(screen,line[i].x,line[i].y,color);
}

void DrawPolygonEdges( const vector<vec4>& vertices,screen* screen ) {
  int V = vertices.size();
  // Transform each vertex from 3D world position to 2D image position:
  vector<ivec2> projectedVertices( V );
  for( int i=0; i<V; ++i ) {
    VertexShader( vertices[i], projectedVertices[i] );
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
}
