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


#define SCREEN_WIDTH 320*2
#define FOCAL_LENGTH SCREEN_WIDTH/2
#define SCREEN_HEIGHT 320*2
#define FULLSCREEN_MODE false
#define SENSITIVITY 0.1f
#define ROTATION_SENSITIVITY 0.05f
#define LIGHT_SENSITIVITY 0.2f
#define PI_F 3.14159265358979f
#define LIGHT_COLOR_INTENSITY 11.f

struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
};

struct Camera {
  vec4 position;
  mat4 R; //rotation
  float yaw; //angle for rotating around y-axis
  float pitch;
  float roll;
};


struct Vertex
{
  vec4 position;
};

struct Light
{
  glm::vec4 pos;
	vec3 color;
	//lightType_t lightType;
};


vec3 white(1,1,1);
vector<Triangle> triangles;
float depthBuffer[SCREEN_WIDTH][SCREEN_HEIGHT];
mat4 transformation_mat;
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vector<Light> lights;
int current_light_index;

/* -------------------------------------------------
 FUNCTIONS
 ---------------------------------------------------*/
void initLights();
void Update(Camera& camera);
void Draw(screen* screen, Camera& camera);
void TransformationMatrix(glm::mat4 &M, Camera& camera);
void initialize_camera(Camera &camera);
void DrawTriangles(vector<Triangle> triangles, screen* screen, Camera& camera);
void Draw(screen* screen, Camera& camera);
void VertexShader( const Vertex& vertices, Pixel& p, Camera& camera );
void Interpolate( Pixel a, Pixel b, vector<Pixel>& result );
void DrawLineSDL( screen* screen, Pixel a, Pixel b, vec3 color );
void DrawPolygonEdges( vector<Vertex>& vertices,screen* screen, Camera &camera );
void PixelShader(const Pixel& p, screen* screen,Camera& camera, vec3 currentColor, vec4 currentNormal);
void initialize_vertices( vector<Vertex>& vertices, Triangle triangle);
void DrawPolygon(const vector<Vertex>& vertices, vec3 currentColor, vec4 normal,screen* screen, Camera& camera );
void ComputePolygonRows(const vector<Pixel>& vertexPixels,vector<Pixel>& leftPixels,vector<Pixel>& rightPixels );
void DrawPolygonRows( const vector<Pixel>& leftPixels,const vector<Pixel>& rightPixels
                ,vec3 currentColor, vec4 normal, screen* screen, Camera& camera);

/******************************************************************************************/

vec3 directLight( int n, vec4 normal, Pixel pixel, Camera &camera){

  vec4 lightPos = lights[n].pos;
  vec3 power = lights[n].color;
  float r = glm::distance(lightPos, pixel.pos3d);//DISTANCE
  vec4 r_hat = glm::normalize(lightPos - pixel.pos3d);//DIRECTION of light
  float dotProduct = glm::dot(normal,r_hat);
  float diff_intensity = std::max(dotProduct,(0.0f));
  vec3 attenuation = power / (4*PI_F*r*r); //colour light / distance
  vec3 total =  diff_intensity*attenuation;
  return total;
}


void ComputePolygonRows(const vector<Pixel>& vertexPixels,vector<Pixel>& leftPixels,vector<Pixel>& rightPixels ){
  int min_y = numeric_limits<int>::max();
  int max_y = numeric_limits<int>::min();

  // 1. Find max and min y-value of the polygon
  for (int i = 0; i < vertexPixels.size(); i++ ){
    if (vertexPixels[i].y > max_y) max_y = vertexPixels[i].y;
    if (vertexPixels[i].y < min_y) min_y = vertexPixels[i].y;
  }

  //computing number of rows
  int rows = max_y-min_y+1;

// 2. Resize leftPixels and rightPixels
  leftPixels.resize(rows); rightPixels.resize(rows);

// 3. Initialize the x-coordinates in leftPixels to something large value
// and the x-coordinates in rightPixels to some really small value
  for( int i=0; i<rows; ++i )
  {
    leftPixels[i].x  = numeric_limits<int>::max();
    rightPixels[i].x = numeric_limits<int>::min();
  }

  // 4. Loop through all edges of the polygon and use
  for( int i=0; i<vertexPixels.size(); ++i )
  {
    //finding the connecting edge
    int j = (i+1)%(vertexPixels.size());
    //finding new rows
    int new_rows = abs(vertexPixels[i].y - vertexPixels[j].y) + 1;
    vector<Pixel> interpolated_line(new_rows);
    Interpolate(vertexPixels[i],vertexPixels[j],interpolated_line);

    for (int new_row_i = 0; new_row_i < new_rows; new_row_i++){
      int row_i = interpolated_line[new_row_i].y - min_y;

      if (interpolated_line[new_row_i].x > rightPixels[row_i ].x ){
        rightPixels[row_i] = interpolated_line[new_row_i];
      }
      if (interpolated_line[new_row_i].x < leftPixels[row_i ].x ){
        leftPixels[row_i] = interpolated_line[new_row_i];
      }
    }
  }
}

void DrawPolygonRows( const vector<Pixel>& leftPixels,const vector<Pixel>& rightPixels
                ,vec3 currentColor, vec4 normal, screen* screen, Camera& camera){
  for (int row = 0; row < rightPixels.size(); row++){
    vector<Pixel> pixels(abs(rightPixels[row].x - leftPixels[row].x + 1 ));
    Interpolate(leftPixels[row], rightPixels[row], pixels);
    for (int point = 0; point < pixels.size(); point++){
      PixelShader(pixels[point], screen, camera, currentColor, normal);
    }
  }
}

void DrawPolygon( const vector<Vertex>& vertices, vec3 currentColor, vec4 normal,screen* screen, Camera& camera )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i )
    VertexShader( vertices[i], vertexPixels[i],camera );
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawPolygonRows( leftPixels, rightPixels,currentColor, normal, screen,camera);
}



int main( int argc, char* argv[] )
{
  initLights();
  current_light_index = 0;
  Camera camera;
  initialize_camera(camera);
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  LoadTestModel(triangles);
  while( NoQuitMessageSDL() ){
      Update(camera);
      SDL_Renderframe(screen);
      Draw(screen,camera);
  }

  SDL_SaveImage( screen, "screenshot.bmp" );
  KillSDL(screen);
  return 0;
}


// void DrawTriangles(vector<Triangle> triangles, screen* screen, Camera& camera){
//   memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
//   vector<Vertex> vertices;
//   for (uint32_t i = 0; i < triangles.size(); i ++){
//     vertices.push_back(triangles[i].v0);
//     vertices.push_back(triangles[i].v1);
//     vertices.push_back(triangles[i].v2);
//   }
//   DrawPolygonEdges(vertices, screen,camera);
// }

/*Place your drawing here*/
void Draw(screen* screen, Camera& camera )
{
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  //clear depth
  for( int y=0; y<SCREEN_HEIGHT; ++y )
    for( int x=0; x<SCREEN_WIDTH; ++x )
      depthBuffer[x][y] = 0;

  /* Clear buffer */
  for( uint32_t i=0; i<triangles.size(); ++i ){
    vector<Vertex> vertices(3);
    initialize_vertices(vertices,triangles[i]);
    vec3 currentColor = triangles[i].color;
    vec4 currentNormal = triangles[i].normal;
    DrawPolygon( vertices,currentColor,currentNormal,screen,camera );
  }
}

void VertexShader( const Vertex& v, Pixel& p, Camera& camera ){
  vec4 p_origin = v.position - camera.position;
  vec4 pixel = p_origin*camera.R;
  p.x = FOCAL_LENGTH*pixel.x/pixel.z + SCREEN_WIDTH/2;
  p.y = FOCAL_LENGTH*pixel.y/pixel.z + SCREEN_HEIGHT/2;
  p.zinv = 1.f/pixel.z;
  p.pos3d = v.position*p.zinv;
}


void PixelShader(const Pixel& p, screen* screen,Camera& camera, vec3 currentColor, vec4 currentNormal)
{
  int x = p.x;
  int y = p.y;

  if( p.zinv > depthBuffer[x][y])
  {
    depthBuffer[x][y] = p.zinv;
    vec3 directlight = directLight(current_light_index,currentNormal,p,camera);
    vec3 illumination = currentColor * (directlight +  indirectLightPowerPerArea);
    PutPixelSDL( screen, x, y, illumination);
  }
}


void Interpolate( Pixel a, Pixel b, vector<Pixel>& result ){
  int N = result.size();
  float stepx =  ((b.x-a.x) / float(std::max(N-1,1)) );
  float stepy =  ((b.y-a.y) / float(std::max(N-1,1)) );
  float stepz = (b.zinv-a.zinv) / float(std::max(N-1,1)) ;
  vec3 step = vec3(stepx,stepy,stepz);
  vec4 step_pos3d = (b.pos3d - a.pos3d) / float(std::max(N-1,1)) ;
  vec3 current = vec3(a.x,a.y,a.zinv);
  vec4 current_pos3d = a.pos3d;

  for( int i=0; i<N; ++i ) {
    result[i].x = current.x;
    result[i].y = current.y;
    result[i].zinv = current.z;
    result[i].pos3d = current_pos3d;
    current += step;
    current_pos3d += step_pos3d;
  }
}

//a - start b - end
void DrawLineSDL( screen* screen, Pixel a, Pixel b, vec3 color ){
  int deltax = (glm::abs( a.x - b.x ));
  int deltay = (glm::abs( a.y - b.y ));

  uint32_t pixels = glm::max( deltax, deltay ) + 1;
  vector<Pixel> line( pixels ); //get the pixel positions of the line
  Interpolate( a, b, line );
  for (uint32_t i = 0; i < pixels; i++)
    PutPixelSDL(screen,line[i].x,line[i].y,color);
}

// void DrawPolygonEdges( vector<vec4>& vertices,screen* screen, Camera &camera ) {
//   int V = vertices.size();
//   // Transform each vertex from 3D world position to 2D image position:
//   vector<Pixel> projectedVertices( V );
//   for( int i=0; i<V; ++i ) {
//     VertexShader( vertices[i], projectedVertices[i],camera );
//   }
//   // Loop over all vertices and draw the edge from it to the next vertex:
//   for( int i=0; i<V; ++i ){
//     int j = (i+1)%V; // The next vertex
//     vec3 color( 1, 1, 1 );
//     DrawLineSDL( screen, projectedVertices[i], projectedVertices[j], color );
//   }
// }

/*Place updates of parameters here*/
void Update(Camera& camera)
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

  const uint8_t* keystate = SDL_GetKeyboardState(NULL);
  if(keystate[SDL_SCANCODE_UP]){
    camera.position.z += SENSITIVITY;
    TransformationMatrix(transformation_mat,camera);
  }
  else if(keystate[SDL_SCANCODE_DOWN]){
    camera.position.z -= SENSITIVITY;
    TransformationMatrix(transformation_mat,camera);
  }
  else if(keystate[SDL_SCANCODE_LEFT]){
    camera.position.x -= SENSITIVITY;
    TransformationMatrix(transformation_mat, camera);
  }
  else if(keystate[SDL_SCANCODE_RIGHT]){
    camera.position.x += SENSITIVITY;
    TransformationMatrix(transformation_mat, camera);
  }
  else if(keystate[SDL_SCANCODE_D]){
    camera.yaw  -= ROTATION_SENSITIVITY;
    TransformationMatrix(transformation_mat, camera);
  }
  else if(keystate[SDL_SCANCODE_A]){
    camera.yaw  += ROTATION_SENSITIVITY;
    TransformationMatrix(transformation_mat, camera);
  }
  // else if (keystate[SDL_SCANCODE_I]){
  //
  // }
}



//yaw - y
mat4 yaw_rotation(Camera& camera){
  vec4 v1(cos(camera.yaw), 0, sin(camera.yaw),0);
  vec4 v2(0,1,0,0);
  vec4 v3(-sin(camera.yaw), 0, cos(camera.yaw),0);
  vec4 v4(0,0,0,1);
  return mat4(v1,v2,v3,v4);
}

//roll - z
mat4 roll_rotation(Camera& camera){
  vec4 v1(cos(camera.roll), -sin(camera.roll), 0 ,0);
  vec4 v2(sin(camera.roll),cos(camera.roll),0,0);
  vec4 v3(0, 0, 1,0);
  vec4 v4(0,0,0,1);
  return mat4(v1,v2,v3,v4);
}

//pitch - x
mat4 pitch_rotation(Camera& camera){
  vec4 v1(1, 0, 0,0);
  vec4 v2(0,cos(camera.pitch),-sin(camera.pitch),0);
  vec4 v3(0, sin(camera.pitch), cos(camera.pitch),0);
  vec4 v4(0,0,0,1);
  return mat4(v1,v2,v3,v4);
}

void TransformationMatrix(glm::mat4 &M, Camera& camera){
  mat4 c = mat4(1.0);
  mat4 c_minus = mat4(1.0);
  for(int i = 0; i < 4 ; i++) {
    c[i][3] = camera.position[i];
    c_minus[i][3] = camera.position[i];
  }

  mat4 yaw_mat = yaw_rotation(camera);
  mat4 roll_mat = roll_rotation(camera);
  mat4 pitch_mat = pitch_rotation(camera);

  camera.R = yaw_mat * roll_mat * pitch_mat;
  M = c * (camera.R * c_minus);
}


void initialize_camera(Camera &camera){
  camera.position = vec4( 0, 0, -2.1,1 );
  camera.yaw = 0;
  camera.pitch = 0;
  camera.roll = 0;

  TransformationMatrix(transformation_mat ,camera );
}

void initLights(){
  vec4 lightPos( 0, -0.5, -0.7, 1.0 );
  vec3 lightPower = LIGHT_COLOR_INTENSITY*vec3( 1, 1, 1 );
  Light light;
  light.pos = lightPos;
  light.color = lightPower;
  lights.push_back(light);
}

void initialize_vertices( vector<Vertex>& vertices, Triangle triangle){
  vertices[0].position = triangle.v0;
  vertices[1].position = triangle.v1;
  vertices[2].position = triangle.v2;
}
