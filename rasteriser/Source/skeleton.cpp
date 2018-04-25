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
#define SCREEN_HEIGHT 320
#define ANTIALIASING 1
#define FOCAL_LENGTH SCREEN_WIDTH/2

#define FULLSCREEN_MODE false
#define SENSITIVITY 0.1f
#define ROTATION_SENSITIVITY 0.05f
#define LIGHT_SENSITIVITY 0.2f
#define PI_F 3.14159265358979f
#define LIGHT_COLOR_INTENSITY 10.f

#define DIFFUSE_COLOR  1.f
#define SPECULAR_COLOR 0.2f
#define AMBIENT_INTENSITY 0.1f
#define SHINY_FACTOR 15.f
#define INDIRECT_LIGHT_INTENSITY 0.3f
#define NEAR 0.1f
#define FAR 3.f
//https://en.wikipedia.org/wiki/Angle_of_view#Measuring_a_camera's_field_of_view
float fov = 2*atanf(SCREEN_HEIGHT/2*FOCAL_LENGTH);
float ratio = SCREEN_HEIGHT/SCREEN_WIDTH;
float Hnear = 2 * tan(fov / 2) * NEAR;
float Wnear = Hnear * ratio;
float Hfar = 2 * tan(fov / 2) * FAR;
float Wfar = Hfar * ratio;


struct Pixel
{
  int x;
  int y;
  float zinv;
  vec4 pos3d;
  float z;
};

struct Camera {
  vec4 position;
  mat4 R; //rotation
  float yaw; //angle for rotating around y-axis
  float pitch;
  float roll;
  std::vector<Plane> cliipingPlanes;
};


struct Vertex
{
  vec4 position;
};

vec3 white(1,1,1);
vector<Triangle> triangles;
float depthBuffer[SCREEN_WIDTH*ANTIALIASING][SCREEN_HEIGHT*ANTIALIASING];
vec3 frameBuffer[SCREEN_WIDTH][SCREEN_HEIGHT];
float shadowMap[SCREEN_WIDTH][SCREEN_HEIGHT];
vec3 AABuffer[SCREEN_WIDTH*ANTIALIASING][SCREEN_HEIGHT*ANTIALIASING];
mat4 transformation_mat;
vec3 indirectLightPowerPerArea = 0.5f*vec3( 1, 1, 1 );
vector<Light> lights;
int current_light_index;
float dof = 3;
int dof_enabled = false;

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
void PixelShader(const Pixel& p, screen* screen,Camera& camera, vec3 currentColor, vec4 currentNormal, Phong phong);
void initialize_vertices( vector<Vertex>& vertices, Triangle triangle);
void DrawPolygon(const vector<Vertex>& v;ertices, vec3 currentColor, vec4 normal,screen* screen, Camera& camera, Phong phong );
void ComputePolygonRows(const vector<Pixel>& vertexPixels,vector<Pixel>& leftPixels,vector<Pixel>& rightPixels );
void DrawPolygonRows( const vector<Pixel>& leftPixels,const vector<Pixel>& rightPixels
                ,vec3 currentColor, vec4 normal, screen* screen, Camera& camera, Phong phong);

/******************************************************************************************/

void make_frustum(Camera &camera)
{


  vec3 right(transformation_mat[0][0], transformation_mat[0][1], transformation_mat[0][2] );
  vec3 down(transformation_mat[1][0], transformation_mat[1][1], transformation_mat[1][2] );
  vec3 forward(transformation_mat[2][0], transformation_mat[2][1], transformation_mat[2][2]);
  vec3 up = -down;

  vec3 far_center = camera.pos + forward*FAR;

  vec3 far_top_left  = far_center + (up * Hfar/2) - (right * Wfar/2);
  vec3 far_top_right = far_center + (up * Hfar/2) + (right * Wfar/2);
  vec3 far_bot_left  = far_center - (up * Hfar/2) - (right * Wfar/2);
  vec3 far_bot_right = far_center - (up * Hfar/2) + (right * Wfar/2);

  vec3 near_center = camera.pos + forward*NEAR;

  vec3 near_top_left  = near_center + (up * Hnear/2) - (right * Wnear/2);
  vec3 near_top_right = near_center + (up * Hnear/2) + (right * Wnear/2);
  vec3 near_bot_left  = near_center - (up * Hnear/2) - (right * Wnear/2);
  vec3 near_bot_right = near_center - (up * Hnear/2) + (right * Wnear/2);


  camera.cliipingPlanes.push_back()


}



vec3 directLight( int n, vec4 normal, Pixel pixel, Camera &camera, Phong phong)
{

  vec4 lightPos = lights[n].pos;
  vec3 power = lights[n].intensity;
  float r = glm::distance(lightPos, pixel.pos3d);//DISTANCE
  vec4 r_hat = glm::normalize(lightPos - pixel.pos3d);//DIRECTION of light
  float dotProduct = glm::dot(normal,r_hat);

  float diffuse_term = std::max(dotProduct,(0.0f));
  vec3 attenuation = power / (4*PI_F*r*r); //colour light / distance

  vec4 viewDir = normalize(-camera.position);

  vec4 H = normalize(r_hat + viewDir );
  float spec_intensity = std::max(glm::dot(normal,H),(0.0f));

  vec3 diffuse = attenuation * diffuse_term * phong.kd ;
  vec3 specular = attenuation * pow(spec_intensity,SHINY_FACTOR)*phong.ks;
  vec3 ambient  =  lights[n].diffuse_color * phong.ka;
  vec3 D = diffuse*lights[n].diffuse_color + specular*lights[n].specular_color + ambient;
  D = clamp(D,vec3(0,0,0),vec3(1,1,1));
    return D;
}


void ComputePolygonRows(const vector<Pixel>& vertexPixels,vector<Pixel>& leftPixels,vector<Pixel>& rightPixels )
{
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

    for (int new_row_i = 0; new_row_i < new_rows; new_row_i++)
    {
      int row_i = interpolated_line[new_row_i].y - min_y;

      if (interpolated_line[new_row_i].x > rightPixels[row_i ].x )
      {
        rightPixels[row_i] = interpolated_line[new_row_i];
      }
      if (interpolated_line[new_row_i].x < leftPixels[row_i ].x )
      {
        leftPixels[row_i] = interpolated_line[new_row_i];
      }
    }
  }
}

void Bresenham(Pixel a, Pixel b, vector<Pixel>& result)
{

  float x0 = a.x; float x1 = b.x;
  float y0 = a.y; float y1 = b.y;

  //no ned for abs since we give right and left pixels
  float dx = x1-x0; float dy = y1-y0;
  result.resize(dx);

  //decision point
  float p = 2*dy-dx;

  vec4 step_pos3d = (b.pos3d - a.pos3d)/dx;
  float step_zinv = (b.zinv - a.zinv)/dx;
  float zinv = a.zinv; vec4 pos3d = a.pos3d;

  int x = x0; int y = y0;

  for(int i = 0; i < dx; i++)
  {
    if(p < 0) {
      x++;
      p = p + 2 * (dy);
    }
    else
    {
      x++;
      y++;
      p = p + 2 * (dy - dx);
    }
    float fi = float(i);
    result[i].x = x;
    result[i].y = y;
    result[i].zinv = a.zinv + step_zinv*(fi);
    result[i].pos3d = a.pos3d+step_pos3d*(fi);
    }
  }


void DrawPolygonRows( const vector<Pixel>& leftPixels,const vector<Pixel>& rightPixels
                ,vec3 currentColor, vec4 normal, screen* screen, Camera& camera, Phong phong )
                {
  for (int row = 0; row < rightPixels.size(); row++)
  {
    vector<Pixel> pixels;
    Bresenham(leftPixels[row], rightPixels[row], pixels);
    for (int point = 0; point < pixels.size(); point++)
    {
      if (pixels[point].x >= 0 && pixels[point].x < SCREEN_WIDTH*ANTIALIASING && pixels[point].y >=0 && pixels[point].y < SCREEN_HEIGHT*ANTIALIASING)
        PixelShader(pixels[point], screen, camera, currentColor, normal, phong);
    }
  }
}


void DrawPolygon( const vector<Vertex>& vertices, vec3 currentColor, vec4 normal,screen* screen, Camera& camera, Phong phong )
{
  int V = vertices.size();
  vector<Pixel> vertexPixels( V );
  for( int i=0; i<V; ++i )
    VertexShader( vertices[i], vertexPixels[i],camera );
  vector<Pixel> leftPixels;
  vector<Pixel> rightPixels;
  ComputePolygonRows( vertexPixels, leftPixels, rightPixels );
  DrawPolygonRows( leftPixels, rightPixels,currentColor, normal, screen,camera, phong);
}

vec3 calc_blur(int i, int j, int size)
{
  vec3 sum(0,0,0);
  int num = 0;
  for (int x = -(size-1)/2; x < (size-1)/2;x++)
  {
    for (int y = -(size-1)/2; y < (size-1)/2;y++)
    {
      if (i+x >= 0 && j+y >= 0 && i+x < SCREEN_WIDTH && j+y < SCREEN_HEIGHT)
      {
        sum += frameBuffer[i+x][j+y];
        num ++;
      }
    }
  }
  return sum/=num;
}

void post_processing(Camera camera)
{
  vec3 processed[SCREEN_WIDTH][SCREEN_HEIGHT];

  #pragma omp for
  for (int i =0; i< SCREEN_WIDTH;i++)
  {
    for (int j =0; j< SCREEN_HEIGHT;j++)
    {
      // vec3 temp(depthBuffer[i][j]);
      float z = abs(camera.position.z - depthBuffer[i][j]);
      float distance = abs(dof - z);
      if (distance <= 0.55 && distance >= 0.25 )
      {
        //skip dof
        processed[i][j] = frameBuffer[i][j];
      }
      else {

        // // //blur according to distance
        if ((distance > 0.55 && distance <= 1) || distance <= 0.15 )
        {
           processed[i][j] = calc_blur(i,j,3);
         }
         if ((distance > 1 && distance <= 1.5) || (distance > 0.15 && distance <= 0.55) )
         {
           processed[i][j] = calc_blur(i,j,5);
          }
          if (distance > 1.5 )
          {
            processed[i][j] = calc_blur(i,j,15);
           }
        }
    }
  }

  for (int i =0; i< SCREEN_WIDTH;i++)
  {
    for (int j =0; j< SCREEN_HEIGHT;j++)
    {
      frameBuffer[i][j] = processed[i][j];
    }
  }
}


void putPixels(screen* screen)
{
  #pragma omp for
  for (int i =0; i< SCREEN_WIDTH;i++)
  {
    for (int j =0; j< SCREEN_HEIGHT;j++)
    {
      // vec3 temp(depthBuffer[i][j]);
      // std::cout << frameBuffer[i][j].x << frameBuffer[i][j].y << frameBuffer[i][j].z << '\n';
      PutPixelSDL(screen, i, j, frameBuffer[i][j]);
    }
  }
}

void putPixelsAA(screen* screen)
{
  #pragma omp for
  for (int i =0; i< SCREEN_WIDTH*ANTIALIASING;i++)
  {
    for (int j =0; j< SCREEN_HEIGHT*ANTIALIASING;j++)
    {
      // vec3 temp(depthBuffer[i][j]);
      // std::cout << frameBuffer[i][j].x << frameBuffer[i][j].y << frameBuffer[i][j].z << '\n';
      PutPixelSDL(screen, i, j, AABuffer[i][j]);
    }
  }
}


void antialiasing()
{

  for( int x=0; x<SCREEN_WIDTH*ANTIALIASING; x+=ANTIALIASING ){
    int truex =(x/ANTIALIASING);
    for( int y=0; y<SCREEN_WIDTH*ANTIALIASING; y+=ANTIALIASING ){
      int truey = (y/ANTIALIASING);
      vec3 sum(0,0,0);
      for (int sample_x = 0; sample_x < ANTIALIASING; sample_x++ )
      {
        for (int sample_y = 0 ; sample_y < ANTIALIASING; sample_y++ )
        {
          sum += AABuffer[sample_x+x][sample_y+y];
        }
      }
      frameBuffer[truex][truey] = sum/float(ANTIALIASING*ANTIALIASING);
    }
  }
}


void lookAtLight(Camera &camera)
{
  int i;
  camera.position += vec4(0,0,0,0);
  camera.pitch = 1.57079633;
  TransformationMatrix(transformation_mat,camera);
}

int main( int argc, char* argv[] )
{
  initLights();
  current_light_index = 0;
  Camera camera;
  initialize_camera(camera);
  // screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  LoadTestModel(triangles);
  lookAtLight(camera);
  while( NoQuitMessageSDL() ){
      Update(camera);
      SDL_Renderframe(screen);
      Draw(screen,camera);
      antialiasing();
      if (dof_enabled) post_processing(camera);
      putPixels(screen);
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
  {
    for( int x=0; x<SCREEN_WIDTH; ++x )
    {
      frameBuffer[x][y] = vec3(0,0,0);
    }
  }

  for( int y=0; y<SCREEN_HEIGHT*ANTIALIASING; ++y )
  {
    for( int x=0; x<SCREEN_WIDTH*ANTIALIASING; ++x )
    {
      depthBuffer[x][y] = 0.f;
      AABuffer[x][y] = vec3(0,0,0);
    }
  }

  /* Clear buffer */

  for( uint32_t i=0; i<triangles.size(); ++i ){
    vector<Vertex> vertices(3);
    initialize_vertices(vertices,triangles[i]);

    vec3 currentColor = triangles[i].color;
    vec4 currentNormal = triangles[i].normal;
    Phong phong = triangles[i].phong;
    DrawPolygon( vertices,currentColor,currentNormal,screen,camera,phong );
  }
}

void VertexShader( const Vertex& v, Pixel& p, Camera& camera ){
  vec4 p_origin = v.position - camera.position;
  vec4 pixel = p_origin*camera.R;
  p.x = FOCAL_LENGTH*ANTIALIASING*pixel.x/pixel.z + SCREEN_WIDTH*ANTIALIASING/2;
  p.y = FOCAL_LENGTH*ANTIALIASING*pixel.y/pixel.z + SCREEN_HEIGHT*ANTIALIASING/2;
  p.zinv = 1.f/pixel.z;
  p.z = pixel.z;
  p.pos3d = v.position*p.zinv;
}


void PixelShader(const Pixel& p, screen* screen,Camera& camera, vec3 currentColor, vec4 currentNormal,Phong phong)
{
  int x = p.x;
  int y = p.y;

  if( p.zinv > depthBuffer[x][y])
  {
    depthBuffer[x][y] = p.zinv;
    vec3 directlight = vec3(0,0,0);
    for (size_t i = 0; i < lights.size(); i++)
     {
      directlight += directLight(i,currentNormal,p,camera, phong);
    }
    vec3 illumination = currentColor * (directlight +  indirectLightPowerPerArea);
    // vec3 illumination =(directlight );
    AABuffer[x][y] = illumination;
  }
}


void Interpolate( Pixel a, Pixel b, vector<Pixel>& result )
{
  int N = result.size();
  float stepx =  ((b.x-a.x) / float(std::max(N-1,1)) );
  float stepy =  ((b.y-a.y) / float(std::max(N-1,1)) );
  float stepz = (b.zinv-a.zinv) / float(std::max(N-1,1)) ;
  vec3 step = vec3(stepx,stepy,stepz);
  vec4 step_pos3d = (b.pos3d - a.pos3d) / float(std::max(N-1,1)) ;
  vec3 current = vec3(a.x,a.y,a.zinv);
  vec4 current_pos3d = a.pos3d;

  for( int i=0; i<N; ++i )
  {
    result[i].x = current.x;
    result[i].y = current.y;
    result[i].zinv = current.z;
    result[i].pos3d = current_pos3d;
    current += step;
    current_pos3d += step_pos3d;
  }
}

//a - start b - end
void DrawLineSDL( screen* screen, Pixel a, Pixel b, vec3 color )
{
  int deltax = (glm::abs( a.x - b.x ));
  int deltay = (glm::abs( a.y - b.y ));

  uint32_t pixels = glm::max( deltax, deltay ) + 1;
  vector<Pixel> line( pixels ); //get the pixel positions of the line
  Interpolate( a, b, line );
  for (uint32_t i = 0; i < pixels; i++)
  {
    if (line[i].x >= 0 && line[i].x < SCREEN_WIDTH && line[i].y >=0 && line[i].y < SCREEN_HEIGHT)
    {
      PutPixelSDL(screen,line[i].x,line[i].y,color);
    }
  }
}

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
  vec4 right(transformation_mat[0][0], transformation_mat[0][1], transformation_mat[0][2], 1 );
  vec4 down(transformation_mat[1][0], transformation_mat[1][1], transformation_mat[1][2], 1 );
  vec4 forward(transformation_mat[2][0], transformation_mat[2][1], transformation_mat[2][2], 1 );

  const uint8_t* keystate = SDL_GetKeyboardState(NULL);
  if(keystate[SDL_SCANCODE_UP])
  {
    camera.position.z += SENSITIVITY;
    TransformationMatrix(transformation_mat,camera);
  }
  else if(keystate[SDL_SCANCODE_DOWN])
  {
    camera.position.z -= SENSITIVITY;
    TransformationMatrix(transformation_mat,camera);
  }

  else if(keystate[SDL_SCANCODE_RIGHT])
  {
    camera.yaw  -= ROTATION_SENSITIVITY;
    TransformationMatrix(transformation_mat, camera);
  }
  else if(keystate[SDL_SCANCODE_LEFT])
  {
    camera.pitch  += ROTATION_SENSITIVITY;
    TransformationMatrix(transformation_mat, camera);
  }

  else if(keystate[SDL_SCANCODE_W])
  {
    lights[current_light_index].pos += forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_S])
  {
    lights[current_light_index].pos -= forward*LIGHT_SENSITIVITY;
  }
  else if (keystate[SDL_SCANCODE_D])
  {
    lights[current_light_index].pos += right*LIGHT_SENSITIVITY;

  }
  else if (keystate[SDL_SCANCODE_A])
  {
    lights[current_light_index].pos -= right*LIGHT_SENSITIVITY;
  }
  else if (keystate[SDL_SCANCODE_C])
  {
    current_light_index = (current_light_index + 1 ) % lights.size();
  }
  else if (keystate[SDL_SCANCODE_O])
  {
    initLights();
  }

}



//yaw - y
mat4 yaw_rotation(Camera& camera)
{
  vec4 v1(cos(camera.yaw), 0, sin(camera.yaw),0);
  vec4 v2(0,1,0,0);
  vec4 v3(-sin(camera.yaw), 0, cos(camera.yaw),0);
  vec4 v4(0,0,0,1);
  return mat4(v1,v2,v3,v4);
}

//roll - z
mat4 roll_rotation(Camera& camera)
{
  vec4 v1(cos(camera.roll), -sin(camera.roll), 0 ,0);
  vec4 v2(sin(camera.roll),cos(camera.roll),0,0);
  vec4 v3(0, 0, 1,0);
  vec4 v4(0,0,0,1);
  return mat4(v1,v2,v3,v4);
}

//pitch - x
mat4 pitch_rotation(Camera& camera)
{
  vec4 v1(1, 0, 0,0);
  vec4 v2(0,cos(camera.pitch),-sin(camera.pitch),0);
  vec4 v3(0, sin(camera.pitch), cos(camera.pitch),0);
  vec4 v4(0,0,0,1);
  return mat4(v1,v2,v3,v4);
}

void TransformationMatrix(glm::mat4 &M, Camera& camera)
{
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



void initialize_camera(Camera &camera)
{
  camera.position = vec4( 0, 0, -2.1,1 );
  camera.yaw = 0;
  camera.pitch = 0;
  camera.roll = 0;

  TransformationMatrix(transformation_mat ,camera );
}

void initLights()
{
  vec4 lightPos( 0, -0.5, -0.7, 1.0 );
  Light light(lightPos,
  vec3(DIFFUSE_COLOR,DIFFUSE_COLOR,DIFFUSE_COLOR),
  vec3(SPECULAR_COLOR,SPECULAR_COLOR,SPECULAR_COLOR),
  vec3(LIGHT_COLOR_INTENSITY,LIGHT_COLOR_INTENSITY,LIGHT_COLOR_INTENSITY),
  vec3(AMBIENT_INTENSITY,AMBIENT_INTENSITY,AMBIENT_INTENSITY), Pointlight );
lights.push_back(light);
}

void initialize_vertices( vector<Vertex>& vertices, Triangle triangle)
{
  vertices[0].position = triangle.v0;
  vertices[1].position = triangle.v1;
  vertices[2].position = triangle.v2;
}
