#include <glm/glm.hpp>
#include <SDL.h>
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <math.h>
#include <limits>
#include <omp.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Camera {
  vec4 position;
  mat4 R; //rotation
  float yaw; //angle for rotating around y-axis
  float pitch;
  float roll;
};

struct Intersection {
  vec4 position;
  float distance;
  int triangleIndex;
};

vector<Triangle> triangles;
vec3 white(  0.75f, 0.75f, 0.75f );
vector<Light> lights;
size_t light_selection;

#define SCREEN_WIDTH 250
#define SCREEN_HEIGHT 250
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH SCREEN_HEIGHT/2
#define SENSITIVITY 0.1f
#define ROTATION_SENSITIVITY 1f
#define LIGHT_SENSITIVITY 0.2f
#define PI_F 3.14159265358979f


#define ANTIALIASING_X 1.f

//LIGHT PARAMETERS
#define DIFFUSE_INTENSITY  0.8f
#define SPECULAR_INTENSITY 0.1f
#define AMBIENT_INTENSITY  0.1f
#define SHINY_FACTOR 15.f
#define LIGHT_COLOR_INTENSITY 14.f

vec3 black(0.0,0.0,0.0);


/* ----------------------------------------------------------------------------*/
/* FUNCTIONS PROTOTYPES                                                        */

void Update(Camera &camera);
void Draw(screen* screen,Camera &camera, std::vector<Triangle> &triangles,Intersection &closestIntersection);
bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection );
vec3 DirectLight( const Intersection& i , Camera &camera);
void initialize_camera(Camera &camera);
vec3 calculateColor(vec3 color,const Intersection& i, Camera &camera );
vec3 pointLight(const Intersection& i, int n, vec4 n_hat, Camera &camera);
void initLights();

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS                                                                   */
/* ----------------------------------------------------------------------------*/

/* ----------------------------------------------------------------------------*/
/* MAIN                                                                   */
/* ----------------------------------------------------------------------------*/

int main( int argc, char* argv[] )
{
  screen *screen = InitializeSDL( SCREEN_WIDTH, SCREEN_HEIGHT, FULLSCREEN_MODE );
  Camera camera;
  initialize_camera(camera);
  initLights();
  light_selection = 0;

  Intersection closestIntersection;
  //Initialize triangles
  if (argc <= 1) {
    printf("Enter model to be loaded\n");
    return 1;
  }

  else {
    if (std::string(argv[1]) == "test"){
      LoadTestModel(triangles);
    }
    else {
      if (!loadObj(std::string(argv[1]),triangles,white)) {
        printf("Can't load file\n");
        return 1;
      }
      std::cout << "Succesfully loaded model" << '\n';
      std::cout << "Number of triangles" << triangles.size() << '\n';

    }
  }

  while( NoQuitMessageSDL() ){
      Update(camera);
      Draw(screen,camera,triangles,closestIntersection);
      SDL_Renderframe(screen);
      // return 0;
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/* ----------------------------------------------------------------------------*/
/* INITIALISERS                                                                   */
/* ----------------------------------------------------------------------------*/

void initLights(){
  vec4 lightPos( 0, 0, -1.5, 1.0 );
  vec3 lightColor = LIGHT_COLOR_INTENSITY * vec3( 1, 1, 1 );
  Light light(lightPos, lightColor, DIFFUSE_INTENSITY, SPECULAR_INTENSITY, AMBIENT_INTENSITY, Pointlight );
  lights.push_back(light);
}

void initialize_camera(Camera &camera){
  camera.position = vec4(0,0,-2,0);
  camera.yaw = 0;
  camera.pitch = 0;
  camera.roll = 0;
}

vec3 IndirectLight(){
  return 0.5f*vec3( 1, 1, 1 );
}

/* ----------------------------------------------------------------------------*/
/* CALCULATE DIRECT AND INDIRECT LIGHT CONTRIBUTION AND COLOR                                                                   */
/* ----------------------------------------------------------------------------*/

vec3 calculateColor(vec3 color,const Intersection& i, Camera &camera ){
  vec3 D = DirectLight(i, camera);
  vec3 I = IndirectLight();
  return color*D+I*color;
}

/* ----------------------------------------------------------------------------*/
/* LIGHT INTENSITY CALCULATION FOR                                             */
/* ----------------------------------------------------------------------------*/

vec3 pointLight(const Intersection& i, int n, vec4 n_hat, Camera &camera){

  vec4 lightPos = lights[n].pos;
  vec3 color = lights[n].color;

  float r = glm::distance(lightPos, i.position);//DISTANCE
  vec4 r_hat = glm::normalize(lightPos - i.position);//DIRECTION of light
  float dotProduct = glm::dot(n_hat,r_hat);
  float diff_intensity = max(dotProduct,(0.0f));
  vec3 attenuation = color / (4*PI_F*r*r); //colour light / distance

  Intersection shadowIntersection;
  vec4 shadowDirection = -r_hat;
  Intersection shadows_intersection;
  bool inter = ClosestIntersection(lightPos,shadowDirection,triangles,shadows_intersection);
  if (inter && (shadows_intersection.distance < r - 0.0001f) ){
    return vec3(0,0,0);
  }

  vec3 diffuse = attenuation *  triangles[i.triangleIndex].phong.diffuse;

  vec4 viewDir = normalize(-camera.position);
  vec4 H = normalize(r_hat + viewDir );
  float spec_intensity = max(glm::dot(n_hat,H),(0.0f));

  vec3 specular = attenuation * pow(spec_intensity,SHINY_FACTOR)*triangles[i.triangleIndex].phong.specular;

  vec3 D = diffuse*lights[n].diffuse_power*diff_intensity + specular*lights[n].specular_power + lights[n].ambient_power * triangles[i.triangleIndex].phong.ambient ;
  D = clamp(D,vec3(0,0,0),vec3(1,1,1));
  return D;
}

/* ----------------------------------------------------------------------------*/
/* DIRECT LIGHT CONTRIBUTION                                                                    */
/* ----------------------------------------------------------------------------*/

vec3 DirectLight( const Intersection& i,Camera &camera ){
  Triangle triangle = triangles[i.triangleIndex];
  vec4 n_hat = (triangle.normal);
  vec3 sum(0,0,0);

  for (size_t n = 0; n < lights.size(); n++){
    vec3 D(0,0,0);
    //function for different lights?
    switch (lights[n].lightType){
      case Pointlight: D = pointLight(i,n,n_hat,camera);
          break;
      case Spotlight: std::cout << "Spotlight not yet implemented" << '\n';
          break;
      case Directional: std::cout << "Directional not yet implemented" << '\n';
          break;
      default: std::cout << "Light not recognised" << '\n';
          break;
    }
    sum += D;
  }
  return sum;
}


/*Place your drawing here*/
void Draw(screen* screen,Camera &camera, std::vector<Triangle> &triangles,Intersection &closestIntersection){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));

  for(int x = 0; x < SCREEN_WIDTH*ANTIALIASING_X; x+=int(ANTIALIASING_X)){
    int truex =(x/ANTIALIASING_X);
    for(int y = 0; y < SCREEN_HEIGHT*ANTIALIASING_X; y+=int(ANTIALIASING_X)){
      int truey = (y/ANTIALIASING_X);
      vec3 sum(0,0,0);
      for (int sample_x = 0; sample_x < ANTIALIASING_X; sample_x++ ){
        for (int sample_y = 0 ; sample_y < ANTIALIASING_X; sample_y++ ){
          vec4 d(x + sample_x - (SCREEN_WIDTH*ANTIALIASING_X/2), y + sample_y - (SCREEN_HEIGHT*ANTIALIASING_X/2), FOCAL_LENGTH*ANTIALIASING_X,1);
          d = camera.R * d;
          bool intersection = ClosestIntersection(camera.position,d,triangles,closestIntersection);
          if (intersection==true) {
            vec3 color = calculateColor(triangles[closestIntersection.triangleIndex].color,closestIntersection, camera);
            sum += color;
          }
        }
      }
      sum /= ANTIALIASING_X*ANTIALIASING_X;
      PutPixelSDL(screen,truex,truey,sum);
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
//roll
void rotation_aroundZ(Camera& camera, int dir){
  camera.roll += dir*SENSITIVITY;
  vec4 v1(cos(camera.roll), -sin(camera.roll), 0 ,0);
  vec4 v2(sin(camera.roll),cos(camera.roll),0,0);
  vec4 v3(0, 0, 1,0);
  vec4 v4(0,0,0,1);
  camera.R = mat4(v1,v2,v3,v4);
}

//pitch
void rotation_aroundX(Camera& camera, int dir){
  camera.pitch += dir*SENSITIVITY;
  vec4 v1(1, 0, 0,0);
  vec4 v2(0,cos(camera.pitch),-sin(camera.pitch),0);
  vec4 v3(0, sin(camera.pitch), cos(camera.pitch),0);
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
    rotation_aroundY(camera,1);
  }
  else if(keystate[SDL_SCANCODE_RIGHT]){
    rotation_aroundY(camera,-1);
  }
  else if(keystate[SDL_SCANCODE_W]){
    lights[light_selection].pos += forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_S]){
    lights[light_selection].pos -= forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_A]){
      lights[light_selection].pos -= right*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_D]){
      lights[light_selection].pos += right*LIGHT_SENSITIVITY;
  }
  else if (keystate[SDL_SCANCODE_I]){
    rotation_aroundX(camera,-1);
  }
  else if (keystate[SDL_SCANCODE_O]){
    initLights();
  }

  else if (keystate[SDL_SCANCODE_K]){
    rotation_aroundX(camera,1);

  }
  else if (keystate[SDL_SCANCODE_J]){
    rotation_aroundZ(camera,1);

  }
  else if (keystate[SDL_SCANCODE_L]){
    rotation_aroundZ(camera,-1);

  }
  else if (keystate[SDL_SCANCODE_B]){
    camera.position.x -= SENSITIVITY; //Camera moves to the left
  }
  else if (keystate[SDL_SCANCODE_V]){
    camera.position.x += SENSITIVITY; //Camera moves to the left
  }
  else if (keystate[SDL_SCANCODE_C]){
    light_selection = (light_selection + 1 ) % lights.size();
  }

}

bool ClosestIntersection(vec4 start,vec4 dir, const vector<Triangle>& triangles,
                         Intersection& closestIntersection ){
  bool flag = false;
  closestIntersection.distance = std::numeric_limits<float>::max();
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
    float v = x.z;

    //Otan valume = fevgun oi mavres grammes
    if(u >= 0 && v >= 0 && (u + v) <= 1 && t >= 0 && t < closestIntersection.distance){
        if (flag == false) flag = true;
        closestIntersection.triangleIndex = i;
        closestIntersection.distance = t;
        closestIntersection.position = start + dir*t;
    }
  }
  return flag;
}
