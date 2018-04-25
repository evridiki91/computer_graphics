#include <glm/glm.hpp>
#include <SDL.h>
#include "SDL.h"
#include "SDLauxiliary.h"
#include "TestModelH.h"
#include <stdint.h>
#include <math.h>
#include <limits>
// #include <omp.h>

using namespace std;
using glm::vec3;
using glm::mat3;
using glm::vec4;
using glm::mat4;

struct Camera
{
  vec4 position;
  mat4 R; //rotation
  float yaw; //angle for rotating around y-axis
  float pitch;
  float roll;
};

struct Intersection
{
  vec4 position;
  float distance;
  int ObjectIndex;
  int objectType; //0 - triangle, 1 -sphere
};

vector<Triangle> triangles;
vector<Sphere> spheres;
vec3 white(  0.75f, 0.75f, 0.75f );
vector<Light> lights;
size_t light_selection;

#define SCREEN_WIDTH 250*2
#define SCREEN_HEIGHT 250*2
#define FULLSCREEN_MODE false
#define FOCAL_LENGTH SCREEN_WIDTH/2
#define FOCAL_LENGTH_2 1.5
#define SENSITIVITY 0.1f
#define ROTATION_SENSITIVITY 1f
#define LIGHT_SENSITIVITY 0.2f
#define PI_F 3.14159265358979f
#define APERTURE 0.2
int MAX_RECURSIVE_DEPTH = 10;
int MODE = 1; //1-AA,2-DOF

#define ANTIALIASING_X 1.f
#define LIGHT_SAMPLES 50
#define LIGHT_LENGTH 0.08f
#define CAMERA_SAMPLES 5

//LIGHT PARAMETERS
#define DIFFUSE_COLOR  4.f
#define SPECULAR_COLOR 0.4f
#define AMBIENT_INTENSITY 0.1f

#define LIGHT_INTENSITY 15.f
#define SHINY_FACTOR 10.f
#define INDIRECT_LIGHT_INTENSITY 0.2f

vec3 black(0.0,0.0,0.0);

/* ----------------------------------------------------------------------------*/
/* FUNCTIONS PROTOTYPES                                                        */
vec3 IndirectLight()
{
  return INDIRECT_LIGHT_INTENSITY*vec3( 1, 1, 1 );
}

void Update(Camera &camera);
void Draw(screen* screen,Camera &camera, Intersection &closestIntersection);
bool ClosestIntersection(vec4 start,vec4 dir,
                         Intersection& closestIntersection );
vec3 DirectLight( const Intersection& i , vec4 camera);
void initialize_camera(Camera &camera);
vec3 calculateColor(vec3 color,const Intersection& i, vec4 camera );
vec3 pointLight(const Intersection& i, int n, vec4 n_hat, vec4 camera);
void initLights();
void initAreaLights();

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
  initAreaLights();
  light_selection = 0;

  Intersection closestIntersection;
  //Initialize triangles

  if (argc <= 1)
  {
    printf("Enter model to be loaded\n");
    return 1;
  }

  else
  {
    if (std::string(argv[1]) == "test")
    {
      LoadTestModel(triangles,spheres);
    }
    else
    {
      if (!loadObj(std::string(argv[1]),triangles,white))
      {
        printf("Can't load file\n");
        return 1;
      }
      std::cout << "Succesfully loaded model" << '\n';
      std::cout << "Number of triangles" << triangles.size() << '\n';

    }
  }

  while( NoQuitMessageSDL() )
  {

      Update(camera);
      Draw(screen,camera,closestIntersection);
      SDL_Renderframe(screen);
      break;
    }

  SDL_SaveImage( screen, "screenshot.bmp" );

  KillSDL(screen);
  return 0;
}

/* ----------------------------------------------------------------------------*/
/* INITIALISERS                                                                   */
/* ----------------------------------------------------------------------------*/

void initAreaLights()
{
  vec4 lightPos( 0, -0.5, -0.7, 1.0 );
  float step = LIGHT_LENGTH/ (LIGHT_SAMPLES-1);
  int samples = LIGHT_SAMPLES*LIGHT_SAMPLES;
  for (float i = -LIGHT_LENGTH/2; i <= LIGHT_LENGTH/2; i+=step )
  {
    for (float j = -LIGHT_LENGTH/2; j <= LIGHT_LENGTH/2; j+=step )
    {
      Light light(lightPos+vec4(1,0,0,1)*i + vec4(0,0,1,1)*j,
        vec3(DIFFUSE_COLOR/samples,DIFFUSE_COLOR/samples,DIFFUSE_COLOR/samples),
        vec3(SPECULAR_COLOR/samples,SPECULAR_COLOR/samples,SPECULAR_COLOR/samples),
        vec3(LIGHT_INTENSITY/samples,LIGHT_INTENSITY/samples,LIGHT_INTENSITY/samples),
        vec3(AMBIENT_INTENSITY/samples,AMBIENT_INTENSITY/samples,AMBIENT_INTENSITY/samples), Pointlight );
      lights.push_back(light);
    }
  }
}

void initLights()
{
  vec4 lightPos( 0, -0.5, -0.7, 1.0 );
      Light light(lightPos,
        vec3(DIFFUSE_COLOR,DIFFUSE_COLOR,DIFFUSE_COLOR),
        vec3(SPECULAR_COLOR,SPECULAR_COLOR,SPECULAR_COLOR),
        vec3(LIGHT_INTENSITY,LIGHT_INTENSITY,LIGHT_INTENSITY),
        vec3(AMBIENT_INTENSITY,AMBIENT_INTENSITY,AMBIENT_INTENSITY), Pointlight );
      lights.push_back(light);
}

void initialize_camera(Camera &camera)
{
  camera.position = vec4(0,0,-2,0);
  camera.yaw = 0;
  camera.pitch = 0;
  camera.roll = 0;
}


/* ----------------------------------------------------------------------------*/
/* CALCULATE DIRECT AND INDIRECT LIGHT CONTRIBUTION AND COLOR                                                                   */
/* ----------------------------------------------------------------------------*/

vec3 calculateColor(vec3 color,const Intersection& i, vec4 camera )
{
  vec3 D = DirectLight(i, camera);
  vec3 I = IndirectLight();
  return color*(D+I);
}

/* ----------------------------------------------------------------------------*/
/* LIGHT INTENSITY CALCULATION FOR                                             */
/* ----------------------------------------------------------------------------*/

vec3 pointLight(const Intersection& i, int n, vec4 n_hat, vec4 camera)
{

  vec4 lightPos = lights[n].pos;
  vec3 intensity_light = lights[n].intensity;

  float r = glm::distance(lightPos, i.position);//DISTANCE
  vec4 r_hat = glm::normalize(lightPos - i.position);//DIRECTION of light

  int type = i.objectType;
  int material = Diffuse;

    if (type == 0)
    {
      material = triangles[i.ObjectIndex].material;
    }
    else if (type == 1 )
    {
      material = spheres[i.ObjectIndex].material;
    }


  Intersection shadows_intersection;
  bool inter = ClosestIntersection(i.position + 0.00001f*r_hat ,r_hat,shadows_intersection);

  if ( material!= Refractive && inter && (shadows_intersection.distance < r )  )
  {
    return vec3(0,0,0);
  }

  float dotProduct = glm::dot(n_hat,r_hat);
  float diffuse_term = max(dotProduct,(0.0f));
  vec3 attenuation = intensity_light / (4*PI_F*r*r); //colour light / distance

  vec4 viewDir = normalize(-camera);
  vec4 H = normalize(r_hat + viewDir );
  float spec_intensity = max(glm::dot(n_hat,H),(0.0f));

  if (i.objectType == 0){
    vec3 diffuse = attenuation * diffuse_term * triangles[i.ObjectIndex].phong.kd ;
    vec3 specular = attenuation * pow(spec_intensity,SHINY_FACTOR)*triangles[i.ObjectIndex].phong.ks;
    vec3 ambient  =  lights[n].diffuse_color * triangles[i.ObjectIndex].phong.ka;
    vec3 D = diffuse*lights[n].diffuse_color + specular*lights[n].specular_color + ambient;
    D = diffuse;
    D = clamp(D,vec3(0,0,0),vec3(1,1,1));
    return D;
  }

  if (i.objectType == 1){
    vec3 diffuse = attenuation * diffuse_term * spheres[i.ObjectIndex].phong.kd ;
    vec3 specular = attenuation * pow(spec_intensity,SHINY_FACTOR)*spheres[i.ObjectIndex].phong.ks;
    vec3 ambient  =  lights[n].diffuse_color * spheres[i.ObjectIndex].phong.ka;
    vec3 D = diffuse*lights[n].diffuse_color + specular*lights[n].specular_color + ambient;
    D = diffuse;

    D = clamp(D,vec3(0,0,0),vec3(1,1,1));
    return D;
  }

  return vec3(0,0,0);
}

vec4 reflection_direction(vec4 inter, vec4 normal )
{
  return (inter - 2*glm::dot(inter,normal)*normal );
}

/* ----------------------------------------------------------------------------*/
/* DIRECT LIGHT CONTRIBUTION                                                                    */
/* ----------------------------------------------------------------------------*/

vec3 DirectLight( const Intersection& i,vec4 camera )
{
  vec4 n_hat;
  if (i.objectType == 0)
  {
    Triangle triangle = triangles[i.ObjectIndex];
    n_hat = (triangle.normal);
  }

  else if (i.objectType == 1){
    Sphere sphere = spheres[i.ObjectIndex];
    n_hat = sphere.get_normal(i.position);
  }
  vec3 sum(0,0,0);

  for (size_t n = 0; n < lights.size(); n++)
  {
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


vec4 refract(const vec4 &I, const vec4 &N, const float &ior)
{
    float cosi = glm::clamp(glm::dot(I, N),-1.f, 1.f );
    float etai = 1, etat = ior;
    vec4 n = N;
    if (cosi < 0) { cosi = -cosi; } else { std::swap(etai, etat); n= -N; }
    float eta = etai / etat;
    float k = 1 - eta * eta * (1 - cosi * cosi);
    return k < 0 ? vec4(0,0,0,0) : eta * I + (eta * cosi - sqrtf(k)) * n;
}

void fresnel(const vec4 &I, const vec4 &N, const float &ior, float &kr)
{
    float cosi = glm::clamp(glm::dot(I, N),-1.f, 1.f);
    float etai = 1, etat = ior;
    if (cosi > 0) { std::swap(etai, etat); }
    // Compute sini using Snell's law
    float sint = etai / etat * sqrtf(std::max(0.f, 1 - cosi * cosi));
    // Total internal reflection
    if (sint >= 1)
    {
        kr = 1;
    }

    else
    {
        float cost = sqrtf(std::max(0.f, 1 - sint * sint));
        cosi = fabsf(cosi);
        float Rs = ((etat * cosi) - (etai * cost)) / ((etat * cosi) + (etai * cost));
        float Rp = ((etai * cosi) - (etat * cost)) / ((etai * cosi) + (etat * cost));
        kr = (Rs * Rs + Rp * Rp) / 2;
    }
}

vec3 shade(vec4 start, vec4 d, Intersection &closestIntersection, int depth, vec3 &color){
  bool intersection = ClosestIntersection(start,d,closestIntersection);

  if(depth > MAX_RECURSIVE_DEPTH) return vec3(0,0,0);

  if (intersection==true)
  {
    if (closestIntersection.objectType == 1)
    {
        switch(spheres[closestIntersection.ObjectIndex].material)
        {
          case Diffuse :
          {
            color += calculateColor(spheres[closestIntersection.ObjectIndex].color,closestIntersection, start);
            return color;
          }
          case Reflective:
          {
                //compute reflection of ray
                vec4 reflected_d = reflection_direction(d,spheres[closestIntersection.ObjectIndex].get_normal(closestIntersection.position));
                color += spheres[closestIntersection.ObjectIndex].phong.ks
                        * shade(closestIntersection.position+ 0.000001f*reflected_d,reflected_d , closestIntersection, depth+1, color);
                return color;
          }

          case Refractive:
          {
            // compute fresnel
            float kr;
            vec4 normal = spheres[closestIntersection.ObjectIndex].get_normal(closestIntersection.position);
            float ior = spheres[closestIntersection.ObjectIndex].ior;
            fresnel(d, normal, ior, kr);
            bool outside = glm::dot(d, normal) < 0;
            vec4 bias = 0.000001f * normal;
            vec3 Refr_color(0,0,0);

            if (kr < 1) {
                vec4 refractionDirection = normalize(refract(d, normal,ior));
                vec4 refractionRayOrig = outside ? closestIntersection.position - bias : closestIntersection.position + bias;
                Refr_color = shade(refractionRayOrig, refractionDirection, closestIntersection,  depth + 1, Refr_color);
            }

            vec3 Refl_color(0,0,0);
            vec4 reflectionDirection = normalize(reflect(d, normal));
            vec4 reflectionRayOrig = outside ? closestIntersection.position + bias : closestIntersection.position - bias;
            Refl_color = shade(reflectionRayOrig, reflectionDirection, closestIntersection,  depth + 1, Refl_color);

            color += Refl_color * kr + Refr_color * (1 - kr);
            return color;
          }

        }
    }

    else if (closestIntersection.objectType == 0)
    {
      switch(triangles[closestIntersection.ObjectIndex].material)
      {
        case Diffuse:
        {
          color += calculateColor(triangles[closestIntersection.ObjectIndex].color,closestIntersection, start);
          return color;
        }

        case Reflective:
        {
              //compute reflection of ray
              vec4 reflected_d = reflection_direction(d,triangles[closestIntersection.ObjectIndex].normal);
              color += triangles[closestIntersection.ObjectIndex].phong.ks
                      * shade(closestIntersection.position+ 0.000001f*reflected_d,reflected_d , closestIntersection, depth+1, color);
              return color;
        }
        case Refractive:
        {
          // compute fresnel
          float kr;
          vec4 normal = triangles[closestIntersection.ObjectIndex].normal;
          float ior = triangles[closestIntersection.ObjectIndex].ior;
          fresnel(d, normal, ior, kr);
          bool outside = glm::dot(d, normal) < 0;
          vec4 bias = 0.000001f * normal;
          vec3 Refr_color(0,0,0);

          if (kr < 1) {
              vec4 refractionDirection = normalize(refract(d, normal,ior));
              vec4 refractionRayOrig = outside ? closestIntersection.position - bias : closestIntersection.position + bias;
              Refr_color = shade(refractionRayOrig, refractionDirection, closestIntersection,  depth + 1, Refr_color);
          }


          vec3 Refl_color(0,0,0);
          vec4 reflectionDirection = normalize(reflect(d, normal));
          vec4 reflectionRayOrig = outside ? closestIntersection.position + bias : closestIntersection.position - bias;
          Refl_color = shade(reflectionRayOrig, reflectionDirection, closestIntersection,  depth + 1, Refl_color);

          // mix the two
          color += Refl_color * kr + Refr_color * (1 - kr);
          return color;
          break;
        }

        default:
        {
          std::cout << "Something went poopoos" << '\n';
          break;
        }
      }
    }
  }
  else return vec3(0,0,0);
}


/*Place your drawing here*/
void Draw(screen* screen,Camera &camera,Intersection &closestIntersection){
  /* Clear buffer */
  memset(screen->buffer, 0, screen->height*screen->width*sizeof(uint32_t));
  switch (MODE) {
    case 1:
    {
      for(int x = 0; x < SCREEN_WIDTH*ANTIALIASING_X; x+=int(ANTIALIASING_X))
      {
        int truex =(x/ANTIALIASING_X);

        for(int y = 0; y < SCREEN_HEIGHT*ANTIALIASING_X; y+=int(ANTIALIASING_X))
        {
          int truey = (y/ANTIALIASING_X);
          vec3 sum(0,0,0);

          for (int sample_x = 0; sample_x < ANTIALIASING_X; sample_x++ )
          {
            for (int sample_y = 0 ; sample_y < ANTIALIASING_X; sample_y++ )
            {

              vec4 d(x + sample_x - (SCREEN_WIDTH*ANTIALIASING_X/2), y + sample_y - (SCREEN_HEIGHT*ANTIALIASING_X/2), FOCAL_LENGTH*ANTIALIASING_X,1);
              d = camera.R * d;
              vec3 initial_color(0,0,0);
              d = glm::normalize(d);
              vec3 color = shade(camera.position,d,closestIntersection,1,initial_color);
              sum += color;
            }
          }
          sum /= ANTIALIASING_X*ANTIALIASING_X;

          PutPixelSDL(screen,truex,truey,sum);
        }
      }
    break;
    }
    case 2:
    {
      vec4 originalCameraPos = camera.position;
      for(int x = 0; x < SCREEN_WIDTH; x++)
      {
        for(int y = 0; y < SCREEN_HEIGHT; y++)
        {
          for (size_t i = 0; i < CAMERA_SAMPLES; i++) {
          }
        }
      }
      break;
    }
  }
}


void rotation_aroundY(Camera& camera, int dir)
{
  camera.yaw += dir*SENSITIVITY;
  vec4 v1(cos(camera.yaw), 0, sin(camera.yaw),0);
  vec4 v2(0,1,0,0);
  vec4 v3(-sin(camera.yaw), 0, cos(camera.yaw),0);
  vec4 v4(0,0,0,1);
  camera.R = mat4(v1,v2,v3,v4);
}
//roll
void rotation_aroundZ(Camera& camera, int dir)
{
  camera.roll += dir*SENSITIVITY;
  vec4 v1(cos(camera.roll), -sin(camera.roll), 0 ,0);
  vec4 v2(sin(camera.roll),cos(camera.roll),0,0);
  vec4 v3(0, 0, 1,0);
  vec4 v4(0,0,0,1);
  camera.R = mat4(v1,v2,v3,v4);
}

//pitch
void rotation_aroundX(Camera& camera, int dir)
{
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
  if(keystate[SDL_SCANCODE_UP])
  {
    camera.position.z += SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_DOWN])
  {
    camera.position.z -= SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_LEFT])
  {
    rotation_aroundY(camera,1);
  }
  else if(keystate[SDL_SCANCODE_RIGHT])
  {
    rotation_aroundY(camera,-1);
  }
  else if(keystate[SDL_SCANCODE_W])
  {
    lights[light_selection].pos += forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_S])
  {
    lights[light_selection].pos -= forward*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_A])
  {
      lights[light_selection].pos -= right*LIGHT_SENSITIVITY;
  }
  else if(keystate[SDL_SCANCODE_D])
  {
      lights[light_selection].pos += right*LIGHT_SENSITIVITY;
  }
  else if (keystate[SDL_SCANCODE_I])
  {
    rotation_aroundX(camera,-1);
  }
  else if (keystate[SDL_SCANCODE_O])
  {
    initLights();
  }

  else if (keystate[SDL_SCANCODE_K])
  {
    rotation_aroundX(camera,1);
  }

  else if (keystate[SDL_SCANCODE_J])
  {
    rotation_aroundZ(camera,1);
  }

  else if (keystate[SDL_SCANCODE_L])
  {
    rotation_aroundZ(camera,-1);
  }

  else if (keystate[SDL_SCANCODE_B])
  {
    camera.position.x -= SENSITIVITY; //Camera moves to the left
  }

  else if (keystate[SDL_SCANCODE_V])
  {
    camera.position.x += SENSITIVITY; //Camera moves to the left
  }

  else if (keystate[SDL_SCANCODE_C])
  {
    light_selection = (light_selection + 1 ) % lights.size();
  }
  else if (keystate[SDL_SCANCODE_U])
  {
    MAX_RECURSIVE_DEPTH ++;
    std::cout << "increased depth to " <<MAX_RECURSIVE_DEPTH << '\n';
  }

}
glm::vec2 sphere_coords_texture(vec3 normal){
  glm::vec2 coord;
  coord.x = (1 + atan2(normal.z, normal.x) / PI_F) * 0.5;
  coord.y = acosf(normal.y) / PI_F;
  return coord;
}

bool ClosestIntersection(vec4 start,vec4 dir, Intersection& closestIntersection )
 {
  bool flag = false;
  closestIntersection.distance = std::numeric_limits<float>::max();
  // mat3 M;

  for (size_t i = 0; i < triangles.size(); i++){

    mat3 A;
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

    if(u >= 0 && v >= 0 && (u + v) <= 1 && t >= 0 && t < closestIntersection.distance)
    {
        if (flag == false) flag = true;
        closestIntersection.ObjectIndex = i;
        closestIntersection.distance = t;
        closestIntersection.position = start + dir*t;
        closestIntersection.objectType = 0;
    }
  }

  for (size_t i = 0; i < spheres.size(); i++) {
    float t0,t1;

    vec3 L = vec3(start) - spheres[i].center;
    vec3 direction = vec3(dir);

    float a = glm::dot(direction,direction);

    float b = 2 * glm::dot(direction,L);

    float c = glm::dot(L,L) - spheres[i].radius*spheres[i].radius;

    if (!solveQuadratic(a, b, c, t0, t1)) continue;

    if (t0 > t1) std::swap(t0, t1);

    if (t0 < 0) {
      t0 = t1; // if t0 is negative, let's use t1 instead
      if (t0 < 0) continue ; // both t0 and t1 are negative
    }
    float t = t0;

    if (flag == false) flag = true;

    if( t < closestIntersection.distance){
      closestIntersection.ObjectIndex = i;
      closestIntersection.distance = t;
      closestIntersection.position = start + dir*t;
      closestIntersection.objectType = 1;
    }
  }
  return flag;
}
