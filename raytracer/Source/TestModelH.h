#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

// Defines a simple test model: The Cornel Box

#include <glm/glm.hpp>
#include <vector>
#include <fstream>
#include <string>       // std::string
#include <iostream>     // std::cout
#include <sstream>      // std::istringstream

#define DIF 0.8f
#define SPC 0.f
#define SPC1 0.5f
#define AMB 0.1f

using glm::vec3;

enum lightType_t { Pointlight, Spotlight, Directional};
enum matType_t { Diffuse, Reflective, Refractive};
class Phong
{
public:
	float kd;
	float ks;
	float ka;

	Phong(float kd, float ks, float ka)
	: kd(kd), ks(ks), ka(ka) {}

};

// class Refractive
// {
// public:
// 	float ior;
// };


class Light
{
public:
	glm::vec4 pos;
	glm::vec3 diffuse_color;
	glm::vec3 specular_color;
	glm::vec3 intensity;
	glm::vec3 ambient_intensity;
	lightType_t lightType;

	Light(glm::vec4 pos,glm::vec3 diffuse_color, glm::vec3 specular_color,
		glm::vec3 intensity, glm::vec3 ambient_intensity,lightType_t lightType)
	: pos(pos), diffuse_color(diffuse_color),specular_color(specular_color),
	intensity(intensity),ambient_intensity(ambient_intensity), lightType(lightType)
	{
		std::cout << "New Light Created" << '\n';
	}
};


// Describe a sphere
class Sphere
 {
public:
	glm::vec3 center;
	float radius;
	glm::vec3 color;
	Phong phong;
	matType_t material;
	float ior;

	glm::vec4 get_normal(glm::vec4 pos_hit){
		glm::vec3 norm =(glm::vec3(pos_hit) - center)/radius;
		return glm::vec4(norm,1);
	}

	Sphere( glm::vec3 center, float radius, glm::vec3 color,
		float diffuse, float specular, float ambient, matType_t material, float ior )
		: center(center), radius(radius), color(color), phong(diffuse, specular, ambient),
		material(material) , ior(ior)
	{	}


};

// Used to describe a triangular surface:
class Triangle
{
public:
	glm::vec4 v0;
	glm::vec4 v1;
	glm::vec4 v2;
	glm::vec4 normal;
	glm::vec3 color;
	Phong phong;
	matType_t material;
	float ior;

	glm::vec4 get_normal(){
		return normal;
	}

	Triangle( glm::vec4 v0, glm::vec4 v1, glm::vec4 v2, glm::vec3 color,
		float diffuse, float specular, float ambient, matType_t material, float ior )
		: v0(v0), v1(v1), v2(v2), color(color), phong(diffuse, specular, ambient), material(material) , ior(ior)
	{
		ComputeNormal();
	}

	void ComputeNormal()
	{
	  glm::vec3 e1 = glm::vec3(v1.x-v0.x,v1.y-v0.y,v1.z-v0.z);
	  glm::vec3 e2 = glm::vec3(v2.x-v0.x,v2.y-v0.y,v2.z-v0.z);
	  glm::vec3 normal3 = glm::normalize( glm::cross( e2, e1 ) );
	  normal.x = normal3.x;
	  normal.y = normal3.y;
	  normal.z = normal3.z;
	  normal.w = 1.0;
	}
};


bool loadObj(std::string path, std::vector<Triangle>& triangles, glm::vec3 color ){
	glm::vec3 white(  0.75f, 0.75f, 0.75f );
	glm::vec3 red(    0.75f, 0.15f, 0.15f );
	glm::vec3 yellow( 0.75f, 0.75f, 0.15f );
	glm::vec3 green(  0.15f, 0.75f, 0.15f );
	glm::vec3 cyan(   0.15f, 0.75f, 0.75f );
	glm::vec3 blue(   0.15f, 0.15f, 0.75f );
	glm::vec3 purple( 0.75f, 0.15f, 0.75f );

	std::vector<glm::vec4> vertices;
	std::vector<glm::vec4> faces;

	std::ifstream file(path); // pass file name as argment
	if( !file ){
	    std::cout << "Can't open file !" << "\n" ;
	    return false;
	}

	std::string line;
	while(getline(file, line)) {


		if (line[0] == '#' ){
			//comment so do nothing
		}
		else if (line.substr(0,2) == "v "){	//checking for v by substring as there is also vt vn
			std::istringstream linestream(line.substr(2)); //istringstream input stream
			glm::vec4 vector;
			linestream >> vector.x;
			linestream >> vector.y;
			linestream >> vector.z;
			vector.w = 1;
			vector.y *=-1;
			vector.z *=-1;
			vertices.push_back(vector);
		}
		// else if (line.substr(0,2) == "vt"){
    //
		// }
		// else if (line.substr(0,2) == "vn"){
    //
		// }

		else if (line[0] == 'f'){
			glm::vec4 face;
		  const char* line_ptr=line.c_str();
			int x,y,z,temp1,temp2,temp3;
	    sscanf (line_ptr, "f %d//%d %d//%d %d//%d",&x,&temp1,&y,&temp2,&z,&temp3); //here it read the line start with f and store the corresponding values in the variables
			face.x = x;
			face.y = y;
			face.z = z;
			face.w = 1;
			//obj starts from 1 index not zero
			face.x--;
			face.y--;
			face.z--;
			faces.push_back(face);
		}

	}

	for (unsigned int i = 0 ; i < faces.size(); i++){
		triangles.push_back(Triangle(vertices[faces[i].x], vertices[faces[i].y], vertices[faces[i].z], color, DIF,SPC,AMB,Diffuse, 0 ) );
	}
	return true;

}


// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1


void LoadTestModel( std::vector<Triangle>& triangles, std::vector<Sphere>& spheres)
{
	using glm::vec3;
	using glm::vec4;

	// Defines colors:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );

	triangles.clear();
	triangles.reserve( 5*2*3 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec4 A(L,0,0,1);
	vec4 B(0,0,0,1);
	vec4 C(L,0,L,1);
	vec4 D(0,0,L,1);

	vec4 E(L,L,0,1);
	vec4 F(0,L,0,1);
	vec4 G(L,L,L,1);
	vec4 H(0,L,L,1);

	// Floor:
	triangles.push_back( Triangle( C, B, A, white,DIF,SPC,AMB, Diffuse, 1.5 ) );
	triangles.push_back( Triangle( C, D, B, white,DIF,SPC,AMB, Diffuse, 1.5 ) );

	// Left wall
	triangles.push_back( Triangle( A, E, C, red,DIF ,SPC1,AMB, Diffuse, 0) );
	triangles.push_back( Triangle( C, E, G, red,DIF ,SPC1,AMB, Diffuse, 0) );

	// Right wall
	triangles.push_back( Triangle( F, B, D, green,DIF ,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle( H, F, D, green,DIF ,SPC,AMB, Diffuse, 0) );

	// Ceiling
	triangles.push_back( Triangle( E, F, G, white,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle( F, H, G, white,DIF ,SPC,AMB, Diffuse, 0) );

	// Back wall
	triangles.push_back( Triangle( G, D, C, white,DIF,SPC ,AMB, Diffuse, 0) );
	triangles.push_back( Triangle( G, H, D, white,DIF,SPC ,AMB, Diffuse, 0) );


	// ---------------------------------------------------------------------------
	// Short block

	A = vec4(290,0,114,1);
	B = vec4(130,0, 65,1);
	C = vec4(240,0,272,1);
	D = vec4( 82,0,225,1);

	E = vec4(290,165,114,1);
	F = vec4(130,165, 65,1);
	G = vec4(240,165,272,1);
	H = vec4( 82,165,225,1);

	// Front
	triangles.push_back( Triangle(E,B,A,yellow, DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(E,F,B,yellow,DIF,SPC ,AMB, Diffuse, 0) );

	// Front
	triangles.push_back( Triangle(F,D,B,yellow, DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(F,H,D,yellow,DIF,SPC,AMB, Diffuse, 0) );

	// BACK
	triangles.push_back( Triangle(H,C,D,yellow,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(H,G,C,yellow,DIF,SPC,AMB, Diffuse, 0) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,yellow,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(E,A,C,yellow,DIF,SPC,AMB, Diffuse, 0) );

	// TOP
	triangles.push_back( Triangle(G,F,E,yellow,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(G,H,F,yellow,DIF,SPC,AMB, Diffuse, 0) );

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec4(423,0,247,1);
	B = vec4(265,0,296,1);
	C = vec4(472,0,406,1);
	D = vec4(314,0,456,1);

	E = vec4(423,330,247,1);
	F = vec4(265,330,296,1);
	G = vec4(472,330,406,1);
	H = vec4(314,330,456,1);

	// Front
	triangles.push_back( Triangle(E,B,A,blue,DIF,SPC,AMB, Diffuse, 2) );
	triangles.push_back( Triangle(E,F,B,blue,DIF,SPC,AMB, Diffuse, 2) );

	// Front
	triangles.push_back( Triangle(F,D,B,blue,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(F,H,D,blue,DIF,SPC,AMB, Diffuse, 0) );

	// BACK
	triangles.push_back( Triangle(H,C,D,blue,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(H,G,C,blue,DIF,SPC,AMB, Diffuse, 0) );

	// LEFT
	triangles.push_back( Triangle(G,E,C,blue,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(E,A,C,blue,DIF,SPC,AMB, Diffuse, 0) );

	// TOP
	triangles.push_back( Triangle(G,F,E,blue,DIF,SPC,AMB, Diffuse, 0) );
	triangles.push_back( Triangle(G,H,F,blue,DIF,SPC,AMB, Diffuse, 0) );




	// ----------------------------------------------
	// Scale to the volume [-1,1]^3r
	// spheres.push_back(Sphere(vec3(400,150,150), 75, white, DIF, SPC1 ,AMB, Refractive, 0.8));
	// spheres.push_back(Sphere(vec3(200,350,200), 100, white, DIF, SPC1 ,AMB, Reflective, 0.8));

	for( size_t i=0; i<triangles.size(); ++i )
	{
		triangles[i].v0 *= 2/L;
		triangles[i].v1 *= 2/L;
		triangles[i].v2 *= 2/L;

		triangles[i].v0 -= vec4(1,1,1,1);
		triangles[i].v1 -= vec4(1,1,1,1);
		triangles[i].v2 -= vec4(1,1,1,1);

		triangles[i].v0.x *= -1;
		triangles[i].v1.x *= -1;
		triangles[i].v2.x *= -1;

		triangles[i].v0.y *= -1;
		triangles[i].v1.y *= -1;
		triangles[i].v2.y *= -1;

		triangles[i].v0.w = 1.0;
		triangles[i].v1.w = 1.0;
		triangles[i].v2.w = 1.0;

		triangles[i].ComputeNormal();
	}

	for( size_t i=0; i<spheres.size(); ++i ){

		spheres[i].center.x   *= 2/L;
		spheres[i].center.y   *= 2/L;
		spheres[i].center.z   *= 2/L;

		spheres[i].radius   *= 2/L;

		spheres[i].center -= vec3(1,1,1);
		spheres[i].center.x *= -1;
		spheres[i].center.y *= -1;

	}

	// triangles.clear();
	// spheres.clear();

}

bool solveQuadratic(const float &a, const float &b, const float &c, float &x0, float &x1)
{
    float discr = b * b - 4 * a * c;
    if (discr < 0) return false;
    else if (discr == 0) {
        x0 = x1 = - 0.5 * b / a;
    }
    else {
        float q = (b > 0) ?
            -0.5 * (b + sqrt(discr)) :
            -0.5 * (b - sqrt(discr));
        x0 = q / a;
        x1 = c / q;
    }

    return true;
}

#endif
