// C++ include
#include <fstream>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>
#include <gif.h> 

// Utilities for the Assignment
#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;

// To hold information about neigbhboring vertices for each vertex
vector<vector<Eigen::Vector3f>> hashNorm;

// Global parameters to be used manipulate image conviniently
int frameBuffer_rows = 4000;
int frameBuffer_cols = 4000;
float x_scale = 2;
float y_scale = 2;
float z_scale = 2;
float x_move = 0;//0.2;
float y_move = 0;//-1;
float z_move = 0;
float gif_delta = 0.2*x_scale;
int delay = 25;
bool isPerspective = false;


void load_off(const string &filename, Eigen::MatrixXd &V, Eigen::MatrixXi &F) {
	std::ifstream in(filename);
	std::string token;
	in >> token;
	int nv, nf, ne;
	in >> nv >> nf >> ne;
	V.resize(nv, 3);
	F.resize(nf, 3);
	for (int i = 0; i < nv; ++i) {
		in >> V(i, 0) >> V(i, 1) >> V(i, 2);
		vector<Eigen::Vector3f> temp;
		hashNorm.push_back(temp);
	}
	for (int i = 0; i < nf; ++i) {
		int s;
		in >> s >> F(i, 0) >> F(i, 1) >> F(i, 2);

		Eigen::Vector3f a (V(F(i,0),0),V(F(i,0),1),V(F(i,0),2));
		Eigen::Vector3f b (V(F(i,1),0),V(F(i,1),1),V(F(i,1),2));
		Eigen::Vector3f c (V(F(i,2),0),V(F(i,2),1),V(F(i,2),2));

		Eigen::Vector3f u = b - a;
		Eigen::Vector3f v = c - a;
		Eigen::Vector3f norm = (u.cross(v)).normalized();

		hashNorm[F(i,0)].push_back(norm);
		hashNorm[F(i,1)].push_back(norm);
		hashNorm[F(i,2)].push_back(norm);
		assert(s == 3);
	}
}

FrameBuffer scale_down_x(const FrameBuffer& fb, const int x){
	assert(fb.rows()%x==0);
	assert(fb.cols()%x==0);

	FrameBuffer out(fb.rows()/x,fb.cols()/x);
	for(unsigned i=0;i<out.rows();i++){
		for(unsigned j=0;j<out.cols();j++){
			Eigen::Vector4f avg = Eigen::Vector4f::Zero();
			for(unsigned ii=0;ii<x;ii++){
				for(unsigned jj=0;jj<x;jj++){
					avg += fb(i*x+ii,j*x+jj).color.cast<float>();
				}
			}
			avg /= (x*x);
			out(i,j).color = avg.cast<uint8_t>();
		}
	}

	return out;
}

int main(int argc, char *argv[]) 
{

	// The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer_1(frameBuffer_rows,frameBuffer_cols);
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer_2(frameBuffer_rows,frameBuffer_cols);
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer_3(frameBuffer_rows,frameBuffer_cols);

	// Global Constants (empty in this example)
	UniformAttributes uniform;
	float aspect_ratio = float(frameBuffer_cols)/float(frameBuffer_rows);
	uniform.view <<
	x_scale,0,0,x_move,
	0,x_scale,0,y_move,
	0,0,z_scale,z_move,
	0,0,0,1;
	if(aspect_ratio < 1){
		uniform.view(0,0) *= aspect_ratio;
		uniform.view(0,3) *= aspect_ratio;
	}
	else {
		uniform.view(1,1) *= 1.0/aspect_ratio;
		uniform.view(1,3) *= 1.0/aspect_ratio;
	}

	float n = 1;
	float f = 0.04;

	if(isPerspective){
		uniform.perspective <<
		n,0,0,0,
		0,n,0,0,
		0,0,n+f,-f*n,
		0,0,0,1;
	}
	else{
		uniform.perspective <<
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1;
	}

	uniform.camera << 0,0,2;
	uniform.light << -3,5,-1;
	uniform.light_intensity << 16,16,16;
	uniform.diffuse_color << 0.5,0.5,0.9;
	uniform.specular_color <<0.2,0.2,0.8;
	uniform.specular_exponent = 256;
	uniform.ambient_color <<1,1,1;
	uniform.ambient_light <<0.4,0.4,0.2;

	// Basic rasterization program
	Program program;

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		VertexAttributes out;
		out.position = uniform.view * uniform.perspective * va.position;
		Eigen::Vector4f temp(va.normal(0),va.normal(1),va.normal(2),1);
		Eigen::Vector4f temp2 = uniform.view * uniform.perspective * temp;
		temp2 = temp2.normalized();
		out.normal <<temp2(0),temp2(1),temp2(2);

		Eigen::Vector3f position(va.position(0),va.position(1),va.position(2));
		Eigen::Vector3f ambient_color = uniform.ambient_color.array() * uniform.ambient_light.array();
		Eigen::Vector3f lights_color(0, 0, 0);
		Eigen::Vector3f Li = (uniform.light - position).normalized();
		Eigen::Vector3f N = out.normal;
		Eigen::Vector3f diffuse = uniform.diffuse_color * max(Li.dot(N), float(0));
		Eigen::Vector3f h = (uniform.camera - position).normalized() + (uniform.light-position).normalized();
		
		Eigen::Vector3f specular = uniform.specular_color * pow(h.normalized().transpose() * N, uniform.specular_exponent);
		specular(0) = max(specular(0),float(0));
		specular(1) = max(specular(1),float(0));
		specular(2) = max(specular(2),float(0));
		Eigen::Vector3f D = uniform.light - position;
		lights_color += (diffuse + specular).cwiseProduct(uniform.light_intensity) /  D.squaredNorm();
		Eigen::Vector3f final_color = lights_color + ambient_color;
		out.color << final_color(0),final_color(1),final_color(2),255;

		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		FragmentAttributes out(va.color(0),va.color(1),va.color(2));
		out.position = va.position;
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		if(fa.position[2] < previous.depth){
			FrameBufferAttributes out(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
			out.depth = fa.position[2];
			return out;
		}
		else{
			return previous;
		}
	};

	Program program_2;

	// The vertex shader is the identity
	program_2.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		VertexAttributes out;
		out.position = uniform.view * uniform.perspective * va.position;
		out.color << 0,1,1,1;
		//out.color =  uniform.perspective * out.color;
		return out;
	};

	// The fragment shader uses a fixed color
	program_2.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		FragmentAttributes out(va.color(0),va.color(1),va.color(2));
		out.position = va.position;
		return out;
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program_2.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		if(fa.position[2] < previous.depth){
			FrameBufferAttributes out(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
			out.depth = fa.position[2];
			return out;
		}
		else{
			return previous;
		}
	};

	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " bunny.off" << endl;
		return 1;
	}

	// Getting Vertices and Facets from input
	string filename = std::string(DATA_DIR) + argv[1];
	ifstream in(filename);
	Eigen::MatrixXd Vertices;
	Eigen::MatrixXi Facets;
	load_off(filename,Vertices,Facets);
	
	// --------------------------------------------------------------------------------------------------------------------------//
	// --------------------------------------------------------------------------------------------------------------------------//
	// --------------------------------------------------------------------------------------------------------------------------//
	// Vertices for wireframe
	vector<VertexAttributes> vertices_1;
	for (int i = 0;i < Facets.size()/3;i++){
		Eigen::Vector3f a (Vertices(Facets(i,0),0),Vertices(Facets(i,0),1),Vertices(Facets(i,0),2));
		Eigen::Vector3f b (Vertices(Facets(i,1),0),Vertices(Facets(i,1),1),Vertices(Facets(i,1),2));
		Eigen::Vector3f c (Vertices(Facets(i,2),0),Vertices(Facets(i,2),1),Vertices(Facets(i,2),2));

		vertices_1.push_back(VertexAttributes(a(0),a(1),a(2)));
		vertices_1.push_back(VertexAttributes(b(0),b(1),b(2)));
		vertices_1.push_back(VertexAttributes(b(0),b(1),b(2)));
		vertices_1.push_back(VertexAttributes(c(0),c(1),c(2)));
		vertices_1.push_back(VertexAttributes(c(0),c(1),c(2)));
		vertices_1.push_back(VertexAttributes(a(0),a(1),a(2)));
	}
	
	rasterize_lines(program_2,uniform,vertices_1,1,frameBuffer_1);
	// Generate image for wireframe
	vector<uint8_t> image_1;
	//frameBuffer = scale_down_x(frameBuffer1,4);
	framebuffer_to_uint8(frameBuffer_1,image_1);
	stbi_write_png("wireframe.png", frameBuffer_1.rows(), frameBuffer_1.cols(), 4, image_1.data(), frameBuffer_1.rows()*4);

	// Generate GIF for wireframe
	const char * fileName_4 = "wireframe.gif";
	vector<uint8_t> image_4;
	GifWriter g_1;
	GifBegin(&g_1, fileName_4, frameBuffer_1.rows(), frameBuffer_1.cols(),delay);
	bool rotate = false;
	for(float i=1;i<=x_scale;i+=gif_delta){
		frameBuffer_1.setConstant(FrameBufferAttributes());
		if(rotate){
			uniform.view(0,0) = -i;
			uniform.view(0,3) = -i*x_move/x_scale;
			rotate = false;
		}
		else{
			uniform.view(0,0) = i;
			uniform.view(0,3) = i*x_move/x_scale;
			rotate = true;
		}
		uniform.view(1,1) = abs(i);
		uniform.view(1,3) = (abs(i))*y_move/y_scale;
		uniform.view(2,3) = abs(i)*x_move/x_scale;
		if(aspect_ratio < 1){
			uniform.view(0,0) *= aspect_ratio;
			uniform.view(0,3) *= aspect_ratio;
		}
		rasterize_lines(program_2,uniform,vertices_1,1,frameBuffer_1);
		framebuffer_to_uint8(frameBuffer_1,image_4);
		GifWriteFrame(&g_1, image_4.data(), frameBuffer_1.rows(), frameBuffer_1.cols(),delay);
	}
	for(float i=x_scale;i>=1;i-=gif_delta){
		frameBuffer_1.setConstant(FrameBufferAttributes());
		if(rotate){
			uniform.view(0,0) = -i;
			uniform.view(0,3) = -i*x_move/x_scale;
			rotate = false;
		}
		else{
			uniform.view(0,0) = i;
			uniform.view(0,3) = i*x_move/x_scale;
			rotate = true;
		}
		uniform.view(1,1) = abs(i);
		uniform.view(1,3) = (abs(i))*y_move/y_scale;
		uniform.view(2,3) = abs(i)*x_move/x_scale;
		if(aspect_ratio < 1){
			uniform.view(0,0) *= aspect_ratio;
			uniform.view(0,3) *= aspect_ratio;
		}
		rasterize_lines(program_2,uniform,vertices_1,1,frameBuffer_1);
		framebuffer_to_uint8(frameBuffer_1,image_4);
		GifWriteFrame(&g_1, image_4.data(), frameBuffer_1.rows(), frameBuffer_1.cols(),delay);
	}
	GifEnd(&g_1);
	uniform.view(0,0) = x_scale;
	uniform.view(0,3) = x_move;
	uniform.view(1,1) = y_scale;
	uniform.view(1,3) = y_move;
	if(aspect_ratio < 1){
		uniform.view(0,0) *= aspect_ratio;
		uniform.view(0,3) *= aspect_ratio;
	}
	else{
		uniform.view(1,1) *= aspect_ratio;
		uniform.view(1,3) *= aspect_ratio;
	}

	// --------------------------------------------------------------------------------------------------------------------------//
	// --------------------------------------------------------------------------------------------------------------------------//
	// --------------------------------------------------------------------------------------------------------------------------//

	// Vertices for flat shading
	vector<VertexAttributes> vertices_2;
	for (unsigned i = 0;i < Facets.size()/3;i++){
		Eigen::Vector3f a (Vertices(Facets(i,0),0),Vertices(Facets(i,0),1),Vertices(Facets(i,0),2));
		Eigen::Vector3f b (Vertices(Facets(i,1),0),Vertices(Facets(i,1),1),Vertices(Facets(i,1),2));
		Eigen::Vector3f c (Vertices(Facets(i,2),0),Vertices(Facets(i,2),1),Vertices(Facets(i,2),2));

		vertices_2.push_back(VertexAttributes(a(0),a(1),a(2)));
		vertices_2.push_back(VertexAttributes(b(0),b(1),b(2)));
		vertices_2.push_back(VertexAttributes(c(0),c(1),c(2)));
		
		
		Eigen::Vector3f u = b - a;
		Eigen::Vector3f v = c - a;
		Eigen::Vector3f norm = (u.cross(v)).normalized();
		

		vertices_2[i*3+0].normal<<norm;
		vertices_2[i*3+1].normal<<norm;
		vertices_2[i*3+2].normal<<norm;
		
			
	}

	rasterize_triangles(program,uniform,vertices_2,frameBuffer_2);
	rasterize_lines(program_2,uniform,vertices_1,1,frameBuffer_2);

	// Generate image for flat
	vector<uint8_t> image_2;
	//frameBuffer = scale_down_x(frameBuffer2,4);
	framebuffer_to_uint8(frameBuffer_2,image_2);
	stbi_write_png("flat.png", frameBuffer_2.rows(), frameBuffer_2.cols(), 4, image_2.data(), frameBuffer_2.rows()*4);

	// Generate GIF for flat
	const char * fileName_5 = "flat.gif";
	vector<uint8_t> image_5;
	GifWriter g_2;
	GifBegin(&g_2, fileName_5, frameBuffer_2.rows(), frameBuffer_2.cols(),delay);
	for(float i=1;i<=x_scale;i+=gif_delta){
		frameBuffer_2.setConstant(FrameBufferAttributes());
		if(rotate){
			uniform.view(0,0) = -i;
			uniform.view(0,3) = -i*x_move/x_scale;
			rotate = false;
		}
		else{
			uniform.view(0,0) = i;
			uniform.view(0,3) = i*x_move/x_scale;
			rotate = true;
		}
		uniform.view(1,1) = abs(i);
		uniform.view(1,3) = (abs(i))*y_move/y_scale;
		uniform.view(2,3) = abs(i)*x_move/x_scale;
		if(aspect_ratio < 1){
			uniform.view(0,0) *= aspect_ratio;
			uniform.view(0,3) *= aspect_ratio;
		}
		rasterize_triangles(program,uniform,vertices_2,frameBuffer_2);
		rasterize_lines(program_2,uniform,vertices_1,1,frameBuffer_2);
		framebuffer_to_uint8(frameBuffer_2,image_5);
		GifWriteFrame(&g_2, image_5.data(), frameBuffer_2.rows(), frameBuffer_2.cols(),delay);
	}
	for(float i=x_scale;i>=1;i-=gif_delta){
		frameBuffer_2.setConstant(FrameBufferAttributes());
		if(rotate){
			uniform.view(0,0) = -i;
			uniform.view(0,3) = -i*x_move/x_scale;
			rotate = false;
		}
		else{
			uniform.view(0,0) = i;
			uniform.view(0,3) = i*x_move/x_scale;
			rotate = true;
		}
		uniform.view(1,1) = abs(i);
		uniform.view(1,3) = (abs(i))*y_move/y_scale;
		uniform.view(2,3) = abs(i)*x_move/x_scale;
		if(aspect_ratio < 1){
			uniform.view(0,0) *= aspect_ratio;
			uniform.view(0,3) *= aspect_ratio;
		}
		rasterize_triangles(program,uniform,vertices_2,frameBuffer_2);
		rasterize_lines(program_2,uniform,vertices_1,1,frameBuffer_2);
		framebuffer_to_uint8(frameBuffer_2,image_5);
		GifWriteFrame(&g_2, image_5.data(), frameBuffer_2.rows(), frameBuffer_2.cols(),delay);
	}
	GifEnd(&g_2);
	uniform.view(0,0) = x_scale;
	uniform.view(0,3) = x_move;
	uniform.view(1,1) = y_scale;
	uniform.view(1,3) = y_move;
	if(aspect_ratio < 1){
		uniform.view(0,0) *= aspect_ratio;
		uniform.view(0,3) *= aspect_ratio;
	}
	else{
		uniform.view(1,1) *= aspect_ratio;
		uniform.view(1,3) *= aspect_ratio;
	}
	
	// --------------------------------------------------------------------------------------------------------------------------//
	// --------------------------------------------------------------------------------------------------------------------------//
	// --------------------------------------------------------------------------------------------------------------------------//

	// Vertices for per vertex shading
	vector<VertexAttributes> vertices_3;
	for (unsigned i = 0;i < Facets.size()/3;i++){
		Eigen::Vector3f a (Vertices(Facets(i,0),0),Vertices(Facets(i,0),1),Vertices(Facets(i,0),2));
		Eigen::Vector3f b (Vertices(Facets(i,1),0),Vertices(Facets(i,1),1),Vertices(Facets(i,1),2));
		Eigen::Vector3f c (Vertices(Facets(i,2),0),Vertices(Facets(i,2),1),Vertices(Facets(i,2),2));

		vertices_3.push_back(VertexAttributes(a(0),a(1),a(2)));
		vertices_3.push_back(VertexAttributes(b(0),b(1),b(2)));
		vertices_3.push_back(VertexAttributes(c(0),c(1),c(2)));
		
		for(unsigned k=0;k<3;k++){
			Eigen::Vector3f norm(0,0,0);
			for(unsigned j=0;j<hashNorm[Facets(i,k)].size();j++){
				norm += hashNorm[Facets(i,k)][j];
			}
			norm /= hashNorm[Facets(i,k)].size();
			vertices_3[i*3+k].normal<<norm;
		}	
			
	}

	rasterize_triangles(program,uniform,vertices_3,frameBuffer_3);

	// Generate image for vertex
	vector<uint8_t> image_3;
	//frameBuffer = scale_down_x(frameBuffer3,4);
	framebuffer_to_uint8(frameBuffer_3,image_3);
	stbi_write_png("vertex.png", frameBuffer_3.rows(), frameBuffer_3.cols(), 4, image_3.data(), frameBuffer_3.rows()*4);
	
	// Generate GIF for vertex
	const char * fileName_6 = "vertex.gif";
	vector<uint8_t> image_6;
	GifWriter g_3;
	GifBegin(&g_3, fileName_6, frameBuffer_3.rows(), frameBuffer_3.cols(),delay);
	for(float i=1;i<=x_scale;i+=gif_delta){
		frameBuffer_3.setConstant(FrameBufferAttributes());
		if(rotate){
			uniform.view(0,0) = -i;
			uniform.view(0,3) = -i*x_move/x_scale;
			rotate = false;
		}
		else{
			uniform.view(0,0) = i;
			uniform.view(0,3) = i*x_move/x_scale;
			rotate = true;
		}
		uniform.view(1,1) = abs(i);
		uniform.view(1,3) = (abs(i))*y_move/y_scale;
		uniform.view(2,3) = abs(i)*x_move/x_scale;
		if(aspect_ratio < 1){
			uniform.view(0,0) *= aspect_ratio;
			uniform.view(0,3) *= aspect_ratio;
		}
		rasterize_triangles(program,uniform,vertices_3,frameBuffer_3);
		framebuffer_to_uint8(frameBuffer_3,image_6);
		GifWriteFrame(&g_3, image_6.data(), frameBuffer_3.rows(), frameBuffer_3.cols(),delay);
	}
	for(float i=x_scale;i>=1;i-=gif_delta){
		frameBuffer_3.setConstant(FrameBufferAttributes());
		if(rotate){
			uniform.view(0,0) = -i;
			uniform.view(0,3) = -i*x_move/x_scale;
			rotate = false;
		}
		else{
			uniform.view(0,0) = i;
			uniform.view(0,3) = i*x_move/x_scale;
			rotate = true;
		}
		uniform.view(1,1) = abs(i);
		uniform.view(1,3) = (abs(i))*y_move/y_scale;
		uniform.view(2,3) = abs(i)*x_move/x_scale;
		if(aspect_ratio < 1){
			uniform.view(0,0) *= aspect_ratio;
			uniform.view(0,3) *= aspect_ratio;
		}
		rasterize_triangles(program,uniform,vertices_3,frameBuffer_3);
		framebuffer_to_uint8(frameBuffer_3,image_6);
		GifWriteFrame(&g_3, image_6.data(), frameBuffer_3.rows(), frameBuffer_3.cols(),delay);
	}
	GifEnd(&g_3);


	return 0;
}
