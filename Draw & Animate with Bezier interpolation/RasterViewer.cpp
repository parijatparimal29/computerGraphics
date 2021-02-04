#include "SDLViewer.h"
#include <Eigen/Core>
#include <unistd.h>
#include <functional>
#include <iostream>

#include "raster.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;

bool ins = false;
bool edt = false;
bool del = false;
bool col = false;
bool v1 = false;
bool v2 = false;
bool v3 = false;
bool translate = false;

Eigen::Vector4f prevColor1(1,0,0,1);
Eigen::Vector4f prevColor2(1,0,0,1);
Eigen::Vector4f prevColor3(1,0,0,1);

bool numpad = false;
int selected = -1;
int vertex_selected = -1;
float trans_x = 0;
float trans_y = 0;
float trans_del_x = 0;
float trans_del_y = 0;

float x1_o = 0;
float y1_o = 0;
float x2_o = 0;
float y2_o = 0;
float x3_o = 0;
float y3_o = 0;

// Used to find the vertex nearest to the cursor
int find_nearest_point(Eigen::Vector4f cursor, vector<VertexAttributes> v_list){
    float nearest = 999999;
    int index = 0;
    for(int i=0;i<v_list.size();i++){
        Eigen::Vector4f delta_val(v_list[i].del_x, v_list[i].del_y, 0,0);
        float dist = (v_list[i].position + delta_val - cursor).norm();
        if(dist < nearest){
            nearest = dist;
            index = i;
        }
    }
    return index;
}

// Used to find the triangle nearest to cursor
int find_nearest_triangle(Eigen::Vector4f cursor, vector<VertexAttributes> v_list){
    float nearest = 10;
    int index = -3;
    for(int i=0;i<v_list.size();i++){
        Eigen::Vector4f delta_val(v_list[i].del_x, v_list[i].del_y, 0,0);
        float dist = (v_list[i].position + delta_val - cursor).norm();
        if(dist < nearest){
            nearest = dist;
            index = i;
        }
    }
    return index/3;
}

// Used for Bézier curve interpolation calculation
float nChoosek(int n, int k){ 
    int res = 1;
    if (k > n - k) 
        k = n - k; 
    for (int i = 0; i < k; ++i) { 
        res *= (n - i); 
        res /= (i + 1); 
    } 
    return float(res); 
} 

int main(int argc, char *args[])
{
    int width = 1500;
    int height = 1000;
    // The Framebuffer storing the image rendered by the rasterizer
	Eigen::Matrix<FrameBufferAttributes,Eigen::Dynamic,Eigen::Dynamic> frameBuffer(width, height);

	// Global Constants
	UniformAttributes uniform;
    
    // View matrix - used for view control
    uniform.view <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,1;

	// Basic rasterization program
	Program program;

    // Rasterizez lines that encapsulates each triangle
    Program progLines;

    // The vertex shader is the identity
	progLines.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{   
        VertexAttributes out = va;

        // translation in vertex shader
        out.position[0] += out.del_x;
        out.position[1] += out.del_y;

        // If the vertex needs to be rotated w.r.t. centroid
        if(out.degree != 0){
            float rot_x0 = out.centroid[0];
            float rot_y0 = out.centroid[1];
            float rot_x1 = out.position[0];
            float rot_y1 = out.position[1];
            float pi = 2*acos(0.0);
            float theta = out.degree * (pi / 18);
            out.position[0] = (rot_x1 - rot_x0)*cos(theta) + (rot_y1 - rot_y0)*sin(theta) + rot_x0;
            out.position[1] = -(rot_x1 - rot_x0)*sin(theta) + (rot_y1 - rot_y0)*cos(theta) + rot_y0;
        }

        // If the vertex needs to be scaled w.r.t. centroid
        if(out.scale != 1){
            float scl_x0 = out.centroid[0];
            float scl_y0 = out.centroid[1];
            float scl_x1 = out.position[0];
            float scl_y1 = out.position[1];
            float mult = pow(1.25,out.scale);
            out.position[0] = (scl_x1 - scl_x0)*mult + scl_x0;
            out.position[1] = (scl_y1 - scl_y0)*mult + scl_y0;
        }

        // For view control
        Eigen::Vector4f finalView = uniform.view * out.position;
        out.position << finalView(0),finalView(1),finalView(2),finalView(3);
        out.color<<1,1,1,1;

		return out;
	};

	// The fragment shader uses a fixed color
	progLines.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	progLines.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// The vertex shader is the identity
	program.VertexShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{   
        VertexAttributes out = va;
        
        // translation in vertex shader
        out.position[0] += out.del_x;
        out.position[1] += out.del_y;
        
        // If the vertex needs to be rotated w.r.t. centroid
        if(out.degree != 0){
            float rot_x0 = out.centroid[0];
            float rot_y0 = out.centroid[1];
            float rot_x1 = out.position[0];
            float rot_y1 = out.position[1];
            float pi = 2*acos(0.0);
            float theta = out.degree * (pi / 18);
            out.position[0] = (rot_x1 - rot_x0)*cos(theta) + (rot_y1 - rot_y0)*sin(theta) + rot_x0;
            out.position[1] = -(rot_x1 - rot_x0)*sin(theta) + (rot_y1 - rot_y0)*cos(theta) + rot_y0;
        }

        // If the vertex needs to be scaled w.r.t. centroid
        if(out.scale != 1){
            float scl_x0 = out.centroid[0];
            float scl_y0 = out.centroid[1];
            float scl_x1 = out.position[0];
            float scl_y1 = out.position[1];
            float mult = pow(1.25,out.scale);
            out.position[0] = (scl_x1 - scl_x0)*mult + scl_x0;
            out.position[1] = (scl_y1 - scl_y0)*mult + scl_y0;
        }

        // For view control
        Eigen::Vector4f finalView = uniform.view * out.position;
        out.position << finalView(0),finalView(1),finalView(2),finalView(3);
		return out;
	};

	// The fragment shader uses a fixed color
	program.FragmentShader = [](const VertexAttributes& va, const UniformAttributes& uniform)
	{
		return FragmentAttributes(va.color(0),va.color(1),va.color(2));
	};

	// The blending shader converts colors between 0 and 1 to uint8
	program.BlendingShader = [](const FragmentAttributes& fa, const FrameBufferAttributes& previous)
	{
		return FrameBufferAttributes(fa.color[0]*255,fa.color[1]*255,fa.color[2]*255,fa.color[3]*255);
	};

	// One triangle in the center of the screen
	vector<VertexAttributes> vertices;

    // Temp data structure to draw intermediate stages of drawing a triangle
    vector<VertexAttributes> temp_vertices;

    //Data structures for animation
    vector<vector<int>> key_rot;
    vector<vector<float>> key_scl;
    vector<vector<float>> key_posx;
    vector<vector<float>> key_posy;

    // Initialize the viewer and the corresponding callbacks
    SDLViewer viewer;
    viewer.init("Viewer Example", width, height);

    viewer.mouse_move = [&](int x, int y, int xrel, int yrel){
        // Action when in insert mode
        if(ins){
            if(v1 && !v2){
                if(temp_vertices.size()==1){
                    temp_vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                         ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                }
                else{
                    temp_vertices[temp_vertices.size()-1].position << ((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                         ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1;
                }
                temp_vertices[temp_vertices.size()-1].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else if(v1 && v2 && !v3){
                if(temp_vertices.size()==2){
                    temp_vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                         ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                }
                else{
                    temp_vertices[temp_vertices.size()-1].position << ((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                         ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1;
                }
                temp_vertices[temp_vertices.size()-1].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
        }

        //Actions when in edit mode
        else if(edt){
            if(translate){
                Eigen::Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                trans_del_x = cursor(0) - trans_x;
                trans_del_y = cursor(1) - trans_y;
                
                // Initial implementation by changing postion of vertices
                //vertices[selected*3+0].position[0] = x1_o + trans_del_x;
                //vertices[selected*3+0].position[1] = y1_o + trans_del_y;
                //vertices[selected*3+1].position[0] = x2_o + trans_del_x;
                //vertices[selected*3+1].position[1] = y2_o + trans_del_y;
                //vertices[selected*3+2].position[0] = x3_o + trans_del_x;
                //vertices[selected*3+2].position[1] = y3_o + trans_del_y;

                vertices[selected*3+0].del_x = x1_o + trans_del_x;
                vertices[selected*3+0].del_y = y1_o + trans_del_y;
                vertices[selected*3+1].del_x = x2_o + trans_del_x;
                vertices[selected*3+1].del_y = y2_o + trans_del_y;
                vertices[selected*3+2].del_x = x3_o + trans_del_x;
                vertices[selected*3+2].del_y = y3_o + trans_del_y;

                Eigen::Vector4f centroid(0,0,0,0);
                for (int k = 0; k < 3; k++) {
                    Eigen::Vector4f delta_val(vertices[selected*3+k].del_x, vertices[selected*3+k].del_y, 0,0);
			        centroid += vertices[selected*3+k].position + delta_val;
		        }
                centroid /= 3;
                vertices[selected*3+0].centroid << centroid;
                vertices[selected*3+1].centroid << centroid;
                vertices[selected*3+2].centroid << centroid;

                vertices[selected*3+0].color << 0,0,1,1;
                vertices[selected*3+1].color << 0,0,1,1;
                vertices[selected*3+2].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
        }
    };

    viewer.mouse_pressed = [&](int x, int y, bool is_pressed, int button, int clicks) {
        
        //Actions when in insert mode
        if(ins){
            if(!is_pressed && !v1){
                v1 = true;
                temp_vertices.clear();
                vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                vertices[vertices.size()-1].color << 1,0,0,1;
                temp_vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                temp_vertices[temp_vertices.size()-1].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else if(!is_pressed && v1 && !v2){
                v2 = true;
                vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                vertices[vertices.size()-1].color << 1,0,0,1;
                temp_vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                temp_vertices[temp_vertices.size()-1].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else if(!is_pressed &&v1 && v2 && !v3){
                vertices.push_back(VertexAttributes(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1));
                vertices[vertices.size()-1].color << 1,0,0,1;
                Eigen::Vector4f centroid(0,0,0,0);
                for (int k = 1; k < 4; k++) {
			        centroid += vertices[vertices.size()-k].position;
		        }
                centroid /= 3;
                vertices[vertices.size()-1].centroid << centroid;
                vertices[vertices.size()-2].centroid << centroid;
                vertices[vertices.size()-3].centroid << centroid;

                v3 = true;
                viewer.redraw_next = true;
                v1 = false;
                v2 = false;
                v3 = false;
            }
        }
        
        //Actions when in edit mode
        else if(edt){
            if(is_pressed){
                Eigen::Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                selected = find_nearest_triangle(cursor, vertices);
                
                trans_x = cursor(0);
                trans_y = cursor(1);

                x1_o = vertices[selected*3+0].del_x;//vertices[selected*3+0].position[0]
                y1_o = vertices[selected*3+0].del_y;//vertices[selected*3+0].position[1]
                x2_o = vertices[selected*3+1].del_x;//vertices[selected*3+1].position[0]
                y2_o = vertices[selected*3+1].del_y;//vertices[selected*3+1].position[1]
                x3_o = vertices[selected*3+2].del_x;//vertices[selected*3+2].position[0]
                y3_o = vertices[selected*3+2].del_y;//vertices[selected*3+2].position[1]

                prevColor1 << vertices[selected*3+0].color;
                prevColor2 << vertices[selected*3+1].color;
                prevColor3 << vertices[selected*3+2].color;

                translate = true;
            }
            else{
                translate = false;

                Eigen::Vector4f centroid(0,0,0,0);
                for (int k = 0; k < 3; k++) {
                    Eigen::Vector4f delta_val(vertices[selected*3+k].del_x, vertices[selected*3+k].del_y, 0,0);
			        centroid += vertices[selected*3+k].position + delta_val;
		        }
                centroid /= 3;
                vertices[selected*3+0].centroid << centroid;
                vertices[selected*3+1].centroid << centroid;
                vertices[selected*3+2].centroid << centroid;

                vertices[selected*3+0].color << prevColor1;
                vertices[selected*3+1].color << prevColor2;
                vertices[selected*3+2].color << prevColor3;
                viewer.redraw_next = true;
            }
        }

        //Actions when in delete mode
        else if(del){
            if(is_pressed){
                Eigen::Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                selected = find_nearest_triangle(cursor, vertices);
                vertices[selected*3+0].color << 0,0,1,1;
                vertices[selected*3+1].color << 0,0,1,1;
                vertices[selected*3+2].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            else{
                vertices.erase(vertices.begin()+selected*3+0,vertices.begin()+selected*3+3);
                viewer.redraw_next = true;
            }
        }

        //Actions when in edit mode and color mode
        else if(col){
            if(!is_pressed){
                Eigen::Vector4f cursor(((float(x)/float(width) * 2) - 1 - uniform.view(0,3))/uniform.view(0,0),
                     ((float(height-1-y)/float(height) * 2) - 1 - uniform.view(1,3))/uniform.view(1,1), 0, 1);
                vertex_selected = find_nearest_point(cursor, vertices);
                vertices[vertex_selected].color << 0,0,1,1;
                viewer.redraw_next = true;
                numpad = true;
            }
        }
    };

    viewer.mouse_wheel = [&](int dx, int dy, bool is_direction_normal) {
    };

    viewer.key_pressed = [&](char key, bool is_pressed, int modifier, int repeat) {
        
        // Events/Action needed when different keys are pressed
        switch (key)
        {
        case 'i':
            ins = true;
            edt = false;
            del = false;
            col = false;
            break;
        case 'o':
            ins = false;
            edt = true;
            del = false;
            col = false;
            break;
        case 'p':
            ins = false;
            edt = false;
            del = true;
            col = false;
            break;
        case 'c':
            ins = false;
            edt = false;
            del = false;
            col = true;
            break;
        case '1':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 1,0,1,1;
                viewer.redraw_next = true;
            }
            break;
        case '2':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 0,1,1,1;
                viewer.redraw_next = true;
            }
            break;
        case '3':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 1,1,0,1;
                viewer.redraw_next = true;
            }
            break;
        case '4':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 0,0,1,1;
                viewer.redraw_next = true;
            }
            break;
        case '5':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 0,1,0,1;
                viewer.redraw_next = true;
            }
            break;
        case '6':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 1,0,0,1;
                viewer.redraw_next = true;
            }
            break;
        case '7':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 0.2,0.8,0.4,1;
                viewer.redraw_next = true;
            }
            break;
        case '8':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 0.1,0.4,0.7,1;
                viewer.redraw_next = true;
            }
            break;
        case '9':
            if(!is_pressed && col && numpad){
                vertices[vertex_selected].color << 0.5,0.2,0.2,1;
                viewer.redraw_next = true;
            }
            break;
        case 'h':
            if(!is_pressed && edt && selected > -1){
                vertices[selected*3+0].degree++;
                vertices[selected*3+1].degree++;
                vertices[selected*3+2].degree++;
                viewer.redraw_next = true;
            }
            break;
        case 'j':
            if(!is_pressed && edt && selected > -1){
                vertices[selected*3+0].degree--;
                vertices[selected*3+1].degree--;
                vertices[selected*3+2].degree--;
                viewer.redraw_next = true;
            }
            break;
        case 'k':
            if(!is_pressed && edt && selected > -1){
                vertices[selected*3+0].scale++;
                vertices[selected*3+1].scale++;
                vertices[selected*3+2].scale++;
                viewer.redraw_next = true;
            }
            break;
        case 'l':
            if(!is_pressed && edt && selected > -1){
                vertices[selected*3+0].scale--;
                vertices[selected*3+1].scale--;
                vertices[selected*3+2].scale--;
                viewer.redraw_next = true;
            }
            break;
        case '='://+
            if(!is_pressed){
                uniform.view(0,0) *= 1.2;
                uniform.view(1,1) *= 1.2;
                viewer.redraw_next = true;
            }
            break;
        case '+':
            if(!is_pressed){
                uniform.view(0,0) *= 1.2;
                uniform.view(1,1) *= 1.2;
                viewer.redraw_next = true;
            }
            break;
        case '-':
            if(!is_pressed){
                uniform.view(0,0) *= 0.8;
                uniform.view(1,1) *= 0.8;
                viewer.redraw_next = true;
            }
            break;
        case 's':
            if(!is_pressed){
                uniform.view(1,3) += 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'd':
            if(!is_pressed){
                uniform.view(0,3) -= 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'w':
            if(!is_pressed){
                uniform.view(1,3) -= 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'a':
            if(!is_pressed){
                uniform.view(0,3) += 0.2;
                viewer.redraw_next = true;
            }
            break;
        case 'z'://K
            if(!is_pressed){
                vector<int> tempD;
                vector<float> tempS;
                vector<float> tempPx;
                vector<float> tempPy;
                for(int i=0;i<vertices.size();i++){
                    tempD.push_back(vertices[i].degree);
                    tempS.push_back(vertices[i].scale);
                    tempPx.push_back(vertices[i].del_x);
                    tempPy.push_back(vertices[i].del_y);
                }
                key_rot.push_back(tempD);
                key_scl.push_back(tempS);
                key_posx.push_back(tempPx);
                key_posy.push_back(tempPy);
            }
            break;
        case 'K':
            if(!is_pressed){
                vector<int> tempD;
                vector<float> tempS;
                vector<float> tempPx;
                vector<float> tempPy;
                for(int i=0;i<vertices.size();i++){
                    tempD.push_back(vertices[i].degree);
                    tempS.push_back(vertices[i].scale);
                    tempPx.push_back(vertices[i].del_x);
                    tempPy.push_back(vertices[i].del_y);
                }
                key_rot.push_back(tempD);
                key_scl.push_back(tempS);
                key_posx.push_back(tempPx);
                key_posy.push_back(tempPy);
            }
            break;
        case 'x'://C
            if(!is_pressed){
                key_rot.clear();
                key_scl.clear();
                key_posx.clear();
                key_posy.clear();
            }
            break;
        case 'C':
            if(!is_pressed){
                key_rot.clear();
                key_scl.clear();
                key_posx.clear();
                key_posy.clear();
            }
            break;
        case 'q'://A
            // Linear Interpolation
            if(!is_pressed){
                for(int k=1;k<key_rot.size();k++){
                    for(float t=0;t<1.1;t+=0.1){
                        for(int i=0;i<vertices.size();i++){
                            float pd0 = key_rot[k-1][i];
                            float pd1 = key_rot[k][i];
                            float pdt = pd0 + t * (pd1 - pd0);
                            vertices[i].degree = pdt;

                            float ps0 = key_scl[k-1][i];
                            float ps1 = key_scl[k][i];
                            float pst = ps0 + t * (ps1 - ps0);
                            vertices[i].scale = pst;

                            float px0 = key_posx[k-1][i];
                            float px1 = key_posx[k][i];
                            float pxt = px0 + t * (px1 - px0);
                            vertices[i].del_x = pxt;

                            float py0 = key_posy[k-1][i];
                            float py1 = key_posy[k][i];
                            float pyt = py0 + t * (py1 - py0);
                            vertices[i].del_y = pyt;
                        }
                        viewer.redraw(viewer);
                        usleep(10000);
                    }
                    
                }
            }
            break;
        case 'A':
            // Linear Interpolation
            if(!is_pressed){
                for(int k=1;k<key_rot.size();k++){
                    for(float t=0;t<1.1;t+=0.1){
                        for(int i=0;i<vertices.size();i++){
                            float pd0 = key_rot[k-1][i];
                            float pd1 = key_rot[k][i];
                            float pdt = pd0 + t * (pd1 - pd0);
                            vertices[i].degree = pdt;

                            float ps0 = key_scl[k-1][i];
                            float ps1 = key_scl[k][i];
                            float pst = ps0 + t * (ps1 - ps0);
                            vertices[i].scale = pst;

                            float px0 = key_posx[k-1][i];
                            float px1 = key_posx[k][i];
                            float pxt = px0 + t * (px1 - px0);
                            vertices[i].del_x = pxt;

                            float py0 = key_posy[k-1][i];
                            float py1 = key_posy[k][i];
                            float pyt = py0 + t * (py1 - py0);
                            vertices[i].del_y = pyt;
                        }
                        viewer.redraw(viewer);
                        usleep(10000);
                    }
                    
                }
            }
            break;
        case 'v'://B
            // Bézier curve Interpolation
            if(!is_pressed){
                for(float t=0;t<1.1;t+=0.1){
                    for(int i=0;i<vertices.size();i++){
                        int n = key_rot.size()-1;
                        float fr = 0;
                        float fs = 0;
                        float fx = 0;
                        float fy = 0;
                        for(int k=0;k<=n;k++){
                            fr += key_rot[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                            fs += key_scl[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                            fx += key_posx[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                            fy += key_posy[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                        }
                        vertices[i].degree = fr;
                        vertices[i].scale = fs;
                        vertices[i].del_x = fx;
                        vertices[i].del_y = fy;
                    }
                    viewer.redraw(viewer);
                    usleep(10000);
                }
            }
            break;
        case 'B':
            // Bézier curve Interpolation
            if(!is_pressed){
                for(float t=0;t<1.1;t+=0.1){
                    for(int i=0;i<vertices.size();i++){
                        int n = key_rot.size()-1;
                        float fr = 0;
                        float fs = 0;
                        float fx = 0;
                        float fy = 0;
                        for(int k=0;k<=n;k++){
                            fr += key_rot[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                            fs += key_scl[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                            fx += key_posx[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                            fy += key_posy[k][i] * nChoosek(n,k) * (pow(t,k)) * (pow((1-t),(n-k)));
                        }
                        vertices[i].degree = fr;
                        vertices[i].scale = fs;
                        vertices[i].del_x = fx;
                        vertices[i].del_y = fy;
                    }
                    viewer.redraw(viewer);
                    usleep(10000);
                }
            }
            break;
        
        default:
            break;
        }
    };

    viewer.redraw = [&](SDLViewer &viewer) {
        // Clear the framebuffer
        for (unsigned i=0;i<frameBuffer.rows();i++)
            for (unsigned j=0;j<frameBuffer.cols();j++)
                frameBuffer(i,j).color << 0,0,0,1;
        if(ins){
            if(v1 && !v2){
                rasterize_lines(program,uniform,temp_vertices,1,frameBuffer);
            }
            else if(v1 && v2 && !v3){
                rasterize_triangles(program,uniform,temp_vertices,frameBuffer);
            }
        }
        vector<VertexAttributes> lines;
        for(int i=0;i<vertices.size()/3;i++){
            lines.push_back(vertices[i*3+0]);
            lines.push_back(vertices[i*3+1]);
            lines.push_back(vertices[i*3+1]);
            lines.push_back(vertices[i*3+2]);
            lines.push_back(vertices[i*3+2]);
            lines.push_back(vertices[i*3+0]);
        }
        rasterize_triangles(program,uniform,vertices,frameBuffer);
        rasterize_lines(progLines,uniform,lines,1,frameBuffer);
       	

        // Buffer for exchanging data between rasterizer and sdl viewer
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> R(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> G(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> B(width, height);
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> A(width, height);

        for (unsigned i=0; i<frameBuffer.rows();i++)
        {
            for (unsigned j=0; j<frameBuffer.cols();j++)
            {
                R(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(0);
                G(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(1);
                B(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(2);
                A(i,frameBuffer.cols()-1-j) = frameBuffer(i,j).color(3);
            }
        }
        viewer.draw_image(R, G, B, A);
    };

    viewer.launch();

    return 0;
}
