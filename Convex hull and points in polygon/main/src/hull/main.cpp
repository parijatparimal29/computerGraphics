////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	// TODO
	double x1 = u.real(), y1 = u.imag();
	double x2 = v.real(), y2 = v.imag();
	double distance = ((y2-y1)*(y2-y1)) + ((x2-x1)*(x2-x1));
	return distance;
}

bool inline salientAngle(Point &a, Point &b, Point &c) {
	// TODO
	double x1 = a.real(), y1 = a.imag();
	double x2 = b.real(), y2 = b.imag();
	double x3 = c.real(), y3 = c.imag();
	double direction = ((y2-y1) * (x3-x2)) - ((y3-y2) * (x2-x1));
	if(direction<0){
		return true;
	}
	if(direction==0){
		if(det(a,c)>det(a,b)){
			return true;
		}
	}
	return false;
}

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		// TODO
		Point x = p1;
		Point y = p2;
		if(salientAngle(p0,x,y)){
			//p0 = p1;
			return true;
		}
		//p0=p2;
		return false;
	}
};




////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	/*Polygon small;
	for(int i=0;i<1000;i++){
		small.push_back(points[i]);
	}
	*/
	Compare order;
	// TODO
	int min = 0;
	for(int i=1;i<points.size();i++){
		if(points[i].imag()<points[min].imag()){
			min=i;
		}
		else if(points[i].imag()==points[min].imag()){
			if(points[i].real()<points[min].real()){
				min=i;
			}
		}
	}
	order.p0 = points[min];
	Point temp = points[0];
	points[0] = points[min];
	points[min] = temp;
	std::sort(points.begin()+1, points.end(), order);
	Polygon hull;
	//hull = points;
	// TODO
	//points.push_back(points[0]);
	std::stack<Point> selected;
	selected.push(points[0]);
	selected.push(points[1]);
	selected.push(points[2]);
	//selected.push(points[2]);
	
	for(int i=3;i<points.size();i++){
		Point curr = selected.top();
		selected.pop();
		if(selected.empty()){
			selected.push(curr);
			selected.push(points[i]);
			continue;
		}
		Point prev = selected.top();
		selected.push(curr);
		while(!salientAngle(prev,curr,points[i])){
			selected.pop();
			if(selected.empty()){
				break;
			}
			curr = selected.top();
			selected.pop();
			if(selected.empty()){
				selected.push(curr);
				break;
			}
			prev = selected.top();
			selected.push(curr);
		}
		selected.push(points[i]);
		//hull.push_back(selected.top());
	}
	
	while(!selected.empty()){
		hull.push_back(selected.top());
		selected.pop();
	}
	
	// use salientAngle(a, b, c) here
	return hull;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	// TODO - Done
	std::string temp;
	getline(in,temp);
	int n = std::stoi(temp);
	while(getline(in,temp)){
		int pos1 = temp.find(" ");
		int pos2 = (temp.substr(pos1+1)).find(" ");
		double x = std::stod(temp.substr(0,pos1));
		double y = std::stod(temp.substr(pos1+1,pos2));
		points.push_back(Point(x,y));
	}
	return points;
}

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
