#include <GLFW/glfw3.h>
#include <math.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <string>

using namespace std;

/*TODO
	DRAW NUMBERS BETTER (use the 8 clock system) (or all text)
	TAKE IN BALANCE NUMBERS AND SEED FROM OUTSIDE FILE
	COLLISION SHATTERING - The more opposed the forces, the more particles shatter off
		as in, if they are going completely opposite directions 50% of the mass is ejected
		however, if going the same direction, just faster, less is ejected
		FIGURE OUT FORMULA FOR THIS?
		
		- When does something shatter vs join?
		- How much debris? Direction of debris?
		- Total momentum before shatter should equal that before
	CHANGE BIG BANG
		so that instead of a negative gravity, particles just start with a velocity opposing the center
		maybe turn off collisions for a bit, unless shattering is implemented?

	check combined momentum of all particles? - worked with all collided
	distribute particles less randomly
	track if speed is increasing or decreasing
	on-screen text rendering
	two clusters? maybe a starting velocity

	ability to input balance settings from file, but ALSO individual points (for suns/black holes/antimatter) and maybe randomization options?

	try collision detection using a quadtree - add to it each time

	out to file instead of console
	//only show accelerating particles

	track time of acceleration, should be exact same to deceleration - to see both approaching and leaving orbit (assuming unchanging orbit - stabilized)
	consistent accel/decel is orbit - figure out what it is orbiting?

	if speed is decreasing at a faster rate
	or increasing at a faster rate - this is interesting

	//maybe do a trail?
	if a particle is so far that the force acting on it is less than the smallest possible value on a double-precision floating number
		then do some crazy math shit to figure out WHEN it returns, and then apply that instead of constant
*/

//BALANCE

int seed;
float G = 9.8f;
float COLLISION_ENERGY_RATIO = 1.0f;
double massMax = 10000000;
double massMin = 1000000;
double positionMax = 10000000;
double positionMin = -10000000;
int QUANTITY = 1000;

// VISUALS
const int WINDOW_WIDTH = 1920;
const int WINDOW_HEIGHT = 1080;
const int FRAMES_PER_SECOND = 60;
const int UPDATES_PER_SECOND = 120;
const double RADIUS_MINIMUM = 0.0015;
const double FORCE_MINIMUM = 0.00001;
double width = positionMax;
double height = positionMax;

double xOffset = 0;
double yOffset = 0;
float cameraSpeed = 0.1f;
int framesToSkip = 1;
int speedMult = 1; //Just multiplied onto G - dangerous way to speed up the simulation (dangerous if they jump past collision - though less dangerous with greater masses)
int UPF = 0;
bool stats = false;
bool paused = false;
bool labels = false;
bool orbitVisibility = false;
int updates = 0;
int frames = 0;
double age = 0;

float randRatio() {
	return static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
}

struct Particle {
	double x, y, xv, yv, xa, ya, rad;
	float red, green, blue;
	double mass;
	double vel = 0;
	int combinations;
	bool accelerating = false;
	int timeAccelerating = 0; //use this to track half the orbit time - for orbit visibility
	Particle(double _x, double _y, double _mass, double _rad) {
		x = _x;
		y = _y;
		xv = 0;
		yv = 0;
		xa = 0;
		ya = 0;
		combinations = 1;
		mass = _mass;
		rad = _rad;
		red = randRatio();
		green = randRatio();
		blue = randRatio();
	}
	Particle(double _x, double _y, double _xv, double _yv, double _mass, double _rad) {
		x = _x;
		y = _y;
		xv = _xv;
		yv = _yv;
		vel = sqrt((xv * xv) + (yv * yv));
		xa = 0;
		ya = 0;
		combinations = 1;
		mass = _mass;
		rad = _rad;
		red = randRatio();
		green = randRatio();
		blue = randRatio();
	}
};

class Sim {
private:
	std::vector<Particle> particles;

	void updateAcceleration() { // O(N^2)
		for (unsigned int i = 0; i < particles.size(); i++) {
			particles[i].xa = 0;
			particles[i].ya = 0;
		}
		for (unsigned int i = 0; i < particles.size(); i++) {
			for (unsigned int j = 0; j < particles.size(); j++) {
				if (i != j) {
					double xd = (particles[i].x - particles[j].x);
					double yd = (particles[i].y - particles[j].y);

					double d = sqrt((xd * xd) + (yd * yd));
					double acc = (G * particles[i].mass * particles[j].mass) / (d * d);
					double xr = xd / d;
					double yr = yd / d;

					double xa = acc * xr;
					double ya = acc * yr;

					particles[j].xa += xa / particles[j].mass;
					particles[j].ya += ya / particles[j].mass;

					//TODO F = ma , therefore acceleration is in face force / mass. not sure what's up with this
				}
			}
		}
	}

	void updateVelocity() {
		for (unsigned int i = 0; i < particles.size(); i++) {
			particles[i].xv += particles[i].xa;
			particles[i].yv += particles[i].ya;

			double vel = sqrt((particles[i].xv * particles[i].xv) + (particles[i].yv * particles[i].yv));
			particles[i].accelerating = vel > particles[i].vel;
			particles[i].vel = vel;

			//if (vel > particles[i].vel) // is accelerating
			//{
			//	if (particles[i].accelerating) // was already accelerating
			//		particles[i].timeAccelerating++;
			//	else //just swi
			//	{
			//		particles[i].accelerating = true;
			//		if (timeAccelerating) //TODO pause if timeAccelerating is not 0?
			//	}
			//}
		}
	}

	void updatePosition() {
		for (unsigned int i = 0; i < particles.size(); i++) {
			particles[i].x += particles[i].xv * speedMult;
			particles[i].y += particles[i].yv * speedMult;
		}
		
		// COLLISION DETECTION (should do something faster later, maybe a quadtree)
		for (unsigned int i = 0; i < particles.size(); i++) {
			for (unsigned int j = i + 1; j < particles.size();) {
				double x1 = particles[j].x;
				double x2 = particles[i].x;
				double y1 = particles[j].y;
				double y2 = particles[i].y;
				double D = sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
				if (D < particles[i].rad + particles[j].rad && particles[i].mass > 0 && particles[j].mass > 0) {
					//THEY INTERSECT, combine particles

					// determine position weightings and center combined mass
					double massTotal = particles[i].mass + particles[j].mass;
					double massRatioA = particles[i].mass / massTotal;
					double massRatioB = particles[j].mass / massTotal;
					double xPos = particles[i].x * massRatioA + particles[j].x * massRatioB;
					double yPos = particles[i].y * massRatioA + particles[j].y * massRatioB;
					particles[i].x = xPos;
					particles[i].y = yPos;

					// color combinations, weighted by mass
					particles[i].red = particles[i].red * float(massRatioA) + particles[j].red * float(massRatioB);
					particles[i].green = particles[i].green * float(massRatioA) + particles[j].green * float(massRatioB);
					particles[i].blue = particles[i].blue * float(massRatioA) + particles[j].blue * float(massRatioB);

					// determine momentums and combine velocities and mass

					double momentumX = particles[i].mass * particles[i].xv + particles[j].mass * particles[j].xv;
					double momentumY = particles[i].mass * particles[i].yv + particles[j].mass * particles[j].yv;
					particles[i].mass += particles[j].mass;
					particles[i].rad = sqrt(particles[i].mass);

					particles[i].xv = momentumX / particles[i].mass * COLLISION_ENERGY_RATIO;
					particles[i].yv = momentumY / particles[i].mass * COLLISION_ENERGY_RATIO;

					// reset accelerations
					particles[i].xa = 0;
					particles[i].ya = 0;
					
					particles[i].combinations += particles[j].combinations;
					particles.erase(particles.begin() + j);
				}
				else
					j++;
			}
		}
	}

public:
	void addParticle(Particle& p) {
		particles.push_back(p);
	}

	vector<Particle>* getParticles() {
		return &particles;
	}

	void update() {
		updateAcceleration();
		updateVelocity();
		updatePosition();
	}
};

Sim sim;

void DrawCircle(float cx, float cy, float r, int num_segments)
{
	//glBegin(GL_LINE_LOOP);
	glBegin(GL_TRIANGLE_FAN);

	for (int ii = 0; ii < num_segments; ii++)
	{
		float theta = 2.0f * 3.1415926f * float(ii) / float(num_segments);//get the current angle

		float x = r * cosf(theta);//calculate the x component
		float y = r * sinf(theta);//calculate the y component

		glVertex2f(x + cx, y + cy);//output vertex

	}
	glEnd();
}

void DrawParticles(Sim sim, double width, double height) {
	vector<Particle>* particles = sim.getParticles();
	for (unsigned int i = 0; i < (*particles).size(); i++) {
		if (!orbitVisibility || (orbitVisibility && sqrt((*particles)[i].xa * (*particles)[i].xa + (*particles)[i].ya * (*particles)[i].ya) > FORCE_MINIMUM)) {
			double scale = width;
			double r = (*particles)[i].rad / scale;
			//double r = (*particles)[i].rad / width;
			if (r < RADIUS_MINIMUM)
				r = RADIUS_MINIMUM;

			double x = ((*particles)[i].x + xOffset) / width;
			double y = ((*particles)[i].y + yOffset) / height;
			glColor3f((*particles)[i].red, (*particles)[i].green, (*particles)[i].blue);
			DrawCircle(float(x), float(y), float(r), 32);

			// NUMBER IT
			if (labels) {
				glColor3f(1.0f, 1.0f, 1.0f);
				glBegin(GL_LINES);

				double seperation = r * 1.5f / (i + 1);
				for (unsigned int l = 0; l < i; l++) {
					glVertex2f(float(x - seperation * (l)), float(y + r));
					glVertex2f(float(x - seperation * (l)), float(y - r * 2));
				}
				glEnd();
			}
		}
	}
}

void DescribeParticles(Sim sim) {
	vector<Particle>* particles = sim.getParticles();
	for (unsigned int i = 0; i < (*particles).size(); i++) {
		Particle p = (*particles)[i];
		double vel = sqrt((p.xv * p.xv) + (p.yv * p.yv));
		double dist = sqrt(p.x * p.x + p.y * p.y);
		double force = sqrt(p.xa * p.xa + p.ya * p.ya) * 100000;
		cout << scientific << setprecision(1) << i << " : A: " << (p.accelerating ? "Y" : "N") << " | C: " << p.combinations << " | R: " << p.rad << " | M: " << p.mass << " | S: " << vel << " | D: " << dist << " | F: " << force << endl;
	}
	cout << endl;
}

void updateConsole() {
	system("cls");
	cout << "Age: " << scientific << setprecision(1) << age << " | " << updates << " UPS | " << frames << " FPS | Seed: " << seed << " | Speed: " << fixed << log2(framesToSkip) + 1 << " & " << log2(speedMult) + 1 << " | P: " << sim.getParticles()->size() << endl;
	if (stats) DescribeParticles(sim);
}

void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_W && (action == GLFW_PRESS || action == GLFW_REPEAT))
		yOffset -= cameraSpeed * height;
	else if (key == GLFW_KEY_S && (action == GLFW_PRESS || action == GLFW_REPEAT))
		yOffset += cameraSpeed * height;
	else if (key == GLFW_KEY_A && (action == GLFW_PRESS || action == GLFW_REPEAT))
		xOffset += cameraSpeed * width;
	else if (key == GLFW_KEY_D && (action == GLFW_PRESS || action == GLFW_REPEAT))
		xOffset -= cameraSpeed * width;
	else if (key == GLFW_KEY_UP && (action == GLFW_PRESS || action == GLFW_REPEAT))
		framesToSkip *= 2;
	else if (key == GLFW_KEY_DOWN && (action == GLFW_PRESS || action == GLFW_REPEAT) && framesToSkip / 2 >= 1) //since it's int division I don't think the framesToSkip/2 >= 1 check is necessary
		framesToSkip /= 2;
	else if (key == GLFW_KEY_RIGHT && (action == GLFW_PRESS || action == GLFW_REPEAT))
		speedMult *= 2;
	else if (key == GLFW_KEY_LEFT && (action == GLFW_PRESS || action == GLFW_REPEAT) && speedMult / 2 >= 1)
		speedMult /= 2;
	else if (key == GLFW_KEY_TAB && action == GLFW_PRESS)
		stats = !stats;
	else if (key == GLFW_KEY_SPACE && action == GLFW_PRESS)
		paused = !paused;
	else if (key == GLFW_KEY_BACKSPACE && action == GLFW_PRESS)
		labels = !labels;
	else if (key == GLFW_KEY_O && action == GLFW_PRESS)
		orbitVisibility = !orbitVisibility;
	else if (key == GLFW_KEY_R && action == GLFW_PRESS)
	{
		xOffset = 0;
		yOffset = 0;
		width = positionMax;
		height = positionMax;
	}
	else if (key == GLFW_KEY_MINUS && action == GLFW_PRESS) {
		width *= 2;
		height *= 2;
		if (width < 0 || height < 0) {
			width = positionMax;
			height = positionMax;
		}
	}
	else if (key == GLFW_KEY_EQUAL && action == GLFW_PRESS && width > 100 && height > 100) {
		width /= 2;
		height /= 2;
	}

	updateConsole();
}

int main(int argc, char** argv)
{
	char fileName[] = "Arguments.txt";
	ifstream file;
	seed = static_cast<int>(time(NULL));

	file.open(fileName);
	file >> G;
	file >> COLLISION_ENERGY_RATIO;
	file >> massMin;
	file >> massMax;
	file >> positionMax;
	positionMin = positionMax * -1;
	file >> QUANTITY;
	file >> seed;
	file.close();

	std::srand(seed);
	
	//double sunMass = massMax * massMax;
	//Particle sun(0, 0, sunMass, sqrt(sunMass));
	//sim.addParticle(sun);	

	//
	//Particle sun2(positionMax / 2, positionMax / 2, sunMass, sqrt(sunMass));
	//sim.addParticle(sun2);
	//Particle sun3(positionMax / 2, positionMax / -2, sunMass, sqrt(sunMass));
	//sim.addParticle(sun3);
	//Particle sun4(positionMax / -2, positionMax / 2, sunMass, sqrt(sunMass));
	//sim.addParticle(sun4);
	//Particle sun5(positionMax / -2, positionMax / -2, sunMass, sqrt(sunMass));
	//sim.addParticle(sun5);

	for (int i = 0; i < QUANTITY; i++) {
		double mass = (randRatio() * (massMax - massMin) + massMin) * (randRatio() + 0.5f);
		double x = (randRatio() * (positionMax - positionMin) + positionMin) * (randRatio() + 0.5f);
		double y = (randRatio() * (positionMax - positionMin) + positionMin) * (randRatio() + 0.5f);
		double rad = sqrt(mass);
		Particle p(x, y, mass, rad);
		//Particle p(x, y, y > 0 ? 1 : -1, 0, mass, rad);
		sim.addParticle(p);
	}

	//// CLUSTER 1
	//int x1 = -1000000;
	//int y1 = -1000000;
	//double xv1 = 2;
	//double yv1 = 3;
	//for (int i = 0; i < QUANTITY; i++) {
	//	double mass = randRatio() * (massMax - massMin) + massMin;
	//	double x = (randRatio() * (positionMax - positionMin) + positionMin) * randRatio() + x1;
	//	double y = (randRatio() * (positionMax - positionMin) + positionMin) * randRatio() + y1;
	//	double rad = sqrt(mass);
	//	Particle p(x, y, xv1, yv1, mass, rad);
	//	sim.addParticle(p);
	//}

	//// CLUSTER 2
	//int x2 = 1000000;
	//int y2 = 1000000;
	//double xv2 = -2;
	//double yv2 = -3;
	//for (int i = 0; i < QUANTITY; i++) {
	//	double mass = randRatio() * (massMax - massMin) + massMin;
	//	double x = (randRatio() * (positionMax - positionMin) + positionMin) * randRatio() + x2;
	//	double y = (randRatio() * (positionMax - positionMin) + positionMin) * randRatio() + y2;
	//	double rad = sqrt(mass);
	//	Particle p(x, y, xv2, yv2, mass, rad);
	//	sim.addParticle(p);
	//}


	GLFWwindow* window;

	/* Initialize the library */
	if (!glfwInit())
		return -1;

	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Particle Gravity", NULL, NULL);
	glfwSetWindowAspectRatio(window, WINDOW_WIDTH, WINDOW_HEIGHT);

	if (!window)
	{
		glfwTerminate();
		return -1;
	}

	/* Make the window's context current */
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, key_callback);

	clock_t last;
	last = clock();

	/* Loop until the user closes the window */
	while (!glfwWindowShouldClose(window))
	{
		if (!paused) {
			sim.update();
			age++;
			updates++;
		}
			UPF++;

		if (UPF > framesToSkip - 1) {
			glClear(GL_COLOR_BUFFER_BIT);

			float aspect = (float)WINDOW_WIDTH / WINDOW_HEIGHT;
			glLoadIdentity();
			glOrtho(-aspect, aspect, -1, 1, -1, 1);

			DrawParticles(sim, width, height);
			/* Swap front and back buffers */
			glfwSwapBuffers(window);
			/* Poll for and process events */
			glfwPollEvents();
			frames++;
			UPF = 0;
		}

		//PER SECOND
		if ((clock() - last) / (double)(CLOCKS_PER_SEC) > 1) {
			updateConsole();
			updates = 0;
			frames = 0;
			last = clock();
		}

		
	}

	glfwTerminate();
	return 0;
}