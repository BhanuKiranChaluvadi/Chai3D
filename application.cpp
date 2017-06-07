
//------------------------------------------------------------------------------
#include "chai3d.h"
#include <Eigen/Dense>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
using namespace Eigen;
//------------------------------------------------------------------------------
#ifndef MACOSX
#include "GL/glut.h"
#else
#include "GLUT/glut.h"
#endif
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
C_STEREO_DISABLED:            Stereo is disabled
C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

// maximum number of devices supported by this application
const int MAX_DEVICES = 2;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all eyes of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a light source to illuminate the eyes in the world
cSpotLight *light;

// a virtual eye
cMultiMesh* eye;

// a virtual eye
cMultiMesh* scope[MAX_DEVICES];


// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice[MAX_DEVICES];

// number of haptic devices detected
int numHapticDevices = 0;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool[MAX_DEVICES];

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelHapticRate;

// lines representing the velocity vector of each haptic device
cShapeLine* insertion_dir[MAX_DEVICES];

// indicates if the haptic simulation currently running
bool simulationRunning = false;

// indicates if the haptic simulation has terminated
bool simulationFinished = true;

// flag for using print (ON/OFF)
bool print = false;

// frequency counter to measure the simulation haptic rate
cFrequencyCounter frequencyCounter;

// information about computer screen and GLUT display window
int screenW;
int screenH;
int windowW;
int windowH;
int windowPosX;
int windowPosY;

// root resource path
string resourceRoot;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
#define RESOURCE_PATH(p)    (char*)((resourceRoot+string(p)).c_str())

//------------------------------------------------------------------------------
// Location of scope insertion points
//------------------------------------------------------------------------------

float resize = 0.5;
float convertion = 0.0174533;  // 1 degree = 0.0174533 radians
const float hole_angle = 33.9;
const double x = -0.00;  // offset determined by trail and error.
const double y = resize * cos(convertion*hole_angle);
const double z = resize * sin(convertion*hole_angle);
const cVector3d insertion_pt[2] = { cVector3d(x , y, z) , cVector3d(x , y, -z) };
cVector3d bottom_pt[2] = { cVector3d(0.0, 0.0, 0.0) , cVector3d(0.0, 0.0, 0.0) };
float depth[2] = { 0, 0 };

bool rotation[2] = { false, false };

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void resizeWindow(int w, int h);

// callback when a key is pressed
void keySelect(unsigned char key, int x, int y);

// callback to render graphic scene
void updateGraphics(void);

// callback of GLUT timer
void graphicsTimer(int data);

// function that closes the application
void close(void);

// main haptics simulation loop
void updateHaptics(void);


int main(int argc, char* argv[])
{
	//--------------------------------------------------------------------------
	// INITIALIZATION
	//--------------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Demo: 22-chrome" << endl;
	cout << "Copyright 2003-2016" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Keyboard Options:" << endl << endl;
	cout << "[1] - Texture   (ON/OFF)" << endl;
	cout << "[2] - Wireframe (ON/OFF)" << endl;
	cout << "[3] - Normals   (ON/OFF)" << endl;
	cout << "[f] - Enable/Disable full screen mode" << endl;
	cout << "[m] - Enable/Disable vertical mirroring" << endl;
	cout << "[x] - Exit application" << endl;
	cout << endl << endl;

	// parse first arg to try and locate resources
	resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);


	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

	// initialize GLUT
	glutInit(&argc, argv);

	// retrieve  resolution of computer display and position window accordingly
	screenW = glutGet(GLUT_SCREEN_WIDTH);
	screenH = glutGet(GLUT_SCREEN_HEIGHT);
	windowW = 0.8 * screenH;
	windowH = 0.5 * screenH;
	windowPosY = (screenH - windowH) / 2;
	windowPosX = windowPosY;

	// initialize the OpenGL GLUT window
	glutInitWindowPosition(windowPosX, windowPosY);
	glutInitWindowSize(windowW, windowH);

	if (stereoMode == C_STEREO_ACTIVE)
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE | GLUT_STEREO);
	else
		glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE);

	// create display context and initialize GLEW library
	glutCreateWindow(argv[0]);

#ifdef GLEW_VERSION
	// initialize GLEW
	glewInit();
#endif

	// setup GLUT options
	glutDisplayFunc(updateGraphics);
	glutKeyboardFunc(keySelect);
	glutReshapeFunc(resizeWindow);
	glutSetWindowTitle("CHAI3D");

	// set fullscreen mode
	if (fullscreen)
	{
		glutFullScreen();
	}

	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	// create a new world.
	world = new cWorld();

	// set the background color of the environment
	world->m_backgroundColor.setBlueLightSky();

	// create a camera and insert it into the virtual world
	camera = new cCamera(world);
	world->addChild(camera);

	// position and oriente the camera
	camera->set(cVector3d(2.5, 0.0, 0.0),    // camera position (eye)
		cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
		cVector3d(0.0, 0.0, 1.0));   // direction of the "up" vector

									 // set the near and far clipping planes of the camera
									 // anything in front/behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);

	// set stereo mode
	camera->setStereoMode(stereoMode);

	// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoEyeSeparation(0.02);
	camera->setStereoFocalLength(2.0);

	// set vertical mirrored display mode
	camera->setMirrorVertical(mirroredDisplay);

	// create a light source
	light = new cSpotLight(world);

	// attach light to camera
	world->addChild(light);

	// enable light source
	light->setEnabled(true);

	// position the light source
	light->setLocalPos(2.0, 0.0, -2.0);

	// define the direction of the light beam
	light->setDir(-1.0, -0.0, 1.0);

	// set uniform concentration level of light 
	light->setSpotExponent(5.0);

	// enable this light source to generate shadows
	light->setShadowMapEnabled(true);

	// set the resolution of the shadow map
	//light->m_shadowMap->setQualityLow();
	light->m_shadowMap->setQualityMedium();

	// set light cone half angle
	light->setCutOffAngleDeg(30);

	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	// get number of haptic devices
	numHapticDevices = handler->getNumDevices();

	// setup each haptic device
	for (int i = 0; i < numHapticDevices; i++)
	{
		// get access to the first available haptic device
		handler->getDevice(hapticDevice[i], i);

		// open a connection to haptic device
		hapticDevice[i]->open();

		// calibrate device (if necessary)
		hapticDevice[i]->calibrate();

		// retrieve information about the current haptic device
		cHapticDeviceInfo info = hapticDevice[i]->getSpecifications();

		// if the haptic devices carries a gripper, enable it to behave like a user switch
		hapticDevice[i]->setEnableGripperUserSwitch(true);

		// create a 3D tool and add it to the world
		tool[i] = new cToolCursor(world);
		world->addChild(tool[i]);

		// connect the haptic device to the tool
		tool[i]->setHapticDevice(hapticDevice[i]);

		// define the radius of the tool (sphere)
		double toolRadius = 0.001;

		// define a radius for the tool
		tool[i]->setRadius(toolRadius);

		// hide the device sphere. only show proxy.
		tool[i]->setShowContactPoints(false, false);

		// create a white cursor
		tool[i]->m_hapticPoint->m_sphereProxy->m_material->setBlack();

		// map the physical workspace of the haptic device to a larger virtual workspace.
		tool[i]->setWorkspaceRadius(1.0);

		// enable if objects in the scene are going to rotate of translate
		// or possibly collide against the tool. If the environment
		// is entirely static, you can set this parameter to "false"
		//tool[i]->enableDynamicObjects(true);

		// haptic forces are enabled only if small forces are first sent to the device;
		// this mode avoids the force spike that occurs when the application starts when 
		// the tool is located inside an eye for instance. 
		tool[i]->setWaitForSmallForce(true);

		// initialize tool by connecting to haptic device
		tool[i]->start();
	}



	//--------------------------------------------------------------------------
	// CREATE OBJECTS
	//--------------------------------------------------------------------------

	// read the scale factor between the physical workspace of the haptic
	// device and the virtual workspace defined for the tool
	double workspaceScaleFactor = tool[0]->getWorkspaceScaleFactor();

	// stiffness properties
	// retrieve information about the current haptic device
	cHapticDeviceInfo info = hapticDevice[0]->getSpecifications();
	double maxStiffness = info.m_maxLinearStiffness / workspaceScaleFactor;

	//--------------------------------------------------------------------------
	// CREATE "EYE"
	//--------------------------------------------------------------------------

	// create a virtual mesh
	eye = new cMultiMesh();

	// add eye to world
	world->addChild(eye);

	// load an eye file
	bool fileload;
	//fileload = eye->loadFromFile(RESOURCE_PATH("../../../resources/models/eye5/eye.obj"));
	fileload = eye->loadFromFile(RESOURCE_PATH("../resources/models/eye/eye.obj"));
	if (!fileload)
	{
	#if defined(_MSVC)
		fileload = eye->loadFromFile("../../../bin/resources/models/eye/eye.obj");
	#endif
	}
	if (!fileload)
	{
		cout << "Error - 3D Model failed to load correctly." << endl;
		close();
		return (-1);
	}

	// disable culling so that faces are rendered on both sides
	eye->setUseCulling(true);

	// resize eye to screen
	eye->scale(0.5);

	// compute collision detection algorithm
	eye->createAABBCollisionDetector(0.001);

	// define a default stiffness for the object
	eye->setStiffness(0.1 * maxStiffness, true);

	// use display list for faster rendering
	eye->setUseDisplayList(true);


	// position and orient object in scene
	eye->setLocalPos(0.0, 0.0, 0.0);
	eye->rotateAboutGlobalAxisDeg(cVector3d(0, 0, 1), -90);
	eye->rotateAboutGlobalAxisDeg(cVector3d(1, 0, 0), 90);

	cMaterial mat;
	mat.setHapticTriangleSides(true, true);
	eye->setMaterial(mat);

	// disbale haptic interaction of eye
	eye->setHapticEnabled(false);

	// display frame
	eye->setShowFrame(false);
	eye->setFrameSize(0.5);

	//--------------------------------------------------------------------------
	// CREATE Scope
	//--------------------------------------------------------------------------

	for (int i = 0; i < numHapticDevices; i++)
	{
		// create a virtual mesh
		scope[i] = new cMultiMesh();

		// add eye to world
		world->addChild(scope[i]);

		// load an eye file
		fileload = scope[i]->loadFromFile(RESOURCE_PATH("../resources/models/endoscope/endoscope.3ds"));
		if (!fileload)
		{
#if defined(_MSVC)
			fileload = scope[i]->loadFromFile("../../../bin/resources/models/endoscope/endoscope.3ds");
#endif
		}
		if (!fileload)
		{
			cout << "Error - 3D Model failed to load correctly." << endl;
			close();
			return (-1);
		}

		// disable culling so that faces are rendered on both sides
		scope[i]->setUseCulling(false);

		// resize eye to screen
		scope[i]->scale(0.2);

		// use display list for faster rendering
		scope[i]->setUseDisplayList(true);

		// location and orientaiton
		scope[i]->setLocalPos(0.0, 0.0, 0.0);

		/*
		if (i == 0)
		scope[i]->rotateAboutLocalAxisDeg(cVector3d(1.0, 0.0, 0.0), 210);
		else
		scope[i]->rotateAboutLocalAxisDeg(cVector3d(1.0, 0.0, 0.0), 30);
		*/

		// display frame
		scope[i]->setShowFrame(false);
		scope[i]->setFrameSize(0.25);

		// create small line to illustrate the orientaion of the scope
		insertion_dir[i] = new cShapeLine();
		world->addChild(insertion_dir[i]);

		if (i == 0)
			scope[i]->rotateAboutLocalAxisDeg(cVector3d(1.0, 0.0, 0.0), -90);
		else
			scope[i]->rotateAboutLocalAxisDeg(cVector3d(1.0, 0.0, 0.0), 30);

	}


	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	cFont *font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic rate of the simulation
	labelHapticRate = new cLabel(font);
	labelHapticRate->m_fontColor.setBlack();
	camera->m_frontLayer->addChild(labelHapticRate);


	//--------------------------------------------------------------------------
	// START SIMULATION
	//--------------------------------------------------------------------------

	// create a thread which starts the main haptics rendering loop
	cThread* hapticsThread = new cThread();
	hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

	// setup callback when application exits
	atexit(close);

	// start the main graphics rendering loop
	glutTimerFunc(50, graphicsTimer, 0);
	glutMainLoop();

	// exit
	return (0);
}

//------------------------------------------------------------------------------

void resizeWindow(int w, int h)
{
	windowW = w;
	windowH = h;
}

//------------------------------------------------------------------------------

void keySelect(unsigned char key, int x, int y)
{
	// option ESC: exit
	if ((key == 27) || (key == 'x'))
	{
		// exit application
		exit(0);
	}

	// option f: toggle fullscreen
	if (key == 'f')
	{
		if (fullscreen)
		{
			windowPosX = glutGet(GLUT_INIT_WINDOW_X);
			windowPosY = glutGet(GLUT_INIT_WINDOW_Y);
			windowW = glutGet(GLUT_INIT_WINDOW_WIDTH);
			windowH = glutGet(GLUT_INIT_WINDOW_HEIGHT);
			glutPositionWindow(windowPosX, windowPosY);
			glutReshapeWindow(windowW, windowH);
			fullscreen = false;
		}
		else
		{
			glutFullScreen();
			fullscreen = true;
		}
	}

	// option m: toggle vertical mirroring
	if (key == 'm')
	{
		mirroredDisplay = !mirroredDisplay;
		camera->setMirrorVertical(mirroredDisplay);
	}

	if (key == 'p')
	{
		print = !print;
		if (print)
			cout << "> Enable print         \r";
		else
			cout << "> Disable print        \r";
	}
}

//------------------------------------------------------------------------------

void close(void)
{
	// stop the simulation
	simulationRunning = false;

	// wait for graphics and haptics loops to terminate
	while (!simulationFinished) { cSleepMs(100); }

	// close haptic device
	for (int i = 0; i<numHapticDevices; i++)
	{
		hapticDevice[i]->close();
	}
}

//------------------------------------------------------------------------------

void graphicsTimer(int data)
{
	// inform the GLUT window to call updateGraphics again (next frame)
	if (simulationRunning)
	{
		glutPostRedisplay();
	}

	glutTimerFunc(50, graphicsTimer, 0);
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic rate data
	labelHapticRate->setText(cStr(frequencyCounter.getFrequency(), 0) + " Hz");

	// update position of label
	labelHapticRate->setLocalPos((int)(0.5 * (windowW - labelHapticRate->getWidth())), 15);


	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world
	camera->renderView(windowW, windowH);

	// swap buffers
	glutSwapBuffers();

	// wait until all GL commands are completed
	glFinish();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;
	int j = 0;
	// main haptic simulation loop
	while (simulationRunning)
	{
		eye->setHapticEnabled(false);
		// update frequency counter
		frequencyCounter.signal(1);

		// compute global reference frames for each eye
		world->computeGlobalPositions(true);

		// haptic deive
		cHapticDeviceInfo info = hapticDevice[0]->getSpecifications();

		// tranfromation matrix
		cTransform transformation = eye->getGlobalTransform();

		// insertion points in global frame
		cVector3d insertion_pt_global_frame, bottom_pt_global_frame;

		for (int i = 0; i < numHapticDevices; i++)
		{

			// update position and orientation of tool
			tool[i]->updateFromDevice();

			// compute interaction forces
			tool[i]->computeInteractionForces();

			// positon
			cVector3d position;
			position = tool[i]->getDeviceGlobalPos();

			// conversion to global frame
			insertion_pt_global_frame = transformation * insertion_pt[i];
			bottom_pt_global_frame = transformation * bottom_pt[i];

			/////////////////////////////////////////////////////////////////////
			// APPLY FORCES
			/////////////////////////////////////////////////////////////////////

			// variables for forces  
			cVector3d force(0, 0, 0);
			cVector3d torque(0, 0, 0);
			double gripperForce = 0.0;
			double Kp = 50; // [N/m]

			cVector3d desiredPosition;
			cVector3d forceField;

			double distance = cDistance(transformation.getLocalPos(), position);
			cVector3d disc_proj = cProjectPointOnPlane(position, cVector3d(0.50, 0.0, 0.0), transformation.getLocalRot().getCol1());

			cVector3d outside_eye_pt;
			if (tool[i]->getUserSwitch(0) == 0)
			{
				outside_eye_pt.set(insertion_pt_global_frame.x() - bottom_pt_global_frame.x(), insertion_pt_global_frame.y() - bottom_pt_global_frame.y(), insertion_pt_global_frame.z() - bottom_pt_global_frame.z());
				outside_eye_pt = outside_eye_pt * 5;
				outside_eye_pt = bottom_pt_global_frame + outside_eye_pt;
				desiredPosition = cProjectPointOnSegment(position, bottom_pt_global_frame, 1 * outside_eye_pt);
				forceField = Kp * (desiredPosition - position);
				force.add(forceField);
			}

			else
			{
				eye->setHapticEnabled(true);
				double radius = cDistance(insertion_pt_global_frame, position);
				desiredPosition = position;
				bottom_pt[i] = cTranspose(transformation.getLocalRot())*desiredPosition;
				cVector3d totDistance;
				cVector3d innerDistance;
				cVector3d normalizedDistance;
				//if (cDistance(disc_proj, position) < 0.15 || distance >= 0.5)
				if (abs(distance) >= 0.4)
				{
					totDistance.set(-position.x(), -position.y(), -position.z());
					totDistance.normalizer(normalizedDistance);
					innerDistance = normalizedDistance * 0.4;
					force.add(1.25 * Kp * (totDistance - innerDistance));
				}
				if (position.x() > 0.34)
				{
					force.add(1.25*Kp*(cVector3d(0.34, 0, 0) - cVector3d(position.x(), 0, 0)));
				}
			}

			// send computed force, torque, and gripper force to haptic device
			hapticDevice[i]->setForceAndTorqueAndGripperForce(force, torque, gripperForce);


			/////////////////////////////////////////////////////////////////////
			// UPDATE SCOPE POSITION AND ORIENTAITON
			/////////////////////////////////////////////////////////////////////

			// Update position of the scope
			scope[i]->setLocalPos(desiredPosition);

			// Update rotation of the scope
			cMatrix3d scope_global_rot = scope[i]->getGlobalRot();
			cVector3d scope_local_col0 = scope[i]->getGlobalRot().getCol0();

			cVector3d align_vector = (insertion_pt_global_frame - bottom_pt_global_frame);
			align_vector.normalize();

			Eigen::Quaterniond quater;
			Vector3d align(align_vector.x(), align_vector.y(), align_vector.z());
			Vector3d scope_col_0(scope_local_col0.x(), scope_local_col0.y(), scope_local_col0.z());
			quater = quater.setFromTwoVectors(scope_col_0, align);
			quater.normalize();

			// Create rotation matrix from the quaternion
			Matrix3f quater_mat = quater.toRotationMatrix().cast<float>();

			cMatrix3d rot_mat;
			rot_mat.set(quater_mat(0, 0), quater_mat(0, 1), quater_mat(0, 2),
				quater_mat(1, 0), quater_mat(1, 1), quater_mat(1, 2),
				quater_mat(2, 0), quater_mat(2, 1), quater_mat(2, 2));

			cMatrix3d final_rotation_mat;
			scope_global_rot.mulr(rot_mat, final_rotation_mat);
			scope[i]->setLocalRot(final_rotation_mat);

		}
	}

	// exit haptics thread
	simulationFinished = true;
}

//------------------------------------------------------------------------------



