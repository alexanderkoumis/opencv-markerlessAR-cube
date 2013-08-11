#include "ardrawingcontext.h"
#include <Windows.h>
#include <gl/GL.h>
#include <gl/GLU.h>

void ardrawingcontextdrawcallback(void* param)
{
	ardrawingcontext * ctx = static_cast<ardrawingcontext*>(param);
	if (ctx)
	{
		ctx->draw();
	}
}

ardrawingcontext::ardrawingcontext(string windowname, Size framesize, const cameracalibration& c) : mistextureinitialized(false), mcalibration(c), mwindowname(windowname)
{
	namedWindow(windowname, WINDOW_OPENGL);
	resizeWindow(windowname, framesize.width, framesize.height);
	setOpenGlContext(windowname);
	setOpenGlDrawCallback(windowname, ardrawingcontextdrawcallback, this);
}

ardrawingcontext::~ardrawingcontext()
{
	setOpenGlDrawCallback(mwindowname, 0, 0);
}

void ardrawingcontext::updatebackground(const cv::Mat& frame)
{
	frame.copyTo(mbackgroundimage);
}

void ardrawingcontext::updatewindow()
{
	updateWindow(mwindowname);
}

void ardrawingcontext::draw()
{
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
	drawcameraframe();
	drawaugmentedscene(); // problem
	glFlush();
}

void ardrawingcontext::drawcameraframe()
{
  // Initialize texture for background image
  if (!mistextureinitialized)
  {
    glGenTextures(1, &mbackgroundtextureid);
    glBindTexture(GL_TEXTURE_2D, mbackgroundtextureid);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    mistextureinitialized = true;
  }

  int w = mbackgroundimage.cols;
  int h = mbackgroundimage.rows;
  
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glBindTexture(GL_TEXTURE_2D, mbackgroundtextureid);

  // Upload new texture data:
  if (mbackgroundimage.channels() == 3)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, mbackgroundimage.data);
  else if(mbackgroundimage.channels() == 4)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, mbackgroundimage.data);
  else if (mbackgroundimage.channels()==1)
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, w, h, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, mbackgroundimage.data);

  const GLfloat bgTextureVertices[] = { 0, 0, w, 0, 0, h, w, h };
  const GLfloat bgTextureCoords[] = { 1, 0, 1, 1, 0, 0, 0, 1 };
  const GLfloat proj[] = { 0, -2.f/w, 0, 0, -2.f/h, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, 1 };

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(proj);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, mbackgroundtextureid);

  // Update attribute values.
  glEnableClientState(GL_VERTEX_ARRAY);
  glEnableClientState(GL_TEXTURE_COORD_ARRAY);

  glVertexPointer(2, GL_FLOAT, 0, bgTextureVertices);
  glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

  glColor4f(1,1,1,1);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

  glDisableClientState(GL_VERTEX_ARRAY);
  glDisableClientState(GL_TEXTURE_COORD_ARRAY);
  glDisable(GL_TEXTURE_2D);
}

void ardrawingcontext::drawaugmentedscene() // problem
{
  // Init augmentation projection
  matrix44 projectionMatrix;
  int w = mbackgroundimage.cols;
  int h = mbackgroundimage.rows;
  buildprojectionmatrix(mcalibration, w, h, projectionMatrix);

  glMatrixMode(GL_PROJECTION);
  glLoadMatrixf(projectionMatrix.data);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  if (ispatternpresent)
  {
    // Set the pattern transformation
    matrix44 glMatrix = patternpose.getmat44();
    glLoadMatrixf(reinterpret_cast<const GLfloat*>(&glMatrix.data[0]));

    // Render model
    drawcoordinateaxis(); // problem
    drawcubemodel();
  }
}

void ardrawingcontext::buildprojectionmatrix(const cameracalibration& calibration, int w, int h, matrix44& projectionmatrix)
{
	float nearplane = 0.01f; // near clipping difference
	float farplane = 100.0f; // far clipping difference

	//camera params
	float f_x = calibration.fx(); // focal length in x axis
	float f_y = calibration.fy(); // focal length in y axis (usually the same)
	float c_x = calibration.cx(); // camera primary point x
	float c_y = calibration.cy(); // camera primary point y

	projectionmatrix.data[0] = -2.0f * f_x / w;
	projectionmatrix.data[1] = 0.0f;
	projectionmatrix.data[2] = 0.0f;
	projectionmatrix.data[3] = 0.0f;

	
	projectionmatrix.data[4] = 0.0f;
	projectionmatrix.data[5] = 2.0f * f_y / h;
	projectionmatrix.data[6] = 0.0f;
	projectionmatrix.data[7] = 0.0f;


	projectionmatrix.data[8] = 2.0f * c_x / w - 1.0f;
	projectionmatrix.data[9] = 2.0f * c_y / h - 1.0f;
	projectionmatrix.data[10] = -(farplane + nearplane) / (farplane - nearplane);
	projectionmatrix.data[11] = -1.0f;

	projectionmatrix.data[12] = 0.0f;
	projectionmatrix.data[13] = 0.0f;
	projectionmatrix.data[14] = -2.0f * farplane * nearplane / (farplane - nearplane);
	projectionmatrix.data[15] = 0.0f;
}

void ardrawingcontext::drawcoordinateaxis()
{
	static float linex[] = {0,0,0,1,0,0};
	static float liney[] = {0,0,0,0,1,0};
	static float linez[] = {0,0,0,0,0,1};

	glLineWidth(2);

	glBegin(GL_LINES);

	glColor3f(1.0f, 0.0f, 0.0f);
	glVertex3fv(linex);
	glVertex3fv(linex + 3);

	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3fv(liney);
	glVertex3fv(liney + 3);

	glColor3f(0.0f, 0.0f, 1.0f);
	glVertex3fv(linez);
	glVertex3fv(linez + 3);

	glEnd();
}

void ardrawingcontext::drawcubemodel()
{
	static const GLfloat LightAmbient[]= { 0.25f, 0.25f, 0.25f, 1.0f }; // Ambient Light Values
	static const GLfloat LightDiffuse[]= { 0.1f, 0.1f, 0.1f, 1.0f }; // Diffuse Light Values
	static const GLfloat LightPosition[]= { 0.0f, 0.0f, 2.0f, 1.0f }; // Light Position
  
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_CURRENT_BIT | GL_ENABLE_BIT | GL_LIGHTING_BIT | GL_POLYGON_BIT);

	glColor4f(0.2f,0.35f,0.3f,0.75f); // Full Brightness, 50% Alpha ( NEW )
	glBlendFunc(GL_ONE,GL_ONE_MINUS_SRC_ALPHA); // Blending Function For Translucency Based On Source Alpha
	glEnable(GL_BLEND);

	glShadeModel(GL_SMOOTH);

	glEnable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glLightfv(GL_LIGHT1, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT1, GL_POSITION, LightPosition);
	glEnable(GL_COLOR_MATERIAL);

	glScalef(0.25,0.25, 0.25);
	glTranslatef(0,0, 1);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	glBegin(GL_QUADS);
	// Front Face
	glNormal3f( 0.0f, 0.0f, 1.0f); // Normal Pointing Towards Viewer
	glVertex3f(-1.0f, -1.0f, 1.0f); // Point 1 (Front)
	glVertex3f( 1.0f, -1.0f, 1.0f); // Point 2 (Front)
	glVertex3f( 1.0f, 1.0f, 1.0f); // Point 3 (Front)
	glVertex3f(-1.0f, 1.0f, 1.0f); // Point 4 (Front)
	// Back Face
	glNormal3f( 0.0f, 0.0f,-1.0f); // Normal Pointing Away From Viewer
	glVertex3f(-1.0f, -1.0f, -1.0f); // Point 1 (Back)
	glVertex3f(-1.0f, 1.0f, -1.0f); // Point 2 (Back)
	glVertex3f( 1.0f, 1.0f, -1.0f); // Point 3 (Back)
	glVertex3f( 1.0f, -1.0f, -1.0f); // Point 4 (Back)
	// Top Face
	glNormal3f( 0.0f, 1.0f, 0.0f); // Normal Pointing Up
	glVertex3f(-1.0f, 1.0f, -1.0f); // Point 1 (Top)
	glVertex3f(-1.0f, 1.0f, 1.0f); // Point 2 (Top)
	glVertex3f( 1.0f, 1.0f, 1.0f); // Point 3 (Top)
	glVertex3f( 1.0f, 1.0f, -1.0f); // Point 4 (Top)
	// Bottom Face
	glNormal3f( 0.0f,-1.0f, 0.0f); // Normal Pointing Down
	glVertex3f(-1.0f, -1.0f, -1.0f); // Point 1 (Bottom)
	glVertex3f( 1.0f, -1.0f, -1.0f); // Point 2 (Bottom)
	glVertex3f( 1.0f, -1.0f, 1.0f); // Point 3 (Bottom)
	glVertex3f(-1.0f, -1.0f, 1.0f); // Point 4 (Bottom)
	// Right face
	glNormal3f( 1.0f, 0.0f, 0.0f); // Normal Pointing Right
	glVertex3f( 1.0f, -1.0f, -1.0f); // Point 1 (Right)
	glVertex3f( 1.0f, 1.0f, -1.0f); // Point 2 (Right)
	glVertex3f( 1.0f, 1.0f, 1.0f); // Point 3 (Right)
	glVertex3f( 1.0f, -1.0f, 1.0f); // Point 4 (Right)
	// Left Face
	glNormal3f(-1.0f, 0.0f, 0.0f); // Normal Pointing Left
	glVertex3f(-1.0f, -1.0f, -1.0f); // Point 1 (Left)
	glVertex3f(-1.0f, -1.0f, 1.0f); // Point 2 (Left)
	glVertex3f(-1.0f, 1.0f, 1.0f); // Point 3 (Left)
	glVertex3f(-1.0f, 1.0f, -1.0f); // Point 4 (Left)
	glEnd();
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	glColor4f(0.2f,0.65f,0.3f,0.35f); // Full Brightness, 50% Alpha ( NEW )
	glBegin(GL_QUADS);
	// Front Face
	glNormal3f( 0.0f, 0.0f, 1.0f); // Normal Pointing Towards Viewer
	glVertex3f(-1.0f, -1.0f, 1.0f); // Point 1 (Front)
	glVertex3f( 1.0f, -1.0f, 1.0f); // Point 2 (Front)
	glVertex3f( 1.0f, 1.0f, 1.0f); // Point 3 (Front)
	glVertex3f(-1.0f, 1.0f, 1.0f); // Point 4 (Front)
	// Back Face
	glNormal3f( 0.0f, 0.0f,-1.0f); // Normal Pointing Away From Viewer
	glVertex3f(-1.0f, -1.0f, -1.0f); // Point 1 (Back)
	glVertex3f(-1.0f, 1.0f, -1.0f); // Point 2 (Back)
	glVertex3f( 1.0f, 1.0f, -1.0f); // Point 3 (Back)
	glVertex3f( 1.0f, -1.0f, -1.0f); // Point 4 (Back)
	// Top Face
	glNormal3f( 0.0f, 1.0f, 0.0f); // Normal Pointing Up
	glVertex3f(-1.0f, 1.0f, -1.0f); // Point 1 (Top)
	glVertex3f(-1.0f, 1.0f, 1.0f); // Point 2 (Top)
	glVertex3f( 1.0f, 1.0f, 1.0f); // Point 3 (Top)
	glVertex3f( 1.0f, 1.0f, -1.0f); // Point 4 (Top)
	// Bottom Face
	glNormal3f( 0.0f,-1.0f, 0.0f); // Normal Pointing Down
	glVertex3f(-1.0f, -1.0f, -1.0f); // Point 1 (Bottom)
	glVertex3f( 1.0f, -1.0f, -1.0f); // Point 2 (Bottom)
	glVertex3f( 1.0f, -1.0f, 1.0f); // Point 3 (Bottom)
	glVertex3f(-1.0f, -1.0f, 1.0f); // Point 4 (Bottom)
	// Right face
	glNormal3f( 1.0f, 0.0f, 0.0f); // Normal Pointing Right
	glVertex3f( 1.0f, -1.0f, -1.0f); // Point 1 (Right)
	glVertex3f( 1.0f, 1.0f, -1.0f); // Point 2 (Right)
	glVertex3f( 1.0f, 1.0f, 1.0f); // Point 3 (Right)
	glVertex3f( 1.0f, -1.0f, 1.0f); // Point 4 (Right)
	// Left Face
	glNormal3f(-1.0f, 0.0f, 0.0f); // Normal Pointing Left
	glVertex3f(-1.0f, -1.0f, -1.0f); // Point 1 (Left)
	glVertex3f(-1.0f, -1.0f, 1.0f); // Point 2 (Left)
	glVertex3f(-1.0f, 1.0f, 1.0f); // Point 3 (Left)
	glVertex3f(-1.0f, 1.0f, -1.0f); // Point 4 (Left)
	glEnd();
 
	glPopAttrib();
}