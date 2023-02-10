#define GL_SILENCE_DEPRECATION

#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <assert.h>
#include <vector>
#include <functional>
#include <numeric>
#include <glm/glm.hpp>

#if !defined(BUFSIZE)
#define BUFSIZE 512
#endif


using namespace std;

/*
//Simple Lines
int num_points=2;
glm::vec3* points = new glm::vec3[num_points];
int num_other_points = 2;
glm::vec3* other_points = new glm::vec3[num_other_points];*/

int pick_control = 0;
int pick_knot = 0;
const int num_points=4;
const int num_other_points = 7;
glm::vec3 points[num_points+num_other_points];
glm::vec3* other_points = &points[num_points];
glm::vec3 translationVector;
int picked_pos=-1;
bool intersectionFound = false;
std::vector<glm::vec3> intersectionPoints;
std::shared_ptr<std::vector<float>> knot_points(nullptr);
std::shared_ptr<std::vector<float>> knot_other_points(nullptr);
int degree = 1;

//Helper functions
template<class T>
vector<T> getVectorForCArray(const T* array, const uint array_size)
{
    auto result =vector<T>(array_size);
    for(int i=0; i<array_size; ++i)
    {
        result[i] = array[i];
    }
    return result;
}

float bendingIndicator(const glm::vec3* p, const uint n)
{
    if(n<=2)
        return 0;
    vector<float> bends(n-2);
    for(int i = 1; i<n-1; ++i)
    {
        bends[i] = glm::length((p[i+1]-p[i])-(p[i]-p[i-1]));
        //cout << k << ": " << bends[i] << endl;
    }
    return *max_element(bends.begin(),bends.end());
}

void line_intersection(const glm::vec3& a_start, const glm::vec3& a_end, const glm::vec3& b_start, const glm::vec3& b_end)
{
    if(a_start.z!=0 || b_start.z!=0 || a_end.z!=0 || b_end.z!=0) //2D only
        return;
    
    auto a_dir = a_end-a_start;
    auto b_dir = b_end-b_start;
    
    float det = a_dir.y * b_dir.x-a_dir.x * b_dir.y;
    // if det is 0, the directional vectors are colinear
    if (det == 0)
        return;

    float t1 = b_dir.y * (a_start.x-b_start.x) + b_dir.x * (b_start.y-a_start.y);
    t1 /= det;
    
    float t2 = a_dir.y * (b_start.x-a_start.x) + a_dir.x * (a_start.y-b_start.y);
    t2 /= -det;
    
    if (t1 < 0 || t1 > 1 || t2 < 0 || t2 > 1)
        return;

    auto p = a_start+t1*a_dir;
    //cout << "Intersection found!" << endl;
    //cout << "(" << p.x << ", " << p.y << ", " << 0 << ")" << endl;
    intersectionFound = true;
    intersectionPoints.push_back(p);
}

bool self_intersection_possible(const vector<glm::vec3>& bezierPolygon)
{
    if(bezierPolygon.size()<=3)
        return false;
    
    vector<glm::vec3> tangents;
    for(int i = 0; i< bezierPolygon.size()-1; ++i)
    {
        tangents.push_back(bezierPolygon[i+1]-bezierPolygon[i]);
    }
    double angle = 0.0;
    for(int i=0; i<tangents.size(); ++i)
    {
        for(int j=0; j<tangents.size(); ++j)
        {
            if(i>j)
            {
                double result = atan2( tangents[i].x*tangents[j].y-tangents[i].y*tangents[j].x, tangents[i].x*tangents[j].x+tangents[i].y*tangents[j].y);
                //cout << "result:" <<result <<endl;
                result += M_PI;
                angle = max(angle,result);
            }
        }
    }
    //cout << "angle: " << angle << endl;
    return angle > M_PI;
}

//Bezier functions

const vector<vector<glm::vec3>> deCasteljau(const glm::vec3* levelZero, const uint n,float t)
{
    vector<vector<glm::vec3>> triangleMat(n);
    triangleMat[0] = getVectorForCArray(levelZero, n);
    
    auto calcNextLevel = [&t](const vector<glm::vec3>& currentLevel, const uint currentSize)
    {
        vector<glm::vec3> nextLevel(currentSize-1);
        for(int i = 0; i<currentSize-1; ++i)
        {
            nextLevel[i] = (1-t)*currentLevel[i] + t*currentLevel[i+1];
        }
        return nextLevel;
    };
    
    for(int i=1; i<n; ++i)
    {
        triangleMat[i] = calcNextLevel(triangleMat[i-1], n-(i-1));
    }
    
    return triangleMat;
}

void plotBezier(const glm::vec3* p, const uint n, const uint k, const float eps)
{
    //std::cout << "Plotting Bezier with p: " << p << ", n: " << n << ", k: " << k << ", eps: " << eps << std::endl;
    if(k==0 || bendingIndicator(p, n) < eps)
    {
        glBegin(GL_LINE_STRIP);
        glColor3f(1,1,0);
        for(int i = 0; i<n; ++i)
        {
            //std::cout << "p(" << p[i].x << ", " << p[i].y << ", " << p[i].z << ") for i: " << i << endl;
            glVertex3f(static_cast<GLfloat>(p[i].x), static_cast<GLfloat>(p[i].y), static_cast<GLfloat>(p[i].z));
        }
        glEnd();
    }
    else
    {
        auto triangleMat = deCasteljau(p, n, 0.5);
        
        glm::vec3 firstPoly[n];
        glm::vec3 secPoly[n];
        for(int i = 0; i<n; ++i)
        {
            firstPoly[i] = triangleMat[i][0];
            secPoly[i]   = triangleMat[i][n-(i+1)];
        }
        
        /*for(int i=0; i<n; ++i)
        {
            std::cout << "firstpoly(" << firstPoly[i].x << ", " << firstPoly[i].y << ", " << firstPoly[i].z << ") for i: " << i  << endl;
        }
        std::cout << endl;
        for(int i=0; i<n; ++i)
        {
            std::cout << "secpoly(" << secPoly[i].x << ", " << secPoly[i].y << ", " << secPoly[i].z << ") for i: " << i << endl;
        }
        std::cout << endl;*/
        
        plotBezier(firstPoly, n, k-1, eps);
        plotBezier(secPoly, n, k-1, eps);
        
    }
}

void bezierIntersect(const glm::vec3* b_arr,const uint b_size, const glm::vec3* c_arr, const uint c_size, const uint k, const float eps, bool self_i_possible = true)
{
    auto ext = [](const vector<glm::vec3>& poly, std::function<const double(const double&, const double&)> func)
    {
        auto result = poly[0];
        for(int i = 1; i < poly.size(); ++i)
        {
            result.x = func(result.x, poly[i].x);
            result.y = func(result.y, poly[i].y);
            result.z = func(result.z, poly[i].z);
        }
        return result;
    };
    
    auto isOverlapping = [](const glm::vec3& a_min, const glm::vec3& a_max, const glm::vec3& b_min, const glm::vec3& b_max)
    {
        return (
                a_max.x >= b_min.x && b_max.x >= a_min.x
            &&
                a_max.y >= b_min.y && b_max.y >= a_min.y
            &&
                a_max.z >= b_min.z && b_max.z >= a_min.z
        );
    };
    
    auto min_wrapper = [](const double& a, const double& b){return std::min(a,b);};
    auto max_wrapper = [](const double& a, const double& b){return std::max(a,b);};
    
    auto b = getVectorForCArray(b_arr, b_size);
    auto c = getVectorForCArray(c_arr, c_size);
    
    if(isOverlapping(ext(b,min_wrapper),ext(b, max_wrapper),ext(c, min_wrapper), ext(c, max_wrapper)))
    {
        if(k==0 || b_size*(b_size-1)*bendingIndicator(b_arr, b_size) > eps)
        {
            //std::cout << "Smoothing b" << std::endl;
            auto triangleMat = deCasteljau(b_arr, b_size, 0.5);
            glm::vec3 firstPoly[b_size];
            glm::vec3 secPoly[b_size];
            for(int i = 0; i<b_size; ++i)
            {
                firstPoly[i] = triangleMat[i][0];
                secPoly[i]   = triangleMat[i][b_size-(i+1)];
            }
            
            self_i_possible = self_i_possible && self_intersection_possible(b);
            if(self_i_possible)
            {
                //cout << "Self intersection possible for b" << endl;
                bezierIntersect(firstPoly, b_size, secPoly, b_size, k-1, eps);
                self_i_possible = false;
            }
            bezierIntersect(firstPoly, b_size, c_arr, c_size, k-1, eps, self_i_possible);
            bezierIntersect(secPoly, b_size, c_arr, c_size, k-1, eps, self_i_possible);
            
        }
        else if (k==0 || c_size*(c_size-1)*bendingIndicator(c_arr, c_size) > eps)
        {
            //std::cout << "Smoothing c" << std::endl;
            auto triangleMat = deCasteljau(c_arr, c_size, 0.5);
            glm::vec3 firstPoly[c_size];
            glm::vec3 secPoly[c_size];
            for(int i = 0; i<c_size; ++i)
            {
                firstPoly[i] = triangleMat[i][0];
                secPoly[i]   = triangleMat[i][c_size-(i+1)];
            }
            
            self_i_possible = self_i_possible && self_intersection_possible(b);
            if(self_i_possible)
            {
                //cout << "Self intersection possible for c" << endl;
                bezierIntersect(firstPoly, c_size, secPoly, c_size, k-1, eps);
                self_i_possible=false;
            }
            bezierIntersect(b_arr, b_size, firstPoly, c_size, k-1, eps, self_i_possible);
            bezierIntersect(b_arr, b_size, secPoly, c_size, k-1, eps, self_i_possible);
        }
        else
        {
            //cout << "Candidate for intersection found" << endl;
            for(int i = 0; i< b_size-1; ++i)
            {
                for(int j = 0; j< c_size-1; ++j)
                {
                    if(b[i] != c[j] && b[i+1] != c[j+1] )
                        line_intersection(b[i], b[i+1], c[j], c[j+1]);
                }
            }
        }
    }
    /*else
    {
        cout << "Bounding Box does not intersect!" << endl;
    }*/
}

//BSpline functions

glm::vec3 deBoor(const float t, const std::vector<glm::vec3>& c, const uint p, const std::vector<float>& T, const uint k)
{
    std::vector<glm::vec3> d;
    for(int i=0; i<p+1; ++i)
    {
            d.push_back(c[i+k-p]);
    }
    //cout << "Used controlpoints: [" << k-p << ", " << k << "]" << endl;
    
    for(int r=1; r<=p; ++r)
    {
        for(int j=p; j>r-1; --j)
        {
            float alpha = (t-T[j+k-p]) / ((float)T[j+1+k-r]-T[j+k-p]);
            /*if(j==p && r==p)
                cout << "Alpha for t=" << t << ", k=" << k <<": " << alpha << endl;*/
            d[j] = ((float)1.0-alpha) * d[j-1] + alpha * d[j];
        }
    }
    //cout << "Result: (" << d[p].x << ", " << d[p].y << ")" << endl;
    return d[p];
}

void plotBSpline(const glm::vec3* c, const uint n, const uint p, float stepsize)
{
    auto createUniformKnots = [](const int p, const int n) -> std::vector<float>
    {
        std::vector<float> T;
        for(int i=0; i<p; ++i)
        {
            T.push_back(0);
        }
        for(int i=0; i<n-(p-1); ++i)
        {
            T.push_back(i);
        }
        for(int i=0; i<p; ++i)
        {
            T.push_back(*std::max_element(T.begin(), T.end()));
        }

        return T;
    };
    
    auto getIntervalIndex = [](float t, const std::vector<float>& T)
    {
        for(int i=0; i <T.size()-1; ++i)
        {
            if(t>=T[i] && t<T[i+1])
                return i;
        }
        for(int i=0; i<T.size(); ++i)
        {
            if(t==T[i])
                return i;
        }
        return -1;
    };
    
    glBegin(GL_LINE_STRIP);
    glColor3f(0,1,0);
    const auto c_vec = getVectorForCArray(c, n);
    std::vector<float> knots;
    //const std::vector<int> knots = {0,0,0,1,2,2,2};
    if(c == points)
    {
        if(knot_points==nullptr)
            knot_points = make_shared<std::vector<float>>(createUniformKnots(p,n));
        
        knots = *knot_points;
    }
    if(c == other_points)
    {
        if(knot_other_points == nullptr)
            knot_other_points = make_shared<std::vector<float>>(createUniformKnots(p,n));
        
        knots = *knot_other_points;
    }
    
    for(float s = (*std::min_element(knots.begin(), knots.end()));
        s < (*std::max_element(knots.begin(), knots.end())); s+=stepsize)
    {
        auto result = deBoor(s, c_vec, p, knots, getIntervalIndex(s, knots));
        glVertex3f(static_cast<GLfloat>(result.x), static_cast<GLfloat>(result.y), static_cast<GLfloat>(result.z));
    }
    
    glEnd();
}

void drawCurve(GLenum mode)
{
    float norm_ps = 8.0;
    float pick_ps = 12.0;
    
    //first Curve
	for(int i=0;i<num_points;i++)
	{
	  if(mode==GL_SELECT)
	    glLoadName(i);
	  
        glPointSize(norm_ps);
        glBegin(GL_POINTS);
        
        glColor3f(1,1,1);
        if(points[pick_control]==points[i])
        {
            glEnd();
            glPointSize(pick_ps);
            glBegin(GL_POINTS);
        }
            
      glVertex3f((GLfloat)points[i].x,(GLfloat)points[i].y,(GLfloat)points[i].z);
	  glEnd();
	}
    
    if(mode==GL_SELECT)
    {
        glPopName();
    }

	glBegin(GL_LINE_STRIP);
    glColor3f(1,1,1);
	for(int i=0;i<num_points;i++)
	{
        glVertex3f((GLfloat)points[i].x,(GLfloat)points[i].y,(GLfloat)points[i].z);
	}
	glEnd();
    
    plotBezier(points, num_points, 6, 0.01);
    plotBSpline(points, num_points, std::min(degree,num_points-1), 0.01);
    
    auto point_vec = getVectorForCArray(points, num_points);
    auto knot_vec = *knot_points;
    auto map_knot_points = [&point_vec,&knot_vec](float kp){
        auto minmax = std::minmax_element(point_vec.begin(), point_vec.end(),
            [](const glm::vec3& first, const glm::vec3& sec){
            return first.x < sec.x;
        });
        float min = minmax.first->x;
        float max = minmax.second->x;
        
        //int max_k = *std::max_element(knot_vec.begin(), knot_vec.end());
        int deg = std::min(degree,(static_cast<int>(point_vec.size())-1));
        float n = knot_vec.size()-(2.0*deg)-1;
        
        return kp*((max-min)/n) + min;
    };
    
    auto min = *std::min_element(point_vec.begin(), point_vec.end(),
                                [](const glm::vec3& first, const glm::vec3& sec){
        return first.y < sec.y;
    });
    
    for(auto it : knot_vec)
    {
        glPointSize(norm_ps);
        glBegin(GL_POINTS);
        glColor3f(0,1,0);
        
        if(pick_knot < knot_vec.size() && it==knot_vec[pick_knot])
        {
            glEnd();
            glPointSize(pick_ps);
            glBegin(GL_POINTS);
        }
        
        //std::cout<< map_knot_points(it)<<std::endl;
        glVertex3d(map_knot_points(it), min.y , 0);
        glEnd();
    }
    
    //Second curve
    for(int i=0;i<num_other_points;i++)
    {
        if(mode==GL_SELECT)
            glLoadName(i+num_points);
        
        glPointSize(norm_ps);
        glBegin(GL_POINTS);
        glColor3f(1,1,1);
        if(points[pick_control]==other_points[i])
        {
            glEnd();
            glPointSize(pick_ps);
            glBegin(GL_POINTS);
            
        }
        glVertex3f((GLfloat)other_points[i].x,(GLfloat)other_points[i].y,(GLfloat)other_points[i].z);
        glEnd();
    }
    
    if(mode==GL_SELECT)
    {
        glPopName();
    }

    glBegin(GL_LINE_STRIP);
    
    for(int i=0;i<num_other_points;i++)
    {
       glVertex3f((GLfloat)other_points[i].x,(GLfloat)other_points[i].y,(GLfloat)other_points[i].z);
    }
    glEnd();
    
    plotBezier(other_points, num_other_points, 6, 0.01);
    plotBSpline(other_points, num_other_points, std::min(degree,num_other_points-1), 0.01);
    
    
    point_vec = getVectorForCArray(other_points, num_other_points);
    knot_vec = *knot_other_points;
    
    min = *std::min_element(point_vec.begin(), point_vec.end(),
                                [](const glm::vec3& first, const glm::vec3& sec){
        return first.y < sec.y;
    });
    
    for(auto it : knot_vec)
    {
        glPointSize(norm_ps);
        glBegin(GL_POINTS);
        glColor3f(0,1,0);
        
        
        if(pick_knot>=knot_points->size() && it == knot_vec[pick_knot-knot_points->size()])
        {
            glEnd();
            glPointSize(pick_ps);
            glBegin(GL_POINTS);
        }
        
        glVertex3d(map_knot_points(it), min.y , 0);
        glEnd();
    }
    
    if(intersectionFound)
    {
        glPointSize(norm_ps);
        glBegin(GL_POINTS);
        for(auto const & it : intersectionPoints)
        {
            glColor3f(1,1,0);
            glVertex3f((GLfloat)it.x,(GLfloat)it.y,(GLfloat)it.z);
        }
        glEnd();
    }
}


int processHits (GLint hits, GLuint buffer[])
{
   //std::cout << "Process hits func" << std::endl;
   unsigned int i, j;
   //GLuint ii, jj;
   GLuint names, *ptr;
   int result=-1;  

   ptr = (GLuint *) buffer;
   for (i = 0; i < hits; i++) { //  for each hit
      names = *ptr;

      ptr++;
      ptr++; 
      ptr++;
      
      for (j = 0; j < names; j++) { //  for each name
	  printf ("%d ", *ptr);
      
	 result=(int)*ptr;
	 
         ptr++;
      }
      printf ("\n");
      
   }
   return result;
}




int pickPoints(int x, int y)
{
    //std::cout << "Pickpoints method" << std::endl;
	GLuint selectBuf[BUFSIZE];
	GLint hits;
	GLint viewport[4];

	glGetIntegerv (GL_VIEWPORT, viewport);
	glSelectBuffer (BUFSIZE, selectBuf);
	(void) glRenderMode(GL_SELECT);
        glInitNames();
        glPushName(0);

    glMatrixMode( GL_PROJECTION );
	glPushMatrix();
	glLoadIdentity();
      
	gluPickMatrix ((GLdouble) x, (GLdouble) (viewport[3] - y),8.0, 8.0, viewport);
	gluPerspective(60.0,(GLfloat) viewport[2] / (GLfloat) viewport[3], 1.0,20.0);
	drawCurve(GL_SELECT);

	glMatrixMode( GL_PROJECTION );
	glPopMatrix();	
	    
	hits = glRenderMode (GL_RENDER);
	int result = processHits (hits, selectBuf);
	
	return result;
}

void mousePress(int button, int state, int x, int y)
{
    //std::cout << "Mousepress listener" << std::endl;
    if((button == GLUT_LEFT_BUTTON)&&(state == GLUT_DOWN)) // && (glutGetModifiers()==GLUT_ACTIVE_CTRL))
       picked_pos=pickPoints(x,y);

    if((button == GLUT_LEFT_BUTTON)&&(state == GLUT_UP))
       picked_pos=-1;

    glutPostRedisplay();
}

void mouseMove(int x, int y)
{
     //std::cout << "Mousemove listener" << std::endl;
     GLint viewport[4];
     glGetIntegerv(GL_VIEWPORT, viewport);
     //GLfloat w=(GLfloat)viewport[2];
     //GLfloat h=(GLfloat)viewport[3];

     GLint new_pos_x=x;
     GLint new_pos_y=viewport[3]-y;

     GLdouble cpm[16];
     glGetDoublev(GL_PROJECTION_MATRIX, cpm);
	
     GLdouble cmvm[16];
     glGetDoublev(GL_MODELVIEW_MATRIX, cmvm);
	
     GLdouble objx, objy, objz;
     GLfloat z;	

     glReadPixels((GLdouble)new_pos_x,(GLdouble)new_pos_y,1, 1, GL_DEPTH_COMPONENT, GL_FLOAT,&z);
     gluUnProject((GLdouble)new_pos_x,(GLdouble)new_pos_y,z,cmvm,cpm,viewport,&objx,&objy,&objz);
    
    if(picked_pos >= 0)
    {
        intersectionFound=false;
        intersectionPoints.clear();
        
    }
    if(picked_pos>=0 && picked_pos < num_points)
        points[picked_pos]=glm::vec3((double)objx,(double)objy,(double)objz);
    else if(picked_pos >= num_points && picked_pos < num_other_points)
        other_points[picked_pos]=glm::vec3(static_cast<double>(objx), static_cast<double>(objy), static_cast<double>(objz));

     glutPostRedisplay();
}


void display(void)
{
    //std::cout << "Display listener" << std::endl;
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glColor3f (1.0, 1.0, 1.0);
    drawCurve(GL_RENDER);
    glutSwapBuffers();
}

static void initalizePoints() {
    points[0]=glm::vec3(-1.0,-0.5,0.0);
    points[1]=glm::vec3(-1.5,2.5,0.0);// swap these
    points[2]=glm::vec3(1.5,2.5,0.0);//  for self intersection
    points[3]=glm::vec3(1.0,-0.5,0.0);
    //points[4]=glm::vec3(1.0,-1.0,0.0);
    //points[5]=glm::vec3(1.5,-1.5,0.0);
    
    other_points[0]=glm::vec3(-3.0,-1.5,0.0);
    other_points[1]=glm::vec3(-3.0,-0.5,0.0);
    other_points[2]=glm::vec3(-2.0,0.0,0.0);
    other_points[3]=glm::vec3(1.0,0.5,0.0);
    other_points[4]=glm::vec3(1.0,1.5,0.0);
    other_points[5]=glm::vec3(1.5,2.0,0.0);
    other_points[6]=glm::vec3(2.0,2.0,0.0);
    
    /*
    //Simple Lines
    points[0]=glm::vec3(-1.0,0.0,0.0);
    points[1]=glm::vec3(1.0,0.0,0.0);
    other_points[0]=glm::vec3(0.0,-1.0,0.0);
    other_points[1]=glm::vec3(0.0,1.0,0.0);
    */
}

void init(void)
{
    //std::cout << "Init" << std::endl;
    glClearColor(0.0,0.0,0.0,0.0);
    glShadeModel(GL_FLAT);
    
    initalizePoints();
    
}

void reshape(GLsizei w, GLsizei h)
{
    //std::cout << "Reshape listener" << std::endl;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0,(GLfloat) w / (GLfloat) h, 1.0,20.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef (0.0, 0.0, -5.0);
}

void keyboard(unsigned char key, int x, int y)
{
    //std::cout << "Keyboard listener" << std::endl;
    const float step = 0.1;
    auto moveCurve =  [step] (glm::vec3* arr, uint len, bool x, bool more)
    {
        for(int i=0; i<len; ++i)
        {
            
            int sign = more ? 1 : -1;
            if(x)
                arr[i].x = arr[i].x + (sign*step);
            else
                arr[i].y = arr[i].y + (sign*step);
        }

        intersectionFound=false;
        intersectionPoints.clear();
        knot_points = nullptr;
        knot_other_points = nullptr;

        glutPostRedisplay();
    };
    auto movePoint = [step] (int p, bool x, bool more)
    {
        int sign = more ? 1 : -1;
        if(x)
            points[p].x = points[p].x + (sign*step);
        else
            points[p].y = points[p].y + (sign*step);
        
        intersectionFound=false;
        intersectionPoints.clear();
        knot_points = nullptr;
        knot_other_points = nullptr;

        glutPostRedisplay();
    };
    auto moveKnot = [step] (int p, shared_ptr<std::vector<float>> kv, bool left)
    {
        std::vector<int> indicesToChange;
        float valToChange = kv->at(p);
        
        for(int i = 0; i<kv->size();++i)
        {
            if(kv->at(i) == valToChange)
            {
                indicesToChange.push_back(i);
            }
        }
        
        if(left)
        {
            const int min = *std::min_element(indicesToChange.begin(), indicesToChange.end());
            if(min == 0 || valToChange-step > kv->at(min-1))
            {
                for(const int idx : indicesToChange)
                {
                    kv->at(idx)-=step;
                }
            }
        }
        else
        {
            const int max = *std::max_element(indicesToChange.begin(), indicesToChange.end());
            if(max == kv->size()-1 || valToChange+step < kv->at(max+1))
            {
                for(const int idx : indicesToChange)
                {
                    kv->at(idx)+=step;
                }
            }
        }
        
        intersectionFound=false;
        intersectionPoints.clear();
        glutPostRedisplay();
    };
    
    switch(key) {
    case 'l': //Loops first curve by switching control points at index 1 & 2
        {
            auto temp = points[2];
            points[2] = points[1];
            points[1] = temp;
            
            knot_points = nullptr;
            intersectionFound=false;
            intersectionPoints.clear();
            
            glutPostRedisplay();
            break;
        }
	case 'i': //Calculates all intersections(including selfintersections) of Beziercurves and draws them
        {
            intersectionFound=false;
            intersectionPoints.clear();
            bezierIntersect(points, num_points, other_points, num_other_points, 6, 0.1);
            glutPostRedisplay();
            break;
        }
    case 'p': //picks next control point
        {
            pick_control++;
            
            if(pick_control >= num_points+num_other_points)
                pick_control = 0;
            
            glutPostRedisplay();
            break;
        }
    case 'k': //picks next knot point (note that there are multiple knots at the beginning and ending of the knot vector)
        {
            pick_knot++;
                
            if(pick_knot >= knot_points->size()+knot_other_points->size())
                pick_knot = 0;
            
            glutPostRedisplay();
            break;
        }
    case 'r'://reset drawing
        {
            for(int i=0; i<num_points; ++i)
            {
                points[i].z=0;
            }
            
            knot_points = nullptr;
            knot_other_points = nullptr;
            intersectionFound=false;
            intersectionPoints.clear();
            
            initalizePoints();
            pick_control = 0;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
        }
	case '1': //change degree of bspline curves to 1
            degree=1;
            knot_points = nullptr;
            knot_other_points = nullptr;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
	case '2': //change degree of bspline curves to 2
            degree=2;
            knot_points = nullptr;
            knot_other_points = nullptr;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
	case '3': //change degree of bspline curves to 3 (note that this is the maximum degree for the first curve)
            degree=3;
            knot_points = nullptr;
            knot_other_points = nullptr;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
    case '4': //change degree of bspline curves to 4
            degree=4;
            knot_points = nullptr;
            knot_other_points = nullptr;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
    case '5': //change degree of bspline curves to 5
            degree=5;
            knot_points = nullptr;
            knot_other_points = nullptr;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
    case '6': //change degree of bspline curves to 6
            degree=6;
            knot_points = nullptr;
            knot_other_points = nullptr;
            pick_knot = 0;
            
            glutPostRedisplay();
            break;
    case 'w': //moves picked control point up
        {
            movePoint(pick_control, false, true);
            break;
        }
    case 'W': //moves curve containing picked control point up (hold shift for uppercase)
        {
            moveCurve(pick_control<num_points?points:other_points,
                      pick_control<num_points?num_points:num_other_points,
                      false, true);
            break;
        }
    case 'a': //moves picked control point left
        {
            movePoint(pick_control,true, false);
            break;
        }
    case 'A': //moves curve containing picked control point left (hold shift for uppercase)
        {
            moveCurve(pick_control<num_points?points:other_points,
                      pick_control<num_points?num_points:num_other_points,
                      true, false);
            break;
        }
    case 's': //moves picked control point down
        {
            movePoint(pick_control,false, false);
            break;
        }
    case 'S': //moves curve containing picked control point down (hold shift for uppercase)
        {
            moveCurve(pick_control<num_points?points:other_points,
                      pick_control<num_points?num_points:num_other_points,
                      false, false);
            break;
        }
    case 'd': //moves picked control point right
        {
            movePoint(pick_control, true, true);
            break;
        }
    case 'D': //moves curve containing picked control point right (hold shift for uppercase)
        {
            moveCurve(pick_control<num_points?points:other_points,
                      pick_control<num_points?num_points:num_other_points,
                      true, true);
            break;
        }
    case 'y': //moves picked knot point left
        {
            if(pick_knot<knot_points->size())
                moveKnot(pick_knot, knot_points, true);
            else
                moveKnot(pick_knot-(int)(knot_points->size()), knot_other_points,true);
            break;
        }
    case 'x': //moves picked knot point right
        {
            if(pick_knot<knot_points->size())
                moveKnot(pick_knot, knot_points, false);
            else
                moveKnot(pick_knot-(int)(knot_points->size()), knot_other_points,false);
            break;
        }
	}
	
}

int main(int argc, char** argv)
{
    glutInit(&argc,argv);
    glutInitDisplayMode( GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH );
    glutInitWindowPosition( 0, 0 );
    glutInitWindowSize( 1024, 768 );
    glutCreateWindow( "GM Uebung SoSe 2020" );
    
    init();

    //glutMouseFunc(mousePress);
    //glutMotionFunc(mouseMove);
    glutKeyboardFunc(keyboard);
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    
    glutMainLoop();
    return 0;		
}

