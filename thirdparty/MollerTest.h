#ifndef MOLLER_TEST_H
#define MOLLER_TEST_H

/********************************************************/
/* AABB-triangle overlap test code                      */
/* by Tomas Akenine-M�ller                              */
/* Function: int triBoxOverlap(float boxcenter[3],      */
/*          float boxhalfsize[3],float triverts[3][3]); */
/* History:                                             */
/*   2001-03-05: released the code in its first version */
/*   2001-06-18: changed the order of the tests, faster */
/*                                                      */
/* Acknowledgement: Many thanks to Pierre Terdiman for  */
/* suggestions and discussions on how to optimize code. */
/* Thanks to David Hunt for finding a ">="-bug!         */
/********************************************************/

/*
Copyright 2020 Tomas Akenine-Möller

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files (the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT
OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <math.h>
#include <stdio.h>

#define MT_X 0
#define MT_Y 1
#define MT_Z 2

#define CROSS(dest,v1,v2) \
          dest[0]=v1[1]*v2[2]-v1[2]*v2[1]; \
          dest[1]=v1[2]*v2[0]-v1[0]*v2[2]; \
          dest[2]=v1[0]*v2[1]-v1[1]*v2[0];

#define DOT(v1,v2) (v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])

#define SUB(dest,v1,v2) \
          dest[0]=v1[0]-v2[0]; \
          dest[1]=v1[1]-v2[1]; \
          dest[2]=v1[2]-v2[2];

#define FINDMINMAX(x0,x1,x2,min,max) \
  min = max = x0;   \
  if(x1<min) min=x1;\
  if(x1>max) max=x1;\
  if(x2<min) min=x2;\
  if(x2>max) max=x2;

inline int planeBoxOverlap(float normal[3],float d, float maxbox[3])
{
    int q;
    float vmin[3],vmax[3];
    for(q=MT_X; q <= MT_Z; q++)
    {
        if(normal[q]>0.0f)
        {
            vmin[q]=-maxbox[q];
            vmax[q]=maxbox[q];
        }
        else
        {
            vmin[q]=maxbox[q];
            vmax[q]=-maxbox[q];
        }
    }
    if(DOT(normal,vmin)+d>0.0f) return 0;
    if(DOT(normal,vmax)+d>=0.0f) return 1;

    return 0;
}


/*======================== MT_X-tests ========================*/
#define AXISTEST_X01(a, b, fa, fb)             \
    p0 = a*v0[MT_Y] - b*v0[MT_Z];                    \
    p2 = a*v2[MT_Y] - b*v2[MT_Z];                    \
        if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    rad = fa * boxhalfsize[MT_Y] + fb * boxhalfsize[MT_Z];   \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_X2(a, b, fa, fb)              \
    p0 = a*v0[MT_Y] - b*v0[MT_Z];                    \
    p1 = a*v1[MT_Y] - b*v1[MT_Z];                    \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[MT_Y] + fb * boxhalfsize[MT_Z];   \
    if(min>rad || max<-rad) return 0;

/*======================== MT_Y-tests ========================*/
#define AXISTEST_Y02(a, b, fa, fb)             \
    p0 = -a*v0[MT_X] + b*v0[MT_Z];                   \
    p2 = -a*v2[MT_X] + b*v2[MT_Z];                       \
        if(p0<p2) {min=p0; max=p2;} else {min=p2; max=p0;} \
    rad = fa * boxhalfsize[MT_X] + fb * boxhalfsize[MT_Z];   \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_Y1(a, b, fa, fb)              \
    p0 = -a*v0[MT_X] + b*v0[MT_Z];                   \
    p1 = -a*v1[MT_X] + b*v1[MT_Z];                       \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[MT_X] + fb * boxhalfsize[MT_Z];   \
    if(min>rad || max<-rad) return 0;

/*======================== MT_Z-tests ========================*/

#define AXISTEST_Z12(a, b, fa, fb)             \
    p1 = a*v1[MT_X] - b*v1[MT_Y];                    \
    p2 = a*v2[MT_X] - b*v2[MT_Y];                    \
        if(p2<p1) {min=p2; max=p1;} else {min=p1; max=p2;} \
    rad = fa * boxhalfsize[MT_X] + fb * boxhalfsize[MT_Y];   \
    if(min>rad || max<-rad) return 0;

#define AXISTEST_Z0(a, b, fa, fb)              \
    p0 = a*v0[MT_X] - b*v0[MT_Y];                \
    p1 = a*v1[MT_X] - b*v1[MT_Y];                    \
        if(p0<p1) {min=p0; max=p1;} else {min=p1; max=p0;} \
    rad = fa * boxhalfsize[MT_X] + fb * boxhalfsize[MT_Y];   \
    if(min>rad || max<-rad) return 0;

inline int triBoxOverlap(float boxcenter[3],float boxhalfsize[3],float triverts[3][3])
{

    /*    use separating axis theorem to test overlap between triangle and box */
    /*    need to test for overlap in these directions: */
    /*    1) the {x,y,z}-directions (actually, since we use the AABB of the triangle */
    /*       we do not even need to test these) */
    /*    2) normal of the triangle */
    /*    3) crossproduct(edge from tri, {x,y,z}-directin) */
    /*       this gives 3x3=9 more tests */
    float v0[3],v1[3],v2[3];
    float min,max,d,p0,p1,p2,rad,fex,fey,fez;
    float normal[3],e0[3],e1[3],e2[3];

    /* This is the fastest branch on Sun */
    /* move everything so that the boxcenter is in (0,0,0) */
    SUB(v0,triverts[0],boxcenter);
    SUB(v1,triverts[1],boxcenter);
    SUB(v2,triverts[2],boxcenter);

    /* compute triangle edges */
    SUB(e0,v1,v0);      /* tri edge 0 */
    SUB(e1,v2,v1);      /* tri edge 1 */
    SUB(e2,v0,v2);      /* tri edge 2 */

    /* Bullet 3:  */
    /*  test the 9 tests first (this was faster) */
    fex = fabs(e0[MT_X]);
    fey = fabs(e0[MT_Y]);
    fez = fabs(e0[MT_Z]);
    AXISTEST_X01(e0[MT_Z], e0[MT_Y], fez, fey);
    AXISTEST_Y02(e0[MT_Z], e0[MT_X], fez, fex);
    AXISTEST_Z12(e0[MT_Y], e0[MT_X], fey, fex);

    fex = fabs(e1[MT_X]);
    fey = fabs(e1[MT_Y]);
    fez = fabs(e1[MT_Z]);
    AXISTEST_X01(e1[MT_Z], e1[MT_Y], fez, fey);
    AXISTEST_Y02(e1[MT_Z], e1[MT_X], fez, fex);
    AXISTEST_Z0(e1[MT_Y], e1[MT_X], fey, fex);

    fex = fabs(e2[MT_X]);
    fey = fabs(e2[MT_Y]);
    fez = fabs(e2[MT_Z]);
    AXISTEST_X2(e2[MT_Z], e2[MT_Y], fez, fey);
    AXISTEST_Y1(e2[MT_Z], e2[MT_X], fez, fex);
    AXISTEST_Z12(e2[MT_Y], e2[MT_X], fey, fex);

    /* Bullet 1: */
    /*  first test overlap in the {x,y,z}-directions */
    /*  find min, max of the triangle each direction, and test for overlap in */
    /*  that direction -- this is equivalent to testing a minimal AABB around */
    /*  the triangle against the AABB */

    /* test in MT_X-direction */
    FINDMINMAX(v0[MT_X], v1[MT_X], v2[MT_X], min, max);
    if(min>boxhalfsize[MT_X] || max < -boxhalfsize[MT_X]) return 0;

    /* test in MT_Y-direction */
    FINDMINMAX(v0[MT_Y], v1[MT_Y], v2[MT_Y], min, max);
    if(min>boxhalfsize[MT_Y] || max < -boxhalfsize[MT_Y]) return 0;

    /* test in MT_Z-direction */
    FINDMINMAX(v0[MT_Z], v1[MT_Z], v2[MT_Z], min, max);
    if(min>boxhalfsize[MT_Z] || max < -boxhalfsize[MT_Z]) return 0;

    /* Bullet 2: */
    /*  test if the box intersects the plane of the triangle */
    /*  compute plane equation of triangle: normal*x+d=0 */
    CROSS(normal,e0,e1);
    d=-DOT(normal,v0);  /* plane eq: normal.x+d=0 */
    if(!planeBoxOverlap(normal,d,boxhalfsize)) return 0;

    return 1;   /* box and triangle overlaps */
}
#endif