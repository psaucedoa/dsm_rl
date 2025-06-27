#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "raylib.h"
#include "rlgl.h"

#define PASS 0
#define SPEED_UP 1
#define SPEED_DOWN 2
#define LEFT 3
#define RIGHT 4
#define BLADE_UP 5
#define BLADE_DOWN 6

#define FREE 0
#define BORDER 1

#define TIMESTEP 0.1

#include <math.h>

#define IX(i,j) ((i)+(N+2)*(j))
#define SWAP(x0,x) {float *tmp=x0;x0=x;x=tmp;}

struct FluidSquare {
    int size;
    float dt;
    float diff;
    float visc;
    
    float *s;
    float *density;
    
    float *Vx;
    float *Vy;

    float *Vx0;
    float *Vy0;
};



FluidSquare *FluidSquareCreate(int size, int diffusion, int viscosity, float dt)
{
    FluidSquare *square = malloc(sizeof(*square));
    int N = size;
    
    square->size = size;
    square->dt = dt;
    square->diff = diffusion;
    square->visc = viscosity;
    
    square->s = calloc(N * N , sizeof(float));
    square->density = calloc(N * N, sizeof(float));
    
    square->Vx = calloc(N * N, sizeof(float));
    square->Vy = calloc(N * N, sizeof(float));
    
    square->Vx0 = calloc(N * N, sizeof(float));
    square->Vy0 = calloc(N * N, sizeof(float));
    
    return square;
}

void FluidSquareFree(FluidSquare *square)
{
    free(square->s);
    free(square->density);
    
    free(square->Vx);
    free(square->Vy);
    
    free(square->Vx0);
    free(square->Vy0);
    
    free(square);
}

void add_source ( int N, float * x, float * s, float dt )
{
    int i, size=(N+2)*(N+2);
    for ( i=0 ; i<size ; i++ ) x[i] += dt*s[i];
}

void set_bnd ( int N, int b, float * x )
{
    int i;
    for ( i=1 ; i<=N ; i++ )
    {
        x[IX(0 ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
        x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
        x[IX(i,0 )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
        x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
    }

    x[IX(0 ,0 )] = 0.5*(x[IX(1,0 )]+x[IX(0 ,1)]);
    x[IX(0 ,N+1)] = 0.5*(x[IX(1,N+1)]+x[IX(0 ,N )]);
    x[IX(N+1,0 )] = 0.5*(x[IX(N,0 )]+x[IX(N+1,1)]);
    x[IX(N+1,N+1)] = 0.5*(x[IX(N,N+1)]+x[IX(N+1,N )]);
}

void diffuse (int b, float * x, float * x0, float diff, float dt, int N)
{
    int i, j, k;
    float a=dt*diff*N*N;
    for ( k=0 ; k<20 ; k++ )
    {
        for ( i=1 ; i<=N ; i++ )
        {
            for ( j=1 ; j<=N ; j++ )
            {
                x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)]+x[IX(i+1,j)]+
                x[IX(i,j-1)]+x[IX(i,j+1)]))/(1+4*a);
            }
        }
        set_bnd ( N, b, x );
    }
}

void advect (int b, float * d, float * d0, float * u, float * v, float dt, int N)
{
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0;
    dt0 = dt*N;
    for ( i=1 ; i<=N ; i++ )
    {
        for ( j=1 ; j<=N ; j++ )
        {
            x = i-dt0*u[IX(i,j)]; y = j-dt0*v[IX(i,j)];
            if (x<0.5) x=0.5; if (x>N+0.5) x=N+ 0.5; i0=(int)x; i1=i0+ 1;
            if (y<0.5) y=0.5; if (y>N+0.5) y=N+ 0.5; j0=(int)y; j1=j0+1;
            s1 = x-i0; s0 = 1-s1; t1 = y-j0; t0 = 1-t1;
            d[IX(i,j)] = s0*(t0*d0[IX(i0,j0)]+t1*d0[IX(i0,j1)])+
            s1*(t0*d0[IX(i1,j0)]+t1*d0[IX(i1,j1)]);
        }
    }
    set_bnd ( N, b, d );
}

void project (float * u, float * v, float * p, float * div, int N)
{
    int i, j, k;
    float h;
    h = 1.0/N;
    for ( i=1 ; i<=N ; i++ )
    {
        for ( j=1 ; j<=N ; j++ )
        {
            div[IX(i,j)] = -0.5*h*(u[IX(i+1,j)]-u[IX(i-1,j)]+
            v[IX(i,j+1)]-v[IX(i,j-1)]);
            p[IX(i,j)] = 0;
        }
    }
    set_bnd ( N, 0, div ); set_bnd ( N, 0, p );
    
    for ( k=0 ; k<20 ; k++ )
    {
        for ( i=1 ; i<=N ; i++ )
        {
            for ( j=1 ; j<=N ; j++ )
            {
                p[IX(i,j)] = (div[IX(i,j)]+p[IX(i-1,j)]+p[IX(i+1,j)]+
                p[IX(i,j-1)]+p[IX(i,j+1)])/4;
            }
        }
        set_bnd ( N, 0, p );
    }

    for ( i=1 ; i<=N ; i++ )
    {
        for ( j=1 ; j<=N ; j++ )
        {
            u[IX(i,j)] -= 0.5*(p[IX(i+1,j)]-p[IX(i-1,j)])/h;
            v[IX(i,j)] -= 0.5*(p[IX(i,j+1)]-p[IX(i,j-1)])/h;
        }
    }
    set_bnd ( N, 1, u ); set_bnd ( N, 2, v );
}

void dens_step ( int N, float * x, float * x0, float * u, float * v, float diff, float dt )
{
    add_source ( N, x, x0, dt );
    SWAP ( x0, x ); diffuse (0, x, x0, diff, dt, N);
    SWAP ( x0, x ); advect (0, x, x0, u, v, dt, N);
}

void vel_step ( int N, float * u, float * v, float * u0, float * v0, float visc, float dt )
{
    add_source ( N, u, u0, dt ); add_source ( N, v, v0, dt );
    SWAP ( u0, u ); diffuse (1, u, u0, visc, dt, N);
    SWAP ( v0, v ); diffuse (2, v, v0, visc, dt, N);
    project (u, v, u0, v0, N );
    SWAP ( u0, u ); SWAP ( v0, v);
    advect (1, u, u0, u0, v0, dt, N); advect (2, v, v0, u0, v0, dt, N);
    project (u, v, u0, v0, N);
}

void FluidSquareStep(FluidSquare *square)
{
    int N          = square->size;
    float visc     = square->visc;
    float diff     = square->diff;
    float dt       = square->dt;
    float *Vx      = square->Vx;
    float *Vy      = square->Vy;
    float *Vx0     = square->Vx0;
    float *Vy0     = square->Vy0;
    float *s       = square->s;
    float *density = square->density;
    
    diffuse(1, Vx0, Vx, visc, dt, N);
    diffuse(2, Vy0, Vy, visc, dt, N);
    
    project(Vx0, Vy0, Vx, Vy, N);
    
    advect(1, Vx, Vx0, Vx0, Vy0, dt, N);
    advect(2, Vy, Vy0, Vx0, Vy0, dt, N);
    
    project(Vx, Vy, Vx0, Vy0, N);
    
    diffuse(0, s, density, diff, dt, N);
    advect(0, density, s, Vx, Vy, dt, N);
}

void FluidSquareAddDensity(FluidSquare *square, int x, int y, int z, float amount)
{
    int N = square->size;
    square->density[IX(x, y)] += amount;
}

void FluidSquareAddVelocity(FluidSquare *square, int x, int y, int z, float amountX, float amountY, float amountZ)
{
    int N = square->size;
    int index = IX(x, y);
    
    square->Vx[index] += amountX;
    square->Vy[index] += amountY;
}

