// Tests for the fragment builder: voxelisation, hull extraction and the inertia tensor.
// Pure geometry, so it runs without the game.

#include "Fragment.h"
#include <cstdio>
#include <cmath>
#include <vector>

using namespace flexrevive;

// Unit cube of side `s` centred at `c`, as 12 triangles.
static void MakeBox(float sx, float sy, float sz, const float* c,
                    std::vector<float>& v, std::vector<int>& idx)
{
    const float hx=sx/2, hy=sy/2, hz=sz/2;
    const float p[8][3] = {{-hx,-hy,-hz},{hx,-hy,-hz},{hx,hy,-hz},{-hx,hy,-hz},
                           {-hx,-hy, hz},{hx,-hy, hz},{hx,hy, hz},{-hx,hy, hz}};
    for (auto& q : p) { v.push_back(q[0]+c[0]); v.push_back(q[1]+c[1]); v.push_back(q[2]+c[2]); }
    const int f[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                          {2,3,7},{2,7,6},{1,2,6},{1,6,5},{0,4,7},{0,7,3}};
    for (auto& t : f) { idx.push_back(t[0]); idx.push_back(t[1]); idx.push_back(t[2]); }
}

static int check(const char* name, bool ok) {
    printf("  [%s] %s\n", ok ? "PASS" : "FAIL", name);
    return ok ? 0 : 1;
}

int main()
{
    int fails = 0;

    { // A 20-unit cube at radius 3.5: expect a solid block of particles, centre at the origin.
        std::vector<float> v; std::vector<int> idx;
        float c[3] = {0,0,0};
        MakeBox(20,20,20,c,v,idx);
        void* a = fragment::BuildFromMesh(v.data(), 8, idx.data(), (int)idx.size(), 3.5f, 0.0f, true);
        fails += check("cube: asset built", a != nullptr);
        if (a) {
            float ctr[3]; float r=0,g=0; int n=0;
            bool ok = fragment::Describe(a, ctr, r, g, n);
            fails += check("cube: describable", ok);
            printf("        particles=%d radius=%.2f gyration=%.2f centre=(%.2f %.2f %.2f)\n", n,r,g,ctr[0],ctr[1],ctr[2]);
            // 20/3.5 -> about 5 cells per axis, so a few dozen to a couple hundred particles
            fails += check("cube: plausible particle count", n >= 27 && n <= 400);
            fails += check("cube: centre near origin", fabsf(ctr[0])<2 && fabsf(ctr[1])<2 && fabsf(ctr[2])<2);
            // corner of a 20-cube is 17.3 from centre; particles sit inside it
            fails += check("cube: radius close to the true half width", r > 8.0f && r < 12.0f);
            fails += check("cube: gyration positive", g > 0.0f);
            fragment::Destroy(a);
            fails += check("cube: destroyed and disowned", !fragment::IsOurs(a));
        }
    }

    { // Offset box: the centre of mass must follow it.
        std::vector<float> v; std::vector<int> idx;
        float c[3] = {100,-50,25};
        MakeBox(30,30,30,c,v,idx);
        void* a = fragment::BuildFromMesh(v.data(), 8, idx.data(), (int)idx.size(), 3.5f, 0.0f, true);
        float ctr[3]; float r=0,g=0; int n=0;
        bool ok = a && fragment::Describe(a, ctr, r, g, n);
        printf("        offset box: particles=%d centre=(%.1f %.1f %.1f)\n", n, ctr[0],ctr[1],ctr[2]);
        fails += check("offset box: centre tracks the mesh",
                       ok && fabsf(ctr[0]-100)<3 && fabsf(ctr[1]+50)<3 && fabsf(ctr[2]-25)<3);
        fragment::Destroy(a);
    }

    { // A flat slab and a cube of the same extent must not have the same gyration.
        std::vector<float> v1, v2; std::vector<int> i1, i2;
        float c[3] = {0,0,0};
        MakeBox(40,40,4,c,v1,i1);
        MakeBox(40,40,40,c,v2,i2);
        void* a1 = fragment::BuildFromMesh(v1.data(), 8, i1.data(), (int)i1.size(), 3.5f, 0.0f, true);
        void* a2 = fragment::BuildFromMesh(v2.data(), 8, i2.data(), (int)i2.size(), 3.5f, 0.0f, true);
        float ctr[3]; float r1=0,g1=0,r2=0,g2=0; int n1=0,n2=0;
        fragment::Describe(a1, ctr, r1, g1, n1);
        fragment::Describe(a2, ctr, r2, g2, n2);
        printf("        slab: n=%d r=%.2f g=%.2f | cube: n=%d r=%.2f g=%.2f\n", n1,r1,g1,n2,r2,g2);
        fails += check("slab has fewer particles than cube", n1 > 0 && n2 > n1);
        fails += check("slab gyration differs from cube", fabsf(g1-g2) > 0.5f);
        fragment::Destroy(a1); fragment::Destroy(a2);
    }

    { // Degenerate and hostile inputs must not build anything or crash.
        fails += check("null vertices rejected",
                       fragment::BuildFromMesh(nullptr, 10, nullptr, 0, 3.5f, 0.0f, true) == nullptr);
        std::vector<float> v; std::vector<int> idx; float c[3]={0,0,0};
        MakeBox(10,10,10,c,v,idx);
        fails += check("zero radius rejected",
                       fragment::BuildFromMesh(v.data(), 8, idx.data(), (int)idx.size(), 0.0f, 0.0f, true) == nullptr);
        // Huge extent at tiny spacing would need a grid past the 64-cell cap
        std::vector<float> big; std::vector<int> bidx;
        MakeBox(10000,10000,10000,c,big,bidx);
        fails += check("oversized grid refused",
                       fragment::BuildFromMesh(big.data(), 8, bidx.data(), (int)bidx.size(), 3.5f, 0.0f, true) == nullptr);
        fragment::Destroy(nullptr);
        fails += check("destroy(null) is safe", true);
        int dummy = 0;
        fails += check("foreign pointer not claimed", !fragment::IsOurs(&dummy));
    }

    { // A sliver thinner than the spacing still has to yield a usable chunk.
        std::vector<float> v; std::vector<int> idx; float c[3]={7,7,7};
        MakeBox(0.5f,0.5f,0.5f,c,v,idx);
        void* a = fragment::BuildFromMesh(v.data(), 8, idx.data(), (int)idx.size(), 3.5f, 0.0f, true);
        float ctr[3]; float r=0,g=0; int n=0;
        bool ok = a && fragment::Describe(a, ctr, r, g, n);
        printf("        sliver: particles=%d centre=(%.2f %.2f %.2f)\n", n, ctr[0],ctr[1],ctr[2]);
        fails += check("sliver yields at least one particle", ok && n >= 1);
        fails += check("sliver centre is the chunk", ok && fabsf(ctr[0]-7)<1);
        fragment::Destroy(a);
    }

    { // Hull and inertia tensor: the basis for settling a chunk on a face.
        std::vector<float> v; std::vector<int> idx; float c[3]={0,0,0};
        MakeBox(30,30,30,c,v,idx);
        void* a = fragment::BuildFromMesh(v.data(), 8, idx.data(), (int)idx.size(), 3.5f, 0.0f, true);
        fragment::Hull shp;
        bool ok = a && fragment::GetHull(a, shp);
        fails += check("cube: shape available", ok);
        if (ok) {
            printf("        hull points=%d radius=%.2f\n", shp.numPoints, shp.radius);
            fails += check("cube: hull has corners and faces", shp.numPoints >= 8 && shp.numPoints <= fragment::kMaxHullPoints);
            fails += check("cube: hull radius positive", shp.radius > 0.0f);
            fails += check("cube: inflation set", shp.inflate > 0.0f);

            // Every hull point must lie inside the chunk it came from.
            bool inside = true;
            for (int k = 0; k < shp.numPoints; ++k)
                for (int ax = 0; ax < 3; ++ax)
                    if (fabsf(shp.points[k*3+ax]) > 16.0f) inside = false;
            fails += check("cube: hull points lie within the mesh", inside);

            // A cube's inertia is isotropic, so the inverse tensor is near a scaled identity.
            const float d0=shp.invInertia[0], d1=shp.invInertia[4], d2=shp.invInertia[8];
            printf("        invI diag = %.6f %.6f %.6f\n", d0,d1,d2);
            fails += check("cube: inverse inertia is isotropic",
                           d0>0 && fabsf(d0-d1)/d0 < 0.15f && fabsf(d0-d2)/d0 < 0.15f);
            bool offDiagSmall = true;
            for (int k : {1,2,3,5,6,7})
                if (fabsf(shp.invInertia[k]) > 0.1f*d0) offDiagSmall = false;
            fails += check("cube: inverse inertia is near diagonal", offDiagSmall);
        }
        fragment::Destroy(a);
    }

    { // A slab must resist rotation differently about its thin axis than its wide ones.
        std::vector<float> v; std::vector<int> idx; float c[3]={0,0,0};
        MakeBox(60,60,6,c,v,idx);
        void* a = fragment::BuildFromMesh(v.data(), 8, idx.data(), (int)idx.size(), 3.5f, 0.0f, true);
        fragment::Hull shp;
        bool ok = a && fragment::GetHull(a, shp);
        if (ok) {
            const float ix=shp.invInertia[0], iy=shp.invInertia[4], iz=shp.invInertia[8];
            printf("        slab invI diag = %.6f %.6f %.6f\n", ix,iy,iz);
            // Mass is spread furthest in x and y, so the moment about z is the largest and
            // its inverse the smallest.
            fails += check("slab: spins most easily about a wide axis", ix > iz && iy > iz);
            fails += check("slab: the two wide axes match", fabsf(ix-iy)/ix < 0.2f);
        } else {
            fails += check("slab: shape available", false);
        }
        fragment::Destroy(a);
    }

    printf("\n%s (%d failure%s)\n", fails ? "FAILURES" : "ALL PASS", fails, fails==1?"":"s");
    return fails ? 1 : 0;
}
