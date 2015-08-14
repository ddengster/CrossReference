/*
=====================================================================================
Copyright (c) 2015 Fan LiangDeng

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgement in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
=====================================================================================
*/
#include "EngineFrontendPCH.h"
#include "cNavMesh.h"

#include "Graphics/iVertexBuffer.h"
#include "Graphics/iShader.h"
#include "Graphics/cStaticMesh.h"
#include "Graphics/cMeshConstruction.h"
#include "Graphics/cMeshQuickDraw.h"
#include "GraphicsManaged/cModelGfxCmp.h"
#include "GraphicsManaged/cTextQuickDraw.h"
#include "GraphicsManaged/cManagedRenderer.h"

#include "cUnitCollision.h"
#include "Utilities/Timer.h"
#include "Input/cMouseManager.h"
#include "Simulation/cRTSSimulation.h"
#include "Entities/cCameraEntities.h"
#include "Entities/cFloorEnt.h"
#include <AntTweakBar.h>

ENGINE_NSP

#define NAVMESH_MAX_VTX 3072
#define NAVMESH_EPSILONSQ NAVMESH_EPSILON * NAVMESH_EPSILON 

cNavMesh::cNavMesh()
{
}

cNavMesh::~cNavMesh()
{ 
  Destroy();
}

bool cNavMesh::Initialize(const Vector3& min, const Vector3& max, const char* navmeshfilename)
{
  mNavmeshFilename = navmeshfilename;

  mSectorGrid.mSrcMesh = this;
  mSectorGrid.mSectorXWidth = 4.0f;
  mSectorGrid.mSectorZHeight = 4.0f;
  mSectorGrid.mWorldMin = min;
  mSectorGrid.mWorldMax = max;
  mSectorGrid.BuildSectors();
  //initial constraint setup
  {
    mVertices.reserve(MEMORY_SIZE_3);
    mVertexSlots.mHighestId = 4;
    mVertices.resize(4);

    mVertices[0].mHandle = 0;
    mVertices[0].mPosition = min;
    mVertices[1].mHandle = 1;
    mVertices[1].mPosition = Vector3(max.x, min.y, min.z);
    mVertices[2].mHandle = 2;
    mVertices[2].mPosition = max;
    mVertices[3].mHandle = 3;
    mVertices[3].mPosition = Vector3(min.x, min.y, max.z);
    for (uint i = 0; i < 4; ++i)
      mSectorGrid.CheckIn(&mVertices[i]);

    sConstraint* boundary_cs = AddConstraint();
    boundary_cs->mPointCount = 4;
    boundary_cs->mPositions[0] = mVertices[0].mPosition;
    boundary_cs->mPositions[1] = mVertices[1].mPosition;
    boundary_cs->mPositions[2] = mVertices[2].mPosition;
    boundary_cs->mPositions[3] = mVertices[3].mPosition;
    boundary_cs->mFlags |= IS_BOUNDARY;

    mFaces.reserve(MEMORY_SIZE_3);
    mFaceSlots.mHighestId = 2;
    mFaces.resize(2);

    mFaces[0].mHandle = 0;
    mFaces[0].mVertex1Idx = 0;
    mFaces[0].mVertex2Idx = 1;
    mFaces[0].mVertex3Idx = 3;
    mFaces[0].mEdge1 = 1;
    mFaces[0].mEdge2 = 4;
    mFaces[0].mEdge3 = 0;

    mFaces[1].mHandle = 1;
    mFaces[1].mVertex1Idx = 2;
    mFaces[1].mVertex2Idx = 3;
    mFaces[1].mVertex3Idx = 1;
    mFaces[1].mEdge1 = 4;
    mFaces[1].mEdge2 = 2;
    mFaces[1].mEdge3 = 3;
    for (uint i = 0; i < 2; ++i)
      mSectorGrid.CheckIn(&mFaces[i]);

    //some boundary edges will have -1 faces and edges
    mEdges.reserve(MEMORY_SIZE_3);
    mEdges.resize(5);
    mEdgeSlots.mHighestId = 5;

    //left
    mEdges[0].mHandle = 0;
    mEdges[0].mOriginVertexIdx = 3;
    mEdges[0].mDestVertexIdx = 0;
    mEdges[0].mRightFaceIdx = 0;
    mEdges[0].mOriginEdgeCW = 4;  //diagonal
    mEdges[0].mDestEdgeCCW = 1;
    mEdges[0].mFlags |= IS_BOUNDARY;
    mEdges[0].AddConstraintIdx(boundary_cs->mHandle);

    //top
    mEdges[1].mHandle = 1;
    mEdges[1].mOriginVertexIdx = 0;
    mEdges[1].mDestVertexIdx = 1;
    mEdges[1].mRightFaceIdx = 0;
    mEdges[1].mOriginEdgeCW = 0;
    mEdges[1].mDestEdgeCCW = 4; //mid diagonal
    mEdges[1].mFlags |= IS_BOUNDARY;
    mEdges[1].AddConstraintIdx(boundary_cs->mHandle);

    //right 
    mEdges[2].mHandle = 2;
    mEdges[2].mOriginVertexIdx = 1;
    mEdges[2].mDestVertexIdx = 2;
    mEdges[2].mRightFaceIdx = 1;
    mEdges[2].mOriginEdgeCW = 4; //mid diagonal
    mEdges[2].mDestEdgeCCW = 3; //bottom
    mEdges[2].mFlags |= IS_BOUNDARY;
    mEdges[2].AddConstraintIdx(boundary_cs->mHandle);

    //bottom
    mEdges[3].mHandle = 3;
    mEdges[3].mOriginVertexIdx = 2;
    mEdges[3].mDestVertexIdx = 3;
    mEdges[3].mRightFaceIdx = 1;
    mEdges[3].mOriginEdgeCW = 2; //right
    mEdges[3].mDestEdgeCCW = 4; //mid diagonal
    mEdges[3].mFlags |= IS_BOUNDARY;
    mEdges[3].AddConstraintIdx(boundary_cs->mHandle);

    //diagonal
    mEdges[4].mHandle = 4;
    mEdges[4].mOriginVertexIdx = 3; //bot left
    mEdges[4].mDestVertexIdx = 1;   //top right
    mEdges[4].mLeftFaceIdx = 0;
    mEdges[4].mRightFaceIdx = 1;
    mEdges[4].mOriginEdgeCCW = 0; //left edge
    mEdges[4].mOriginEdgeCW = 3; //bot edge
    mEdges[4].mDestEdgeCCW = 2; //right
    mEdges[4].mDestEdgeCW = 1; //top

    for (uint i = 0; i < 5; ++i)
      mSectorGrid.CheckIn(&mEdges[i]);
  }
  
  //test insert vertex
#if 0
  {
    //rotating square test
    float r = 0.0f * GenericMath::DEG_TO_RAD;
    float cosr = cos(r);
    float sinr = sin(r);
    Vector3 sqvtx[4];
    sqvtx[0] = Vector3(-1.0f, 0, 0.0f);
    sqvtx[1] = Vector3(1.0f, 0, 0.0f);
    sqvtx[2] = Vector3(0.0f, 0, -1.0f);
    sqvtx[3] = Vector3(0.0f, 0, 1.0f);

    sqvtx[0] = Vector3(sqvtx[0].x * cosr - sqvtx[0].z * sinr, 0, sqvtx[0].x * sinr + sqvtx[0].z * cosr);
    sqvtx[1] = Vector3(sqvtx[1].x * cosr - sqvtx[1].z * sinr, 0, sqvtx[1].x * sinr + sqvtx[1].z * cosr);
    sqvtx[2] = Vector3(sqvtx[2].x * cosr - sqvtx[2].z * sinr, 0, sqvtx[2].x * sinr + sqvtx[2].z * cosr);
    sqvtx[3] = Vector3(sqvtx[3].x * cosr - sqvtx[3].z * sinr, 0, sqvtx[3].x * sinr + sqvtx[3].z * cosr);

    TIMER_START;

    InsertVertexConstraint(sqvtx[0]);
    InsertVertexConstraint(sqvtx[1]);
    InsertVertexConstraint(sqvtx[2]);
    InsertVertexConstraint(sqvtx[3]);

    float time = 0.0f;
    TIMER_STOP(time);
    LOG("Time taken: %f", time);
  }
#endif

#if 1
  //load from file
  {
    TIMER_START;

    char linebuf[512];
    bool blockcomment = false;
    bool polystart = false, holestart = false, blockerstart = false;
    ObjectHandle cur_cs_identifier = NULL_OBJECTHANDLE;
    std::vector<Vector3> polypoints;

    //FILE* f = fopen("../EngineAssets/NavMeshTest/TestVtxInsertion.txt", "rb+");
    //FILE* f = fopen("../EngineAssets/NavMeshTest/TestSegmentInsertion.txt", "rb+");
    FILE* f = fopen("../EngineAssets/NavMeshTest/TestA.txt", "rb+");
    //FILE* f = fopen(navmeshfilename, "rb+");
    if (f == nullptr)
      LOG("Failed to load navmesh %s, using defaults...", navmeshfilename);
    else
    {
      while (1)
      {
        if (fgets(linebuf, 512, f) == nullptr)
          break;

        if (!blockcomment && linebuf[0] == '/' && linebuf[1] == '*')
        {
          blockcomment = true;
          continue;
        }
        else if (blockcomment && linebuf[0] == '*' && linebuf[1] == '/')
        {
          blockcomment = false;
          continue;
        }
        else if (blockcomment)
          continue;

        if (linebuf[0] == '/' && linebuf[1] == '/') //single line comment
          continue;
        if (!polystart && strstr(linebuf, "polystart"))
        {
          polystart = true;
          int ident = -1;
          sscanf(linebuf, "polystart %d", &ident);
          cur_cs_identifier = (ObjectHandle)ident;
        }
        else if (!holestart && strstr(linebuf, "holestart"))
          holestart = true;
        else if (!blockerstart && strstr(linebuf, "blockerstart"))
          blockerstart = true;

        if (polystart && strstr(linebuf, "polyend"))
        {
          polystart = false;
          if (!polypoints.empty())
            InsertNPointPolyConstraint(polypoints.data(), (uint)polypoints.size(), 0, cur_cs_identifier);
          polypoints.clear();
        }
        else if (holestart && strstr(linebuf, "holeend"))
        {
          holestart = false;
          if (!polypoints.empty())
            Systems::mRTSSimulation->CarveHole(polypoints.data(), (uint)polypoints.size(), false);
            //InsertNPointPolyConstraint(polypoints.data(), (uint)polypoints.size(), 0, cur_cs_identifier);
          polypoints.clear();
        }
        else if (blockerstart && strstr(linebuf, "blockerend"))
        {
          blockerstart = false;
          if (!polypoints.empty())
            Systems::mRTSSimulation->ReclaimLand(polypoints.data(), (uint)polypoints.size(), false, false);
          //InsertNPointPolyConstraint(polypoints.data(), (uint)polypoints.size(), 0, cur_cs_identifier);
          polypoints.clear();
        }

        else if ((polystart || holestart || blockerstart) && strstr(linebuf, "pt:"))
        {
          float v1 = 0.0f, v2 = 0.0f, v3 = 0.0f;
          sscanf(linebuf, "pt: %f %f %f", &v1, &v2, &v3);
          polypoints.emplace_back(v1, v2, v3);
        }

        char token[32] = { 0 };
        int length = 0;
        float v1 = 0.0f, v2 = 0.0f, v3 = 0.0f, v4 = 0.0f, v5 = 0.0f, v6 = 0.0f;
        sscanf(linebuf, "%s%n", token, &length);
        //int args_processed = sscanf(linebuf, "%s: %f %f %f %f %f %f", token, &v1, &v2, &v3, &v4, &v5, &v6);
        if (strstr(token, "vtx:") != nullptr)
        {
          char c = linebuf[length + 1];
          if (c == '[')
          {
            char offset_token[12] = { 0 };
            int args_processed = sscanf(linebuf + length + 1, "[%3c] %f %f %f", offset_token, &v1, &v2, &v3);
            if (args_processed == 4)
            {
              if (strstr(offset_token, "min"))
                InsertVertex(min + Vector3(v1, v2, v3));
              else if (strstr(offset_token, "max"))
                InsertVertex(max + Vector3(v1, v2, v3));
            }
            else
              LOGC("failed to process vertex insertion");
          }
          else
          {
            int args_processed = sscanf(linebuf + length, "%f %f %f", &v1, &v2, &v3);
            if (args_processed == 3)
              InsertVertex(Vector3(v1, v2, v3));
            else
              LOGC("failed to process vertex insertion");
          }
        }
        else if (strstr(token, "seg:") != nullptr)
        {
          char tokens[8][32] = { 0 };
          int args_processed = sscanf(linebuf + length, "%s %s %s %s %s %s %s %s %s", tokens[0], tokens[1], tokens[2], tokens[3], tokens[4], tokens[5], tokens[6], tokens[7]);
          if (args_processed == 6)
          {
            sscanf(tokens[0], "%f", &v1);
            sscanf(tokens[1], "%f", &v2);
            sscanf(tokens[2], "%f", &v3);
            sscanf(tokens[3], "%f", &v4);
            sscanf(tokens[4], "%f", &v5);
            sscanf(tokens[5], "%f", &v6);
            InsertSegmentConstraint(Vector3(v1, v2, v3), Vector3(v4, v5, v6));
          }
          else if (args_processed == 7 && tokens[0][0] == '[')
          {
            char offset_token[12] = { 0 };
            sscanf(tokens[0], "[%3c]", offset_token);
            sscanf(tokens[1], "%f", &v1);
            sscanf(tokens[2], "%f", &v2);
            sscanf(tokens[3], "%f", &v3);
            sscanf(tokens[4], "%f", &v4);
            sscanf(tokens[5], "%f", &v5);
            sscanf(tokens[6], "%f", &v6);
            if (strstr(offset_token, "min"))
              InsertSegmentConstraint(min + Vector3(v1, v2, v3), Vector3(v4, v5, v6));
            else if (strstr(offset_token, "max"))
              InsertSegmentConstraint(max + Vector3(v1, v2, v3), Vector3(v4, v5, v6));
          }
          else if (args_processed == 7 && tokens[3][0] == '[')
          {
            char offset_token[12] = { 0 };
            sscanf(tokens[0], "%f", &v1);
            sscanf(tokens[1], "%f", &v2);
            sscanf(tokens[2], "%f", &v3);
            sscanf(tokens[3], "[%3c]", offset_token);
            sscanf(tokens[4], "%f", &v4);
            sscanf(tokens[5], "%f", &v5);
            sscanf(tokens[6], "%f", &v6);
            if (strstr(offset_token, "min"))
              InsertSegmentConstraint(Vector3(v1, v2, v3), min + Vector3(v4, v5, v6));
            else if (strstr(offset_token, "max"))
              InsertSegmentConstraint(Vector3(v1, v2, v3), max + Vector3(v4, v5, v6));
          }
          else if (args_processed == 8)
          {
            char offset_token[2][12] = { 0 };
            sscanf(tokens[0], "[%3c]", offset_token[0]);
            sscanf(tokens[1], "%f", &v1);
            sscanf(tokens[2], "%f", &v2);
            sscanf(tokens[3], "%f", &v3);
            sscanf(tokens[4], "[%3c]", offset_token[1]);
            sscanf(tokens[5], "%f", &v4);
            sscanf(tokens[6], "%f", &v5);
            sscanf(tokens[7], "%f", &v6);
            if (strstr(offset_token[0], "min") && strstr(offset_token[1], "min"))
              InsertSegmentConstraint(min + Vector3(v1, v2, v3), min + Vector3(v4, v5, v6));
            else if (strstr(offset_token[0], "min") && strstr(offset_token[1], "max"))
              InsertSegmentConstraint(min + Vector3(v1, v2, v3), max + Vector3(v4, v5, v6));
            else if (strstr(offset_token[0], "max") && strstr(offset_token[1], "min"))
              InsertSegmentConstraint(max + Vector3(v1, v2, v3), min + Vector3(v4, v5, v6));
            else if (strstr(offset_token[0], "max") && strstr(offset_token[1], "max"))
              InsertSegmentConstraint(max + Vector3(v1, v2, v3), max + Vector3(v4, v5, v6));
          }
        }
        ValidateNavMesh();
      };
      fclose(f);
      float time = 0.0f;
      TIMER_STOP(time);
      LOG("Time taken: %f", time);
    }
  }
#endif
  //DeleteConstraint(0);
  //InsertSegmentConstraint(Vector3(min.x + 3, 0, min.z + 1.5f), Vector3(min.x + 4.5f, 0, min.z + 5.0f));
  //InsertSegmentConstraint(Vector3(min.x + 2.0f, 0, min.z + 2.0f), Vector3(min.x + 9.0f, 0, min.z + 2.0f));
  //InsertSegmentConstraint(Vector3(-11.7f, 0, -11.48f), Vector3(-9.37f, 0, -14.47f));
  
  //DeleteConstraint(Vector3(min.x + 3, 0, min.z + 1.5f), Vector3(min.x + 4.5f, 0, min.z + 5.0f));
  //DeleteConstraint(Vector3(min.x + 2.0f, 0, min.z + 2.0f), Vector3(min.x + 9.0f, 0, min.z + 2.0f));
  //InsertVertex(Vector3(-16, 0, 0));
  //InsertVertex(Vector3(16, 0, 0));
  //InsertSegmentConstraint(Vector3(-14, 0, 2), Vector3(-12, 0, 0));
  //InsertSegmentConstraint(Vector3(-16, 0, 0), Vector3(16, 0, 0));
  //DeleteConstraint(3);
  
  ValidateNavMesh();

  //gfx debug construction
  {
    mNavGfxCmp = nullptr;
#if 0
    mNavGfxCmp = (cModelGfxCmp*)cModelGfxCmp::GetFactory()->Allocate();

    uint size_per_vtx = sizeof(sWireFrameColorVtxFmt);
    char* data = new char[NAVMESH_MAX_VTX * size_per_vtx];
    memset(data, 0, NAVMESH_MAX_VTX * size_per_vtx);

    Color4 lightgreen(0, 0.95f, 0, 0.25f);
    uint vtxcount = mTriangleCount * 3;
    uint vtxidx = 0;
    sWireFrameColorVtxFmt* vertices = (sWireFrameColorVtxFmt*)data;
    for (uint i = 0; i < mTriangleCount; ++i)
    {
      for (uint j = 0; j < 3; ++j)
      {
        vertices[vtxidx].x = mTriangles[i].mPoints[j].x;
        vertices[vtxidx].y = mTriangles[i].mPoints[j].y;
        vertices[vtxidx].z = mTriangles[i].mPoints[j].z;
        vertices[vtxidx].mColor = lightgreen.ConvertRGBAToU32();

        ++vtxidx;
      }
    }

    Any vbufferargs[] = { NAVMESH_MAX_VTX, BUF_USG_DYNAMIC, cMeshConstruction::mColorVertexShader, (void*)data };
    iVertexBuffer* vbuffer = iVertexBuffer::GetFactory()->AllocateAndInitialize<iVertexBuffer>(ANYCOUNT(vbufferargs), vbufferargs);

    String name("navmesh");
    Any smeshargs[] = { &name, PRIMITIVE_TRI_LIST, vbuffer, nullptr };
    cStaticMesh* mesh = cStaticMesh::GetFactory()->GetOrCreate<cStaticMesh>(0, ANYCOUNT(smeshargs), smeshargs, 0, false);
    mesh->SetVerticesToRender(vtxcount);
    delete[] data;

    sModelGfxCmpParams p;
    p.mMaterialName = "Color";
    p.mMeshName = "navmesh";
    mNavGfxCmp->InitializeNoTextureSet(p);
    mNavGfxCmp->SetTransformPtr(&mTransform);

    mTransform = Matrix4::IDENTITY;
    mTransform.SetTranslate(Vector3(0, 0.0005f, 0.0f));
#endif
  }
  mDebugContext.mMesh = this;
  mDebugContext.mAStarNodes.reserve(MEMORY_SIZE_4);
  mDebugPathfinder.ClearAll();
  return true;
}

void cNavMesh::Destroy()
{
  mDebugPathfinder.ClearAll();
  mSectorGrid.Destroy();
  cModelGfxCmp::GetFactory()->DestroyAndDeallocate(&mNavGfxCmp);
}

bool cNavMesh::SaveNavmesh()
{
  //save to 
  FILE* f = fopen(mNavmeshFilename.c_str(), "wb+");
  if (f == nullptr)
  {
    LOG("Failed to save navmesh %s", mNavmeshFilename.c_str());
    return false;
  }

  for (uint i = 0; i < mConstraints.size(); ++i)
  {
    if (mConstraints[i].mPointCount != 0 && !mConstraints[i].IsInsertedByUnit() && !mConstraints[i].InFreeList()
      && !(mConstraints[i].mFlags & IS_BOUNDARY))
    {
      char buf[512] = { 0 };
      if (mConstraints[i].mIdentifier != NULL_OBJECTHANDLE)
        snprintf(buf, 512, "polystart %d\r\n", mConstraints[i].mIdentifier);
      else
        snprintf(buf, 512, "polystart\r\n");

      for (uint j = 0; j < mConstraints[i].mPointCount; ++j)
      {
        char ptbuf[512] = { 0 };
        snprintf(ptbuf, 512, "pt: %g %g %g\r\n", mConstraints[i].mPositions[j].x,
          mConstraints[i].mPositions[j].y, mConstraints[i].mPositions[j].z);

        strcat_s(buf, ptbuf);
      }
      strcat_s(buf, "polyend\r\n");
      fwrite(buf, strlen(buf), 1, f);
    }
  }
  fclose(f);
  LOG("Navmesh %s save successful.", mNavmeshFilename.c_str());
  return true;
}

void cNavMesh::BuildDebugATB(TwBar* viewbar)
{
  //TwAddButton(navmeshdebugbar, "ShowGeomBtn", nullptr, nullptr, "label='Show Geom' key=F1 help='Shows all debug geometry'");
  TwAddVarRW(viewbar, "DisplayEdgesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mDisplayEdges, "label='Draw Edges' help='Draw edges' group='Navmesh'");
  TwAddVarRW(viewbar, "ShowEdgesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowEdgeInfo, "label='Show Edges' help='Shows all debug information for edges' group='Navmesh'");
  TwAddVarRW(viewbar, "ShowFacesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowFaceInfo, "label='Show Faces' help='Shows all debug information for faces' group='Navmesh'");
  TwAddVarRW(viewbar, "ShowVerticesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowVertexInfo, "label='Show Vertices' help='Shows all debug information for vertices' group='Navmesh'");
  TwAddVarRW(viewbar, "ShowSectorGridBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowSectorGrid, "label='Show SectorGrid' help='Shows the sector grid' group='Navmesh'");
  TwAddVarRW(viewbar, "MOViewBtn", TW_TYPE_BOOLCPP, &mDebugControls.mMouseOverViewMode, "label='Mouseover View' help='Uses the mouse to show info' group='Navmesh'");
  
  //more specific debug info for edges
  TwAddVarRW(viewbar, "ShowEdgeAdjacenciesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowEdgeAdjacencies, "label='Show Edge Adjacencies' help='Show edge adjacencies' group='NavInfo'");
  TwAddVarRW(viewbar, "ShowEdgeConstraintsBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowEdgeConstraints, "label='Show Edge Constraints' help='Show edge constraints' group='NavInfo'");
  TwAddVarRW(viewbar, "ShowFaceAdjacenciesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mShowFaceAdjacencies, "label='Show Face Adjacencies' help='Show face adjacencies' group='NavInfo'");

  TwAddVarRW(viewbar, "ShowNavGameplay", TW_TYPE_BOOLCPP, &mDebugControls.mShowFaceInfoGameplay, "label='Show NavGameplay Info' help='Show edge constraints' group='NavmeshGameplay'");

  //debugging specific edges
  TwAddVarRW(viewbar, "ShowSpecificNavBtn", TW_TYPE_BOOLCPP, &mDebugControls.mDebugOnlySpecified, "label='Show Specified Elemets' help='Show specific navmesh elements' group='SpecificNavDebug'");
  TwAddVarRW(viewbar, "DebugEdge1RW", TW_TYPE_INT32, &mDebugControls.mDebugEdge1, "label='Debug Edge 1' help='Specific edge #1' group='SpecificNavDebug'");
  TwAddVarRW(viewbar, "DebugEdge2RW", TW_TYPE_INT32, &mDebugControls.mDebugEdge2, "label='Debug Edge 2' help='Specific edge #2' group='SpecificNavDebug'");
  TwAddVarRW(viewbar, "DebugFace1RW", TW_TYPE_INT32, &mDebugControls.mDebugFace1, "label='Debug Face 1' help='Specific face #1' group='SpecificNavDebug'");
  TwAddVarRW(viewbar, "DebugFace2RW", TW_TYPE_INT32, &mDebugControls.mDebugFace2, "label='Debug Edge 2' help='Specific face #2' group='SpecificNavDebug'");

  TwDefine("ViewDebugBar/NavInfo  group=Navmesh");
  TwDefine("ViewDebugBar/SpecificNavDebug  group=Navmesh");
  TwDefine("ViewDebugBar/NavmeshGameplay  group=Navmesh");

  auto validatecb = [](void* clientdata)
  {
    cNavMesh* m = (cNavMesh*)clientdata;
    m->ValidateNavMesh(true);
  };
  TwAddButton(viewbar, "ValidateNavMeshBtn", validatecb, this, "label='Validate NavMesh' help='Checks the navmesh for incorrect data' group='Navmesh'");

  TwAddSeparator(viewbar, nullptr, nullptr);
  TwAddVarRW(viewbar, "DisplayPathingFacesBtn", TW_TYPE_BOOLCPP, &mDebugControls.mDisplayPathfinder, "label='Display Pathfound Faces' help='Shows results of pathfinding' group='Pathfinder'");
  TwAddVarRW(viewbar, "StartPathingBtn", TW_TYPE_BOOLCPP, &mDebugControls.mStartRecordStartEndPts, "label='Start Pathing mode..' help='Starts pathing mode' group='Pathfinder'");
}

bool cNavMesh::PickConstraintedPoint(const Vector3& position, sPickedCSPtRes& result, float width, float height, float depth)
{
  for (uint i = 0; i < mConstraints.size(); ++i)
  {
    if (mConstraints[i].mPointCount == 0 || mConstraints[i].IsInsertedByUnit() || mConstraints[i].InFreeList()
      || (mConstraints[i].mFlags & IS_BOUNDARY))
      continue;
    //result.mConstraintHandle = mConstraints[i].mHandle;
    for (uint j = 0; j < mConstraints[i].mPointCount; ++j)
    {
      Vector3 centerpos = mConstraints[i].mPositions[j];
      AABB3D pointbox(centerpos, width, height, depth);
      if (pointbox.Intersects(position))
      {
        result.mConstraintHandle = mConstraints[i].mHandle;
        result.mPointIdx = j;
        result.mCSIdentifier = mConstraints[i].mIdentifier;
        return true;
      }
    }
  }
  return false;
}

void cNavMesh::Update(float dt)
{
  if (mDebugControls.mShowSectorGrid)
    mSectorGrid.DrawSectors();

  if (mDebugControls.mDisplayEdges)
  {
    for (int i = 0; i < mEdges.size(); ++i)
    {
      if (!mEdges[i].InFreeList() && mEdges[i].AreVerticesValid())
      {
        Vector3 origin = GetVertex(mEdges[i].mOriginVertexIdx)->mPosition;
        origin.y = NAVMESH_DBG_YOFFSET;
        Vector3 dest = GetVertex(mEdges[i].mDestVertexIdx)->mPosition;
        dest.y = NAVMESH_DBG_YOFFSET;

        Color4 color(Color4::GREEN);
        if (mEdges[i].IsConstrainted())
          color = Color4::RED;
        cMeshQuickDraw::DrawLine3D(origin, dest, color);
      }
    }
  }

  auto edgeDrawFunc = [this](sNavEdge* edge)
  {
    Vector3 mid = GetVertex(edge->mOriginVertexIdx)->mPosition.MidPoint(GetVertex(edge->mDestVertexIdx)->mPosition);

    Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(mid);
    char buf[256] = { 0 };
    if (mDebugControls.mShowEdgeAdjacencies)
    {
      snprintf(buf, 256, "Edge_%d\nVertices(%d to %d)\nOriginEdge(CCW/CW):%d,%d\nDestEdge(CCW/CW):%d,%d\nFace(Left/Right):%d,%d",
        edge->mHandle, edge->mOriginVertexIdx, edge->mDestVertexIdx, edge->mOriginEdgeCCW, edge->mOriginEdgeCW, edge->mDestEdgeCCW, edge->mDestEdgeCW,
        edge->mLeftFaceIdx, edge->mRightFaceIdx);
    }
    else
    {
      snprintf(buf, 256, "e%d", edge->mHandle);
    }

    if (mDebugControls.mShowEdgeConstraints && !edge->mConstraintIndices.empty())
    {
      char constraintsbuf[64] = "\nConstraints:";
      for (uint i = 0; i < edge->mConstraintIndices.size(); ++i)
      {
        ObjectHandle h = edge->mConstraintIndices[i];

        char tiny[32] = { 0 };
        _itoa(h, tiny, 10);
        strcat_s(constraintsbuf, tiny);
        if (i != (edge->mConstraintIndices.size() - 1))
          strcat_s(constraintsbuf, ",");
      }

      strcat_s(buf, constraintsbuf);
    }
    cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);
  };
  auto faceDrawFunc = [this](sNavFace* f)
  {
    Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(f->GetFaceMidPoint(this));
    char buf[128] = { 0 };
    if (mDebugControls.mShowFaceAdjacencies)
    {
      snprintf(buf, 128, "Face_%d\nVertices(%d,%d,%d)\nEdges(%d, %d, %d)\nFlags:%d\n",
        f->mHandle, f->mVertex1Idx, f->mVertex2Idx, f->mVertex3Idx, f->mEdge1, f->mEdge2, f->mEdge3, f->mFlags);
    }
    else
    {
      snprintf(buf, 128, "f%d", f->mHandle);
    }
    cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);
  };

  if (!mDebugControls.mMouseOverViewMode && !mDebugControls.mDebugOnlySpecified)
  {
    //edge information debug
    if (mDebugControls.mShowEdgeInfo)
    {
      for (int i = 0; i < mEdges.size(); ++i)
      {
        if (!mEdges[i].InFreeList() && mEdges[i].AreVerticesValid())
          edgeDrawFunc(&mEdges[i]);
      }
    }
    //face info debug
    if (mDebugControls.mShowFaceInfo)
    {
      for (int i = 0; i < mFaces.size(); ++i)
      {
        if (mFaces[i].InFreeList())
          continue;
        //if (mFaces[i].mHandle == 65552)
        //continue;
        if (GetVertex(mFaces[i].mVertex1Idx) == nullptr || GetVertex(mFaces[i].mVertex2Idx) == nullptr || GetVertex(mFaces[i].mVertex3Idx) == nullptr)
          continue;
        faceDrawFunc(&mFaces[i]);
      }
    }
    //vertex information debug
    if (mDebugControls.mShowVertexInfo)
    {
      for (int i = 0; i < mVertices.size(); ++i)
      {
        if (mVertices[i].InFreeList())
          continue;
        Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(mVertices[i].mPosition);
        char buf[32] = { 0 };
        snprintf(buf, 32, "v%d", mVertices[i].mHandle);
        cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);
      }
    }
  }

  if (mDebugControls.mDebugOnlySpecified)
  {
    if (mDebugControls.mDebugEdge1 != NULL_OBJECTHANDLE && GetEdge(mDebugControls.mDebugEdge1, false))
      edgeDrawFunc(GetEdge(mDebugControls.mDebugEdge1, false));
    if (mDebugControls.mDebugEdge2 != NULL_OBJECTHANDLE && GetEdge(mDebugControls.mDebugEdge2, false))
      edgeDrawFunc(GetEdge(mDebugControls.mDebugEdge2, false));
    if (mDebugControls.mDebugFace1 != NULL_OBJECTHANDLE && GetFace(mDebugControls.mDebugFace1, false))
      faceDrawFunc(GetFace(mDebugControls.mDebugFace1, false));
    if (mDebugControls.mDebugFace2 != NULL_OBJECTHANDLE && GetFace(mDebugControls.mDebugFace2, false))
      faceDrawFunc(GetFace(mDebugControls.mDebugFace2, false));
  }

  if (mDebugControls.mMouseOverViewMode)
  {
    Ray3D ray = Systems::mRTSSimulation->mRTSCamera->PickRay((uint)Systems::mMouseMgr->GetMouseX(), (uint)Systems::mMouseMgr->GetMouseY());
    float interp = 0.0f;
    ray.Intersects(cFloorEnt::mPlane, interp);

    Vector3 ptOnFloor = ray.mOrigin + ray.mDirection * interp;
    for (int i = 0; i < mFaces.size(); ++i)
    {
      if (mFaces[i].InFreeList())
        continue;

      sNavVertex* v1 = GetVertex(mFaces[i].mVertex1Idx);
      sNavVertex* v2 = GetVertex(mFaces[i].mVertex2Idx);
      sNavVertex* v3 = GetVertex(mFaces[i].mVertex3Idx);

      bool intersect = AccurateTriangle_Pt_Intersection(v1->mPosition, v2->mPosition, v3->mPosition, ptOnFloor);
      if (intersect)
      {
        //show info on surrounding navmesh elements(esp. edges)
        if (mDebugControls.mShowFaceInfo)
          faceDrawFunc(&mFaces[i]);
        if (mDebugControls.mShowEdgeInfo)
        {
          edgeDrawFunc(GetEdge(mFaces[i].mEdge1));
          edgeDrawFunc(GetEdge(mFaces[i].mEdge2));
          edgeDrawFunc(GetEdge(mFaces[i].mEdge3));
        }
        break; //only 1
      }
    }
  }

  if (mDebugControls.mShowFaceInfoGameplay)
  {
    for (int i = 0; i < mFaces.size(); ++i)
    {
      if (mFaces[i].InFreeList())
        continue;

      if (mFaces[i].mFlags & FACE_REPRESENTS_HOLE)
      {
        Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(mFaces[i].GetFaceMidPoint(this));
        char buf[128] = { 0 };
        snprintf(buf, 128, "Hole_%d\n", mFaces[i].mHandle);

        cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);
      }
      else if (mFaces[i].mFlags & FACE_REPRESENTS_BLOCKER)
      {
        Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(mFaces[i].GetFaceMidPoint(this));
        char buf[128] = { 0 };
        snprintf(buf, 128, "Blocker_%d\n", mFaces[i].mHandle);

        cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);
      }
    }
  }

  //test pathing
  static const float testradius = 1.0f;
  auto doDebugPathing = [this]()
  {
    LOG("Pathing end at %f,%f,%f", mDebugControls.mPathEnd.x, mDebugControls.mPathEnd.y, mDebugControls.mPathEnd.z);
    mDebugControls.mWayPointFaces.clear();
    mDebugControls.mWayPoints.clear();

    mDebugPathfinder.ClearAll();

    TIMER_START;

    Vector3 adjustedend;
    cPathfinder::FindPath(&mDebugContext, mDebugControls.mPathStart, mDebugControls.mPathEnd, mDebugControls.mWayPointFaces, adjustedend, testradius);

    float time = 0.0f;
    TIMER_STOP(time);
    LOG("Pathfind time: %f", time);

    {
      TIMER_START;

      cPathfinder::ProduceFunnelGuidedPath(&mDebugContext, mDebugControls.mPathStart, adjustedend,
        mDebugControls.mWayPointFaces, mDebugControls.mWayPoints, testradius, &mDebugPathfinder);

      cPathfinder::DoPathStraightening(mDebugControls.mWayPoints, testradius);
      //cPathfinder::ModifyEndpointNoIntersection(mDebugControls.mWayPoints, testradius);

      TIMER_STOP(time);
      LOG("Funnel time: %f", time);
    }

    /*if (!mDebugControls.mWayPointFaces.empty())
    mDebugControls.mPatherPosition = GetFace(mDebugControls.mWayPointFaces[0])->GetFaceMidPoint(this);
    */
    if (!mDebugControls.mWayPoints.empty())
      mDebugControls.mPatherPosition = mDebugControls.mWayPoints[0];

    mDebugControls.mPatherSegment = 0;
    mDebugControls.mPatherTime = 0.0f;

    mDebugContext.mAStarNodes.clear();
  };

  if (mDebugControls.mStartRecordStartEndPts)
  {
    if (Systems::mMouseMgr->IsButtonTriggered(MOUSEBUTTON::LBUTTON))
    {
      if (mDebugControls.mPathingMode == PATHING_MODE_STOP)
      {
        mDebugControls.mPathStart = Systems::mRTSSimulation->SelectGroundTarget();
        mDebugControls.mPathingMode = PATHING_MODE_START;

        LOG("Pathing started at %f,%f,%f", mDebugControls.mPathStart.x, mDebugControls.mPathStart.y, mDebugControls.mPathStart.z);
      }
      else if (mDebugControls.mPathingMode == PATHING_MODE_START)
      {
        mDebugControls.mPathEnd = Systems::mRTSSimulation->SelectGroundTarget();
        mDebugControls.mPathingMode = PATHING_MODE_STOP;

        doDebugPathing();
      }
    }
  }
  else
    mDebugControls.mPathingMode = PATHING_MODE_STOP;

  if (mDebugControls.mDebugPathFromATB)
  {
    doDebugPathing();
    mDebugControls.mDebugPathFromATB = false;
  }


  if (mDebugControls.mDisplayPathfinder && !mDebugControls.mWayPoints.empty())
  {
    mDebugPathfinder.DrawFunnelAlgo(testradius);

    for (int i = 0; i < ((int)mDebugControls.mWayPoints.size() - 1); ++i)
    {
      Vector3 v1 = mDebugControls.mWayPoints[i];
      v1.y = NAVMESH_DBG_YOFFSET;
      Vector3 v2 = mDebugControls.mWayPoints[i + 1];
      v2.y = NAVMESH_DBG_YOFFSET;
      cMeshQuickDraw::DrawLine3D(v1, v2, Color4::WHITE);
    }

    //traverse each face in 0.4f seconds time
    mDebugControls.mPatherTime += dt;
    if (mDebugControls.mPatherTime > 0.4f)
    {
      mDebugControls.mPatherSegment++;
      mDebugControls.mPatherTime = 0.0f;
    }

    uint totalsegments = (uint)mDebugControls.mWayPoints.size() - 1;
    if (totalsegments == 0)
    {
      mDebugControls.mPatherPosition = mDebugControls.mWayPoints[0];
      mDebugControls.mPatherPosition.y = NAVMESH_DBG_YOFFSET;
      cMeshQuickDraw::DrawWorldCircle(Cylinder(mDebugControls.mPatherPosition, testradius, 0.1f), Color4::RED);
    }
    else
    {
      if (mDebugControls.mPatherSegment >= totalsegments)
        mDebugControls.mPatherSegment = 0; //reset

      Vector3 v1 = mDebugControls.mWayPoints[mDebugControls.mPatherSegment];
      Vector3 v2 = mDebugControls.mWayPoints[mDebugControls.mPatherSegment + 1];
      
      float percent = mDebugControls.mPatherTime / 0.4f;
      mDebugControls.mPatherPosition = v1 + percent * (v2 - v1);
      mDebugControls.mPatherPosition.y = NAVMESH_DBG_YOFFSET;

      cMeshQuickDraw::DrawWorldCircle(Cylinder(mDebugControls.mPatherPosition, testradius, 0.1f), Color4::WHITE);
    }
  }
}

ObjectHandle cNavMesh::InsertSegmentConstraint(const Vector3& pt1, const Vector3& pt2, ObjectHandle constrainthandle, int stopatstep)
{
  ObjectHandle startvtxh = InsertVertex(pt1);
  ObjectHandle endvtxh = InsertVertex(pt2);

  if (startvtxh == endvtxh) //no segments caused by duplicate vtx
    return NULL_OBJECTHANDLE;
  if (constrainthandle == NULL_OBJECTHANDLE)
  {
    //add fresh constraint
    sConstraint* cs = AddConstraint();
    constrainthandle = cs->mHandle;
    cs->mPointCount = 2;
    cs->mPositions[0] = pt1;
    cs->mPositions[1] = pt2;
  }

  if (stopatstep & BEFORE_INSERT_SEGMENT_STEP)
    return constrainthandle;

  //orientate left to right for cleaner code and less conditional checks
  sNavVertex* startvtx = GetVertex(startvtxh);
  sNavVertex* endvtx = GetVertex(endvtxh);
  if (startvtx->mPosition.x > endvtx->mPosition.x)
  {
    //swap
    sNavVertex* tempe = startvtx;
    startvtx = endvtx;
    endvtx = tempe;

    ObjectHandle temph = startvtxh;
    endvtxh = startvtxh;
    startvtxh = temph;
  }

  //form the constraint

  //1) Find all unconstrainted edges intersected by pt1, pt2

  //use a limited plane since we're dealing with a 2d surface with elevations
  //note our y-plane is not correctly aligned with the world's z-plane, we have to correct it
  LineSegment3D newedge(startvtx->mPosition, endvtx->mPosition);
  LineSegment2D newedgeprojected(Vector2(startvtx->mPosition.x, startvtx->mPosition.z), Vector2(endvtx->mPosition.x, endvtx->mPosition.z));
  Vector2 edgevectorprojected = newedgeprojected.mEnd - newedgeprojected.mStart;

  //compute edge bounding box for quick early rejection tests
  /*
  Vector3 box1min = startvtx->mPosition;
  box1min.MakeFloor(endvtx->mPosition);
  box1min.x -= NAVMESH_EPSILON;
  box1min.z -= NAVMESH_EPSILON;
  Vector3 box1max = startvtx->mPosition;
  box1max.MakeCeil(endvtx->mPosition);
  box1max.x += NAVMESH_EPSILON;
  box1max.z += NAVMESH_EPSILON;
  */
  auto handlealreadyexists = [](std::vector<ObjectHandle>& r, ObjectHandle input) -> bool
  {
    for (uint i = 0; i < r.size(); ++i)
    {
      if (r[i] == input)
        return true;
    }
    return false;
  };

  struct IntersectionInfo
  {
    IntersectionInfo(ObjectHandle h, bool is_vertex, bool constrainted = false, const Vector3& pt = Vector3::ZERO)
      :mIntersectedVertexOrEdge(h), mIsVertex(is_vertex), mIsConstrainted(constrainted), mIntersectPt(pt), mFaceGameplayFlags(0)
    { }

    void SetCrossedEdge(cNavMesh* m, sNavEdge* e, LineSegment2D& travel_edge)
    {
      mIntersectedVertexOrEdge = e->mHandle;
      mIsConstrainted = e->IsConstrainted();
      mIsVertex = false;
      if (e->IsConstrainted())
      {
        sNavVertex* v1 = m->GetVertex(e->mOriginVertexIdx);
        sNavVertex* v2 = m->GetVertex(e->mDestVertexIdx);
        LineSegment2D crossededge(Vector2(v1->mPosition.x, v1->mPosition.z), Vector2(v2->mPosition.x, v2->mPosition.z));
        Vector2 intersectpt;
        if (travel_edge.Intersects(crossededge, intersectpt))
        {
          /*
          //note: if want to do reprojection, also modify the other intersection pt calculations
          //20th april 2015: reprojection has more fp errors!
          float fraction = (intersectpt - newedgeprojected.mStart).DotProduct(edgevectorprojected) / edgevectorprojected.SquaredLength();
          Vector3 intersectpt3d = v1->mPosition + fraction * (v2->mPosition - v1->mPosition);
          intersected_info.emplace_back(e->mHandle, e->IsConstrainted(), intersectpt3d);
          */
          mIntersectPt = Vector3(intersectpt.x, 0, intersectpt.y);
        }
        else
          ASSERTLOGC(0, "no intersection pt!");
      }
    }

    void SetCrossedEdge(cNavMesh*, sNavEdge* e, const Vector2& intersect_pt)
    {
      //intersect_pt already computed
      mIntersectedVertexOrEdge = e->mHandle;
      mIsConstrainted = e->IsConstrainted();
      mIsVertex = false;
      mIntersectPt = Vector3(intersect_pt.x, 0, intersect_pt.y);
    }

    ObjectHandle mIntersectedVertexOrEdge;
    Vector3 mIntersectPt; //only if is constrainted
    bool mIsConstrainted;
    bool mIsVertex;
    uint mFaceGameplayFlags;
  };

  auto getNextIntersectInfoFromVertex = [this, constrainthandle, &newedgeprojected]
    (sNavVertex* vtx, sNavVertex* endvtx, IntersectionInfo& info_out, sNavFace** out_face) -> bool /*return true if edge exists*/
  {
    std::vector<sSectorCoords> sectors_to_check;
    mSectorGrid.LocateSectors(sectors_to_check, vtx->mPosition);
    for (uint i = 0; i < sectors_to_check.size(); ++i)
    {
      sSector* sector = mSectorGrid.GetSector(sectors_to_check[i]);
      //check if edge exists already
      for (uint j = 0; j < sector->mEdgesInSector.size(); ++j)
      {
        sNavEdge* e = GetEdge(sector->mEdgesInSector[j]);
        if ((e->mOriginVertexIdx == vtx->mHandle && e->mDestVertexIdx == endvtx->mHandle) || 
          (e->mOriginVertexIdx == endvtx->mHandle && e->mDestVertexIdx == vtx->mHandle))
        {
          e->AddConstraintIdx(constrainthandle);
          return true; //an edge which contains the 2 vertices is already formed, return true
        }
      }

      for (uint j = 0; j < sector->mTrianglesInSector.size(); ++j)
      {
        sNavFace* f = GetFace(sector->mTrianglesInSector[j]);
        sNavVertex* opp_vtx1 = nullptr, *opp_vtx2 = nullptr, *cur_vtx = nullptr;

        //ensure triangle shares a vtx with the vertex your're shooting from
        if (f->mVertex1Idx == vtx->mHandle)
        {
          cur_vtx = GetVertex(f->mVertex1Idx);
          opp_vtx1 = GetVertex(f->mVertex2Idx);
          opp_vtx2 = GetVertex(f->mVertex3Idx);
        }
        else if (f->mVertex2Idx == vtx->mHandle)
        {
          cur_vtx = GetVertex(f->mVertex2Idx);
          opp_vtx1 = GetVertex(f->mVertex3Idx);
          opp_vtx2 = GetVertex(f->mVertex1Idx);
        }
        else if (f->mVertex3Idx == vtx->mHandle)
        {
          cur_vtx = GetVertex(f->mVertex3Idx);
          opp_vtx1 = GetVertex(f->mVertex1Idx);
          opp_vtx2 = GetVertex(f->mVertex2Idx);
        }

        if (!cur_vtx || !opp_vtx1 || !opp_vtx2)
          continue; //not sharing any vtx, skip

        //check if the path intersects any vertices
        LineSegment3D e(vtx->mPosition, endvtx->mPosition); 
        if (e.Intersects(opp_vtx1->mPosition, NAVMESH_EPSILON))
        {
          //vtx to vtx case
          info_out.mIntersectedVertexOrEdge = opp_vtx1->mHandle;
          info_out.mIntersectPt = opp_vtx1->mPosition;
          info_out.mIsVertex = true;
          info_out.mIsConstrainted = false;
          *out_face = nullptr;
          return false;
        }
        if (e.Intersects(opp_vtx2->mPosition, NAVMESH_EPSILON))
        {
          //vtx to vtx case
          info_out.mIntersectedVertexOrEdge = opp_vtx2->mHandle;
          info_out.mIntersectPt = opp_vtx2->mPosition;
          info_out.mIsVertex = true;
          info_out.mIsConstrainted = false;
          *out_face = nullptr;
          return false;
        }

        //check for the crossed edge
        Vector3 cur_to_opp1 = opp_vtx1->mPosition - cur_vtx->mPosition;
        Vector3 opp1_to_opp2 = opp_vtx2->mPosition - opp_vtx1->mPosition;
        Vector3 opp2_to_cur = cur_vtx->mPosition - opp_vtx2->mPosition;

        Vector3 cur_to_end = endvtx->mPosition - cur_vtx->mPosition;
        Vector3 opp1_to_end = endvtx->mPosition - opp_vtx1->mPosition;
        Vector3 opp2_to_end = endvtx->mPosition - opp_vtx2->mPosition;

        bool side1 = cur_to_opp1.CrossProductObtainSign(cur_to_end) >= 0.0f;
        bool side2 = opp1_to_opp2.CrossProductObtainSign(opp1_to_end) >= 0.0f;
        bool side3 = opp2_to_cur.CrossProductObtainSign(opp2_to_end) >= 0.0f;

        if (side1 == false && side1 == side3)
        {
          //vertex to crossed edge case
          //found_startface = true;
          *out_face = f;
          if (side1 == side2) //endpt within the same triangle or not?
            return true;
            //found_endface = true;
          else
          {
            sNavEdge* edgecrossed = GetEdgeFromFace(f, opp_vtx1->mHandle, opp_vtx2->mHandle);
            info_out.SetCrossedEdge(this, edgecrossed, newedgeprojected);
          }
          return false;
        }
      }
    }

    LOGC("Did not find a face corresponding to the vertex and line.");
    *out_face = nullptr;
    return false;
  };

  auto getNextIntersectInfoFromCrossedEdge = [this, &newedgeprojected, &newedge]
    (sNavEdge* crossededge, sNavFace* facebefore_crossededge, sNavVertex* endvtx, IntersectionInfo& info_out, sNavFace** out_nextface) -> bool
  {
    sNavFace* currentface = nullptr;
    if (facebefore_crossededge->mHandle == crossededge->mLeftFaceIdx)
      currentface = GetFace(crossededge->mRightFaceIdx);
    else
      currentface = GetFace(crossededge->mLeftFaceIdx);

    if (currentface->mVertex1Idx == endvtx->mHandle || currentface->mVertex2Idx == endvtx->mHandle || currentface->mVertex3Idx == endvtx->mHandle)
    {
      *out_nextface = currentface;
      return true;
    }

    //classify away vertices and vertices forming the crossed edge in clockwise manner
    ObjectHandle away_vtxh = NULL_OBJECTHANDLE, v1h = NULL_OBJECTHANDLE, v2h = NULL_OBJECTHANDLE;
    if ((currentface->mVertex1Idx == crossededge->mOriginVertexIdx && currentface->mVertex2Idx == crossededge->mDestVertexIdx) ||
      (currentface->mVertex1Idx == crossededge->mDestVertexIdx && currentface->mVertex2Idx == crossededge->mOriginVertexIdx))
    {
      away_vtxh = currentface->mVertex3Idx;
      v1h = currentface->mVertex1Idx;
      v2h = currentface->mVertex2Idx;
    }
    else if ((currentface->mVertex2Idx == crossededge->mOriginVertexIdx && currentface->mVertex3Idx == crossededge->mDestVertexIdx) ||
      (currentface->mVertex2Idx == crossededge->mDestVertexIdx && currentface->mVertex3Idx == crossededge->mOriginVertexIdx))
    {
      away_vtxh = currentface->mVertex1Idx;
      v1h = currentface->mVertex2Idx;
      v2h = currentface->mVertex3Idx;
    }
    else if ((currentface->mVertex3Idx == crossededge->mOriginVertexIdx && currentface->mVertex1Idx == crossededge->mDestVertexIdx) ||
      (currentface->mVertex3Idx == crossededge->mDestVertexIdx && currentface->mVertex1Idx == crossededge->mOriginVertexIdx))
    {
      away_vtxh = currentface->mVertex2Idx;
      v1h = currentface->mVertex3Idx;
      v2h = currentface->mVertex1Idx;
    }

    sNavVertex* away = GetVertex(away_vtxh);
    //check if the away vertex is on the path 
    if (newedge.Intersects(away->mPosition, NAVMESH_EPSILON))
    {
      //crossed edge to vertex case
      info_out.mIntersectedVertexOrEdge = away->mHandle;
      info_out.mIntersectPt = away->mPosition;
      info_out.mIsVertex = true;
      info_out.mIsConstrainted = false;
      *out_nextface = currentface;
      return false;
    }

    sNavVertex* v1 = GetVertex(v1h);
    sNavVertex* v2 = GetVertex(v2h);

    LineSegment2D v2_to_away(Vector2(v2->mPosition.x, v2->mPosition.z), Vector2(away->mPosition.x, away->mPosition.z));
    LineSegment2D away_to_v1(Vector2(away->mPosition.x, away->mPosition.z), Vector2(v1->mPosition.x, v1->mPosition.z));

    Vector2 intersectpt;
    if (v2_to_away.Intersects(newedgeprojected, intersectpt))
    {
      //crossed edge to crossed edge case
      sNavEdge* intersectededge = GetEdgeFromFace(currentface, crossededge->mHandle, v2h, away_vtxh);
      info_out.SetCrossedEdge(this, intersectededge, intersectpt);
      *out_nextface = currentface;
      return false;
    }
    else if (away_to_v1.Intersects(newedgeprojected, intersectpt))
    {
      //crossed edge to crossed edge case
      sNavEdge* intersectededge = GetEdgeFromFace(currentface, crossededge->mHandle, away_vtxh, v1h);
      info_out.SetCrossedEdge(this, intersectededge, intersectpt);
      *out_nextface = currentface;
      return false;
    }

    LOGC("This current face does not appear to be on the path of intersection!");
    *out_nextface = nullptr;
    return false;
  };

  std::vector<ObjectHandle> faces_to_remove;
  std::vector<IntersectionInfo> intersected_info;
  intersected_info.reserve(16);

  sNavFace* current_face = nullptr;
  IntersectionInfo intersection_info(NULL_OBJECTHANDLE, false);
  
  bool reachedend = getNextIntersectInfoFromVertex(startvtx, endvtx, intersection_info, &current_face);
  if (reachedend) //only vertex to vertex case fills this condition
    return constrainthandle;

  //not intersections found, yet we cannot continue
  if (!reachedend && intersection_info.mIntersectedVertexOrEdge == NULL_OBJECTHANDLE)
    return constrainthandle;

  //Vtx to crossed edge case, add face to remove
  if (!intersection_info.mIsVertex && current_face)
  {
    faces_to_remove.push_back(current_face->mHandle);
    if (current_face->mFlags & FACE_REPRESENTS_HOLE)
      intersection_info.mFaceGameplayFlags |= FACE_REPRESENTS_HOLE;
  }
  intersected_info.push_back(intersection_info);
  
  uint lastfaceflags = 0;
  while (!reachedend)
  {
    ObjectHandle h = intersection_info.mIntersectedVertexOrEdge;
    if (intersection_info.mIsVertex)
    {
      intersection_info = IntersectionInfo(NULL_OBJECTHANDLE, false);
      reachedend = getNextIntersectInfoFromVertex(GetVertex(h), endvtx, intersection_info, &current_face);
    }
    else
    {
      intersection_info = IntersectionInfo(NULL_OBJECTHANDLE, false);
      sNavFace* next_face = nullptr;
      reachedend = getNextIntersectInfoFromCrossedEdge(GetEdge(h), current_face, endvtx, intersection_info, &next_face);
      current_face = next_face;
    }

    //case for Vtx to crossed edge, crossed edge to crossed edge, crossed edge to vtx
    if (current_face)
    {
      faces_to_remove.push_back(current_face->mHandle);
      if (current_face->mFlags & FACE_REPRESENTS_HOLE)
        intersection_info.mFaceGameplayFlags |= FACE_REPRESENTS_HOLE;
    }

    if (!reachedend)
      intersected_info.push_back(intersection_info);
    else if(current_face)
      lastfaceflags = current_face->mFlags;
  } 

  //remove faces of intersected unconstrainted edges
  for (uint i = 0; i < faces_to_remove.size(); ++i)
    RemoveFace(faces_to_remove[i]);
  
  auto edgeAlreadyIntersected = [this](std::vector<IntersectionInfo>& intersectinfo, ObjectHandle h) -> bool
  {
    for (uint i = 0; i < intersectinfo.size(); ++i)
    {
      if (!intersectinfo[i].mIsVertex && intersectinfo[i].mIntersectedVertexOrEdge == h)
        return true;
    }
    return false;
  };

  //build shells, obtain intersection points from constrainted edges
  std::vector<Shell> shells;
  Shell first_shell;
  first_shell.mStartVtx = startvtx->mHandle;
  shells.push_back(first_shell);

  std::vector<ObjectHandle> verticeschecked;
  Vector2 perpendicularvecprojected(-edgevectorprojected.y, edgevectorprojected.x);

  auto collectVerticesOfShell = [this, &intersected_info, edgeAlreadyIntersected](ObjectHandle endvtx,
    ObjectHandle firstedge, ObjectHandle firsthostvtx, std::vector<ObjectHandle>& collect, bool moveccw)
  {
    ObjectHandle currentedgeh = firstedge;
    ObjectHandle currenthostvtxh = firsthostvtx;
    //traverse edges until you hit the end vtx of the shell
    do
    {
      collect.push_back(currenthostvtxh);
      do
      {
        //keep traveling ccw as long as the edge isnt one of the removed ones
        sNavEdge* e = GetEdge(currentedgeh);
        if (currenthostvtxh == e->mOriginVertexIdx)
          currentedgeh = moveccw ? e->mOriginEdgeCCW : e->mOriginEdgeCW;
        else if (currenthostvtxh == e->mDestVertexIdx)
          currentedgeh = moveccw ? e->mDestEdgeCCW : e->mDestEdgeCW;

      } while (edgeAlreadyIntersected(intersected_info, currentedgeh));

      sNavEdge* next_shell_edge = GetEdge(currentedgeh);
      if (next_shell_edge->mOriginVertexIdx == currenthostvtxh)
        currenthostvtxh = next_shell_edge->mDestVertexIdx;
      else if (next_shell_edge->mDestVertexIdx == currenthostvtxh)
        currenthostvtxh = next_shell_edge->mOriginVertexIdx;

      //current_shell.mShellEdgesAbove.push_back(currentedgeh);
    } while (currenthostvtxh != endvtx);
  };

  for (uint i = 0; i < intersected_info.size(); ++i)
  {
    if (!intersected_info[i].mIsVertex && intersected_info[i].mIsConstrainted)
    {
      LOG("Insert constraint %d: Constrainted Edge %d intersected.", constrainthandle, intersected_info[i].mIntersectedVertexOrEdge);
      
      Shell& current_shell = shells.back();
      Shell next_shell;

      sNavVertex* startvtx = GetVertex(current_shell.mStartVtx);
      Vector3 startpos = startvtx->mPosition;
      
      //add the intersected vertex for the new edge and split edge
      sNavVertex* newendvtx = AddVertex();
      newendvtx->mPosition = intersected_info[i].mIntersectPt;
      current_shell.mEndVtx = newendvtx->mHandle;
      
      Vector3 vec = newendvtx->mPosition - startpos;
      Vector2 vec_projected(vec.x, vec.z);
      float vec_sqlength = vec_projected.SquaredLength();

      //allocations
      ObjectHandle edge_newVtxToOldOriginHandle = AddEdgeGetHandle();
      ObjectHandle edge_newVtxToOldDestHandle = AddEdgeGetHandle();
      current_shell.mBaseEdgeHandle = AddEdgeGetHandle();

      sNavEdge* edge_newVtxToOldOrigin = GetEdge(edge_newVtxToOldOriginHandle);
      sNavEdge* edge_newVtxToOldDest = GetEdge(edge_newVtxToOldDestHandle);
      sNavEdge* intersected_constrainted_edge = GetEdge(intersected_info[i].mIntersectedVertexOrEdge);
      sNavEdge* current_shell_baseedge = GetEdge(current_shell.mBaseEdgeHandle);

      //split the edge
      SplitEdge(intersected_constrainted_edge, newendvtx, edge_newVtxToOldOrigin, edge_newVtxToOldDest);

      //build up the shell vertices around the edge
      ObjectHandle initial_ccw_edge_origin = NULL_OBJECTHANDLE, initial_cw_edge_origin = NULL_OBJECTHANDLE;
      //first detect the initial ccw and cw edges about the two endpoints
      FindRotationEdgesAboutVtx(current_shell.mStartVtx, vec_projected, vec_sqlength, initial_ccw_edge_origin, initial_cw_edge_origin);

      //add the new base edge
      current_shell_baseedge->mOriginVertexIdx = current_shell.mStartVtx;
      current_shell_baseedge->mDestVertexIdx = newendvtx->mHandle;
      current_shell_baseedge->AddConstraintIdx(constrainthandle);
      //base edge adjacencies are settled in retriangulation step
      current_shell.mBaseEdgeHandle = current_shell_baseedge->mHandle;

      {
        //update adjacencies for the split edges
        sNavEdge* oldoriginccw = GetEdge(intersected_constrainted_edge->mOriginEdgeCCW);
        if (oldoriginccw->mOriginVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
        {
          oldoriginccw->mOriginEdgeCW = edge_newVtxToOldOrigin->mHandle;
          edge_newVtxToOldOrigin->mDestEdgeCCW = oldoriginccw->mHandle;
        }
        else if (oldoriginccw->mDestVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
        {
          oldoriginccw->mDestEdgeCW = edge_newVtxToOldOrigin->mHandle;
          edge_newVtxToOldOrigin->mDestEdgeCCW = oldoriginccw->mHandle;
        }

        sNavEdge* oldorigincw = GetEdge(intersected_constrainted_edge->mOriginEdgeCW);
        if (oldorigincw->mOriginVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
        {
          oldorigincw->mOriginEdgeCCW = edge_newVtxToOldOrigin->mHandle;
          edge_newVtxToOldOrigin->mDestEdgeCW = oldorigincw->mHandle;
        }
        else if (oldorigincw->mDestVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
        {
          oldorigincw->mDestEdgeCCW = edge_newVtxToOldOrigin->mHandle;
          edge_newVtxToOldOrigin->mDestEdgeCW = oldorigincw->mHandle;
        }

        sNavEdge* olddestccw = GetEdge(intersected_constrainted_edge->mDestEdgeCCW);
        if (olddestccw->mOriginVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
        {
          olddestccw->mOriginEdgeCW = edge_newVtxToOldDest->mHandle;
          edge_newVtxToOldDest->mDestEdgeCCW = olddestccw->mHandle;
        }
        else if (olddestccw->mDestVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
        {
          olddestccw->mDestEdgeCW = edge_newVtxToOldDest->mHandle;
          edge_newVtxToOldDest->mDestEdgeCCW = olddestccw->mHandle;
        }

        sNavEdge* olddestcw = GetEdge(intersected_constrainted_edge->mDestEdgeCW);
        if (olddestcw->mOriginVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
        {
          olddestcw->mOriginEdgeCCW = edge_newVtxToOldDest->mHandle;
          edge_newVtxToOldDest->mDestEdgeCW = olddestcw->mHandle;
        }
        else if (olddestcw->mDestVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
        {
          olddestcw->mDestEdgeCCW = edge_newVtxToOldDest->mHandle;
          edge_newVtxToOldDest->mDestEdgeCW = olddestcw->mHandle;
        }
      }

      mSectorGrid.CheckOut(GetEdge(intersected_info[i].mIntersectedVertexOrEdge));

      mSectorGrid.CheckIn(edge_newVtxToOldOrigin);
      mSectorGrid.CheckIn(edge_newVtxToOldDest);

      { 
        //get shell edges for the positive side (ccw)
        ObjectHandle firstedgeh = initial_ccw_edge_origin;
        ObjectHandle firsthostvtxh = NULL_OBJECTHANDLE;

        sNavEdge* firstedge = GetEdge(firstedgeh);
        if (current_shell.mStartVtx == firstedge->mOriginVertexIdx)
          firsthostvtxh = firstedge->mDestVertexIdx;
        else if (current_shell.mStartVtx == firstedge->mDestVertexIdx)
          firsthostvtxh = firstedge->mOriginVertexIdx;

        collectVerticesOfShell(current_shell.mEndVtx, firstedgeh, firsthostvtxh, 
          current_shell.mShellVerticesAbove, false);
      }

      { 
        //get shell edges for the negative side (cw)
        ObjectHandle firstedgeh = initial_cw_edge_origin;
        ObjectHandle firsthostvtxh = NULL_OBJECTHANDLE;

        sNavEdge* firstedge = GetEdge(firstedgeh);
        if (current_shell.mStartVtx == firstedge->mOriginVertexIdx)
          firsthostvtxh = firstedge->mDestVertexIdx;
        else if (current_shell.mStartVtx == firstedge->mDestVertexIdx)
          firsthostvtxh = firstedge->mOriginVertexIdx;

        collectVerticesOfShell(current_shell.mEndVtx, firstedgeh, firsthostvtxh,
          current_shell.mShellVerticesBelow, true);
      }

      mSectorGrid.CheckIn(current_shell_baseedge);

      if (intersected_info[i].mFaceGameplayFlags & FACE_REPRESENTS_HOLE)
        current_shell.mGameplayFlags |= FACE_REPRESENTS_HOLE;

      //add the vertices to the new shell
      next_shell.mStartVtx = newendvtx->mHandle;
      shells.push_back(next_shell);
    }
    else if (intersected_info[i].mIsVertex)
    {
      Shell& current_shell = shells.back();
      current_shell.mEndVtx = intersected_info[i].mIntersectedVertexOrEdge;

       ObjectHandle h = mSectorGrid.FindEdgeInSector(current_shell.mStartVtx, current_shell.mEndVtx);
       if (h != NULL_OBJECTHANDLE)
       {
         //valid edge, vtx to vtx case
         sNavEdge* existing_edge = GetEdge(h);
         existing_edge->AddConstraintIdx(constrainthandle);
         current_shell.mBaseEdgeHandle = NULL_OBJECTHANDLE;
       }
       else
       {
         //no existing edge, probably a constrainted edge to vertex or 
         //a vertex to vertex(with no edge between them) case.
         //in this case, we need to build our shells and retriangulate the areas
         sNavEdge* finalbaseedge = AddEdge();
         finalbaseedge->mOriginVertexIdx = current_shell.mStartVtx;
         finalbaseedge->mDestVertexIdx = current_shell.mEndVtx;
         finalbaseedge->AddConstraintIdx(constrainthandle);

         current_shell.mBaseEdgeHandle = finalbaseedge->mHandle;

         Vector3 vec = GetVertex(current_shell.mEndVtx)->mPosition - GetVertex(current_shell.mStartVtx)->mPosition;
         Vector2 vec_projected(vec.x, vec.z);
         float vec_sqlength = vec_projected.SquaredLength();

         ObjectHandle initial_ccw_edge_origin = NULL_OBJECTHANDLE, initial_cw_edge_origin = NULL_OBJECTHANDLE;
         FindRotationEdgesAboutVtx(current_shell.mStartVtx, vec_projected, vec_sqlength, initial_ccw_edge_origin, initial_cw_edge_origin);

         {
           //get shell edges for the positive side (ccw)
           ObjectHandle firstedgeh = initial_ccw_edge_origin;
           ObjectHandle firsthostvtxh = NULL_OBJECTHANDLE;

           sNavEdge* firstedge = GetEdge(firstedgeh);
           if (current_shell.mStartVtx == firstedge->mOriginVertexIdx)
             firsthostvtxh = firstedge->mDestVertexIdx;
           else if (current_shell.mStartVtx == firstedge->mDestVertexIdx)
             firsthostvtxh = firstedge->mOriginVertexIdx;

           collectVerticesOfShell(current_shell.mEndVtx, firstedgeh, firsthostvtxh,
             current_shell.mShellVerticesAbove, false);

           //get shell edges for the negative side (cw)
           firstedgeh = initial_cw_edge_origin;
           firsthostvtxh = NULL_OBJECTHANDLE;

           firstedge = GetEdge(firstedgeh);
           if (current_shell.mStartVtx == firstedge->mOriginVertexIdx)
             firsthostvtxh = firstedge->mDestVertexIdx;
           else if (current_shell.mStartVtx == firstedge->mDestVertexIdx)
             firsthostvtxh = firstedge->mOriginVertexIdx;

           collectVerticesOfShell(current_shell.mEndVtx, firstedgeh, firsthostvtxh,
             current_shell.mShellVerticesBelow, true);
         }
         if (finalbaseedge) //only if we created it
           mSectorGrid.CheckIn(finalbaseedge);

       }

      Shell next_shell;
      next_shell.mStartVtx = current_shell.mEndVtx;
      shells.push_back(next_shell);
    }
    else
    {
      //unconstrainted edge
      LOG("Insert constraint %d: Unconstrainted Edge %d intersected.", constrainthandle, intersected_info[i].mIntersectedVertexOrEdge);
      if (intersected_info[i].mFaceGameplayFlags & FACE_REPRESENTS_HOLE)
      {
        Shell& current_shell = shells.back();
        current_shell.mGameplayFlags |= FACE_REPRESENTS_HOLE;
      }
    }
  }

  {
    //form the final edge of the shell
    Shell& last_shell = shells.back();
    last_shell.mEndVtx = endvtx->mHandle;

    if (lastfaceflags & FACE_REPRESENTS_HOLE)
      last_shell.mGameplayFlags |= FACE_REPRESENTS_HOLE;
    if (lastfaceflags & FACE_REPRESENTS_BLOCKER)
      last_shell.mGameplayFlags |= FACE_REPRESENTS_BLOCKER;

    sNavEdge* finalbaseedge = nullptr;
    ObjectHandle lastedge = mSectorGrid.FindEdgeInSector(last_shell.mStartVtx, last_shell.mEndVtx);
    if (lastedge == NULL_OBJECTHANDLE) //edge doesnt exist yet
    {
      finalbaseedge = AddEdge();
      finalbaseedge->mOriginVertexIdx = last_shell.mStartVtx;
      finalbaseedge->mDestVertexIdx = last_shell.mEndVtx;
      finalbaseedge->AddConstraintIdx(constrainthandle);
      last_shell.mBaseEdgeHandle = finalbaseedge->mHandle;

      Vector3 vec = GetVertex(last_shell.mEndVtx)->mPosition - GetVertex(last_shell.mStartVtx)->mPosition;
      Vector2 vec_projected(vec.x, vec.z);
      float vec_sqlength = vec_projected.SquaredLength();

      ObjectHandle initial_ccw_edge_origin = NULL_OBJECTHANDLE, initial_cw_edge_origin = NULL_OBJECTHANDLE;
      FindRotationEdgesAboutVtx(last_shell.mStartVtx, vec_projected, vec_sqlength, initial_ccw_edge_origin, initial_cw_edge_origin);

      {
        //get shell edges for the positive side (ccw)
        ObjectHandle firstedgeh = initial_ccw_edge_origin;
        ObjectHandle firsthostvtxh = NULL_OBJECTHANDLE;

        sNavEdge* firstedge = GetEdge(firstedgeh);
        if (last_shell.mStartVtx == firstedge->mOriginVertexIdx)
          firsthostvtxh = firstedge->mDestVertexIdx;
        else if (last_shell.mStartVtx == firstedge->mDestVertexIdx)
          firsthostvtxh = firstedge->mOriginVertexIdx;

        collectVerticesOfShell(last_shell.mEndVtx, firstedgeh, firsthostvtxh,
          last_shell.mShellVerticesAbove, false);

        //get shell edges for the negative side (cw)
        firstedgeh = initial_cw_edge_origin;
        firsthostvtxh = NULL_OBJECTHANDLE;

        firstedge = GetEdge(firstedgeh);
        if (last_shell.mStartVtx == firstedge->mOriginVertexIdx)
          firsthostvtxh = firstedge->mDestVertexIdx;
        else if (last_shell.mStartVtx == firstedge->mDestVertexIdx)
          firsthostvtxh = firstedge->mOriginVertexIdx;

        collectVerticesOfShell(last_shell.mEndVtx, firstedgeh, firsthostvtxh,
          last_shell.mShellVerticesBelow, true);
      }
      if (finalbaseedge) //only if we created it
        mSectorGrid.CheckIn(finalbaseedge);
    }
    else
    {
      sNavEdge* existing_edge = GetEdge(lastedge);
      existing_edge->AddConstraintIdx(constrainthandle);
      last_shell.mBaseEdgeHandle = NULL_OBJECTHANDLE;
    }
    
  }

  for (uint i = 0; i < intersected_info.size(); ++i)
  {
    if (!intersected_info[i].mIsVertex)
      RemoveEdge(intersected_info[i].mIntersectedVertexOrEdge);
  }

  if (stopatstep & BEFORE_RETRIANGULATE_STEP)
    return constrainthandle;

  //todo: some help for debugging this area?
  for (uint i = 0; i < shells.size(); ++i)
  {
    Shell& current_shell = shells[i];
    if (current_shell.mBaseEdgeHandle == NULL_OBJECTHANDLE) //edge already exists, no need for retriangulate
      continue;

    RetriangulateWithBaseEdge(current_shell);
  }
  return constrainthandle;
}

ObjectHandle cNavMesh::InsertNPointPolyConstraint(Vector3* pts, uint count, uint extraflags, ObjectHandle cur_cs_identifier)
{
  if (count == 1 || count == 0)
    return NULL_OBJECTHANDLE;

  //add constraint
  if (count > MAX_CONSTRAINT_POINTS)
  {
    LOG("%d points given, only supporting %d points", count, MAX_CONSTRAINT_POINTS);
    count = MAX_CONSTRAINT_POINTS;
  }

  sConstraint* cs = AddConstraint();
  for (uint i = 0; i < count; ++i)
    cs->mPositions[i] = pts[i];
  cs->mPointCount = count;
  cs->mFlags |= extraflags;
  cs->mIdentifier = cur_cs_identifier;
  
  uint segments = count - 1;
  for (uint i = 0; i < segments; ++i)
    InsertSegmentConstraint(pts[i], pts[i+1], cs->mHandle);

  if (count != 2)
    InsertSegmentConstraint(pts[0], pts[segments], cs->mHandle);

  return cs->mHandle;
}

void cNavMesh::DeleteConstraint(ObjectHandle constrainthandle)
{
  sConstraint* cs = GetConstraint(constrainthandle);
  if (!cs || cs->mPointCount == 0)
    return;
  
  sSectorCoords sectors[MAX_SECTORS_PER_POINT];
  uint sectorcount = MAX_SECTORS_PER_POINT;
  mSectorGrid.LocateSectors(sectors, &sectorcount, cs->mPositions[0]);

  sNavVertex* startvertex = nullptr;
  sNavEdge* startedge = nullptr;
  for (uint i = 0; i < sectorcount; ++i)
  {
    sSector* sector = mSectorGrid.GetSector(sectors[i]);
    for (uint j = 0; j < sector->mVerticesInSector.size(); ++j)
    {
      sNavVertex* v = GetVertex(sector->mVerticesInSector[j]);
      if (v->mPosition.IsEqual(cs->mPositions[0], NAVMESH_EPSILON))
      {
        startvertex = v;
        break;
      }
    }

    for (uint j = 0; j < sector->mEdgesInSector.size(); ++j)
    {
      sNavEdge* e = GetEdge(sector->mEdgesInSector[j]);
      if (e->HasConstraintIdx(constrainthandle) && (e->mOriginVertexIdx == startvertex->mHandle || e->mDestVertexIdx == startvertex->mHandle))
      {
        startedge = e;
        break;
      }
    }
    if (startvertex)
      break;
  }
  RemoveConstraint(cs);

  if (!startvertex)
  {
    LOGC("Cannot find a vertex with the same constraint");
    return;
  }

  auto alreadyTested = [this](std::vector<ObjectHandle>& testagainst, ObjectHandle h) -> bool
  {
    for (uint i = 0; i < testagainst.size(); ++i)
    {
      if (testagainst[i] == h)
        return true;
    }
    return false;
  };

  //traverse and get all edges with the same constraint
  std::vector<ObjectHandle> constrainted_edges;
  std::vector<ObjectHandle> vertices_of_csedges;
  constrainted_edges.reserve(16);
  vertices_of_csedges.reserve(16);

  std::deque<ObjectHandle> constrainted_edges_deq;
  constrainted_edges_deq.push_front(startedge->mHandle);
  auto getEdgesWithSameConstraintAboutVtx = [this, &constrainted_edges_deq, &constrainted_edges, alreadyTested](ObjectHandle vtxh, ObjectHandle constraintidx)
  {
    sNavVertex* vtx = GetVertex(vtxh);
    sSectorCoords sectors[MAX_SECTORS_PER_POINT];
    uint sectorcount = MAX_SECTORS_PER_POINT;
    mSectorGrid.LocateSectors(sectors, &sectorcount, vtx->mPosition);

    for (uint i = 0; i < sectorcount; ++i)
    {
      sSector* sector = mSectorGrid.GetSector(sectors[i]);
      for (uint j = 0; j < sector->mEdgesInSector.size(); ++j)
      {
        sNavEdge* e = GetEdge(sector->mEdgesInSector[j]);
        if (e->HasConstraintIdx(constraintidx) && !alreadyTested(constrainted_edges, e->mHandle))
          constrainted_edges_deq.push_front(e->mHandle);
      }
    }
  };

  while (!constrainted_edges_deq.empty())
  {
    ObjectHandle edgeh = constrainted_edges_deq.front();
    constrainted_edges_deq.pop_front();

    constrainted_edges.push_back(edgeh);

    sNavEdge* curedge = GetEdge(edgeh);
    if (!alreadyTested(vertices_of_csedges, curedge->mOriginVertexIdx))
    {
      vertices_of_csedges.push_back(curedge->mOriginVertexIdx);
      //push to the deque all edges with the same constraint
      getEdgesWithSameConstraintAboutVtx(curedge->mOriginVertexIdx, constrainthandle);
    }
    if (!alreadyTested(vertices_of_csedges, curedge->mDestVertexIdx))
    {
      vertices_of_csedges.push_back(curedge->mDestVertexIdx);
      getEdgesWithSameConstraintAboutVtx(curedge->mDestVertexIdx, constrainthandle);
    }
  } 

  //erase constraints from the edges
  for (uint i = 0; i < constrainted_edges.size(); ++i)
  {
    sNavEdge* e = GetEdge(constrainted_edges[i]);
    for (uint j = 0; j < e->mConstraintIndices.size(); ++j)
    {
      if (e->mConstraintIndices[j] == constrainthandle)
      {
        e->mConstraintIndices.erase(e->mConstraintIndices.begin() + j);
        break;
      }
    }
  }
  
  //all vertices traversed are vertices connected to edges with the constraint
  for (uint i = 0; i < vertices_of_csedges.size(); ++i)
  {
    sSectorCoords sectors2[MAX_SECTORS_PER_POINT];
    sectorcount = MAX_SECTORS_PER_POINT;
    sNavVertex* vtx = GetVertex(vertices_of_csedges[i]);
    mSectorGrid.LocateSectors(sectors2, &sectorcount, vtx->mPosition);

    std::vector<ObjectHandle> unconstrainted_edges;
    uint connected_constrainted_edges = 0;
    sNavEdge* connected_c_edge_2only[2] = { nullptr, nullptr };
    for (uint z = 0; z < sectorcount; ++z)
    { 
      sSector* sector = mSectorGrid.GetSector(sectors2[z]);
      for (uint j = 0; j < sector->mEdgesInSector.size(); ++j)
      {
        sNavEdge* e = GetEdge(sector->mEdgesInSector[j]);
        if (e &&
          (e->mOriginVertexIdx == vtx->mHandle || e->mDestVertexIdx == vtx->mHandle))
        {
          if (!e->IsConstrainted())
            unconstrainted_edges.push_back(e->mHandle);
          else
          {
            if (connected_constrainted_edges < 2)
              connected_c_edge_2only[connected_constrainted_edges] = e;

            ++connected_constrainted_edges;
          }
        }
      }
    }

    if (connected_constrainted_edges == 0)
    {
      if (!unconstrainted_edges.empty())
        RetriangulateHoleFromRemovedVertex(vtx, unconstrainted_edges);
      
      RemoveVertex(vertices_of_csedges[i]);
      //ValidateNavMesh();
    }
    else if (connected_constrainted_edges == 2)
    {
      if (connected_c_edge_2only[0]->mConstraintIndices == connected_c_edge_2only[1]->mConstraintIndices &&
        IsCollinear(connected_c_edge_2only[0], connected_c_edge_2only[1]))
      {
        sNavVertex* v1, *v2;
        v1 = GetVertex(GetOtherNonSharedVertex(connected_c_edge_2only[0], connected_c_edge_2only[1], connected_c_edge_2only[1]));
        v2 = GetVertex(GetOtherNonSharedVertex(connected_c_edge_2only[0], connected_c_edge_2only[1], connected_c_edge_2only[0]));
        
        if (v1->mPosition.x > v2->mPosition.x)
        {
          //swap to ensure left to right, 
          //gives us less complex cases that require us to evaluate going cw/ccw is above or below the line
          auto* temp = v1;
          v1 = v2;
          v2 = temp;
        }

        auto indices = connected_c_edge_2only[0]->mConstraintIndices;

        //custom function to collect edges of shell
        auto collectVerticesOfShell = [this](ObjectHandle endvtx,
          ObjectHandle firstedge, ObjectHandle firsthostvtx,
          std::vector<ObjectHandle>& collect, bool moveccw, std::vector<ObjectHandle>& edges_to_ignore)
        {
          auto edgeAlreadyIntersected = [](std::vector<ObjectHandle>& edges, ObjectHandle h) -> bool
          {
            //this function defers from the other function in InsertSegmentConstraint
            for (uint i = 0; i < edges.size(); ++i)
            {
              if (edges[i] == h) 
                return true;
            }
            return false;
          };

          ObjectHandle currentedgeh = firstedge;
          ObjectHandle currenthostvtxh = firsthostvtx;
          //traverse edges until you hit the end vtx of the shell
          do
          {
            collect.push_back(currenthostvtxh);
            do
            {
              //keep traveling ccw/cw as long as the edge isnt one of the removed ones
              sNavEdge* e = GetEdge(currentedgeh);
              if (currenthostvtxh == e->mOriginVertexIdx)
                currentedgeh = moveccw ? e->mOriginEdgeCCW : e->mOriginEdgeCW;
              else if (currenthostvtxh == e->mDestVertexIdx)
                currentedgeh = moveccw ? e->mDestEdgeCCW : e->mDestEdgeCW;

            } while (edgeAlreadyIntersected(edges_to_ignore, currentedgeh));

            sNavEdge* next_shell_edge = GetEdge(currentedgeh);
            if (next_shell_edge->mOriginVertexIdx == currenthostvtxh)
              currenthostvtxh = next_shell_edge->mDestVertexIdx;
            else if (next_shell_edge->mDestVertexIdx == currenthostvtxh)
              currenthostvtxh = next_shell_edge->mOriginVertexIdx;

          } while (currenthostvtxh != endvtx);
        };

        Shell shell_to_retriangulate;

        ObjectHandle firstedgeh = NULL_OBJECTHANDLE;
        if (connected_c_edge_2only[0]->mOriginVertexIdx == v1->mHandle)
          firstedgeh = connected_c_edge_2only[0]->mOriginEdgeCCW;
        else if (connected_c_edge_2only[0]->mDestVertexIdx == v1->mHandle)
          firstedgeh = connected_c_edge_2only[0]->mDestEdgeCCW;

        if (firstedgeh != NULL_OBJECTHANDLE)
        {
          sNavEdge* firstedge = GetEdge(firstedgeh);
          ObjectHandle firsthostvtxh = GetOtherNonSharedVertex(connected_c_edge_2only[0], firstedge, connected_c_edge_2only[0]);

          collectVerticesOfShell(v2->mHandle, firstedgeh, firsthostvtxh,
            shell_to_retriangulate.mShellVerticesBelow, true, unconstrainted_edges);
        }

        firstedgeh = NULL_OBJECTHANDLE;
        if (connected_c_edge_2only[0]->mOriginVertexIdx == v1->mHandle)
          firstedgeh = connected_c_edge_2only[0]->mOriginEdgeCW;
        else if (connected_c_edge_2only[0]->mDestVertexIdx == v1->mHandle)
          firstedgeh = connected_c_edge_2only[0]->mDestEdgeCW;

        if (firstedgeh != NULL_OBJECTHANDLE)
        {
          sNavEdge* firstedge = GetEdge(firstedgeh);
          ObjectHandle firsthostvtxh = GetOtherNonSharedVertex(connected_c_edge_2only[0], firstedge, connected_c_edge_2only[0]);

          collectVerticesOfShell(v2->mHandle, firstedgeh, firsthostvtxh,
            shell_to_retriangulate.mShellVerticesAbove, false, unconstrainted_edges);
        }

        //remove the edges and faces
        for (uint j = 0; j < unconstrainted_edges.size(); ++j)
        {
          sNavEdge* e = GetEdge(unconstrainted_edges[j]);
          RemoveFace(e->mLeftFaceIdx);
          RemoveFace(e->mRightFaceIdx);
          RemoveEdge(e);
        }

        bool mergeboundary = false;
        if (connected_c_edge_2only[0]->IsBoundary() && connected_c_edge_2only[1]->IsBoundary())
          mergeboundary = true;

        RemoveFace(connected_c_edge_2only[0]->mLeftFaceIdx);
        RemoveFace(connected_c_edge_2only[0]->mRightFaceIdx);
        RemoveEdge(connected_c_edge_2only[0]);
        RemoveFace(connected_c_edge_2only[1]->mLeftFaceIdx);
        RemoveFace(connected_c_edge_2only[1]->mRightFaceIdx);
        RemoveEdge(connected_c_edge_2only[1]);
        RemoveVertex(vertices_of_csedges[i]);

        //form the new edge
        sNavEdge* newedge = AddEdge();
        newedge->mOriginVertexIdx = v1->mHandle;
        newedge->mDestVertexIdx = v2->mHandle;
        newedge->mConstraintIndices = indices;
        if (mergeboundary)
          newedge->mFlags |= IS_BOUNDARY;
        mSectorGrid.CheckIn(newedge);

        shell_to_retriangulate.mStartVtx = v1->mHandle;
        shell_to_retriangulate.mEndVtx = v2->mHandle;
        shell_to_retriangulate.mBaseEdgeHandle = newedge->mHandle;
        RetriangulateWithBaseEdge(shell_to_retriangulate);
        //ValidateNavMesh(); //for checking
      }
    }
  }
}

bool cNavMesh::ValidateNavMesh(bool logatend)
{
  bool haserrors = false;
  for (uint i = 0; i < mEdges.size(); ++i)
  {
    if (mEdges[i].InFreeList())
      continue;

    //rotation checks
    sNavEdge* o_cw = GetEdge(mEdges[i].mOriginEdgeCW, false);
    if (!o_cw)
    {
      if (!mEdges[i].IsBoundary())
      {
        LOG("Edge %d mOriginEdgeCW does not exist!", mEdges[i].mHandle);
        haserrors = true;
      }
    }
    else
    {
      if (mEdges[i].mOriginVertexIdx == o_cw->mOriginVertexIdx)
      {
        if (o_cw->mOriginEdgeCCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mOriginEdgeCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mOriginEdgeCW);
          haserrors = true;
        }
      }
      else if (mEdges[i].mOriginVertexIdx == o_cw->mDestVertexIdx)
      {
        if (o_cw->mDestEdgeCCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mOriginEdgeCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mOriginEdgeCW);
          haserrors = true;
        }
      }
    }

    sNavEdge* o_ccw = GetEdge(mEdges[i].mOriginEdgeCCW, false);
    if (!o_ccw)
    {
      if (!mEdges[i].IsBoundary())
      {
        LOG("Edge %d mOriginEdgeCCW does not exist!", mEdges[i].mHandle);
        haserrors = true;
      }
    }
    else
    {
      if (mEdges[i].mOriginVertexIdx == o_ccw->mOriginVertexIdx)
      {
        if (o_ccw->mOriginEdgeCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mOriginEdgeCCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mOriginEdgeCCW);
          haserrors = true;
        }
      }
      else if (mEdges[i].mOriginVertexIdx == o_ccw->mDestVertexIdx)
      {
        if (o_ccw->mDestEdgeCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mOriginEdgeCCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mOriginEdgeCCW);
          haserrors = true;
        }
      }
    }

    sNavEdge* d_cw = GetEdge(mEdges[i].mDestEdgeCW, false);
    if (!d_cw)
    {
      if (!mEdges[i].IsBoundary())
      {
        LOG("Edge %d mDestEdgeCW does not exist!", mEdges[i].mHandle);
        haserrors = true;
      }
    }
    else
    {
      if (mEdges[i].mDestVertexIdx == d_cw->mOriginVertexIdx)
      {
        if (d_cw->mOriginEdgeCCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mDestEdgeCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mDestEdgeCW);
          haserrors = true;
        }
      }
      else if (mEdges[i].mDestVertexIdx == d_cw->mDestVertexIdx)
      {
        if (d_cw->mDestEdgeCCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mDestEdgeCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mDestEdgeCW);
          haserrors = true;
        }
      }
    }

    sNavEdge* d_ccw = GetEdge(mEdges[i].mDestEdgeCCW, false);
    if (!d_ccw)
    {
      if (!mEdges[i].IsBoundary())
      {
        LOG("Edge %d mDestEdgeCCW does not exist!", mEdges[i].mHandle);
        haserrors = true;
      }
    }
    else
    {
      if (mEdges[i].mDestVertexIdx == d_ccw->mOriginVertexIdx)
      {
        if (d_ccw->mOriginEdgeCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mDestEdgeCCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mDestEdgeCCW);
          haserrors = true;
        }
      }
      else if (mEdges[i].mDestVertexIdx == d_ccw->mDestVertexIdx)
      {
        if (d_ccw->mDestEdgeCW != mEdges[i].mHandle)
        {
          LOG("Edge %d mDestEdgeCCW %d might be wrong!", mEdges[i].mHandle, mEdges[i].mDestEdgeCCW);
          haserrors = true;
        }
      }
    }
  }

  for (uint i = 0; i < mFaces.size(); ++i)
  {
    if (mFaces[i].InFreeList())
      continue;

    {
      sNavVertex* v1 = GetVertex(mFaces[i].mVertex1Idx);
      sNavVertex* v2 = GetVertex(mFaces[i].mVertex2Idx);
      sNavVertex* v3 = GetVertex(mFaces[i].mVertex3Idx);

      if (!v1)
      {
        LOG("Face %d vertex 1 does not exist!", mFaces[i].mHandle);
        haserrors = true;
      }
      if (!v2)
      {
        LOG("Face %d vertex 2 does not exist!", mFaces[i].mHandle);
        haserrors = true;
      }
      if (!v3)
      {
        LOG("Face %d vertex 3 does not exist!", mFaces[i].mHandle);
        haserrors = true;
      }

      if (v1 && v2 && v3)
      {
        Vector3 v1_to_v2 = v2->mPosition - v1->mPosition;
        Vector3 v2_to_v3 = v3->mPosition - v2->mPosition;
        Vector3 v3_to_v1 = v1->mPosition - v3->mPosition;

        bool checkSide1 = v1_to_v2.CrossProductObtainSign(v2_to_v3) >= 0.0f;
        bool checkSide2 = v2_to_v3.CrossProductObtainSign(v3_to_v1) >= 0.0f;
        bool checkSide3 = v3_to_v1.CrossProductObtainSign(v1_to_v2) >= 0.0f;

        bool allpositive = checkSide1 && checkSide2 && checkSide3;
        if (allpositive)
        {
          LOG("Face %d might have ccw vertices! Vertices(%d, %d, %d)", mFaces[i].mHandle, v1->mHandle, v2->mHandle, v3->mHandle);
          haserrors = true;
        }
      }
    }

    {
      sNavEdge* e1 = GetEdge(mFaces[i].mEdge1);
      sNavEdge* e2 = GetEdge(mFaces[i].mEdge2);
      sNavEdge* e3 = GetEdge(mFaces[i].mEdge3);

      if (!e1)
      {
        LOG("Face %d edge 1 does not exist!", mFaces[i].mHandle);
        haserrors = true;
      }
      if (!e2)
      {
        LOG("Face %d edge 2 does not exist!", mFaces[i].mHandle);
        haserrors = true;
      }
      if (!e3)
      {
        LOG("Face %d edge 3 does not exist!", mFaces[i].mHandle);
        haserrors = true;
      }

      if (e1 && e2 && e3)
      {
        auto getVector = [this](sNavEdge* e) -> Vector3
        {
          sNavVertex* v1 = GetVertex(e->mOriginVertexIdx);
          sNavVertex* v2 = GetVertex(e->mDestVertexIdx);
          if (v1 && v2)
            return v2->mPosition - v1->mPosition;
          return Vector3::ZERO;
        };

        Vector3 e1vec = getVector(e1);
        Vector3 e2vec = getVector(e2);
        Vector3 e3vec = getVector(e3);
        bool checkSide1 = e1vec.CrossProductObtainSign(e2vec) >= 0.0f;
        bool checkSide2 = e2vec.CrossProductObtainSign(e3vec) >= 0.0f;
        bool checkSide3 = e3vec.CrossProductObtainSign(e1vec) >= 0.0f;

        bool allpositive = checkSide1 && checkSide2 && checkSide3;
        if (allpositive)
        {
          LOG("Face %d might have ccw edges! Edges(%d, %d, %d)", mFaces[i].mHandle, e1->mHandle, e2->mHandle, e3->mHandle);
          haserrors = true;
        }
      }
    }
  }

  if (logatend)
    LOGC("NavMesh validation done.");
  return !haserrors;
}

void cNavMesh::InsertHole(Vector3* points, uint ptcount)
{
  if (ptcount <= 2)
  {
    LOGC("Cannot insert a hole with < 2 points!");
    return;
  }
  InsertNPointPolyConstraint(points, ptcount, IS_INSERTED_BY_UNIT | FACE_REPRESENTS_HOLE);

  {
    //scan the area for faces whose barycenters are inside the polygon
    Vector3 minboxpt = points[0], maxboxpt = points[0];
    for (uint i = 1; i < ptcount; ++i)
    {
      if (minboxpt.x > points[i].x)
        minboxpt.x = points[i].x;
      if (minboxpt.z > points[i].z)
        minboxpt.z = points[i].z;

      if (maxboxpt.x < points[i].x)
        maxboxpt.x = points[i].x;
      if (maxboxpt.z < points[i].z)
        maxboxpt.z = points[i].z;
    }

    AABB3D box(minboxpt, maxboxpt);
    std::vector<sSectorCoords> sectorstosearch;
    mSectorGrid.LocateSectorsOfBox(sectorstosearch, box);

    for (auto sector : sectorstosearch)
    {
      sSector* s = mSectorGrid.GetSector(sector);
      for (auto triangleh : s->mTrianglesInSector)
      {
        sNavFace* f = GetFace(triangleh);
        Vector3 midpt = f->GetFaceMidPoint(this);
        
        //LOG("%d: (%f, %f, %f)", f->mHandle, midpt.x, midpt.y, midpt.z);
        if (f->mFlags & FACE_REPRESENTS_HOLE)
          continue;
        if (!box.Intersects(midpt)) //broadphase
          continue;
        
        bool intersect = Point_NPointPoly_Intersection2D(midpt, points, ptcount);
        if (intersect)
        {
          f->mFlags |= FACE_REPRESENTS_HOLE;
          f->mFlags &= ~FACE_REPRESENTS_BLOCKER;
        }
      }
    }
  } 

  ValidateNavMesh(true);
}

ObjectHandle cNavMesh::InsertVertex(const Vector3& point, int stopatstep)
{
  ObjectHandle ret = NULL_OBJECTHANDLE;
  PointLocateResult res = LocatePoint(point);
  if (res.mResultId == IS_EXISTING_VTX)
  {
    return res.mVertexId;
  }
  else if (res.mResultId == ON_EXISTING_EDGE)
  {
    sNavEdge* edge = GetEdge(res.mEdgeId);

    sNavEdge temp_edge = *edge; //we will be adding edges, so the old pointer might not be valid
    
    sNavVertex* newvtx = AddVertex();
    newvtx->mPosition = res.mProjectedPoint;
    mSectorGrid.CheckIn(newvtx);

    ret = newvtx->mHandle;
    ObjectHandle inserted_vtx_handle = newvtx->mHandle;

    //also need to account for edges without left or right faces. This situation usally occurs at boundary edges
    bool hasleftface = edge->mLeftFaceIdx != NULL_OBJECTHANDLE;
    bool hasrightface = edge->mRightFaceIdx != NULL_OBJECTHANDLE;

    //due to realloc issues, allocate all objects first before using their pointers
    ObjectHandle edge_newVtxToOldOriginHandle = AddEdgeGetHandle();
    ObjectHandle edge_newVtxToOldDestHandle = AddEdgeGetHandle();
    sNavEdge* edge_newVtxToOldOrigin = GetEdge(edge_newVtxToOldOriginHandle);
    sNavEdge* edge_newVtxToOldDest = GetEdge(edge_newVtxToOldDestHandle);

    ObjectHandle newedgeLHandle = NULL_OBJECTHANDLE;
    ObjectHandle newedgeRHandle = NULL_OBJECTHANDLE;
    sNavEdge* newedgeL = nullptr;
    sNavEdge* newedgeR = nullptr;
    if (hasleftface)
      newedgeLHandle = AddEdgeGetHandle();
    if (hasrightface)
      newedgeRHandle = AddEdgeGetHandle();
    if (hasleftface)
      newedgeL = GetEdge(newedgeLHandle);
    if (hasrightface)
      newedgeR = GetEdge(newedgeRHandle);
    
    //again, mind the reallocation in std::vectors
    ObjectHandle topleft = NULL_OBJECTHANDLE;
    ObjectHandle botleft = NULL_OBJECTHANDLE;
    ObjectHandle topright = NULL_OBJECTHANDLE;
    ObjectHandle botright = NULL_OBJECTHANDLE;
    if (hasleftface)
    {
      topleft = AddFaceGetHandle();
      botleft = AddFaceGetHandle();
    }
    if (hasrightface)
    {
      topright = AddFaceGetHandle();
      botright = AddFaceGetHandle();
    }

    sNavFace* topleftface = nullptr;
    sNavFace* botleftface = nullptr;
    sNavFace* toprightface = nullptr;
    sNavFace* botrightface = nullptr;
    if (hasleftface)
    {
      topleftface = GetFace(topleft);
      botleftface = GetFace(botleft);
    }
    if (hasrightface)
    {
      toprightface = GetFace(topright);
      botrightface = GetFace(botright);
    }

    {
      //do the edge splits
      SplitEdge(&temp_edge, newvtx, edge_newVtxToOldOrigin, edge_newVtxToOldDest);

      edge_newVtxToOldOrigin->mDestEdgeCW = temp_edge.mOriginEdgeCW;
      edge_newVtxToOldDest->mDestEdgeCCW = temp_edge.mDestEdgeCCW;

      edge_newVtxToOldOrigin->mOriginEdgeCCW = edge_newVtxToOldDest->mHandle;
      edge_newVtxToOldDest->mOriginEdgeCW = edge_newVtxToOldOrigin->mHandle;

      if (temp_edge.IsBoundary())
      {
        edge_newVtxToOldOrigin->mFlags |= IS_BOUNDARY;
        edge_newVtxToOldDest->mFlags |= IS_BOUNDARY;

        //special edge cases for boundary edges
        if (!hasleftface)
        {
          edge_newVtxToOldOrigin->mOriginEdgeCW = NULL_OBJECTHANDLE;
          edge_newVtxToOldOrigin->mDestEdgeCCW = NULL_OBJECTHANDLE;
          edge_newVtxToOldDest->mOriginEdgeCCW = NULL_OBJECTHANDLE;
          edge_newVtxToOldDest->mDestEdgeCW = NULL_OBJECTHANDLE;
        }
        if (!hasrightface)
        {
          edge_newVtxToOldOrigin->mOriginEdgeCCW = NULL_OBJECTHANDLE;
          edge_newVtxToOldOrigin->mDestEdgeCW = NULL_OBJECTHANDLE;
          edge_newVtxToOldDest->mOriginEdgeCW = NULL_OBJECTHANDLE;
          edge_newVtxToOldDest->mDestEdgeCCW = NULL_OBJECTHANDLE;
        }
      }
      else
      {
        edge_newVtxToOldDest->mOriginEdgeCCW = edge_newVtxToOldOrigin->mHandle;
        edge_newVtxToOldOrigin->mOriginEdgeCW = edge_newVtxToOldDest->mHandle;

        edge_newVtxToOldOrigin->mDestEdgeCCW = temp_edge.mOriginEdgeCCW;
        edge_newVtxToOldDest->mDestEdgeCW = temp_edge.mDestEdgeCW;
      }
      mSectorGrid.CheckIn(edge_newVtxToOldOrigin);
      mSectorGrid.CheckIn(edge_newVtxToOldDest);
    }

    if (stopatstep & AFTER_EDGE_SPLIT_STEP)
      return ret;

    {
      //form the 2 new crossed edges
      if (hasleftface)
      {
        ObjectHandle endpt_L = NULL_OBJECTHANDLE;
        if (GetEdge(temp_edge.LeftFaceOriginEdge())->mOriginVertexIdx != temp_edge.mOriginVertexIdx)
          endpt_L = GetEdge(temp_edge.LeftFaceOriginEdge())->mOriginVertexIdx;
        else
          endpt_L = GetEdge(temp_edge.LeftFaceOriginEdge())->mDestVertexIdx;
        ASSERTLOGC(endpt_L != NULL_OBJECTHANDLE, "Invalid vertices");

        newedgeL->mOriginVertexIdx = inserted_vtx_handle;
        newedgeL->mDestVertexIdx = endpt_L;
        newedgeL->mOriginEdgeCCW = edge_newVtxToOldOrigin->mHandle;
        newedgeL->mOriginEdgeCW = edge_newVtxToOldDest->mHandle;
        newedgeL->mDestEdgeCCW = temp_edge.mDestEdgeCW;
        newedgeL->mDestEdgeCW = temp_edge.mOriginEdgeCCW;
        mSectorGrid.CheckIn(newedgeL);

        edge_newVtxToOldOrigin->mOriginEdgeCW = newedgeL->mHandle;
        edge_newVtxToOldDest->mOriginEdgeCCW = newedgeL->mHandle;
      }

      if (hasrightface)
      {
        ObjectHandle endpt_R = NULL_OBJECTHANDLE;
        if (GetEdge(temp_edge.RightFaceOriginEdge())->mOriginVertexIdx != temp_edge.mOriginVertexIdx)
          endpt_R = GetEdge(temp_edge.RightFaceOriginEdge())->mOriginVertexIdx;
        else
          endpt_R = GetEdge(temp_edge.RightFaceOriginEdge())->mDestVertexIdx;
        ASSERTLOGC(endpt_R != NULL_OBJECTHANDLE, "Invalid vertices");

        newedgeR->mOriginVertexIdx = inserted_vtx_handle;
        newedgeR->mDestVertexIdx = endpt_R;
        newedgeR->mOriginEdgeCCW = edge_newVtxToOldDest->mHandle;
        newedgeR->mOriginEdgeCW = edge_newVtxToOldOrigin->mHandle;
        newedgeR->mDestEdgeCCW = temp_edge.mOriginEdgeCW;
        newedgeR->mDestEdgeCW = temp_edge.mDestEdgeCCW;
        mSectorGrid.CheckIn(newedgeR);

        edge_newVtxToOldOrigin->mOriginEdgeCCW = newedgeR->mHandle;
        edge_newVtxToOldDest->mOriginEdgeCW = newedgeR->mHandle;
      }
    }

    {
      //update faces

      //our split and new edges point outwards from the newly inserted vtx
      //note: ensure the vertices and edges are in clockwise order
      if (hasleftface)
      {
        sNavFace* f = GetFace(temp_edge.mLeftFaceIdx);
        CopyGameplayFlags(f, botleftface);
        CopyGameplayFlags(f, topleftface);

        RemoveFace(temp_edge.mLeftFaceIdx);
        botleftface->mVertex1Idx = edge_newVtxToOldOrigin->mOriginVertexIdx;
        botleftface->mVertex2Idx = edge_newVtxToOldOrigin->mDestVertexIdx;
        botleftface->mVertex3Idx = newedgeL->mDestVertexIdx;
        botleftface->mEdge1 = edge_newVtxToOldOrigin->mHandle;
        botleftface->mEdge2 = edge_newVtxToOldOrigin->mDestEdgeCCW;
        botleftface->mEdge3 = newedgeL->mHandle;

        topleftface->mVertex1Idx = edge_newVtxToOldDest->mOriginVertexIdx;
        topleftface->mVertex2Idx = newedgeL->mDestVertexIdx;
        topleftface->mVertex3Idx = edge_newVtxToOldDest->mDestVertexIdx;
        topleftface->mEdge1 = edge_newVtxToOldDest->mHandle;
        topleftface->mEdge2 = newedgeL->mHandle;
        topleftface->mEdge3 = edge_newVtxToOldDest->mDestEdgeCW;

        mSectorGrid.CheckIn(botleftface);
        mSectorGrid.CheckIn(topleftface);

        newedgeL->mLeftFaceIdx = botleft;
        newedgeL->mRightFaceIdx = topleft;
      }

      if (hasrightface)
      {
        sNavFace* f = GetFace(temp_edge.mRightFaceIdx);
        CopyGameplayFlags(f, botrightface);
        CopyGameplayFlags(f, toprightface);

        RemoveFace(temp_edge.mRightFaceIdx);
        botrightface->mVertex1Idx = edge_newVtxToOldOrigin->mOriginVertexIdx;
        botrightface->mVertex2Idx = newedgeR->mDestVertexIdx;
        botrightface->mVertex3Idx = edge_newVtxToOldOrigin->mDestVertexIdx;
        botrightface->mEdge1 = edge_newVtxToOldOrigin->mHandle;
        botrightface->mEdge2 = newedgeR->mHandle;
        botrightface->mEdge3 = edge_newVtxToOldOrigin->mDestEdgeCW;

        toprightface->mVertex1Idx = edge_newVtxToOldDest->mOriginVertexIdx;
        toprightface->mVertex2Idx = edge_newVtxToOldDest->mDestVertexIdx;
        toprightface->mVertex3Idx = newedgeR->mDestVertexIdx;
        toprightface->mEdge1 = edge_newVtxToOldDest->mHandle;
        toprightface->mEdge2 = edge_newVtxToOldDest->mDestEdgeCCW;
        toprightface->mEdge3 = newedgeR->mHandle;

        mSectorGrid.CheckIn(botrightface);
        mSectorGrid.CheckIn(toprightface);

        //update faces of edges
        edge_newVtxToOldOrigin->mLeftFaceIdx = botright;
        edge_newVtxToOldOrigin->mRightFaceIdx = botleft;
        edge_newVtxToOldDest->mLeftFaceIdx = topleft;
        edge_newVtxToOldDest->mRightFaceIdx = topright;

        newedgeR->mLeftFaceIdx = topright;
        newedgeR->mRightFaceIdx = botright;
      }
    }

    sNavEdge* shell_L = nullptr, *shell_T = nullptr, *shell_R = nullptr, *shell_B = nullptr;
    {
      //update the edges of the quadrilateral shell
      //if (hasleftface)
      {
        if (edge_newVtxToOldOrigin->mDestEdgeCCW != NULL_OBJECTHANDLE)
        {
          shell_L = GetEdge(edge_newVtxToOldOrigin->mDestEdgeCCW);
          if (shell_L->mOriginVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
          {
            shell_L->mOriginEdgeCW = edge_newVtxToOldOrigin->mHandle;
            shell_L->mDestEdgeCCW = newedgeLHandle;
            shell_L->mRightFaceIdx = botleft;
          }
          else if (shell_L->mDestVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
          {
            shell_L->mDestEdgeCW = edge_newVtxToOldOrigin->mHandle;
            shell_L->mOriginEdgeCCW = newedgeLHandle;
            shell_L->mLeftFaceIdx = botleft;
          }
          else
            ASSERTLOGC(0, "shell_L edge does not share vertices!");
        }

        if (edge_newVtxToOldDest->mDestEdgeCW != NULL_OBJECTHANDLE)
        {
          shell_T = GetEdge(edge_newVtxToOldDest->mDestEdgeCW);
          if (shell_T->mOriginVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
          {
            shell_T->mOriginEdgeCCW = edge_newVtxToOldDest->mHandle;
            shell_T->mDestEdgeCW = newedgeLHandle;
            shell_T->mLeftFaceIdx = topleft;
          }
          else if (shell_T->mDestVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
          {
            shell_T->mDestEdgeCCW = edge_newVtxToOldDest->mHandle;
            shell_T->mOriginEdgeCW = newedgeLHandle;
            shell_T->mRightFaceIdx = topleft;
          }
          else
            ASSERTLOGC(0, "shell_T edge does not share vertices!");
        }
      }

      //if (hasrightface)
      {
        if (edge_newVtxToOldDest->mDestEdgeCCW != NULL_OBJECTHANDLE)
        {
          shell_R = GetEdge(edge_newVtxToOldDest->mDestEdgeCCW);
          if (shell_R->mOriginVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
          {
            shell_R->mOriginEdgeCW = edge_newVtxToOldDest->mHandle;
            shell_R->mDestEdgeCCW = newedgeRHandle;
            shell_R->mRightFaceIdx = topright;
          }
          else if (shell_R->mDestVertexIdx == edge_newVtxToOldDest->mDestVertexIdx)
          {
            shell_R->mDestEdgeCW = edge_newVtxToOldDest->mHandle;
            shell_R->mOriginEdgeCCW = newedgeRHandle;
            shell_R->mLeftFaceIdx = topright;
          }
          else
            ASSERTLOGC(0, "shell_R edge does not share vertices!");
        }

        if (edge_newVtxToOldOrigin->mDestEdgeCW != NULL_OBJECTHANDLE)
        {
          shell_B = GetEdge(edge_newVtxToOldOrigin->mDestEdgeCW);
          if (shell_B->mOriginVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
          {
            shell_B->mOriginEdgeCCW = edge_newVtxToOldOrigin->mHandle;
            shell_B->mDestEdgeCW = newedgeRHandle;
            shell_B->mLeftFaceIdx = botright;
          }
          else if (shell_B->mDestVertexIdx == edge_newVtxToOldOrigin->mDestVertexIdx)
          {
            shell_B->mDestEdgeCCW = edge_newVtxToOldOrigin->mHandle;
            shell_B->mOriginEdgeCW = newedgeRHandle;
            shell_B->mRightFaceIdx = botright;
          }
          else
            ASSERTLOGC(0, "shell_B edge does not share vertices!");
        }
      }
    }

    //push 4 edges of the quadilateral shell
    std::deque<ObjectHandle> edgestack;
    if (hasleftface)
    {
      edgestack.push_back(temp_edge.mOriginEdgeCCW);
      edgestack.push_back(temp_edge.mDestEdgeCW);
    }
    if (hasrightface)
    {
      edgestack.push_back(temp_edge.mDestEdgeCCW);
      edgestack.push_back(temp_edge.mOriginEdgeCW);
    }

    RemoveEdge(&temp_edge);
    if (stopatstep & BEFORE_FLIP_STEP)
      return ret;
    FlipEdges(inserted_vtx_handle, edgestack);
    
  }
  else if (res.mResultId == ON_EXISTING_FACE)
  {
    sNavVertex* newvtx = AddVertex();
    newvtx->mPosition = res.mProjectedPoint;
    mSectorGrid.CheckIn(newvtx);
    ret = newvtx->mHandle;
    ObjectHandle inserted_vtx_handle = newvtx->mHandle;

    ObjectHandle newedge1h = AddEdgeGetHandle();
    ObjectHandle newedge2h = AddEdgeGetHandle();
    ObjectHandle newedge3h = AddEdgeGetHandle();

    sNavEdge* newedge1 = GetEdge(newedge1h);
    sNavEdge* newedge2 = GetEdge(newedge2h);
    sNavEdge* newedge3 = GetEdge(newedge3h);

    ObjectHandle newface1h = AddFaceGetHandle();
    ObjectHandle newface2h = AddFaceGetHandle();
    ObjectHandle newface3h = AddFaceGetHandle();
    
    sNavFace* face = GetFace(res.mFaceId);
    sNavFace* face1 = GetFace(newface1h);
    sNavFace* face2 = GetFace(newface2h);
    sNavFace* face3 = GetFace(newface3h);

    //triangular shell
    sNavEdge* shell_edge1 = GetEdge(face->mEdge1);
    sNavEdge* shell_edge2 = GetEdge(face->mEdge2);
    sNavEdge* shell_edge3 = GetEdge(face->mEdge3);

    {
      //form new edges, originating from the new vertex. Base all of this on the edge information given in the face
      newedge1->mOriginVertexIdx = inserted_vtx_handle;
      newedge1->mDestVertexIdx = GetSharedVertex(shell_edge1, shell_edge3);
      newedge1->mOriginEdgeCCW = newedge3->mHandle;
      newedge1->mOriginEdgeCW = newedge2->mHandle;
      newedge1->mDestEdgeCCW = shell_edge1->mHandle;
      newedge1->mDestEdgeCW = shell_edge3->mHandle;
      newedge1->mLeftFaceIdx = face3->mHandle;
      newedge1->mRightFaceIdx = face1->mHandle;

      newedge2->mOriginVertexIdx = inserted_vtx_handle;
      newedge2->mDestVertexIdx = GetSharedVertex(shell_edge1, shell_edge2);
      newedge2->mOriginEdgeCCW = newedge1->mHandle;
      newedge2->mOriginEdgeCW = newedge3->mHandle;
      newedge2->mDestEdgeCCW = shell_edge2->mHandle;
      newedge2->mDestEdgeCW = shell_edge1->mHandle;
      newedge2->mLeftFaceIdx = face1->mHandle;
      newedge2->mRightFaceIdx = face2->mHandle;

      newedge3->mOriginVertexIdx = inserted_vtx_handle;
      newedge3->mDestVertexIdx = GetSharedVertex(shell_edge2, shell_edge3);
      newedge3->mOriginEdgeCCW = newedge2->mHandle;
      newedge3->mOriginEdgeCW = newedge1->mHandle;
      newedge3->mDestEdgeCCW = shell_edge3->mHandle;
      newedge3->mDestEdgeCW = shell_edge2->mHandle;
      newedge3->mLeftFaceIdx = face2->mHandle;
      newedge3->mRightFaceIdx = face3->mHandle;

      mSectorGrid.CheckIn(newedge1);
      mSectorGrid.CheckIn(newedge2);
      mSectorGrid.CheckIn(newedge3);
    }

    {
      //update faces
      CopyGameplayFlags(face, face1);
      CopyGameplayFlags(face, face2);
      CopyGameplayFlags(face, face3);

      RemoveFace(face);

      face1->mVertex1Idx = inserted_vtx_handle;
      face1->mVertex2Idx = GetSharedVertex(shell_edge3, shell_edge1);
      face1->mVertex3Idx = GetSharedVertex(shell_edge1, shell_edge2);
      face1->mEdge1 = newedge1->mHandle;
      face1->mEdge2 = shell_edge1->mHandle;
      face1->mEdge3 = newedge2->mHandle;

      face2->mVertex1Idx = inserted_vtx_handle;
      face2->mVertex2Idx = GetSharedVertex(shell_edge1, shell_edge2);
      face2->mVertex3Idx = GetSharedVertex(shell_edge2, shell_edge3);
      face2->mEdge1 = newedge2->mHandle;
      face2->mEdge2 = shell_edge2->mHandle;
      face2->mEdge3 = newedge3->mHandle;

      face3->mVertex1Idx = inserted_vtx_handle;
      face3->mVertex2Idx = GetSharedVertex(shell_edge2, shell_edge3);
      face3->mVertex3Idx = GetSharedVertex(shell_edge1, shell_edge3);
      face3->mEdge1 = newedge3->mHandle;
      face3->mEdge2 = shell_edge3->mHandle;
      face3->mEdge3 = newedge1->mHandle;

      mSectorGrid.CheckIn(face1);
      mSectorGrid.CheckIn(face2);
      mSectorGrid.CheckIn(face3);
    }

    {
      //update shells
      if (newedge1->mDestVertexIdx == shell_edge1->mOriginVertexIdx)
      {
        shell_edge1->mOriginEdgeCW = newedge1->mHandle;
        shell_edge1->mDestEdgeCCW = newedge2->mHandle;
        shell_edge1->mRightFaceIdx = face1->mHandle;
      }
      else if (newedge1->mDestVertexIdx == shell_edge1->mDestVertexIdx)
      {
        shell_edge1->mDestEdgeCW = newedge1->mHandle;
        shell_edge1->mOriginEdgeCCW = newedge2->mHandle;
        shell_edge1->mLeftFaceIdx = face1->mHandle;
      }
      else
        ASSERTLOGC(0, "newedge1 does not share a vertex with shell_edge1!");

      if (newedge2->mDestVertexIdx == shell_edge2->mOriginVertexIdx)
      {
        shell_edge2->mOriginEdgeCW = newedge2->mHandle;
        shell_edge2->mDestEdgeCCW = newedge3->mHandle;
        shell_edge2->mRightFaceIdx = face2->mHandle;
      }
      else if (newedge2->mDestVertexIdx == shell_edge2->mDestVertexIdx)
      {
        shell_edge2->mDestEdgeCW = newedge2->mHandle;
        shell_edge2->mOriginEdgeCCW = newedge3->mHandle;
        shell_edge2->mLeftFaceIdx = face2->mHandle;
      }
      else
        ASSERTLOGC(0, "newedge2 does not share a vertex with shell_edge2!");

      if (newedge3->mDestVertexIdx == shell_edge3->mOriginVertexIdx)
      {
        shell_edge3->mOriginEdgeCW = newedge3->mHandle;
        shell_edge3->mDestEdgeCCW = newedge1->mHandle;
        shell_edge3->mRightFaceIdx = face3->mHandle;
      }
      else if (newedge3->mDestVertexIdx == shell_edge3->mDestVertexIdx)
      {
        shell_edge3->mDestEdgeCW = newedge3->mHandle;
        shell_edge3->mOriginEdgeCCW = newedge1->mHandle;
        shell_edge3->mLeftFaceIdx = face3->mHandle;
      }
      else
        ASSERTLOGC(0, "newedge3 does not share a vertex with shell_edge3!");
    }

    std::deque<ObjectHandle> edgestack;
    edgestack.push_back(shell_edge1->mHandle);
    edgestack.push_back(shell_edge2->mHandle);
    edgestack.push_back(shell_edge3->mHandle);
    if (stopatstep & BEFORE_FLIP_STEP)
      return ret;
    FlipEdges(inserted_vtx_handle, edgestack);
  }
  else
  {
    LOG("Failed to insert point %f, %f, %f", point.x, point.y, point.z);
  }
  return ret;
}

PointLocateResult cNavMesh::LocatePoint(const Vector3& pt)
{
  std::vector<sSectorCoords> sectors;
  sectors.reserve(4);
  mSectorGrid.LocateSectors(sectors, pt);

  PointLocateResult res;

  for (uint j = 0; j < sectors.size(); ++j)
  {
    sSectorCoords s = sectors[j];
    uint arrayidx = mSectorGrid.GetArrayIdx(s.mX, s.mZ);
    sSector* sector = mSectorGrid.mSectors + arrayidx;

    //mSectorGrid.
    for (int i = 0; i < sector->mVerticesInSector.size(); ++i)
    {
      sNavVertex* v = GetVertex(sector->mVerticesInSector[i]);
      if (!v->InFreeList() && v->mPosition.IsEqual(pt, NAVMESH_EPSILON))
      {
        res.mVertexId = v->mHandle;
        res.mResultId = IS_EXISTING_VTX;
        res.mProjectedPoint = v->mPosition;
        return res;
      }
    }

    for (int i = 0; i < sector->mEdgesInSector.size(); ++i)
    {
      sNavEdge* e = GetEdge(sector->mEdgesInSector[i]);
      if (!e->InFreeList() && e->AreVerticesValid())
      {
        LineSegment3D lineseg(GetVertex(e->mOriginVertexIdx)->mPosition, GetVertex(e->mDestVertexIdx)->mPosition);
        Vector3 intersectpt;

        if (lineseg.Intersects(pt, intersectpt, NAVMESH_EPSILON))
        {
          res.mEdgeId = e->mHandle;
          res.mResultId = ON_EXISTING_EDGE;
          res.mProjectedPoint = pt;
          return res;
        }
      }
    }

    for (int i = 0; i < sector->mTrianglesInSector.size(); ++i)
    {
      sNavFace* f = GetFace(sector->mTrianglesInSector[i]);
      if (f->InFreeList())
        continue;
      
      Vector3 p1 = GetVertex(f->mVertex1Idx)->mPosition;
      Vector3 p2 = GetVertex(f->mVertex2Idx)->mPosition;
      Vector3 p3 = GetVertex(f->mVertex3Idx)->mPosition;

      Vector3 a = p2 - p1;
      Vector3 b = p3 - p1;

      Vector3 normal = a.CrossProduct(b);
      Plane p(normal, p1);
      /*
      if (f->mHandle == 65538 || f->mHandle == 65541)
      DebugBreak();*/
      if (p.GetSide(pt) == PLANE_SIDE_ON)
      { 
        //do accurate point in triangle test
        //http://totologic.blogspot.fr/2014/01/accurate-point-in-triangle-test.html
        bool intersect = AccurateTriangle_Pt_Intersection(p1, p2, p3, pt, NAVMESH_EPSILON);
        if (!intersect)
          continue;

        //pass
        res.mFaceId = f->mHandle;
        res.mResultId = ON_EXISTING_FACE;
        //res.mProjectedPoint = GetVertex(mFaces[i].mVertex1Idx)->mPosition + frame_a * a + frame_b * b; //major fp precision loss
        res.mProjectedPoint = pt;
        return res;
      }
    }
  }
  return res;
}

void cNavMesh::FlipEdges(ObjectHandle vtxidx, std::deque<ObjectHandle>& edgestack)
{
  Vector3 p = GetVertex(vtxidx)->mPosition;

  while (!edgestack.empty())
  {
    ObjectHandle edgeidx = edgestack.front();
    edgestack.pop_front();
    sNavEdge* e = GetEdge(edgeidx);

    if (e && e->mConstraintIndices.empty() && !e->IsBoundary() && IsFlippable(e))
    {
      Vector3 v1 = GetVertex(e->mOriginVertexIdx)->mPosition;
      Vector3 v2 = GetVertex(e->mDestVertexIdx)->mPosition;

      Vector3 circlecenter = Vector3::ZERO;
      float circlesqradius = 0.0f;
      bool validcircle = CircleFrom3Pts(v1, v2, p, circlecenter, circlesqradius);
      if (!validcircle)
      {
        LOGC("Constructed Circle not valid");
        //continue;
      }

      ObjectHandle testervtx = NULL_OBJECTHANDLE;
      sNavEdge* leftfaceedge = GetEdge(e->LeftFaceOriginEdge());
      ObjectHandle leftothervtx = GetOtherNonSharedVertex(e, leftfaceedge, e);
      sNavEdge* rightfaceedge = GetEdge(e->RightFaceOriginEdge());
      ObjectHandle rightothervtx = GetOtherNonSharedVertex(e, rightfaceedge, e);
      if (leftothervtx == vtxidx)
        testervtx = rightothervtx;
      else if (rightothervtx == vtxidx)
        testervtx = leftothervtx;

      sNavVertex* testvtx = GetVertex(testervtx);
      float testervtx_sqdist = circlecenter.SquaredDistance(testvtx->mPosition);
      if (!validcircle || (testervtx_sqdist < (circlesqradius - NAVMESH_EPSILONSQ)))
      {
        if (leftothervtx == vtxidx)
        {
          edgestack.push_front(rightfaceedge->mHandle);
          edgestack.push_front(e->RightFaceForwardEdge());
          Flip(e);
        }
        else if (rightothervtx == vtxidx)
        {
          edgestack.push_front(leftfaceedge->mHandle);
          edgestack.push_front(e->LeftFaceForwardEdge());
          Flip(e);
        }
      }
    }
  }
}

void cNavMesh::Flip(sNavEdge* edge)
{
  if (!edge->mConstraintIndices.empty())
    return;

  mSectorGrid.CheckOut(edge);
  //IsFlippable() should be already checked
  //edge cannot be constrainted
  sNavEdge* leftoriginedge = GetEdge(edge->LeftFaceOriginEdge());
  sNavEdge* rightoriginedge = GetEdge(edge->RightFaceOriginEdge());
  sNavEdge* leftforwardedge = GetEdge(edge->mDestEdgeCW);
  sNavEdge* rightforwardedge = GetEdge(edge->mDestEdgeCCW);

  ObjectHandle leftvtx = GetOtherNonSharedVertex(edge, leftoriginedge, edge);
  ObjectHandle rightvtx = GetOtherNonSharedVertex(edge, rightoriginedge, edge);
  ObjectHandle oldorigin = edge->mOriginVertexIdx;
  ObjectHandle olddest = edge->mDestVertexIdx;

  //connect edge to other 2 vertices
  edge->mOriginVertexIdx = leftvtx;
  edge->mDestVertexIdx = rightvtx;
  edge->mOriginEdgeCW = leftoriginedge->mHandle;
  edge->mOriginEdgeCCW = leftforwardedge->mHandle;
  edge->mDestEdgeCCW = rightoriginedge->mHandle;
  edge->mDestEdgeCW = rightforwardedge->mHandle;
  mSectorGrid.CheckIn(edge);

  //update faces
  uint flagscopied = 0;
  {
    sNavFace* lface = GetFace(edge->mLeftFaceIdx);
    sNavFace* rface = GetFace(edge->mRightFaceIdx);

    //only non constrainted edges are flipped, and holes/gameplay stuff are always
    //encompassed by constrainted edges, hence both faces are either non-holes or holes.
    if ((lface->mFlags & FACE_REPRESENTS_HOLE) || (rface->mFlags & FACE_REPRESENTS_HOLE))
      flagscopied |= FACE_REPRESENTS_HOLE;
    if ((lface->mFlags & FACE_REPRESENTS_BLOCKER) || (rface->mFlags & FACE_REPRESENTS_BLOCKER))
      flagscopied |= FACE_REPRESENTS_BLOCKER;
  }
  RemoveFace(edge->mLeftFaceIdx);
  RemoveFace(edge->mRightFaceIdx);

  ObjectHandle newleftfaceh = AddFaceGetHandle();
  ObjectHandle newrightfaceh = AddFaceGetHandle();  

  sNavFace* newleftface = GetFace(newleftfaceh);
  sNavFace* newrightface = GetFace(newrightfaceh);

  newleftface->mFlags |= flagscopied;
  newrightface->mFlags |= flagscopied;

  newleftface->mVertex1Idx = leftvtx;
  newleftface->mVertex2Idx = olddest;
  newleftface->mVertex3Idx = rightvtx;

  newleftface->mEdge1 = edge->mHandle;
  newleftface->mEdge2 = leftforwardedge->mHandle;
  newleftface->mEdge3 = rightforwardedge->mHandle;

  newrightface->mVertex1Idx = leftvtx;
  newrightface->mVertex2Idx = rightvtx;
  newrightface->mVertex3Idx = oldorigin;

  newrightface->mEdge1 = edge->mHandle;
  newrightface->mEdge2 = rightoriginedge->mHandle;
  newrightface->mEdge3 = leftoriginedge->mHandle;

  mSectorGrid.CheckIn(newleftface);
  mSectorGrid.CheckIn(newrightface);

  edge->mLeftFaceIdx = newleftfaceh;
  edge->mRightFaceIdx = newrightfaceh;

  //update old edges
  {
    //edges at the old origin
    if (leftoriginedge->mOriginVertexIdx == oldorigin)
    {
      leftoriginedge->mOriginEdgeCW = rightoriginedge->mHandle;
      leftoriginedge->mDestEdgeCCW = edge->mHandle;
      leftoriginedge->mRightFaceIdx = newrightfaceh;
    }
    else if (leftoriginedge->mDestVertexIdx == oldorigin)
    {
      leftoriginedge->mDestEdgeCW = rightoriginedge->mHandle;
      leftoriginedge->mOriginEdgeCCW = edge->mHandle;
      leftoriginedge->mLeftFaceIdx = newrightfaceh;
    }
    else
      ASSERTLOGC(0, "left origin edge does not share vertices!");

    if (rightoriginedge->mOriginVertexIdx == oldorigin)
    {
      rightoriginedge->mOriginEdgeCCW = leftoriginedge->mHandle;
      rightoriginedge->mDestEdgeCW = edge->mHandle;
      rightoriginedge->mLeftFaceIdx = newrightfaceh;
    }
    else if (rightoriginedge->mDestVertexIdx == oldorigin)
    {
      rightoriginedge->mDestEdgeCCW = leftoriginedge->mHandle;
      rightoriginedge->mOriginEdgeCW = edge->mHandle;
      rightoriginedge->mRightFaceIdx = newrightfaceh;
    }
    else
      ASSERTLOGC(0, "right origin edge does not share vertices!");

    //forward edges at the old dest
    if (leftforwardedge->mOriginVertexIdx == olddest) //origin of the left dest edge shares old dest
    {
      leftforwardedge->mOriginEdgeCCW = rightforwardedge->mHandle;
      leftforwardedge->mDestEdgeCW = edge->mHandle;
      leftforwardedge->mLeftFaceIdx = newleftfaceh;
    }
    else if (leftforwardedge->mDestVertexIdx == olddest)
    {
      leftforwardedge->mDestEdgeCCW = rightforwardedge->mHandle;
      leftforwardedge->mOriginEdgeCW = edge->mHandle;
      leftforwardedge->mRightFaceIdx = newleftfaceh;
    }
    else
      ASSERTLOGC(0, "left dest edge does not share vertices!");

    if (rightforwardedge->mOriginVertexIdx == olddest)
    {
      rightforwardedge->mOriginEdgeCW = leftforwardedge->mHandle;
      rightforwardedge->mDestEdgeCCW = edge->mHandle;
      rightforwardedge->mRightFaceIdx = newleftfaceh;
    }
    else if (rightforwardedge->mDestVertexIdx == olddest)
    {
      rightforwardedge->mDestEdgeCW = leftforwardedge->mHandle;
      rightforwardedge->mOriginEdgeCCW = edge->mHandle;
      rightforwardedge->mLeftFaceIdx = newleftfaceh;
    }
    else
      ASSERTLOGC(0, "right dest edge does not share vertices!");
  }
}

bool cNavMesh::IsFlippable(sNavEdge* edge)
{
  //IsFlippable()

  //Flippable edges also cannot have reflex vertices in its quadrilaterial shell, (or no 4 sided concave quads)
  //figure 3.35
  //https://books.google.com.sg/books?id=SxY1Xrr12DwC&pg=PA130&lpg=PA130&dq=edge+flip+collinear&source=bl&ots=xVxsPGEe43&sig=LbdZpsp6x1-p_wRSZ3HQrvtOpr4&hl=en&sa=X&ei=cAklVf7LKNaRuATf14DQCA&ved=0CCYQ6AEwAw#v=onepage&q=edge%20flip%20collinear&f=false

  if (edge->mLeftFaceIdx == NULL_OBJECTHANDLE || edge->mRightFaceIdx == NULL_OBJECTHANDLE) //no faces; cannot flip
    return false;

  //edge cannot be constrainted
  sNavEdge* leftoriginedge = GetEdge(edge->LeftFaceOriginEdge());
  sNavEdge* rightoriginedge = GetEdge(edge->RightFaceOriginEdge());

  ObjectHandle leftvtx = GetOtherNonSharedVertex(edge, leftoriginedge, edge);
  ObjectHandle rightvtx = GetOtherNonSharedVertex(edge, rightoriginedge, edge);
  ObjectHandle oldorigin = edge->mOriginVertexIdx;
  ObjectHandle olddest = edge->mDestVertexIdx;

  //get 4 vertices of the quadrilateral
  sNavVertex* left_vtx = GetVertex(leftvtx);
  sNavVertex* right_vtx = GetVertex(rightvtx);
  sNavVertex* oldorigin_vtx = GetVertex(oldorigin);
  sNavVertex* olddest_vtx = GetVertex(olddest);

  Vector3 origin_to_left = left_vtx->mPosition - oldorigin_vtx->mPosition;
  Vector3 left_to_dest = olddest_vtx->mPosition - left_vtx->mPosition;
  Vector3 dest_to_right = right_vtx->mPosition - olddest_vtx->mPosition;
  Vector3 right_to_orgin = oldorigin_vtx->mPosition - right_vtx->mPosition;

  float val1 = origin_to_left.CrossProductObtainSign(left_to_dest);
  if (val1 == 0.0f) //collinear, cannot flip
    return false;
  float val2 = left_to_dest.CrossProductObtainSign(dest_to_right);
  if (val2 == 0.0f)
    return false;
  float val3 = dest_to_right.CrossProductObtainSign(right_to_orgin);
  if (val3 == 0.0f)
    return false;
  float val4 = right_to_orgin.CrossProductObtainSign(origin_to_left);
  if (val4 == 0.0f)
    return false;

  //all must be either -ve or positive
  bool allnegative = (val1 < 0.0f && val2 < 0.0f && val3 < 0.0f && val4 < 0.0f);
  bool allpositive = (val1 > 0.0f && val2 > 0.0f && val3 > 0.0f && val4 > 0.0f);
  if (!allnegative && !allpositive)
    return false;
  return true;
}

void cNavMesh::FindRotationEdgesAboutVtx(ObjectHandle sourcevtxh, const Vector2& edgeprojected, float edge_sqlength, ObjectHandle& ccw_edge, ObjectHandle& cw_edge)
{
  Vector2 perp_edge(-edgeprojected.y, edgeprojected.x);

  sNavVertex* src = GetVertex(sourcevtxh);

  float largestval_positiveside = -FLT_MAX;
  float largestval_negativeside = -FLT_MAX;
  bool is_same_dir_positiveside = false;
  bool is_same_dir_negativeside = false;

  std::vector<sSectorCoords> sectors;
  mSectorGrid.LocateSectors(sectors, src->mPosition);
  for (uint i = 0; i < sectors.size(); ++i)
  {
    sSector* sector = mSectorGrid.GetSector(sectors[i]);
    for (uint k = 0; k < sector->mEdgesInSector.size(); ++k)
    {
      sNavEdge* e = GetEdge(sector->mEdgesInSector[k]);
      ObjectHandle vtx1h = NULL_OBJECTHANDLE, vtx2h = NULL_OBJECTHANDLE;
      if (e->mOriginVertexIdx == sourcevtxh)
      {
        vtx1h = e->mOriginVertexIdx;
        vtx2h = e->mDestVertexIdx;
      }
      else if (e->mDestVertexIdx == sourcevtxh)
      {
        vtx1h = e->mDestVertexIdx;
        vtx2h = e->mOriginVertexIdx;
      }

      if (vtx1h == NULL_OBJECTHANDLE || vtx2h == NULL_OBJECTHANDLE)
        continue;
      
      sNavVertex* v1 = GetVertex(vtx1h);
      sNavVertex* v2 = GetVertex(vtx2h);

      Vector3 vec = v2->mPosition - v1->mPosition;
      float projdotval = edgeprojected.DotProduct(Vector2(vec.x, vec.z));
      bool points_in_same_dir = (projdotval >= 0.0f); 
      
      float perpdotval = perp_edge.DotProduct(Vector2(vec.x, vec.z));
      if (perpdotval >= 0.0f) //positive side of inserted edge
      {
        //edges that point in same direction have a priority over those that dont
        /*if (points_in_same_dir == false && is_same_dir_positiveside == true)
          continue;
        */
        //see derivation reference
        float projdotvalsq = projdotval * projdotval;
        float divisor = edge_sqlength * vec.SquaredLength();
        float val = projdotvalsq / divisor;

        if (points_in_same_dir == false)
          val = -val;

        if (largestval_positiveside < val)
        {
          largestval_positiveside = val;
          ccw_edge = sector->mEdgesInSector[k];
          is_same_dir_positiveside = points_in_same_dir;
        }
        if (val > 1.0f) //cos 0 to 90 does not have values beyond 0 and 1
          ASSERTLOGC(0, "Computed value more than 1!");
      }
      else
      {
        /*if (points_in_same_dir == false && is_same_dir_negativeside == true)
          continue;*/

        float projdotvalsq = projdotval * projdotval;
        float divisor = edge_sqlength * vec.SquaredLength();
        float val = projdotvalsq / divisor;

        if (points_in_same_dir == false)
          val = -val;

        if (largestval_negativeside < val)
        {
          largestval_negativeside = val;
          cw_edge = sector->mEdgesInSector[k];
          is_same_dir_negativeside = points_in_same_dir;
        }

        if (val > 1.0f) //cos 0 to 90 does not have values beyond 0 and 1
          ASSERTLOGC(0, "Computed value more than 1!");
      }
      
    }
  }


}

void cNavMesh::SplitEdge(sNavEdge* edgetosplit, sNavVertex* inserted_vtx, sNavEdge* edge_newVtxToOldOrigin, sNavEdge* edge_newVtxToOldDest)
{
  //new edges assumed allocated
  //adjacencies are to be settled by the caller
  //sector grid checkins/checkouts is also done by caller

  edge_newVtxToOldOrigin->mOriginVertexIdx = inserted_vtx->mHandle;
  edge_newVtxToOldOrigin->mDestVertexIdx = edgetosplit->mOriginVertexIdx;
  edge_newVtxToOldOrigin->mConstraintIndices = edgetosplit->mConstraintIndices;

  edge_newVtxToOldDest->mOriginVertexIdx = inserted_vtx->mHandle;
  edge_newVtxToOldDest->mDestVertexIdx = edgetosplit->mDestVertexIdx;
  edge_newVtxToOldDest->mConstraintIndices = edgetosplit->mConstraintIndices;
}

void cNavMesh::SplitVertices(ObjectHandle baseedgeh, std::vector<ObjectHandle>& vertices_samesideof_edge, ObjectHandle& splitter_vtxh_out,
                             std::vector<ObjectHandle>& verticesOutL, std::vector<ObjectHandle>& verticesOutR)
{
  sNavEdge* baseedge = GetEdge(baseedgeh);
  Vector3 basept_a = GetVertex(baseedge->mOriginVertexIdx)->mPosition;
  Vector3 basept_b = GetVertex(baseedge->mDestVertexIdx)->mPosition;

  sNavVertex* first_vtx = GetVertex(vertices_samesideof_edge[0]);

  Vector3 circlecenter(Vector3::ZERO);
  float radiussq = 0.0f;
  bool circleformed = CircleFrom3Pts(basept_a, basept_b, first_vtx->mPosition, circlecenter, radiussq);
  if (circleformed == false)
    ASSERTLOGC(0, "Failed to form the circle!");

  ObjectHandle splitting_vtxh = NULL_OBJECTHANDLE;
  uint splitter_idx = 0;
  splitting_vtxh = first_vtx->mHandle;

  const float epsilon = 0.0001f * 0.00001f;
  for (uint i = 1; i < vertices_samesideof_edge.size(); ++i)
  {
    sNavVertex* vtx = GetVertex(vertices_samesideof_edge[i]);
    
    if (circlecenter.SquaredDistance(vtx->mPosition) < (radiussq - epsilon))
    {
      bool circleformed = CircleFrom3Pts(basept_a, basept_b, vtx->mPosition, circlecenter, radiussq);
      if (circleformed == false)
        ASSERTLOGC(0, "Failed to form the circle!");

      splitting_vtxh = vertices_samesideof_edge[i];
      splitter_idx = i;
    }
  }

  //split into before and after the vertices; order of projection values is already maintained by the initial sort
  for (uint i = 0; i < vertices_samesideof_edge.size(); ++i)
  {
    if (i < splitter_idx)
      verticesOutL.push_back(vertices_samesideof_edge[i]);
    else if (i > splitter_idx)
      verticesOutR.push_back(vertices_samesideof_edge[i]);
  }
  
  //assign base edges
  splitter_vtxh_out = splitting_vtxh;
}

sNavEdge* cNavMesh::GetEdgeFromFace(sNavFace* f, ObjectHandle edge_excluded, ObjectHandle vtx1h, ObjectHandle vtx2h)
{
  ObjectHandle otheredge1 = NULL_OBJECTHANDLE, otheredge2 = NULL_OBJECTHANDLE;
  if (f->mEdge1 == edge_excluded)
  {
    otheredge1 = f->mEdge2;
    otheredge2 = f->mEdge3;
  }
  else if (f->mEdge2 == edge_excluded)
  {
    otheredge1 = f->mEdge1;
    otheredge2 = f->mEdge3;
  }
  else if (f->mEdge3 == edge_excluded)
  {
    otheredge1 = f->mEdge1;
    otheredge2 = f->mEdge2;
  }

  sNavEdge* a = GetEdge(otheredge1);
  if ((a->mOriginVertexIdx == vtx1h && a->mDestVertexIdx == vtx2h) || (a->mOriginVertexIdx == vtx2h && a->mDestVertexIdx == vtx1h))
    return a;
  else
  {
    sNavEdge* b = GetEdge(otheredge2);
    if ((b->mOriginVertexIdx == vtx1h && b->mDestVertexIdx == vtx2h) || (b->mOriginVertexIdx == vtx2h && b->mDestVertexIdx == vtx1h))
      return b;
  }
  return nullptr;
}

sNavEdge* cNavMesh::GetEdgeFromFace(sNavFace* f, ObjectHandle vtx1, ObjectHandle vtx2)
{
  sNavEdge* e1 = GetEdge(f->mEdge1);
  sNavEdge* e2 = GetEdge(f->mEdge2);
  sNavEdge* e3 = GetEdge(f->mEdge3);

  if ((e1->mOriginVertexIdx == vtx1 && e1->mDestVertexIdx == vtx2) || (e1->mOriginVertexIdx == vtx2 && e1->mDestVertexIdx == vtx1))
    return e1;
  if ((e2->mOriginVertexIdx == vtx1 && e2->mDestVertexIdx == vtx2) || (e2->mOriginVertexIdx == vtx2 && e2->mDestVertexIdx == vtx1))
    return e2;
  if ((e3->mOriginVertexIdx == vtx1 && e3->mDestVertexIdx == vtx2) || (e3->mOriginVertexIdx == vtx2 && e3->mDestVertexIdx == vtx1))
    return e3;
  return nullptr;
}

void cNavMesh::RetriangulateHoleFromRemovedVertex(sNavVertex* vtx, std::vector<ObjectHandle>& connected_edges)
{
  std::vector<ObjectHandle> faces_created;

  if (connected_edges.empty())
    return;

  std::deque<ObjectHandle> shell_vertices_cw;
  std::vector<ObjectHandle> edges_tracked;
  edges_tracked.reserve(16);

  sNavEdge* startedge = GetEdge(connected_edges[0]);
  sNavEdge* edge = startedge;
  
  //traverse ccw and obtain all the shell vertices
  do
  {
    //add other vtx
    if (edge->mOriginVertexIdx == vtx->mHandle)
    {
      shell_vertices_cw.push_back(edge->mDestVertexIdx);
      edges_tracked.push_back(edge->mDestEdgeCW);
      edge = GetEdge(edge->mOriginEdgeCW);
    }
    else if (edge->mDestVertexIdx == vtx->mHandle)
    {
      shell_vertices_cw.push_back(edge->mOriginVertexIdx);
      edges_tracked.push_back(edge->mOriginEdgeCW);
      edge = GetEdge(edge->mDestEdgeCW);
    }

  } while (edge != startedge);


  //remove old edges
  for (uint j = 0; j < connected_edges.size(); ++j)
  {
    sNavEdge* e = GetEdge(connected_edges[j]);
    RemoveFace(e->mLeftFaceIdx);
    RemoveFace(e->mRightFaceIdx);
    RemoveEdge(e);
  }

  auto findEdge = [this](ObjectHandle vtx1, ObjectHandle vtx2, std::vector<ObjectHandle>& tracked) -> sNavEdge*
  {
    for (uint i = 0; i < tracked.size(); ++i)
    {
      sNavEdge* e = GetEdge(tracked[i]);
      if (e->mOriginVertexIdx == vtx1 && e->mDestVertexIdx == vtx2)
        return e;
      else if (e->mOriginVertexIdx == vtx2 && e->mDestVertexIdx == vtx1)
        return e;
    }
    return nullptr;
  };
  auto formNewEar = [this, findEdge]
    (sNavVertex* a, sNavVertex* b, sNavVertex* c, std::vector<ObjectHandle>& tracked)
  {
    sNavEdge* newedge = AddEdge();
    newedge->mOriginVertexIdx = a->mHandle;
    newedge->mDestVertexIdx = c->mHandle;
    sNavFace* newface = AddFace();

    sNavEdge* a_b_edge = findEdge(a->mHandle, b->mHandle, tracked);
    sNavEdge* b_c_edge = findEdge(b->mHandle, c->mHandle, tracked);
    if (a_b_edge->mOriginVertexIdx == newedge->mOriginVertexIdx)
    {
      a_b_edge->mOriginEdgeCW = newedge->mHandle;
      a_b_edge->mDestEdgeCCW = b_c_edge->mHandle;
      a_b_edge->mRightFaceIdx = newface->mHandle;
    }
    else if (a_b_edge->mDestVertexIdx == newedge->mOriginVertexIdx)
    {
      a_b_edge->mDestEdgeCW = newedge->mHandle;
      a_b_edge->mOriginEdgeCCW = b_c_edge->mHandle;
      a_b_edge->mLeftFaceIdx = newface->mHandle;
    }

    if (b_c_edge->mOriginVertexIdx == newedge->mDestVertexIdx)
    {
      b_c_edge->mOriginEdgeCCW = newedge->mHandle;
      b_c_edge->mDestEdgeCW = a_b_edge->mHandle;
      b_c_edge->mLeftFaceIdx = newface->mHandle;
    }
    else if (b_c_edge->mDestVertexIdx == newedge->mDestVertexIdx)
    {
      b_c_edge->mDestEdgeCCW = newedge->mHandle;
      b_c_edge->mOriginEdgeCW = a_b_edge->mHandle;
      b_c_edge->mRightFaceIdx = newface->mHandle;
    }

    newedge->mOriginEdgeCCW = a_b_edge->mHandle;
    newedge->mDestEdgeCW = b_c_edge->mHandle;
    newedge->mLeftFaceIdx = newface->mHandle;
    mSectorGrid.CheckIn(newedge);

    //new face
    newface->mVertex1Idx = a->mHandle;
    newface->mVertex2Idx = b->mHandle;
    newface->mVertex3Idx = c->mHandle;

    newface->mEdge1 = a_b_edge->mHandle;
    newface->mEdge2 = b_c_edge->mHandle;
    newface->mEdge3 = newedge->mHandle;
    mSectorGrid.CheckIn(newface);
    
    tracked.push_back(newedge->mHandle);
  };
  
  uint consecutive_ccw_collinear_sets = 0;
  while (shell_vertices_cw.size() > 3 && consecutive_ccw_collinear_sets < (shell_vertices_cw.size() - 2))
  {
    //for every 3 triplets of vertices, if they are cw, 
    //form a circle and test if any other point is within that circle
    //if not, it is an ear

    //look at first 3 vertices 
    sNavVertex* v1 = GetVertex(shell_vertices_cw.front());
    shell_vertices_cw.pop_front();
    sNavVertex* v2 = GetVertex(shell_vertices_cw.front());
    shell_vertices_cw.pop_front();
    sNavVertex* v3 = GetVertex(shell_vertices_cw.front());
    shell_vertices_cw.pop_front();

    Vector3 v1_to_v2 = v2->mPosition - v1->mPosition;
    Vector3 v2_to_v3 = v3->mPosition - v2->mPosition;

    float orient1 = v1_to_v2.CrossProductObtainSign(v2_to_v3);

    if (orient1 >= 0.0f)
    {
      //ccw/collinear
      shell_vertices_cw.push_front(v3->mHandle);
      shell_vertices_cw.push_front(v2->mHandle);
      shell_vertices_cw.push_back(v1->mHandle); //push to the back
      ++consecutive_ccw_collinear_sets;
    }
    else if (orient1 < 0.0f)
    {
      consecutive_ccw_collinear_sets = 0;
      //cw. form a circle, test against remaining vertices of shell_vertices_cw

      Vector3 circlecenter(Vector3::ZERO);
      float circleradiussq = 0.0f;
      CircleFrom3Pts(v1->mPosition, v2->mPosition, v3->mPosition, circlecenter, circleradiussq);

      bool haspointwithincircle = false;
      for (uint i = 0; i < shell_vertices_cw.size(); ++i)
      {
        sNavVertex* testvtx = GetVertex(shell_vertices_cw[i]);
        float diff_sqdist = circlecenter.SquaredDistance(testvtx->mPosition);
        if (diff_sqdist < (circleradiussq - NAVMESH_EPSILONSQ))
        { 
          //within circle
          haspointwithincircle = true;
          break;
        }
      }

      if (haspointwithincircle)
      {
        shell_vertices_cw.push_front(v3->mHandle);
        shell_vertices_cw.push_front(v2->mHandle);
        shell_vertices_cw.push_back(v1->mHandle); //push to the back
      }
      else
      {
        //form ear
        formNewEar(v1, v2, v3, edges_tracked);
        shell_vertices_cw.push_front(v3->mHandle);
        shell_vertices_cw.push_front(v1->mHandle);
      }
    }
  } 

  //create the last face
  sNavVertex* v1 = GetVertex(shell_vertices_cw.front());
  shell_vertices_cw.pop_front();
  sNavVertex* v2 = GetVertex(shell_vertices_cw.front());
  shell_vertices_cw.pop_front();
  sNavVertex* v3 = GetVertex(shell_vertices_cw.front());
  shell_vertices_cw.pop_front();
  
  Vector3 v1_to_v2 = v2->mPosition - v1->mPosition;
  Vector3 v2_to_v3 = v3->mPosition - v2->mPosition;

  float orient1 = v1_to_v2.CrossProductObtainSign(v2_to_v3);

  if (orient1 < 0.0f) //only cw vertices for last face
  {
    sNavFace* lastface = AddFace();
    lastface->mVertex1Idx = v1->mHandle;
    lastface->mVertex2Idx = v2->mHandle;
    lastface->mVertex3Idx = v3->mHandle;

    sNavEdge* e1 = findEdge(v1->mHandle, v2->mHandle, edges_tracked);
    sNavEdge* e2 = findEdge(v2->mHandle, v3->mHandle, edges_tracked);
    sNavEdge* e3 = findEdge(v3->mHandle, v1->mHandle, edges_tracked);

    lastface->mEdge1 = e1->mHandle;
    lastface->mEdge2 = e2->mHandle;
    lastface->mEdge3 = e3->mHandle;

    {
      //fix e1 adjacencies
      if (e1->mOriginVertexIdx == v1->mHandle)
      {
        e1->mOriginEdgeCW = e3->mHandle;
        e1->mDestEdgeCCW = e2->mHandle;
        e1->mRightFaceIdx = lastface->mHandle;
      }
      else if (e1->mDestVertexIdx == v1->mHandle)
      {
        e1->mDestEdgeCW = e3->mHandle;
        e1->mOriginEdgeCCW = e2->mHandle;
        e1->mLeftFaceIdx = lastface->mHandle;
      }

      //fix e2 adjacencies
      if (e2->mOriginVertexIdx == v2->mHandle)
      {
        e2->mOriginEdgeCW = e1->mHandle;
        e2->mDestEdgeCCW = e3->mHandle;
        e2->mRightFaceIdx = lastface->mHandle;
      }
      else if (e2->mDestVertexIdx == v2->mHandle)
      {
        e2->mDestEdgeCW = e1->mHandle;
        e2->mOriginEdgeCCW = e3->mHandle;
        e2->mLeftFaceIdx = lastface->mHandle;
      }

      //fix e3 adjacencies
      if (e3->mOriginVertexIdx == v3->mHandle)
      {
        e3->mOriginEdgeCW = e2->mHandle;
        e3->mDestEdgeCCW = e1->mHandle;
        e3->mRightFaceIdx = lastface->mHandle;
      }
      else if (e3->mDestVertexIdx == v3->mHandle)
      {
        e3->mDestEdgeCW = e2->mHandle;
        e3->mOriginEdgeCCW = e1->mHandle;
        e3->mLeftFaceIdx = lastface->mHandle;
      }
    }

    mSectorGrid.CheckIn(lastface); 
  }
}

bool cNavMesh::IsCollinear(sNavEdge* e1, sNavEdge* e2)
{
  ObjectHandle sharedh = GetSharedVertex(e1, e2);
  ObjectHandle v1h = GetOtherNonSharedVertex(e1, e2, e2);
  ObjectHandle v2h = GetOtherNonSharedVertex(e1, e2, e1);

  sNavVertex* shared = GetVertex(sharedh);
  sNavVertex* v1 = GetVertex(v1h);
  sNavVertex* v2 = GetVertex(v2h);

  //http://mathworld.wolfram.com/Collinear.html
  float area = shared->mPosition.x * (v1->mPosition.z - v2->mPosition.z) +
    v1->mPosition.x * (v2->mPosition.z - shared->mPosition.z) +
    v2->mPosition.x * (shared->mPosition.z - v1->mPosition.z);

  if (area == 0.0f)
    return true;
  return false;
}

void cNavMesh::RetriangulateWithBaseEdge(const Shell& shell)
{
  //add final edge. note that base edges are oriented left to right
  sNavEdge* baseedge = GetEdge(shell.mBaseEdgeHandle);

  typedef std::vector<ObjectHandle> Vertices;

  std::deque<Vertices> cache;
  cache.emplace_front(shell.mShellVerticesBelow);
  cache.emplace_front(shell.mShellVerticesAbove);

  std::deque<VtxInfo> baseedges;
  baseedges.emplace_front(baseedge->mHandle, false);
  baseedges.emplace_front(baseedge->mHandle, true);

  //retriangulate via divide and conquer
  do
  {
    //every iteration is gauranteed to make 1 face as long as there are vertices
    Vertices vertices = cache.front();
    ObjectHandle baseedgeh = baseedges.front().mBaseEdgeH;
    bool verticesOnPositiveSide = baseedges.front().mOnPositiveSide;

    cache.pop_front();
    baseedges.pop_front();

    Vertices verticesL, verticesR;
    ObjectHandle splitter_vtx = NULL_OBJECTHANDLE, newedgeLh = NULL_OBJECTHANDLE, newedgeRh = NULL_OBJECTHANDLE;
    if (vertices.size() > 1)
    {
      SplitVertices(baseedgeh, vertices, splitter_vtx, verticesL, verticesR);

      //interpretations of results: 
      //1) no edges on one of the sides: form only one edge of the other side, and update the edge info
      //always form 1 triangle as long as we find the splitter_vtx

      //form/find the right edge
      bool newedgeRAlreadyExists = false;
      sNavEdge* base = GetEdge(baseedgeh);
      newedgeRh = mSectorGrid.FindEdgeInSector(splitter_vtx, base->mDestVertexIdx);
      if (newedgeRh == NULL_OBJECTHANDLE)
      {
        sNavEdge* newrightedge = AddEdge();
        newrightedge->mOriginVertexIdx = splitter_vtx;
        newrightedge->mDestVertexIdx = base->mDestVertexIdx;

        newedgeRh = newrightedge->mHandle;
        mSectorGrid.CheckIn(newrightedge);
      }
      else
        newedgeRAlreadyExists = true;

      //form/find the left edge
      bool newedgeLAlreadyExists = false;
      newedgeLh = mSectorGrid.FindEdgeInSector(splitter_vtx, base->mOriginVertexIdx);
      if (newedgeLh == NULL_OBJECTHANDLE)
      {
        sNavEdge* newleftedge = AddEdge();
        newleftedge->mOriginVertexIdx = base->mOriginVertexIdx;
        newleftedge->mDestVertexIdx = splitter_vtx;

        newedgeLh = newleftedge->mHandle;
        mSectorGrid.CheckIn(newleftedge);
      }
      else
        newedgeLAlreadyExists = true;

      if (!verticesR.empty())
      {
        cache.emplace_front(verticesR);
        baseedges.emplace_front(newedgeRh, verticesOnPositiveSide);
      }
      if (!verticesL.empty())
      {
        cache.emplace_front(verticesL);
        baseedges.emplace_front(newedgeLh, verticesOnPositiveSide);
      }
    }
    else
    {
      if (vertices.size() == 1)
      {
        splitter_vtx = vertices[0];
        sNavEdge* base = GetEdge(baseedgeh);
        newedgeLh = mSectorGrid.FindEdgeInSector(base->mOriginVertexIdx, splitter_vtx);
        newedgeRh = mSectorGrid.FindEdgeInSector(splitter_vtx, base->mDestVertexIdx);
      }
      else if (vertices.size() == 0)
      {
        //ASSERTLOGC(0, "Need 1 external vtx to form a triangle!");
      }
    }

    ObjectHandle newfaceh = NULL_OBJECTHANDLE;
    sNavFace* newface = nullptr;
    sNavEdge *e1 = nullptr, *e2 = nullptr, *e3 = nullptr;

    if (splitter_vtx != NULL_OBJECTHANDLE)
    {
      newface = AddFace();
      newface->mEdge1 = baseedgeh;
      if (verticesOnPositiveSide)
      {
        newface->mEdge2 = newedgeRh;
        newface->mEdge3 = newedgeLh;
      }
      else
      {
        newface->mEdge2 = newedgeLh;
        newface->mEdge3 = newedgeRh;
      }

      e1 = GetEdge(newface->mEdge1);
      e2 = GetEdge(newface->mEdge2);
      e3 = GetEdge(newface->mEdge3);

      newface->mVertex1Idx = GetSharedVertex(e1, e2);
      newface->mVertex2Idx = GetSharedVertex(e2, e3);
      newface->mVertex3Idx = GetSharedVertex(e3, e1);
      if (shell.mGameplayFlags & FACE_REPRESENTS_HOLE)
        newface->mFlags |= FACE_REPRESENTS_HOLE;

      newfaceh = newface->mHandle;
      mSectorGrid.CheckIn(newface);
    }

    //set adjacency information: Refer to diagrams
    {
      sNavEdge* base = GetEdge(baseedgeh);
      sNavEdge* newedgeL = GetEdge(newedgeLh, false);
      sNavEdge* newedgeR = GetEdge(newedgeRh, false);

      //set base edge, e2, e3 adjacencies
      if (verticesOnPositiveSide)
      {
        base->mOriginEdgeCW = newedgeLh;
        base->mDestEdgeCCW = newedgeRh;
        base->mRightFaceIdx = newfaceh;

        if (newedgeL)
        {
          ObjectHandle lsharedvtx = GetSharedVertex(newedgeL, base);
          if (lsharedvtx == newedgeL->mOriginVertexIdx)
          {
            newedgeL->mOriginEdgeCCW = baseedgeh;
            newedgeL->mDestEdgeCW = newedgeRh;
            newedgeL->mLeftFaceIdx = newfaceh;
          }
          else if (lsharedvtx == newedgeL->mDestVertexIdx)
          {
            newedgeL->mDestEdgeCCW = baseedgeh;
            newedgeL->mOriginEdgeCW = newedgeRh;
            newedgeL->mRightFaceIdx = newfaceh;
          }
        }

        if (newedgeR)
        {
          ObjectHandle rsharedvtx = GetSharedVertex(newedgeR, base);
          if (rsharedvtx == newedgeR->mOriginVertexIdx)
          {
            newedgeR->mOriginEdgeCW = baseedgeh;
            newedgeR->mDestEdgeCCW = newedgeLh;
            newedgeR->mRightFaceIdx = newfaceh;
          }
          else if (rsharedvtx == newedgeR->mDestVertexIdx)
          {
            newedgeR->mDestEdgeCW = baseedgeh;
            newedgeR->mOriginEdgeCCW = newedgeLh;
            newedgeR->mLeftFaceIdx = newfaceh;
          }
        }
      }
      else
      {
        base->mOriginEdgeCCW = newedgeLh;
        base->mDestEdgeCW = newedgeRh;
        base->mLeftFaceIdx = newfaceh;

        if (newedgeL)
        {
          ObjectHandle lsharedvtx = GetSharedVertex(newedgeL, base);
          if (lsharedvtx == newedgeL->mOriginVertexIdx)
          {
            newedgeL->mOriginEdgeCW = baseedgeh;
            newedgeL->mDestEdgeCCW = newedgeRh;
            newedgeL->mRightFaceIdx = newfaceh;
          }
          else if (lsharedvtx == newedgeL->mDestVertexIdx)
          {
            newedgeL->mDestEdgeCW = baseedgeh;
            newedgeL->mOriginEdgeCCW = newedgeRh;
            newedgeL->mLeftFaceIdx = newfaceh;
          }
        }

        if (newedgeR)
        {
          ObjectHandle rsharedvtx = GetSharedVertex(newedgeR, base);
          if (rsharedvtx == newedgeR->mOriginVertexIdx)
          {
            newedgeR->mOriginEdgeCCW = baseedgeh;
            newedgeR->mDestEdgeCW = newedgeLh;
            newedgeR->mLeftFaceIdx = newfaceh;
          }
          else if (rsharedvtx == newedgeR->mDestVertexIdx)
          {
            newedgeR->mDestEdgeCCW = baseedgeh;
            newedgeR->mOriginEdgeCW = newedgeLh;
            newedgeR->mRightFaceIdx = newfaceh;
          }
        }
      }
    }
  } while (!cache.empty());
}

sNavVertex* cNavMesh::AddVertex()
{
  sNavVertex* ret = nullptr;
  ObjectHandle handle = mVertexSlots.GrantHandle();
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mVertices.size())
    ret = &mVertices[arrayidx];
  else
  {
    mVertices.push_back(sNavVertex());
    ret = &mVertices[mVertices.size() - 1];
  }

  ret->mHandle = handle;
  ret->mFlags = 0; // reset flags
  return ret;
}

sNavEdge* cNavMesh::AddEdge()
{
  sNavEdge* ret = nullptr;
  ObjectHandle handle = mEdgeSlots.GrantHandle();
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mEdges.size())
    ret = &mEdges[arrayidx];
  else
  {
    mEdges.push_back(sNavEdge());
    ret = &mEdges[mEdges.size() - 1];
  }

  ret->mHandle = handle;
  ret->mFlags = 0; // reset flags
  return ret;
}

sNavFace* cNavMesh::AddFace()
{
  sNavFace* ret = nullptr;
  ObjectHandle handle = mFaceSlots.GrantHandle();
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mFaces.size())
    ret = &mFaces[arrayidx];
  else
  {
    mFaces.push_back(sNavFace());
    ret = &mFaces[mFaces.size() - 1];
  }

  ret->mHandle = handle;
  ret->mFlags = 0; // reset flags
  return ret;
}

sConstraint* cNavMesh::AddConstraint()
{
  sConstraint* ret = nullptr;
  ObjectHandle handle = mConstraintSlots.GrantHandle();
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mConstraints.size())
    ret = &mConstraints[arrayidx];
  else
  {
    mConstraints.emplace_back();
    ret = &mConstraints[mConstraints.size() - 1];
  }

  ret->mHandle = handle;
  ret->mPointCount = 0;
  ret->mFlags = 0;
  ret->mIdentifier = NULL_OBJECTHANDLE;
  return ret;
}

sNavVertex* cNavMesh::GetVertex(ObjectHandle handle)
{
  sNavVertex* ret = nullptr;
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mVertices.size())
  {
    if (mVertices[arrayidx].mHandle == handle)
      ret = &mVertices[arrayidx];
  }

  //Warn
  if (!ret)
    LOG("Vertex %d not found", handle);
  return ret;
}

sNavEdge* cNavMesh::GetEdge(ObjectHandle handle, bool warn)
{
  sNavEdge* ret = nullptr;
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mEdges.size())
  {
    if (mEdges[arrayidx].mHandle == handle)
      ret = &mEdges[arrayidx];
  }

  //Warn
  if (warn && !ret)
    LOG("Edge %d not found", handle);
  return ret;
}

sNavFace* cNavMesh::GetFace(ObjectHandle handle, bool warn)
{
  sNavFace* ret = nullptr;
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mFaces.size())
  {
    if (mFaces[arrayidx].mHandle == handle)
      ret = &mFaces[arrayidx];
  }

  //Warn
  if (warn && !ret)
    LOG("Face %d not found", handle);
  return ret;
}

sConstraint* cNavMesh::GetConstraint(ObjectHandle handle)
{
  sConstraint* ret = nullptr;
  ushort arrayidx = GetHandleID(handle);
  if (arrayidx < (ushort)mConstraints.size())
  {
    if (mConstraints[arrayidx].mHandle == handle)
      ret = &mConstraints[arrayidx];
  }

  //Warn
  if (!ret)
    LOG("Constraint %d not found", handle);
  return ret;
}

void cNavMesh::RemoveVertex(ObjectHandle handle)
{
  if (handle == NULL_OBJECTHANDLE)
    return;
  ushort arrayidx = GetHandleID(handle);
  if (!mVertexSlots.HandleExists(handle) && arrayidx < (ushort)mVertices.size())
  {
    if (mVertices[arrayidx].mHandle == handle)
    {
      mSectorGrid.CheckOut(&mVertices[arrayidx]);
      mVertices[arrayidx].mFlags |= IN_FREELIST;
      mVertexSlots.RecycleHandle(handle);
    }
  }
}

void cNavMesh::RemoveEdge(ObjectHandle handle)
{
  if (handle == NULL_OBJECTHANDLE)
    return;
  ushort arrayidx = GetHandleID(handle);
  if (!mEdgeSlots.HandleExists(handle) && arrayidx < (ushort)mEdges.size())
  {
    if (mEdges[arrayidx].mHandle == handle)
    {
      mSectorGrid.CheckOut(&mEdges[arrayidx]);
      mEdges[arrayidx].mFlags |= IN_FREELIST;
      mEdges[arrayidx].mConstraintIndices.clear();
      mEdgeSlots.RecycleHandle(handle);
    }
  }
}

void cNavMesh::RemoveFace(ObjectHandle handle)
{
  if (handle == NULL_OBJECTHANDLE)
    return;
  ushort arrayidx = GetHandleID(handle);
  if (!mFaceSlots.HandleExists(handle) && arrayidx < (ushort)mFaces.size())
  {
    if (mFaces[arrayidx].mHandle == handle)
    {
      mSectorGrid.CheckOut(&mFaces[arrayidx]);
      mFaces[arrayidx].mFlags |= IN_FREELIST;
      mFaceSlots.RecycleHandle(handle);
    }
  }
}

void cNavMesh::RemoveConstraint(ObjectHandle handle)
{
  if (handle == NULL_OBJECTHANDLE)
    return;
  ushort arrayidx = GetHandleID(handle);
  if (!mConstraintSlots.HandleExists(handle) && arrayidx < (ushort)mConstraints.size())
  {
    if (mConstraints[arrayidx].mHandle == handle)
    {
      mConstraints[arrayidx].mFlags |= IN_FREELIST;
      mConstraintSlots.RecycleHandle(handle);
    } 
  }
}

float cNavMesh::ComputeMaximumWidthSquared(ObjectHandle triangleh, ObjectHandle edge1oftriangleh, ObjectHandle edge2oftriangleh)
{
  sNavFace* face = GetFace(triangleh);
  sNavEdge* edge1 = GetEdge(edge1oftriangleh);
  sNavEdge* edge2 = GetEdge(edge2oftriangleh);
  if (!face || !edge1 || !edge2)
    return 0.0f;

  ObjectHandle sharedvtxh = GetSharedVertex(edge1, edge2);
  ObjectHandle vtxedge1h = GetOtherNonSharedVertex(edge1, edge2, edge2);
  ObjectHandle vtxedge2h = GetOtherNonSharedVertex(edge1, edge2, edge1);

  sNavVertex* vtxshared = GetVertex(sharedvtxh);
  sNavVertex* vtxedge1 = GetVertex(vtxedge1h);
  sNavVertex* vtxedge2 = GetVertex(vtxedge2h);

  //case 1: triangles with obtuse angles
  Vector3 v1_to_shared = vtxshared->mPosition - vtxedge1->mPosition;
  Vector3 v1_to_v2 = vtxedge2->mPosition - vtxedge1->mPosition;
  float dotval = v1_to_shared.DotProduct(v1_to_v2);
  if (dotval < 0.0f) //angle formed by shared,v1,v2 is obtuse
    return v1_to_shared.SquaredLength(); //return v1 to shared length

  Vector3 v2_to_shared = vtxshared->mPosition - vtxedge2->mPosition;
  Vector3 v2_to_v1 = vtxedge1->mPosition - vtxedge2->mPosition;
  dotval = v2_to_shared.DotProduct(v2_to_v1);
  if (dotval < 0.0f) //angle formed by shared,v2,v1 is obtuse
    return v2_to_shared.SquaredLength(); //return v1 to shared length

  ObjectHandle e3h = GetOtherEdgeFromFace(face, edge1oftriangleh, edge2oftriangleh);
  sNavEdge* e3 = GetEdge(e3h);
  //case 2: constrainted edge c
  if (e3->IsConstrainted())
  {
    LineSegment2D e(vtxedge2->mPosition.x, vtxedge2->mPosition.z, vtxedge1->mPosition.x, vtxedge1->mPosition.z);
    float distsq = e.GetClosestDistanceToPointSquared(Vector2(vtxshared->mPosition.x, vtxshared->mPosition.z));
    return distsq;
  }

  //case 3: unconstrainted edge c
  float v2_toshared_sqlength = v2_to_shared.SquaredLength();
  float v1_toshared_sqlength = v1_to_shared.SquaredLength();
  float minsqdist = v2_toshared_sqlength > v1_toshared_sqlength ? v1_toshared_sqlength : v2_toshared_sqlength;
  
  struct EvaluateInfo
  {
    EvaluateInfo()
      :mFaceFrom(nullptr), mCrossingEdge(nullptr)
    {}
    sNavFace* mFaceFrom;
    sNavEdge* mCrossingEdge;
  };
  std::deque<EvaluateInfo> to_evaluate;
  EvaluateInfo i;
  i.mFaceFrom = face;
  i.mCrossingEdge = e3;
  to_evaluate.push_front(i);
  
  do
  {
    EvaluateInfo eval = to_evaluate.front();
    to_evaluate.pop_front();

    sNavEdge* currentedge = eval.mCrossingEdge;
    sNavFace* currentface = nullptr;

    //move to next triangle
    if (currentedge->mLeftFaceIdx == eval.mFaceFrom->mHandle)
      currentface = GetFace(currentedge->mRightFaceIdx);
    else if (currentedge->mRightFaceIdx == eval.mFaceFrom->mHandle)
      currentface = GetFace(currentedge->mLeftFaceIdx);

    ObjectHandle oppositevtxh = GetNonSharedVertexFromFace(currentface, currentedge);
    sNavVertex* oppositevtx = GetVertex(oppositevtxh);

    //find edges to check against
    ObjectHandle new_v1h = NULL_OBJECTHANDLE, new_v2h = NULL_OBJECTHANDLE;
    if (currentface->mVertex1Idx == oppositevtxh)
    {
      new_v1h = currentface->mVertex2Idx;
      new_v2h = currentface->mVertex3Idx;
    }
    else if (currentface->mVertex2Idx == oppositevtxh)
    {
      new_v1h = currentface->mVertex3Idx;
      new_v2h = currentface->mVertex1Idx;
    }
    else if (currentface->mVertex3Idx == oppositevtxh)
    {
      new_v1h = currentface->mVertex1Idx;
      new_v2h = currentface->mVertex2Idx;
    }

    sNavVertex* new_v1 = GetVertex(new_v1h);
    sNavVertex* new_v2 = GetVertex(new_v2h);

    static const int NO_CONTINUE = 0;
    static const int CONTINUE = 1;

    auto evaluateNewEdge = [this, &v1_to_shared, &vtxshared](sNavFace* face, sNavVertex* mid, sNavVertex* newend, float& mindistsq) -> int
    { 
      //check both angles must be acute 
      //we already computed v1_to_shared
      Vector3 mid_to_end = newend->mPosition - mid->mPosition;
      float dotval = v1_to_shared.DotProduct(mid_to_end);
      if (dotval < 0.0f) //angle is obtuse
        return NO_CONTINUE; //stop, do not continue

      Vector3 end_to_mid = -mid_to_end;
      Vector3 end_to_start = vtxshared->mPosition - newend->mPosition;
      dotval = end_to_mid.DotProduct(end_to_start);
      if (dotval < 0.0f) 
        return NO_CONTINUE;

      LineSegment2D seg(mid->mPosition.x, mid->mPosition.z, newend->mPosition.x, newend->mPosition.z);
      float distsq = seg.GetClosestDistanceToPointSquared(Vector2(vtxshared->mPosition.x, vtxshared->mPosition.z));
      if (mindistsq <= distsq) //edge doesnt intersect the curvature
        return NO_CONTINUE;

      //beyond this, the edge intersects the curvature

      sNavEdge* considered_edge = GetEdgeFromFace(face, mid->mHandle, newend->mHandle);
      if (considered_edge->IsConstrainted())
      {
        mindistsq = distsq; //take the distance to the edge since it's intersected and we cannot proceed
        return NO_CONTINUE;
      }
      else
        //edge intersects the curvature, have to search beyond
        return CONTINUE;
    };

    int result = evaluateNewEdge(currentface, new_v2, oppositevtx, minsqdist);
    if (result == CONTINUE)
    {
      EvaluateInfo new_e;
      new_e.mCrossingEdge = GetEdgeFromFace(currentface, new_v2->mHandle, oppositevtx->mHandle);
      new_e.mFaceFrom = currentface;
      to_evaluate.push_front(new_e);
    }

    result = evaluateNewEdge(currentface, new_v1, oppositevtx, minsqdist);
    if (result == CONTINUE)
    {
      EvaluateInfo new_e;
      new_e.mCrossingEdge = GetEdgeFromFace(currentface, new_v2->mHandle, oppositevtx->mHandle);
      new_e.mFaceFrom = currentface;
      to_evaluate.push_front(new_e);
    }
  } while (!to_evaluate.empty());
  return minsqdist;
}

float cNavMesh::GetArcLengthDist(sNavEdge* e1, sNavEdge* e2, float radius)
{
  ObjectHandle shared = GetSharedVertex(e1, e2);

  sNavVertex* e1_v1 = GetVertex(e1->mOriginVertexIdx);
  sNavVertex* e1_v2 = GetVertex(e1->mDestVertexIdx);

  Vector3 e1_vec;
  if (e1_v1->mHandle == shared)
    e1_vec = e1_v2->mPosition - e1_v1->mPosition;
  else if (e1_v2->mHandle == shared)
    e1_vec = e1_v1->mPosition - e1_v2->mPosition;

  sNavVertex* e2_v1 = GetVertex(e2->mOriginVertexIdx);
  sNavVertex* e2_v2 = GetVertex(e2->mDestVertexIdx);

  Vector3 e2_vec;
  if (e2_v1->mHandle == shared)
    e2_vec = e2_v2->mPosition - e2_v1->mPosition;
  else if (e2_v2->mHandle == shared)
    e2_vec = e2_v1->mPosition - e2_v2->mPosition;

  float dotval = e1_vec.DotProduct(e2_vec);
  float denom = e1_vec.Length() * e2_vec.Length();
  
  float angle_rad = acos(dotval / denom);
  return angle_rad * radius;
}

ObjectHandle GetNonSharedVertexFromFace(sNavFace* face, sNavEdge* edge)
{
  if ((face->mVertex1Idx == edge->mOriginVertexIdx && face->mVertex2Idx == edge->mDestVertexIdx) ||
    (face->mVertex2Idx == edge->mOriginVertexIdx && face->mVertex1Idx == edge->mDestVertexIdx))
    return face->mVertex3Idx;
  if ((face->mVertex1Idx == edge->mOriginVertexIdx && face->mVertex3Idx == edge->mDestVertexIdx) ||
    (face->mVertex3Idx == edge->mOriginVertexIdx && face->mVertex1Idx == edge->mDestVertexIdx))
    return face->mVertex2Idx;
  if ((face->mVertex2Idx == edge->mOriginVertexIdx && face->mVertex3Idx == edge->mDestVertexIdx) ||
    (face->mVertex3Idx == edge->mOriginVertexIdx && face->mVertex2Idx == edge->mDestVertexIdx))
    return face->mVertex1Idx;

  return NULL_OBJECTHANDLE;
}

ObjectHandle GetSharedVertex(sNavEdge* edge1, sNavEdge* edge2)
{
  if (edge1->mOriginVertexIdx == edge2->mOriginVertexIdx)
    return edge2->mOriginVertexIdx;
  else if (edge1->mDestVertexIdx == edge2->mOriginVertexIdx)
    return edge2->mOriginVertexIdx;
  else if (edge1->mDestVertexIdx == edge2->mDestVertexIdx)
    return edge2->mDestVertexIdx;
  else if (edge1->mOriginVertexIdx == edge2->mDestVertexIdx)
    return edge2->mDestVertexIdx;
  else
    ASSERTLOGC(0, "edges do not share vertices!");
  return NULL_OBJECTHANDLE;
}

ObjectHandle GetOtherNonSharedVertex(sNavEdge* edge1, sNavEdge* edge2, sNavEdge* notofedge)
{
  ObjectHandle shared = GetSharedVertex(edge1, edge2);

  if (edge1 == notofedge)
  {
    return ((edge2->mOriginVertexIdx == shared) ? edge2->mDestVertexIdx : edge2->mOriginVertexIdx);
  }
  else if (edge2 == notofedge)
  {
    return ((edge1->mOriginVertexIdx == shared) ? edge1->mDestVertexIdx : edge1->mOriginVertexIdx);
  }
  else
    ASSERTLOGC(0, "non-shared vtx not found!");
  return NULL_OBJECTHANDLE;
}

ObjectHandle GetOtherEdgeFromFace(sNavFace* face, ObjectHandle e1, ObjectHandle e2)
{
  if ((face->mEdge1 == e1 && face->mEdge2 == e2) || (face->mEdge2 == e1 && face->mEdge1 == e2))
    return face->mEdge3;
  else if ((face->mEdge2 == e1 && face->mEdge3 == e2) || (face->mEdge3 == e1 && face->mEdge2 == e2))
    return face->mEdge1;
  else if ((face->mEdge3 == e1 && face->mEdge1 == e2) || (face->mEdge1 == e1 && face->mEdge3 == e2))
    return face->mEdge2;
  else
    ASSERTLOGC(0, "cannot find 3rd edge due to non-shared edges!");
  return NULL_OBJECTHANDLE;
}

void CopyGameplayFlags(sNavFace* src, sNavFace* dest)
{
  if (src->mFlags & FACE_REPRESENTS_HOLE)
    dest->mFlags |= FACE_REPRESENTS_HOLE;
  if (src->mFlags & FACE_REPRESENTS_BLOCKER)
    dest->mFlags |= FACE_REPRESENTS_BLOCKER;
}

bool CircleFrom3Pts(const Vector3& a, const Vector3& b, const Vector3& c, Vector3& circlecenter, float& radiussq)
{
  //based on http://en.wikipedia.org/wiki/Circumscribed_circle ; we're interpreting this as a projected 2d surface, y = 0
  float denom = 2.0f * (a.x * (b.z - c.z) + b.x * (c.z - a.z) + c.x * (a.z - b.z));
  if (denom == 0.0f)
    return false; //3 points collinear?

  circlecenter.x = (a.SquaredLength() * (b.z - c.z) + b.SquaredLength() * (c.z - a.z) + c.SquaredLength() * (a.z - b.z)) / denom;
  circlecenter.y = 0.0f;
  circlecenter.z = (a.SquaredLength() * (c.x - b.x) + b.SquaredLength() * (a.x - c.x) + c.SquaredLength() * (b.x - a.x)) / denom;
  radiussq = circlecenter.SquaredDistance(a);
  return true;
}

Vector3 sNavFace::GetFaceMidPoint(cNavMesh* m)
{
  sNavVertex* v1 = m->GetVertex(mVertex1Idx);
  sNavVertex* v2 = m->GetVertex(mVertex2Idx);
  sNavVertex* v3 = m->GetVertex(mVertex3Idx);

  return (v1->mPosition + v2->mPosition + v3->mPosition) / 3.0f;
}

END_NSP

