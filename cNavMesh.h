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
#ifndef _CNAVMESH_H
#define _CNAVMESH_H

#include "EnginePrereqs.h"
#include "ResourceMgt/SlotMap.h"
#include "cSectorGrid.h"
#include "cPathfinder.h"

typedef struct CTwBar TwBar;
ENGINE_NSP

#define NAVMESH_DBG_YOFFSET 0.05f
#define NAVMESH_EPSILON 0.05f

class cModelGfxCmp;
struct sNavEdge;
class cNavMesh;

enum PointLocateResultId
{
  NO_LOC_RESULTS,
  IS_EXISTING_VTX,
  ON_EXISTING_EDGE,
  ON_EXISTING_FACE
};

struct PointLocateResult
{
  PointLocateResultId mResultId = NO_LOC_RESULTS;
  ObjectHandle mVertexId = NULL_OBJECTHANDLE;
  ObjectHandle mEdgeId = NULL_OBJECTHANDLE;
  ObjectHandle mFaceId = NULL_OBJECTHANDLE;
  Vector3 mProjectedPoint;
};

enum PRIM_FLAGS
{
  IN_FREELIST = 1,
  IS_CONSTRAINTED = 1 << 1,
  IS_BOUNDARY = 1 << 2,
  IS_INSERTED_BY_UNIT = 1 << 3,
  //face specific flags.
  //having neither of these flags turned on means the face represents solid ground
  FACE_REPRESENTS_HOLE = 1 << 4,
  FACE_REPRESENTS_BLOCKER = 1 << 5
};

#define MAX_CONSTRAINT_POINTS 24
struct sConstraint
{
  Vector3 mPositions[MAX_CONSTRAINT_POINTS]; //all points in the constraint are linked
  uint mPointCount = 0;
  ObjectHandle mHandle = NULL_OBJECTHANDLE;
  ObjectHandle mIdentifier = NULL_OBJECTHANDLE;
  int mFlags = 0;

  bool InFreeList() { return mFlags & IN_FREELIST; }
  bool IsInsertedByUnit() { return (mFlags & IS_INSERTED_BY_UNIT) ? true : false; }
};

//mesh sub-primitives. All handles should be assigned via their respective slotmaps
struct sNavVertex
{
  ObjectHandle mHandle;
  Vector3 mPosition;
  int mFlags = 0;

  bool InFreeList() { return mFlags & IN_FREELIST; }
};

struct sNavFace //triangular face
{
  ObjectHandle mHandle = NULL_OBJECTHANDLE;
  //vertices in (prefarably) clockwise order
  ObjectHandle mVertex1Idx = NULL_OBJECTHANDLE;
  ObjectHandle mVertex2Idx = NULL_OBJECTHANDLE;
  ObjectHandle mVertex3Idx = NULL_OBJECTHANDLE;
  //edges in clockwise order
  ObjectHandle mEdge1 = NULL_OBJECTHANDLE;
  ObjectHandle mEdge2 = NULL_OBJECTHANDLE;
  ObjectHandle mEdge3 = NULL_OBJECTHANDLE;
  
  int mFlags = 0;

  //ExtractVertices();
  bool InFreeList() { return mFlags & IN_FREELIST; }
  Vector3 GetFaceMidPoint(cNavMesh* m);
};

struct sNavEdge
{
  ObjectHandle mHandle = NULL_OBJECTHANDLE;
  //int mAlternateEdgeIdx = NULL_OBJECTHANDLE; //the other side of the same edge
  ObjectHandle mOriginVertexIdx = NULL_OBJECTHANDLE;  //source of the edge 'dart'
  ObjectHandle mDestVertexIdx = NULL_OBJECTHANDLE;  //dest of the edge 'dart'
  
  //left and right WRT source vertex (origin)
  ObjectHandle mLeftFaceIdx = NULL_OBJECTHANDLE;
  ObjectHandle mRightFaceIdx = NULL_OBJECTHANDLE;

  ObjectHandle mOriginEdgeCCW = NULL_OBJECTHANDLE;
  ObjectHandle mOriginEdgeCW = NULL_OBJECTHANDLE;

  //from perspective of Dest to origin
  ObjectHandle mDestEdgeCCW = NULL_OBJECTHANDLE;
  ObjectHandle mDestEdgeCW = NULL_OBJECTHANDLE;

  //left and right faces WRT source vertex (origin to dest or dest to origin); warning: doesnt check for a left/right face. 
  ObjectHandle LeftFaceOriginEdge()   const { return mOriginEdgeCCW; }
  ObjectHandle LeftFaceForwardEdge()  const { return mDestEdgeCW; }
  ObjectHandle RightFaceOriginEdge()  const { return mOriginEdgeCW; }
  ObjectHandle RightFaceForwardEdge() const { return mDestEdgeCCW; }

  bool AreVerticesValid() { return mOriginVertexIdx != NULL_OBJECTHANDLE && mDestVertexIdx != NULL_OBJECTHANDLE; }

  std::vector<int> mConstraintIndices;
  int mFlags = 0;

  void AddConstraintIdx(int idx)
  {
    for (uint i = 0; i < mConstraintIndices.size(); ++i)
    {
      if (mConstraintIndices[i] == idx)
        return; //no duplicates
    }
    mConstraintIndices.push_back(idx);
  }
  bool HasConstraintIdx(int constraintidx)
  {
    for (uint i = 0; i < mConstraintIndices.size(); ++i)
    {
      if (mConstraintIndices[i] == constraintidx)
        return true;
    }
    return false;
  }

  bool InFreeList() { return (mFlags & IN_FREELIST) != 0; }
  bool IsBoundary() { return (mFlags & IS_BOUNDARY) != 0; }
  bool IsConstrainted() { return IsBoundary() || !mConstraintIndices.empty(); }
};

struct sPickedCSPtRes
{
  ObjectHandle mConstraintHandle = NULL_OBJECTHANDLE;
  uint mPointIdx = (uint)-1;
  ObjectHandle mCSIdentifier = NULL_OBJECTHANDLE;
};
#define PATHING_MODE_STOP 0
#define PATHING_MODE_START 1

struct sNavMeshDebugControls
{
  bool mDebugOnlySpecified = false;
  ObjectHandle mDebugEdge1 = NULL_OBJECTHANDLE;
  ObjectHandle mDebugEdge2 = NULL_OBJECTHANDLE;
  ObjectHandle mDebugFace1 = NULL_OBJECTHANDLE;
  ObjectHandle mDebugFace2 = NULL_OBJECTHANDLE;

  bool mDisplayEdges = false;
  bool mShowEdgeInfo = false;
  bool mShowFaceInfo = false;
  bool mShowVertexInfo = false;
  bool mShowSectorGrid = false;
  bool mMouseOverViewMode = false;

  bool mShowEdgeAdjacencies = false;
  bool mShowEdgeConstraints = false;
  bool mShowFaceAdjacencies = false;

  //gameplay features
  bool mShowFaceInfoGameplay = true;

  //
  bool mDisplayPathfinder = false;
  bool mStartRecordStartEndPts = false;
  int mPathingMode = PATHING_MODE_STOP;
  Vector3 mPathStart = Vector3::ZERO;
  Vector3 mPathEnd = Vector3::ZERO;
  std::vector<ObjectHandle> mWayPointFaces;
  std::vector<Vector3> mWayPoints;
  Vector3 mPatherPosition = Vector3::ZERO;
  uint mPatherSegment;
  float mPatherTime;

  bool mDebugPathFromATB = false;
};

//corresponds to 1 shell of vertices above and below, until the intersection point of each intersected constrainted edge
struct Shell
{
  Shell()
    :mStartVtx(NULL_OBJECTHANDLE), mEndVtx(NULL_OBJECTHANDLE), mBaseEdgeHandle(NULL_OBJECTHANDLE),
    mGameplayFlags(0)
  { }
  //cant do ObjectHandle mStartVtx = NULL_OBJECTHANDLE; in local struct? Probably msvc2013 fuckery
  ObjectHandle mStartVtx;
  ObjectHandle mEndVtx;
  ObjectHandle mBaseEdgeHandle;

  std::vector<ObjectHandle> mShellVerticesAbove;
  std::vector<ObjectHandle> mShellVerticesBelow;
  uint mGameplayFlags;
};

enum INSERTION_STEPS
{
  BEFORE_FLIP_STEP = 1 << 0,
  AFTER_EDGE_SPLIT_STEP = 1 << 1,
  BEFORE_INSERT_SEGMENT_STEP = 1 << 2,
  BEFORE_RETRIANGULATE_STEP = 1 << 3
};

//Navmesh
class cNavMesh
{
public: 
  cNavMesh();
  ~cNavMesh();

  bool Initialize(const Vector3& min, const Vector3& max, const char* navmeshfilename);
  void Destroy();

  void GetNavMeshLimits(Vector3& worldmin, Vector3& worldmax) { worldmin = mSectorGrid.mWorldMin; worldmax = mSectorGrid.mWorldMax; }

  bool SaveNavmesh();
  void BuildDebugATB(TwBar* viewbar);
  bool PickConstraintedPoint(const Vector3& position, sPickedCSPtRes& result, float width = 0.425f, float height = 0.0275f, float depth = 0.425f);

  void Update(float dt);
  ObjectHandle InsertSegmentConstraint(const Vector3& pt1, const Vector3& pt2, ObjectHandle constrainthandle = NULL_OBJECTHANDLE, int stopatstep = 0);
  ObjectHandle InsertNPointPolyConstraint(Vector3* pts, uint count, uint extraflags = 0, ObjectHandle identifier = NULL_OBJECTHANDLE);

  void DeleteConstraint(ObjectHandle constrainthandle);
  bool ValidateNavMesh(bool logatend = false); //true -> no errors

  Matrix4 mTransform;
  cModelGfxCmp* mNavGfxCmp;

  std::vector<sNavVertex> mVertices;
  SimpleSlotMap mVertexSlots;

  std::vector<sNavFace> mFaces;
  SimpleSlotMap mFaceSlots;

  std::vector<sNavEdge> mEdges;
  SimpleSlotMap mEdgeSlots;

  std::vector<sConstraint> mConstraints;
  SimpleSlotMap mConstraintSlots;

  sNavMeshDebugControls mDebugControls;
  cPathfinderContext mDebugContext;
  cPathfinderDebug mDebugPathfinder;

public:
  //game-related functions
  void InsertHole(Vector3* points, uint ptcount);

public:
  ObjectHandle InsertVertex(const Vector3& point, int stopatstep = 0);
  PointLocateResult LocatePoint(const Vector3& pt);
protected:
  void FlipEdges(ObjectHandle vtxidx, std::deque<ObjectHandle>& edgestack);
  void Flip(sNavEdge* edge);
  bool IsFlippable(sNavEdge* edge);
  void SplitEdge(sNavEdge* edgetosplit, sNavVertex* inserted_vtx, sNavEdge* edge_newVtxToOldOrigin, sNavEdge* edge_newVtxToOldDest);

  void FindRotationEdgesAboutVtx(ObjectHandle sourcevtxh, const Vector2& edgeprojected, float edge_sqlength, ObjectHandle& ccw_edge, ObjectHandle& cw_edge);
  
  void SplitVertices(ObjectHandle baseedgeh, std::vector<ObjectHandle>& vertices_samesideof_edge, ObjectHandle& splitter_vtxh_out,
    std::vector<ObjectHandle>& verticesOutL, std::vector<ObjectHandle>& verticesOutR);

  sNavEdge* GetEdgeFromFace(sNavFace* f, ObjectHandle edge_excluded, ObjectHandle vtx1h, ObjectHandle vtx2h);
  sNavEdge* GetEdgeFromFace(sNavFace* f, ObjectHandle vtx1, ObjectHandle vtx2);

  void RetriangulateHoleFromRemovedVertex(sNavVertex* vtx, std::vector<ObjectHandle>& connected_edges);
  bool IsCollinear(sNavEdge* e1, sNavEdge* e2);
  void RetriangulateWithBaseEdge(const Shell& shell);

public:
  sNavVertex* AddVertex();
  sNavEdge* AddEdge();
  sNavFace* AddFace();
  sConstraint* AddConstraint();

  ObjectHandle AddVertexGetHandle() { return AddVertex()->mHandle; }
  ObjectHandle AddEdgeGetHandle()   { return AddEdge()->mHandle; }
  ObjectHandle AddFaceGetHandle()   { return AddFace()->mHandle; }
  ObjectHandle AddConstraintGetHandle() { return AddConstraint()->mHandle; }

  sNavVertex* GetVertex(ObjectHandle handle);
  sNavEdge* GetEdge(ObjectHandle handle, bool warn = true);
  sNavFace* GetFace(ObjectHandle handle, bool warn = true);
  sConstraint* GetConstraint(ObjectHandle handle);
  
  void RemoveVertex(ObjectHandle handle);
  void RemoveEdge(ObjectHandle handle);
  void RemoveFace(ObjectHandle handle);
  void RemoveConstraint(ObjectHandle handle);
  void RemoveVertex(sNavVertex* vtx) { RemoveVertex(vtx->mHandle); }
  void RemoveEdge(sNavEdge* edge) { RemoveEdge(edge->mHandle); }
  void RemoveFace(sNavFace* face)    { RemoveFace(face->mHandle); }
  void RemoveConstraint(sConstraint* cs) { RemoveConstraint(cs->mHandle); }

  cSectorGrid mSectorGrid;
  String mNavmeshFilename;

public:
  //mathematical computations + pathing
  float ComputeMaximumWidthSquared(ObjectHandle triangleh, ObjectHandle edge1oftriangleh, ObjectHandle edge2oftriangleh);
  float GetArcLengthDist(sNavEdge* e1, sNavEdge* e2, float radius);

};

ObjectHandle GetNonSharedVertexFromFace(sNavFace* face, sNavEdge* edge);
ObjectHandle GetSharedVertex(sNavEdge* edge1, sNavEdge* edge2);
ObjectHandle GetOtherNonSharedVertex(sNavEdge* edge1, sNavEdge* edge2, sNavEdge* notofedge);
ObjectHandle GetOtherEdgeFromFace(sNavFace* face, ObjectHandle e1, ObjectHandle e2);
void CopyGameplayFlags(sNavFace* src, sNavFace* dest);

struct VtxInfo
{
  VtxInfo(ObjectHandle h, bool below = false)
    :mBaseEdgeH(h), mOnPositiveSide(below)
  { }
  ObjectHandle mBaseEdgeH = NULL_OBJECTHANDLE;
  bool mOnPositiveSide = false;
};

bool CircleFrom3Pts(const Vector3& a, const Vector3& b, const Vector3& c, Vector3& circlecenter, float& radiussq);

END_NSP

#endif
