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
#ifndef _CPATHFINDER_H
#define _CPATHFINDER_H

#include "EnginePrereqs.h"
#include <queue>
#include <algorithm>

ENGINE_NSP

class cNavMesh;
struct sNavFace;
struct sNavEdge;
class cPathfinderDebug;

struct AStarNode
{
  AStarNode()
    :mHandle(NULL_OBJECTHANDLE), mParentNodeHandle(NULL_OBJECTHANDLE), 
    mFaceNode(nullptr), mEntryEdge(nullptr), mCost(0.0f)
  {}

  ObjectHandle mHandle;
  ObjectHandle mParentNodeHandle;
  sNavFace* mFaceNode;
  sNavEdge* mEntryEdge;
  float mCost;

  bool operator() (const AStarNode& l, const AStarNode& r) const
  {
    return l.mCost < r.mCost;
  }
};

enum FUNNEL_ORIENTATION
{
  FUNNEL_INTERNAL = 0,
  FUNNEL_EXTERNAL
};

struct sFunnelOpcodes
{
  //basically 4 bit patterns, evaluating to true or false
  FUNNEL_ORIENTATION mPositiveAPositiveB = FUNNEL_INTERNAL; //opcode 11
  FUNNEL_ORIENTATION mNegativeAPositiveB = FUNNEL_INTERNAL; //opcode 01
  FUNNEL_ORIENTATION mPositiveANegativeB = FUNNEL_INTERNAL; //opcode 10
  FUNNEL_ORIENTATION mNegativeANegativeB = FUNNEL_INTERNAL; //opcode 00
};

class cPathfinderContext
{
public:
  cNavMesh* mMesh;

  //astar context
  AStarNode* GetNode(ObjectHandle h);
  ObjectHandle AddNewNodeGetHandle();
  AStarNode* AddNewNode();
  std::vector<AStarNode> mAStarNodes;

  inline void ClearAll()
  {
    mAStarNodes.clear();
  }
  //outputs
  //std::vector<ObjectHandle> mAStarResults; //handles to faces


};

class cPathfinder
{
public: 
  static void FindPath(cPathfinderContext* context, const Vector3& start, const Vector3& end, 
    std::vector<ObjectHandle>& astarnodes_handles_out, Vector3& adjustedend_out, float radius = 0.1f);
  static void ProduceFunnelGuidedPath(cPathfinderContext* context, const Vector3& start, const Vector3& end, 
    std::vector<ObjectHandle>& astar_nodes, std::vector<Vector3>& pathpoints, 
    float radius = 0.1f, cPathfinderDebug* pathdbg = nullptr);
  static void DoPathStraightening(std::vector<Vector3>& pathpoints, float radius);
  static void ModifyEndpointNoIntersection(std::vector<Vector3>& pathpoints, float radius);
};

struct sFeelerDebugInfo
{
  sFeelerDebugInfo(const Vector3& pos, ObjectHandle h, bool p)
    :mPos(pos), mVtxHandle(h), mPassed(p)
  {}
  Vector3 mPos;
  ObjectHandle mVtxHandle;
  bool mPassed;
};

class cPathfinderDebug
{
public:
  void ClearAll();

  //funnel debug
  void DrawFunnelAlgo(float radius = 0.15f);

  void AddApex(const Vector3& apexpos);
  void AddFeelerA(const Vector3& feelerpos, ObjectHandle h, bool passed);
  void AddFeelerB(const Vector3& feelerpos, ObjectHandle h, bool passed);

  //1 apex per funnel buildup
  std::vector<Vector3> mApexes;
  typedef std::vector<sFeelerDebugInfo> Feelers;

  //2 sets of feelers per funnel
  std::vector<Feelers> mFeelersA;
  std::vector<Feelers> mFeelersB;

  static bool mDrawFeelers;
};

template <typename T, typename Container, typename Comparator>
class CustomPriorityQueue : public std::priority_queue<T, Container, Comparator>
{
public:
  CustomPriorityQueue(const Comparator& compare)
    :std::priority_queue<T, Container, Comparator>(compare)
  {}

  std::vector<T>& GetContainer() { return c; }

  void Resort(const Comparator& compare)
  {
    //may be very inefficient
    //std::make_heap(reinterpret_cast<T**>(&top()), reinterpret_cast<T**>(&top()) + size(), compare);
    std::make_heap(c.begin(), c.end(), compare);
  }
};

END_NSP

#endif
