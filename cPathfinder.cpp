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
#include "cPathfinder.h"
#include "cNavMesh.h"
#include "cUnitCollision.h"
#include "Simulation/cRTSSimulation.h"

#include "Graphics/cMeshQuickDraw.h"
#include "GraphicsManaged/cTextQuickDraw.h"
#include "GraphicsManaged/cManagedRenderer.h"

ENGINE_NSP

//#define LOG_PATHFINDER 1 //enable for more information

AStarNode* cPathfinderContext::GetNode(ObjectHandle h)
{
  if (h >= mAStarNodes.size())
  {
    LOG("AStarNode Handle %d not found", h);
    return nullptr;
  }
  return &mAStarNodes[h];
}

ObjectHandle cPathfinderContext::AddNewNodeGetHandle()
{
  auto node = AddNewNode();
  return node->mHandle;
}

AStarNode* cPathfinderContext::AddNewNode()
{
  mAStarNodes.emplace_back();
  AStarNode* node = &mAStarNodes[mAStarNodes.size() - 1];
  node->mHandle = (ObjectHandle)(mAStarNodes.size() - 1);
  return node;
}

void cPathfinder::FindPath(cPathfinderContext* context, const Vector3& start, const Vector3& end, std::vector<ObjectHandle>& astarnodes_handles_out, Vector3& adjustedend_out, float radius)
{
  cNavMesh* navmesh = context->mMesh;
  auto findTriangle = [&navmesh](sSectorCoords* coords, uint count, const Vector3& pt) -> ObjectHandle
  {
    ObjectHandle ret = NULL_OBJECTHANDLE;

    for (uint i = 0; i < count; ++i)
    {
      sSector* s = navmesh->mSectorGrid.GetSector(coords[i]);
      if (!s)
        continue;
      for (uint j = 0; j < s->mTrianglesInSector.size(); ++j)
      {
        sNavFace* f = navmesh->GetFace(s->mTrianglesInSector[j]);
        sNavVertex* v1 = navmesh->GetVertex(f->mVertex1Idx);
        sNavVertex* v2 = navmesh->GetVertex(f->mVertex2Idx);
        sNavVertex* v3 = navmesh->GetVertex(f->mVertex3Idx);

        //Not so accurate test for triangle. todo: can make this w
        Vector3 p1 = v1->mPosition;
        Vector3 p2 = v2->mPosition;
        Vector3 p3 = v3->mPosition;

        bool intersect = AccurateTriangle_Pt_Intersection(p1, p2, p3, pt, NAVMESH_EPSILON);
        if (!intersect)
          continue;

        //pass
        ret = f->mHandle;
        return ret;
      }

      if (ret != NULL_OBJECTHANDLE)
        break;
    }
    return ret;
  };

  adjustedend_out = end;
  //ensure end is within the world's bounds. Steering should handle the case whereby the boid hits a constrainted edge
  Vector3 worldmin, worldmax;
  context->mMesh->GetNavMeshLimits(worldmin, worldmax);
  if (adjustedend_out.x < worldmin.x)
    adjustedend_out.x = worldmin.x;
  if (adjustedend_out.x > worldmax.x)
    adjustedend_out.x = worldmax.x;
  if (adjustedend_out.z < worldmin.z)
    adjustedend_out.z = worldmin.z;
  if (adjustedend_out.z > worldmax.z)
    adjustedend_out.z = worldmax.z;

  ObjectHandle start_trih = NULL_OBJECTHANDLE;
  ObjectHandle end_trih = NULL_OBJECTHANDLE;

  //find start triangle
  sSectorCoords startsectors[MAX_SECTORS_PER_POINT];
  uint count = MAX_SECTORS_PER_POINT;
  navmesh->mSectorGrid.LocateSectors(startsectors, &count, start);
  start_trih = findTriangle(startsectors, count, start);

  //find end triangle
  sSectorCoords endsectors[MAX_SECTORS_PER_POINT];
  count = MAX_SECTORS_PER_POINT;
  navmesh->mSectorGrid.LocateSectors(endsectors, &count, adjustedend_out);
  end_trih = findTriangle(endsectors, count, adjustedend_out);

  if (start_trih == NULL_OBJECTHANDLE)
  {
    LOGC("Start triangle not found. Aborting pathfinding");
    return;
  }
  if (end_trih == NULL_OBJECTHANDLE)
  {
    LOGC("End triangle not found. Aborting pathfinding");
    return;
  }

  //LOG("Finding path from triangle %d to %d with radius %.2f", start_trih, end_trih, radius);

  //perform Astar algo
  //Each face is a node in the graph
  auto getAdjacentFace = [](sNavFace* f, sNavEdge* edge) -> ObjectHandle
  {
    if (edge->mLeftFaceIdx == f->mHandle)
      return edge->mRightFaceIdx;
    else if (edge->mRightFaceIdx == f->mHandle)
      return edge->mLeftFaceIdx;
    LOGC("Face does not share edge!");
    return NULL_OBJECTHANDLE;
  };

  struct CostCompare
  {
    CostCompare(cPathfinderContext* context)
      :mContext(context)
    {}
    cPathfinderContext* mContext;

    bool operator() (ObjectHandle a, ObjectHandle b) const
    {
      AStarNode* node_a = mContext->GetNode(a);
      AStarNode* node_b = mContext->GetNode(b);
      return node_a->mCost > node_b->mCost;
    }
  };

  std::vector<sNavFace*> closedset; //triangles already evaluated

  CostCompare c(context);
  CustomPriorityQueue<ObjectHandle, std::vector<ObjectHandle>, CostCompare> openset_astarnodes(c); //triangles to be traversed, aka frontier
  closedset.reserve(32);

  AStarNode* startnode = context->AddNewNode();
  startnode->mFaceNode = navmesh->GetFace(start_trih);
  openset_astarnodes.emplace(startnode->mHandle);

  ObjectHandle closestnongoalnodeh = NULL_OBJECTHANDLE;
  float closestnongoalnode_hval = FLT_MAX;

  auto inClosedSet = [&closedset](sNavFace* f) -> bool
  {
    for (uint i = 0; i < closedset.size(); ++i)
    {
      if (closedset[i] == f)
        return true;
    }
    return false;
  };

  auto getNodeInOpenSet = [context, &openset_astarnodes](sNavFace* corresponding_f) -> AStarNode*
  {
    auto& container = openset_astarnodes.GetContainer();
    for (uint i = 0; i < container.size(); ++i)
    {
      AStarNode* n = context->GetNode(container[i]);
      if (n->mFaceNode == corresponding_f)
        return n;
    }
    return nullptr;
  };

  auto processNewNode = [navmesh, context, &start, &adjustedend_out, radius, getNodeInOpenSet, inClosedSet, &openset_astarnodes, &closestnongoalnode_hval, &closestnongoalnodeh]
    (AStarNode* current, sNavEdge* edgecrossed, sNavFace* nextface, float radius)
  {
    if (edgecrossed->IsConstrainted())
      return;

    if (current->mEntryEdge)
    {
      float maxwidthsq = navmesh->ComputeMaximumWidthSquared(current->mFaceNode->mHandle, current->mEntryEdge->mHandle, edgecrossed->mHandle);
      float diametersq = (radius * 2.0f) * (radius * 2.0f);
      //LOG("Squared Width through edges %d to %d of face %d is %f", current->mEntryEdge->mHandle, edgecrossed->mHandle, current->mFaceNode->mHandle, maxwidthsq);

      if (maxwidthsq < diametersq)
        return;
    }
    if (inClosedSet(nextface)) //do not process faces in closed set
      return;

    sNavVertex* v1 = navmesh->GetVertex(edgecrossed->mOriginVertexIdx);
    sNavVertex* v2 = navmesh->GetVertex(edgecrossed->mDestVertexIdx);

    LineSegment2D edgelineseg(v1->mPosition.x, v1->mPosition.z, v2->mPosition.x, v2->mPosition.z);
    float heuristic_val = sqrt(edgelineseg.GetClosestDistanceToPointSquared(Vector2(adjustedend_out.x, adjustedend_out.z)));

    //first g-val, distance between start pt and entering edge
    float g_val1 = sqrt(edgelineseg.GetClosestDistanceToPointSquared(Vector2(start.x, start.z)));

    //second g-val, arc-length distance from entering edge of current to the entering edge of the next node
    float g_val2 = 0.0f;
    if (current->mEntryEdge != nullptr)
      g_val2 = current->mCost + navmesh->GetArcLengthDist(current->mEntryEdge, edgecrossed, radius);
    else
      g_val2 = current->mCost;

    float g_val3 = 0.0f;
    if (current->mEntryEdge != nullptr)
    {
      sNavVertex* entry_v1 = navmesh->GetVertex(current->mEntryEdge->mOriginVertexIdx);
      sNavVertex* entry_v2 = navmesh->GetVertex(current->mEntryEdge->mDestVertexIdx);
      LineSegment2D entryedgelineseg(entry_v1->mPosition.x, entry_v1->mPosition.z, entry_v2->mPosition.x, entry_v2->mPosition.z);
      float heuristic_val_parent = sqrt(entryedgelineseg.GetClosestDistanceToPointSquared(Vector2(adjustedend_out.x, adjustedend_out.z)));
      g_val3 = current->mCost + (heuristic_val_parent - heuristic_val);
    }
    else
    {
      //start node, no entry edge
      float heuristic_val_parent = start.Distance(adjustedend_out);
      g_val3 = current->mCost + (heuristic_val_parent - heuristic_val);
    }

    float final_gval = g_val1;
    final_gval = final_gval < g_val2 ? g_val2 : final_gval;
    final_gval = final_gval < g_val3 ? g_val3 : final_gval;

    AStarNode* existingnode = getNodeInOpenSet(nextface);
    if (existingnode)
    {
      if (final_gval < existingnode->mCost)
      {
        //replace the node and resort
        existingnode->mFaceNode = nextface;
        existingnode->mParentNodeHandle = current->mHandle;
        existingnode->mEntryEdge = edgecrossed;
        existingnode->mCost = final_gval;
        openset_astarnodes.Resort(CostCompare(context));

        if (heuristic_val < closestnongoalnode_hval)
        {
          closestnongoalnode_hval = heuristic_val;
          closestnongoalnodeh = existingnode->mHandle;
        }
      }
    }
    //else if (neighbour in closedset and cost < g(neighbour)) //only applies for inadmissible heuristics
    //remove neighbour from close
    else if (!inClosedSet(nextface))//not in open and not in closed, unexplored node
    {
      //add new node
      AStarNode* newnode = context->AddNewNode();
      newnode->mFaceNode = nextface;
      newnode->mParentNodeHandle = current->mHandle;
      newnode->mEntryEdge = edgecrossed;
      newnode->mCost = final_gval;
      openset_astarnodes.push(newnode->mHandle);

      if (heuristic_val < closestnongoalnode_hval)
      {
        closestnongoalnode_hval = heuristic_val;
        closestnongoalnodeh = newnode->mHandle;
      }
    }
  };

  ObjectHandle goalnodeh = NULL_OBJECTHANDLE;
  do
  {
    ObjectHandle nodeh = openset_astarnodes.top();
    openset_astarnodes.pop();
    AStarNode* node = context->GetNode(nodeh);
    if (node->mFaceNode->mHandle == end_trih) //reached goal, by a*'s defination we are gauranteed the first goal node opened is the optimal path
    {
      goalnodeh = node->mHandle;
      break;
    }
    closedset.push_back(node->mFaceNode);

    //get adjacent neighbour nodes, (ie triangles)
    sNavFace* f = node->mFaceNode;
    sNavEdge* e1 = navmesh->GetEdge(f->mEdge1);
    sNavEdge* e2 = navmesh->GetEdge(f->mEdge2);
    sNavEdge* e3 = navmesh->GetEdge(f->mEdge3);

    ObjectHandle f1h = getAdjacentFace(f, e1);
    ObjectHandle f2h = getAdjacentFace(f, e2);
    ObjectHandle f3h = getAdjacentFace(f, e3);

    //boundary edges might have some of their faces missing
    auto isEntryEdge = [](sNavEdge* entry, sNavEdge* e) -> bool
    {
      if (!entry)
        return false;
      return entry == e;
    };
    if (!isEntryEdge(node->mEntryEdge, e1) && f1h != NULL_OBJECTHANDLE)
    {
      sNavFace* f1 = navmesh->GetFace(f1h);
      processNewNode(node, e1, f1, radius);
    }
    if (!isEntryEdge(node->mEntryEdge, e2) && f2h != NULL_OBJECTHANDLE)
    {
      sNavFace* f2 = navmesh->GetFace(f2h);
      processNewNode(node, e2, f2, radius);
    }
    if (!isEntryEdge(node->mEntryEdge, e3) && f3h != NULL_OBJECTHANDLE)
    {
      sNavFace* f3 = navmesh->GetFace(f3h);
      processNewNode(node, e3, f3, radius);
    }
  } while (!openset_astarnodes.empty());

  if (goalnodeh == NULL_OBJECTHANDLE)
  {
    if (closestnongoalnodeh != NULL_OBJECTHANDLE)
    {
      AStarNode* nongoalclosest = context->GetNode(closestnongoalnodeh);
      //LOGC("Path not found, searching for the closest triangle...");
      //the next best node we can find is the one with the lowest g-value(cost) and the lowest h-val(distance to the goal point)
      AStarNode* node = nongoalclosest;

      //collect from the last node to the first. 
      do
      {
        //LOG("Face: %d", node->mFaceNode->mHandle);
        astarnodes_handles_out.push_back(node->mHandle);
        if (node->mParentNodeHandle == NULL_OBJECTHANDLE)
          node = nullptr;
        else
          node = context->GetNode(node->mParentNodeHandle);
      } while (node);

      //omit nodes that seem to move away from the unreachable goal
      //ObjectHandle prevh = NULL_OBJECTHANDLE;
      float smallestdistsq_to_pt = FLT_MAX;
      ObjectHandle closestfaceh = NULL_OBJECTHANDLE;
      int closestnode_index = -1;
      sNavEdge* closestedge = nullptr;
      Vector3 closestpt;

      //last node to the first, get the closest triangle
      for (uint i = 0; i < astarnodes_handles_out.size(); ++i)
      {
        ObjectHandle faceh = context->GetNode(astarnodes_handles_out[i])->mFaceNode->mHandle;

        sNavFace* face = navmesh->GetFace(faceh);
        sNavEdge* e1 = navmesh->GetEdge(face->mEdge1);
        sNavEdge* e2 = navmesh->GetEdge(face->mEdge2);
        sNavEdge* e3 = navmesh->GetEdge(face->mEdge3);

        auto chooseClosestEdgeToPoint = [navmesh, faceh, &closestfaceh, &closestnode_index, i, &smallestdistsq_to_pt, &closestedge, &closestpt, &adjustedend_out]
          (sNavEdge* e)
        {
          sNavVertex* v1 = navmesh->GetVertex(e->mOriginVertexIdx);
          sNavVertex* v2 = navmesh->GetVertex(e->mDestVertexIdx);
          LineSegment2D l(v1->mPosition.x, v1->mPosition.z, v2->mPosition.x, v2->mPosition.z);

          Vector2 closestpt2d;
          float distsq = 0.0f;
          l.GetClosestPointOnLineToPt(Vector2(adjustedend_out.x, adjustedend_out.z), closestpt2d, distsq);

          if (distsq < smallestdistsq_to_pt)
          {
            smallestdistsq_to_pt = distsq;
            closestpt.Set(closestpt2d.x, 0, closestpt2d.y);
            closestedge = e;
            closestfaceh = faceh;
            closestnode_index = i;
          }
        };
        chooseClosestEdgeToPoint(e1);
        chooseClosestEdgeToPoint(e2);
        chooseClosestEdgeToPoint(e3);
      } 

      if (closestnode_index != -1)
      {
        //pop_front to the closest triangle
        astarnodes_handles_out.erase(astarnodes_handles_out.begin(), astarnodes_handles_out.begin() + closestnode_index); 
      }

      adjustedend_out = closestpt;
      std::reverse(astarnodes_handles_out.begin(), astarnodes_handles_out.end());
    }
    else
      LOGC("Path not found for goal nodes and even closest goal nodes");
    return;
  }

  //goal found
  AStarNode* node = context->GetNode(goalnodeh);
  do
  {
    astarnodes_handles_out.push_back(node->mHandle);
    //LOG("Face: %d", node->mFaceNode->mHandle);
    if (node->mParentNodeHandle == NULL_OBJECTHANDLE)
      node = nullptr;
    else
      node = context->GetNode(node->mParentNodeHandle);
  } while (node);
  std::reverse(astarnodes_handles_out.begin(), astarnodes_handles_out.end());
}

struct sFunnel
{
  Vector3 mApexPos;
  Vector3 mFeelerAPos;
  Vector3 mFeelerBPos;

  Vector3 mFeelerAToApex;
  Vector3 mApexToFeelerB;
  Vector3 mPerpVecA;
  Vector3 mPerpVecB;

  sNavVertex* mApexVtx = nullptr;
  sNavVertex* mFeelerAVtx = nullptr;
  sNavVertex* mFeelerBVtx = nullptr;
  
  uint mFeelerAPortal;
  uint mFeelerBPortal;

  sFunnelOpcodes mOpcode;

  inline void ComputeVectorsAndPerpendiculars()
  {
    mFeelerAToApex = mApexPos - mFeelerAPos;
    mApexToFeelerB = mFeelerBPos - mApexPos;

    mPerpVecA = Vector3(-mFeelerAToApex.z, 0, mFeelerAToApex.x);
    mPerpVecB = Vector3(-mApexToFeelerB.z, 0, mApexToFeelerB.x);
  }
  
  inline void ComputeSideA()
  {
    mFeelerAToApex = mApexPos - mFeelerAPos;
    mPerpVecA = Vector3(-mFeelerAToApex.z, 0, mFeelerAToApex.x);
  }

  inline void ComputeSideB()
  {
    mApexToFeelerB = mFeelerBPos - mApexPos;
    mPerpVecB = Vector3(-mApexToFeelerB.z, 0, mApexToFeelerB.x);
  }

  inline void ComputeOpcode(const Vector3& internalpt)
  {
    float crossval = (mFeelerAToApex.x * mApexToFeelerB.z - mFeelerAToApex.z * mApexToFeelerB.x);
    bool positiveCrossP = crossval > 0.0f;
    bool zeroCrossP = crossval == 0.0f;
    bool negativeCrossP = crossval < 0.0f;

    Vector3 apex_to_pt = internalpt - mApexPos;

    float dotval1 = mPerpVecA.DotProduct(apex_to_pt);
    float dotval2 = mPerpVecB.DotProduct(apex_to_pt);

    if (positiveCrossP)
    {
      if (dotval1 >= 0.0f && dotval2 >= 0.0f)
      {
        mOpcode.mPositiveAPositiveB = FUNNEL_INTERNAL;
        mOpcode.mNegativeAPositiveB = mOpcode.mPositiveANegativeB = mOpcode.mNegativeANegativeB = FUNNEL_EXTERNAL;
      }
      else if (dotval1 < 0.0f || dotval2 < 0.0f)
      {
        mOpcode.mPositiveAPositiveB = FUNNEL_EXTERNAL;
        mOpcode.mNegativeAPositiveB = mOpcode.mPositiveANegativeB = mOpcode.mNegativeANegativeB = FUNNEL_INTERNAL;
      }
    }
    else if (negativeCrossP)
    {
      if (dotval1 < 0.0f && dotval2 < 0.0f)
      {
        mOpcode.mNegativeANegativeB = FUNNEL_INTERNAL;
        mOpcode.mNegativeAPositiveB = mOpcode.mPositiveANegativeB = mOpcode.mPositiveAPositiveB = FUNNEL_EXTERNAL;
      }
      else if (dotval1 >= 0.0f || dotval2 >= 0.0f)
      {
        mOpcode.mNegativeANegativeB = FUNNEL_EXTERNAL;
        mOpcode.mNegativeAPositiveB = mOpcode.mPositiveANegativeB = mOpcode.mPositiveAPositiveB = FUNNEL_INTERNAL;
      }
    }
    else if (zeroCrossP)
    {
      //ret.mNegativeAPositiveB and ret.mPositiveANegativeB evaluate to a single point
      if (dotval1 >= 0.0f)
      {
        mOpcode.mPositiveAPositiveB = mOpcode.mNegativeAPositiveB = mOpcode.mPositiveANegativeB = FUNNEL_INTERNAL;
        mOpcode.mNegativeANegativeB = FUNNEL_EXTERNAL;
      }
      else
      {
        mOpcode.mPositiveAPositiveB = FUNNEL_EXTERNAL;
        mOpcode.mNegativeANegativeB = mOpcode.mNegativeAPositiveB = mOpcode.mPositiveANegativeB = FUNNEL_INTERNAL;
      }
    }
  }

  inline FUNNEL_ORIENTATION ClassifyPoint(const Vector3& pttoclassify)
  {
    Vector3 apex_to_pt = pttoclassify - mApexPos;
    float dotval1 = mPerpVecA.DotProduct(apex_to_pt);
    float dotval2 = mPerpVecB.DotProduct(apex_to_pt);
    if (dotval1 >= 0.0f && dotval2 >= 0.0f)
      return mOpcode.mPositiveAPositiveB;
    else if (dotval1 >= 0.0f && dotval2 < 0.0f)
      return mOpcode.mPositiveANegativeB;
    else if (dotval1 < 0.0f && dotval2 >= 0.0f)
      return mOpcode.mNegativeAPositiveB;
    else if (dotval1 < 0.0f && dotval2 < 0.0f)
      return mOpcode.mNegativeANegativeB;
    return FUNNEL_EXTERNAL;
  }
};

void cPathfinder::ProduceFunnelGuidedPath(cPathfinderContext* context, const Vector3& start, const Vector3& end,
                                              std::vector<ObjectHandle>& astar_nodes, std::vector<Vector3>& pathpoints, 
                                              float radius, cPathfinderDebug* pathdbg)
{
  cNavMesh* navmesh = context->mMesh;
  if (astar_nodes.empty())
    return;
  if (astar_nodes.size() == 1)
  {
    pathpoints.push_back(start);
    pathpoints.push_back(end);
    return;
  }
  //radius = 0.0f;

  
  Vector3 initial_apex(start);
  
  //last tested vertices for each side
  sNavVertex* last_tested_sideA = nullptr, *last_tested_sideB = nullptr;
  uint sideA_portal = 0, sideB_portal = 0;
  bool needreinitfunnel = false, restartedfromSideJ = false, restartedfromSideK = false;

  std::vector<ObjectHandle> verticesSideJ;
  verticesSideJ.reserve(astar_nodes.size());
  std::vector<ObjectHandle> verticesSideK;
  verticesSideK.reserve(astar_nodes.size());
  std::vector<Vector3> grownVtxPositionsSideJ;
  grownVtxPositionsSideJ.reserve(astar_nodes.size());
  std::vector<Vector3> grownVtxPositionsSideK;
  grownVtxPositionsSideK.reserve(astar_nodes.size());

  {
    //gather vertices on each side of the portal (duplicates included), can probably do this in a single loop
    //note that the reference of indices of astar nodes to vertices on each side differs by 1
    //ie.  astarnode[1]->mEntryEdge.vertices = {verticesSideJ[0] and verticesSideK[0]}
    //and.  astarnode[i]->mEntryEdge.vertices = {verticesSideJ[i-1] and verticesSideK[i-1]}
    // and thus astarnode[0] represents the start point(mEntryEdge is null)
    AStarNode* firstportal = context->GetNode(astar_nodes[1]);
    verticesSideJ.push_back(firstportal->mEntryEdge->mOriginVertexIdx);
    ObjectHandle trackerh = firstportal->mEntryEdge->mDestVertexIdx;
    for (uint i = 2; i < astar_nodes.size(); ++i)
    {
      sNavEdge* portal = context->GetNode(astar_nodes[i])->mEntryEdge;
      ObjectHandle last_vtxh = verticesSideJ.back();

      if (portal->mOriginVertexIdx == last_vtxh || portal->mDestVertexIdx == last_vtxh)
      {
        verticesSideJ.emplace_back(last_vtxh);
        if (portal->mOriginVertexIdx == last_vtxh)
          trackerh = portal->mDestVertexIdx;
        else if (portal->mDestVertexIdx == last_vtxh)
          trackerh = portal->mOriginVertexIdx;
      }
      else
      {
        if (portal->mOriginVertexIdx == trackerh)
          verticesSideJ.emplace_back(portal->mDestVertexIdx);
        else if (portal->mDestVertexIdx == trackerh)
          verticesSideJ.emplace_back(portal->mOriginVertexIdx);
      }
    }

    verticesSideK.push_back(firstportal->mEntryEdge->mDestVertexIdx);
    trackerh = firstportal->mEntryEdge->mOriginVertexIdx;
    for (uint i = 2; i < astar_nodes.size(); ++i)
    {
      sNavEdge* portal = context->GetNode(astar_nodes[i])->mEntryEdge;
      ObjectHandle last_vtxh = verticesSideK.back();

      if (portal->mOriginVertexIdx == last_vtxh || portal->mDestVertexIdx == last_vtxh)
      {
        verticesSideK.emplace_back(last_vtxh);
        if (portal->mOriginVertexIdx == last_vtxh)
          trackerh = portal->mDestVertexIdx;
        else if (portal->mDestVertexIdx == last_vtxh)
          trackerh = portal->mOriginVertexIdx;
      }
      else
      {
        if (portal->mOriginVertexIdx == trackerh)
          verticesSideK.emplace_back(portal->mDestVertexIdx);
        else if (portal->mDestVertexIdx == trackerh)
          verticesSideK.emplace_back(portal->mOriginVertexIdx);
      }
    }
    /*
    if (pathdbg)
      pathdbg->AddApex(Vector3::ZERO);
      */

    auto getNextNonRepeatVtxHandle = [](std::vector<ObjectHandle>& handles, ObjectHandle cur_h, int portal) -> ObjectHandle
    {
      //get 1st non-repeat vtx
      for (int j = portal; j < (int)handles.size(); ++j)
      {
        if (handles[j] != cur_h)
          return handles[j];
      }
      return NULL_OBJECTHANDLE;
    };

    //grow vertex positions by the radius
    for (uint i = 1; i < astar_nodes.size(); ++i)
    {
      int portalidx = (int)i - 1;
      int prevportal = portalidx - 1;
      int nextportal = portalidx + 1;

      auto growVtx = [radius](const Vector3& sharedpt, const Vector3& ptbefore, const Vector3& ptafter, const Vector3& internalpt)
        -> Vector3
      {
        Vector3 shared_to_internal = internalpt - sharedpt;
        
        Vector3 v1 = sharedpt - ptbefore;
        Vector3 v1_perp(-v1.z, 0.0f, v1.x);
        if (v1_perp.DotProduct(shared_to_internal) < 0.0f)
          v1_perp = -v1_perp;

        Vector3 v2 = ptafter - sharedpt;
        Vector3 v2_perp(-v2.z, 0.0f, v2.x);

        float dotval = v1.DotProduct(v2);
        if (dotval < 0.0f)
        {
          //-ve side, v2perp should point opposite of v1perp
          if (v1_perp.DotProduct(v2_perp) >= 0.0f)
            v2_perp = -v2_perp;
        }
        else if (dotval == 0.0f)
        {
          //perpendicular, use cross product value
          float crossval = (v1.x * v2.z - v1.z * v2.x);
          if (crossval >= 0.0f) //left side perp
            v2_perp = (v2_perp.DotProduct(-v1) < 0.0f) ? -v2_perp : v2_perp;
          else if (crossval < 0.0f) //right side perp
            v2_perp = (v2_perp.DotProduct(-v1) >= 0.0f) ? -v2_perp : v2_perp;
        }
        else //if ()
        {
          //+ve side, v2perp should point +ve of v1perp
          if (v1_perp.DotProduct(v2_perp) < 0.0f)
            v2_perp = -v2_perp;
        }

        v1_perp = v1_perp.NormalisedCopy() * radius;
        v2_perp = v2_perp.NormalisedCopy() * radius;

        //since we're about to do line intersections to get the best (approximate) point,
        //figure out if we need to rescale the point (esp. reflex protrusions)
        bool needrescale = true;
        Vector3 ptbefore_to_internalpt = internalpt - ptbefore;
        float crossproductsign = v1.x * v2.z - v2.x * v1.z;
        if (crossproductsign >= 0.0f)
        { 
          float c2 = v1.x * ptbefore_to_internalpt.z - ptbefore_to_internalpt.x * v1.z;
          if (c2 >= 0.0f)
            needrescale = false;
        }
        else if (crossproductsign < 0.0f)
        {
          float c2 = v1.x * ptbefore_to_internalpt.z - ptbefore_to_internalpt.x * v1.z;
          if (c2 < 0.0f)
            needrescale = false;
        }

        Vector3 ls1_start = ptbefore + v1_perp, ls1_end = sharedpt + v1_perp;
        Vector3 ls2_start = sharedpt + v2_perp, ls2_end = ptafter + v2_perp;
        
        float line1param = 0.0f, line2param = 0.0f;
        LineIntersectRes res = GetLineIntersectionParams(Vector2(ls1_start.x, ls1_start.z), Vector2(ls1_end.x, ls1_end.z),
                                  Vector2(ls2_start.x, ls2_start.z), Vector2(ls2_end.x, ls2_end.z),
                                  line1param, line2param);

        if (res == INFINITE_INTERSECT)
        {
          Vector3 intersectpt = ls1_start + (ls1_end - ls1_start) * line1param;
          if (!needrescale)
            return intersectpt;
          //do rescale
          float distsq = intersectpt.SquaredDistance(sharedpt);
          float epsilon = 0.25f;
          float radiussq = (radius + epsilon) * (radius + epsilon);
          if (radiussq < distsq)
          {
            //reduce the distance. this usually occurs when lines are close to being parallel to each other
            Vector3 shared_to_intersect = intersectpt - sharedpt;
            return sharedpt + (radius + epsilon) * shared_to_intersect.NormalisedCopy();
          } 
          else
            return intersectpt;
        }
        else //if (res == PARALLEL)
          return ls1_end;
      };

      //compute side j
      {
        ObjectHandle prev_vtxh = NULL_OBJECTHANDLE;
        if (prevportal >= 0)
          prev_vtxh = verticesSideJ[prevportal];
        ObjectHandle current_vtxh = verticesSideJ[portalidx];
        if (prev_vtxh == current_vtxh) //repeat vtx, already computed
        {
          Vector3 p = grownVtxPositionsSideJ.back();
          grownVtxPositionsSideJ.emplace_back(p);
        }
        else
        {
          Vector3 prev_vtxpos(0, 0, 0), shared_vtxpos(0, 0, 0), next_vtxpos(0, 0, 0), internal_vtxpos(0, 0, 0);
          if (prevportal < 0)
            prev_vtxpos = start;
          else
          {
            ObjectHandle prev_vtxh = verticesSideJ[prevportal];
            prev_vtxpos = context->mMesh->GetVertex(prev_vtxh)->mPosition;
          }

          shared_vtxpos = context->mMesh->GetVertex(current_vtxh)->mPosition;
          internal_vtxpos = context->mMesh->GetVertex(verticesSideK[portalidx])->mPosition;

          ObjectHandle next_vtxh = getNextNonRepeatVtxHandle(verticesSideJ, current_vtxh, nextportal);
          if (next_vtxh == NULL_OBJECTHANDLE)
            next_vtxpos = end;
          else
            next_vtxpos = context->mMesh->GetVertex(next_vtxh)->mPosition;

          Vector3 grown_vtxpos = growVtx(shared_vtxpos, prev_vtxpos, next_vtxpos, internal_vtxpos);
          grownVtxPositionsSideJ.emplace_back(grown_vtxpos);
          /*
          if (pathdbg)
            pathdbg->AddFeelerA(grown_vtxpos, 9999, true);
            */
        }
      }

      //compute side K
      {
        ObjectHandle prev_vtxh = NULL_OBJECTHANDLE;
        if (prevportal >= 0)
          prev_vtxh = verticesSideK[prevportal];
        ObjectHandle current_vtxh = verticesSideK[portalidx];
        if (prev_vtxh == current_vtxh) //repeat vtx, already computed
        {
          Vector3 p = grownVtxPositionsSideK.back();
          grownVtxPositionsSideK.emplace_back(p);
        }
        else
        {
          Vector3 prev_vtxpos(0, 0, 0), shared_vtxpos(0, 0, 0), next_vtxpos(0, 0, 0), internal_vtxpos(0, 0, 0);
          if (prevportal < 0)
            prev_vtxpos = start;
          else
          {
            ObjectHandle prev_vtxh = verticesSideK[prevportal];
            prev_vtxpos = context->mMesh->GetVertex(prev_vtxh)->mPosition;
          }

          shared_vtxpos = context->mMesh->GetVertex(current_vtxh)->mPosition;
          internal_vtxpos = context->mMesh->GetVertex(verticesSideJ[portalidx])->mPosition;

          ObjectHandle next_vtxh = getNextNonRepeatVtxHandle(verticesSideK, current_vtxh, nextportal);
          if (next_vtxh == NULL_OBJECTHANDLE)
            next_vtxpos = end;
          else
            next_vtxpos = context->mMesh->GetVertex(next_vtxh)->mPosition;

          Vector3 grown_vtxpos = growVtx(shared_vtxpos, prev_vtxpos, next_vtxpos, internal_vtxpos);
          grownVtxPositionsSideK.emplace_back(grown_vtxpos);
          /*
          if (pathdbg)
            pathdbg->AddFeelerA(grown_vtxpos, 9999, true);
            */
        }
      }
    }
#if 1
    //final step:
    //ensure the first point after the initial portal must be on the other side of the portal(side effect of growing pts)
    //if the above fails, we need to grow our initial apex point to ensure it stays within bounds,
    
    //warning: still has some cases whereby we're not supposed to flip, 
    //if it affects the pathfinding hard this code might be the problem
    Vector3 initialA = grownVtxPositionsSideJ[0];
    Vector3 initialB = grownVtxPositionsSideK[0];
    Vector3 nextpoint = end;
    if (astar_nodes.size() > 2)
    {
      if (verticesSideJ[0] == verticesSideJ[1]) //k advanced
        nextpoint = grownVtxPositionsSideK[1];
      else if (verticesSideK[0] == verticesSideK[1]) //j advanced
        nextpoint = grownVtxPositionsSideJ[1];
    }

    Vector3 to_apex = initial_apex - initialA;
    Vector3 to_nextpt = nextpoint - initialA;
    Vector3 a_b_perp = initialB - initialA;
    a_b_perp.Set(-a_b_perp.z, 0, a_b_perp.x);

    float dval1 = a_b_perp.DotProduct(to_apex);
    float dval2 = a_b_perp.DotProduct(to_nextpt);
    if ((dval1 >= 0.0f && dval2 >= 0.0f) || (dval1 < 0.0f && dval2 < 0.0f)) //both on same side
    {
      //attempt to grow the initial funnel point
      
      //try#1: mirror the apex about the initial portal
      float param = GetClosestParamOnBiInfLineToPoint(Vector2(initialA.x, initialA.z), Vector2(initialB.x, initialB.z), Vector2(initial_apex.x, initial_apex.z));
      Vector3 ptOnLine = initialA + param * (initialB - initialA);
      Vector3 ptOnLineToApex = initial_apex - ptOnLine;

      static const float mirror_const = 1.25f; //mirror is usually 2.0f, but we only need it to be slightly away
      initial_apex = initial_apex + (mirror_const * -ptOnLineToApex);
      initial_apex.y = 0.0f;
    }
#endif
  }

  auto getVertexHandleAfter = [&end, context, navmesh, &astar_nodes, &verticesSideJ, &verticesSideK]
    (ObjectHandle vtxh, uint startportal, uint* portalret) -> ObjectHandle
  {
    if (verticesSideJ[startportal] == vtxh)
    {
      for (uint k = startportal; k<verticesSideJ.size(); ++k)
      {
        if (verticesSideJ[k] != vtxh)
        {
          if (portalret)
            *portalret = k;
          return verticesSideJ[k];
        }
      }
      if (portalret)
        *portalret = (uint)verticesSideJ.size();
      return NULL_OBJECTHANDLE; //aka the end point
    }
    else if (verticesSideK[startportal] == vtxh)
    {
      for (uint k = startportal; k<verticesSideK.size(); ++k)
      {
        if (verticesSideK[k] != vtxh)
        {
          if (portalret)
            *portalret = k;
          return verticesSideK[k];
        }
      }
      if (portalret)
        *portalret = (uint)verticesSideK.size();
      return NULL_OBJECTHANDLE; //aka the end point
    }
    else
      LOGC("Vertex does not share with the correct portal!");
    return NULL_OBJECTHANDLE; //aka the end point
  };

  auto getGrownVtxPositionSideJ = [end, &grownVtxPositionsSideJ](uint portalidx) -> Vector3
  {
    if (portalidx >= grownVtxPositionsSideJ.size())
      return end;
    return grownVtxPositionsSideJ[portalidx];
  };

  auto getGrownVtxPositionSideK = [end, &grownVtxPositionsSideK](uint portalidx) -> Vector3
  {
    if (portalidx >= grownVtxPositionsSideK.size())
      return end;
    return grownVtxPositionsSideK[portalidx];
  };

  uint portalcount = (uint)verticesSideJ.size();
  sFunnel funnel;
  funnel.mApexPos = initial_apex;

  //rules: FeelerA corresponds to SideJ
  //rules: FeelerB corresponds to SideK
  {
    //initialization using the 1st portal
    //AStarNode* secondnode = context->GetNode(astar_nodes[1]);
    funnel.mFeelerAVtx = navmesh->GetVertex(verticesSideJ[0]);
    funnel.mFeelerBVtx = navmesh->GetVertex(verticesSideK[0]);

    funnel.mFeelerAPos = grownVtxPositionsSideJ[0];
    funnel.mFeelerBPos = grownVtxPositionsSideK[0];

    last_tested_sideA = funnel.mFeelerAVtx;
    last_tested_sideB = funnel.mFeelerBVtx;

    funnel.ComputeVectorsAndPerpendiculars();
    funnel.ComputeOpcode((funnel.mFeelerAPos + funnel.mFeelerBPos) * 0.5f);
    
    //pathpoints.push_back(funnel.mApexPos); //initial funnel position might be inappropriate for pathing
    pathpoints.push_back(start);

    if (pathdbg)
    {
      pathdbg->AddApex(funnel.mApexPos);
      pathdbg->AddFeelerA(funnel.mFeelerAPos, funnel.mFeelerAVtx->mHandle, true);
      pathdbg->AddFeelerB(funnel.mFeelerBPos, funnel.mFeelerBVtx->mHandle, true);
      //LOG("First feelers- a: %d b: %d", funnel.mFeelerAVtx->mHandle, funnel.mFeelerBVtx->mHandle);
    }
  }

  uint portalidx = 1;
FunnelMain:
  for (; portalidx < portalcount; ++portalidx)
  {
    //case for apex initial start not established yet
    if (needreinitfunnel)
    {
      uint nodeidx = portalidx + 1;
      AStarNode* cur_node = context->GetNode(astar_nodes[nodeidx]);
      sNavEdge* portaledge = cur_node->mEntryEdge;

      //compute apex position (boosted by radii)
      //go backwards and forwards each portal to find vertices before and after the apex on the same side
      funnel.mFeelerAVtx = funnel.mFeelerBVtx = nullptr;
      pathpoints.push_back(funnel.mApexPos);
      if (pathdbg)
        pathdbg->AddApex(funnel.mApexPos);

      Vector3 funnel_internalpt(0, 0, 0);
      
#ifdef LOG_PATHFINDER
      LOG("New apex %d, PORTAL at %d, whose face is: %d", funnel.mApexVtx ? funnel.mApexVtx->mHandle : -1, portalidx, cur_node->mFaceNode->mHandle);
#endif
      if (!funnel.mApexVtx) //our apex is null, which indicates the endpoint. stop at endpoint
        return;

      //find the initial funnel feelers
      if (restartedfromSideJ) //apex on side J, corresponding to funnel.mFeelerAPos
      { 
        if (funnel.mApexVtx->mHandle == portaledge->mOriginVertexIdx)
          funnel.mFeelerBVtx = navmesh->GetVertex(portaledge->mDestVertexIdx);
        else if (funnel.mApexVtx->mHandle == portaledge->mDestVertexIdx)
          funnel.mFeelerBVtx = navmesh->GetVertex(portaledge->mOriginVertexIdx);
        sideB_portal = portalidx;

        ObjectHandle feelerAh = getVertexHandleAfter(funnel.mApexVtx->mHandle, portalidx, &sideA_portal);
        if (feelerAh != NULL_OBJECTHANDLE) //end position
          funnel.mFeelerAVtx = navmesh->GetVertex(feelerAh);
#ifdef LOG_PATHFINDER
        else
          LOGC("Handle for feeler A not found. Should be the endpoint!");
#endif
        last_tested_sideA = funnel.mApexVtx;
        last_tested_sideB = funnel.mFeelerBVtx;
        /*
        ObjectHandle vtxAfterfeelerBh = getVertexHandleAfter(funnel.mFeelerBVtx->mHandle, portalidx, nullptr);
        funnel_internal_vtx = navmesh->GetVertex(vtxAfterfeelerBh);
        */
        uint portalidxfound = 0;
        getVertexHandleAfter(funnel.mFeelerBVtx->mHandle, portalidx, &portalidxfound);
        funnel_internalpt = getGrownVtxPositionSideK(portalidxfound);
      }
      else if (restartedfromSideK) //apex on side K, corresponding to funnel.mFeelerBPos
      {
        if (funnel.mApexVtx->mHandle == portaledge->mOriginVertexIdx)
          funnel.mFeelerAVtx = navmesh->GetVertex(portaledge->mDestVertexIdx);
        else if (funnel.mApexVtx->mHandle == portaledge->mDestVertexIdx)
          funnel.mFeelerAVtx = navmesh->GetVertex(portaledge->mOriginVertexIdx);
        sideA_portal = portalidx;

        ObjectHandle feelerBh = getVertexHandleAfter(funnel.mApexVtx->mHandle, portalidx, &sideB_portal);
        if (feelerBh != NULL_OBJECTHANDLE) //end position
          funnel.mFeelerBVtx = navmesh->GetVertex(feelerBh);
#ifdef LOG_PATHFINDER
        else
          LOGC("Handle for feeler B not found. Should be the endpoint!");
#endif
        last_tested_sideA = funnel.mFeelerAVtx;
        last_tested_sideB = funnel.mApexVtx;
        /*
        ObjectHandle vtxAfterfeelerAh = getVertexHandleAfter(funnel.mFeelerAVtx->mHandle, portalidx, nullptr);
        funnel_internal_vtx = navmesh->GetVertex(vtxAfterfeelerAh);
        */
        uint portalidxfound = 0;
        getVertexHandleAfter(funnel.mFeelerAVtx->mHandle, portalidx, &portalidxfound);
        funnel_internalpt = getGrownVtxPositionSideJ(portalidxfound);
      }
#ifdef LOG_PATHFINDER
      LOG("New feelers a: %d b: %d", funnel.mFeelerAVtx ? funnel.mFeelerAVtx->mHandle : -1, funnel.mFeelerBVtx ? funnel.mFeelerBVtx->mHandle : -1);
      LOG("SideA: %d, SideB: %d", sideA_portal, sideB_portal);
#endif
      funnel.mFeelerAPos = getGrownVtxPositionSideJ(sideA_portal);
      funnel.mFeelerBPos = getGrownVtxPositionSideK(sideB_portal);

      funnel.ComputeVectorsAndPerpendiculars();
      
      Vector3 midpt = (funnel.mFeelerAPos + funnel.mFeelerBPos) * 0.5f;
      Vector3 apex_to_midpt = midpt - funnel.mApexPos;

      funnel.ComputeOpcode(funnel.mApexPos + 0.15f * apex_to_midpt);
      FUNNEL_ORIENTATION orient = funnel.ClassifyPoint(funnel_internalpt);
      if (orient == FUNNEL_EXTERNAL)
      {
        if ((nodeidx + 1) < astar_nodes.size())
        {
          AStarNode* next_node = context->GetNode(astar_nodes[nodeidx + 1]);
          sNavEdge* nextportaledge = next_node->mEntryEdge;
          if (nextportaledge->mOriginVertexIdx == funnel.mApexVtx->mHandle || nextportaledge->mDestVertexIdx == funnel.mApexVtx->mHandle)
          {   
            funnel.ComputeOpcode(funnel.mApexPos - 0.15f * apex_to_midpt);//flip midpt
          }
        }
        else
        {
          Vector3 apex_to_midpt = midpt - funnel.mApexPos;
          funnel.ComputeOpcode(funnel.mApexPos - apex_to_midpt);
        }
      }

      if (pathdbg)
      {
        pathdbg->AddFeelerA(funnel.mFeelerAPos, funnel.mFeelerAVtx ? funnel.mFeelerAVtx->mHandle : -1, true);
        pathdbg->AddFeelerB(funnel.mFeelerBPos, funnel.mFeelerBVtx ? funnel.mFeelerBVtx->mHandle : -1, true);
      }

      needreinitfunnel = restartedfromSideJ = restartedfromSideK = false;
      continue;
    }

    //main body
    ObjectHandle vtxAh = verticesSideJ[portalidx];
    ObjectHandle vtxBh = verticesSideK[portalidx];
    sNavVertex* vtxA = navmesh->GetVertex(vtxAh);
    sNavVertex* vtxB = navmesh->GetVertex(vtxBh);

    if (last_tested_sideA && vtxAh == last_tested_sideA->mHandle)
    {
      //try to extend feeler b 
      last_tested_sideB = vtxB;
      Vector3 extend_vtx_radiusexp = getGrownVtxPositionSideK(portalidx);

      FUNNEL_ORIENTATION orient = funnel.ClassifyPoint(extend_vtx_radiusexp);
      if (orient == FUNNEL_INTERNAL)
      {
        //update new feeler B, recompute funnel.mOpcode
        funnel.mFeelerBVtx = vtxB;
        funnel.mFeelerBPos = extend_vtx_radiusexp;
        funnel.ComputeSideB();

        sideB_portal = portalidx;
        if (pathdbg)
          pathdbg->AddFeelerB(extend_vtx_radiusexp, vtxB->mHandle, true);

        Vector3 midpt = (funnel.mFeelerAPos + funnel.mFeelerBPos) * 0.5f;
        FUNNEL_ORIENTATION midptorient = funnel.ClassifyPoint(midpt);
        Vector3 internalpt = midpt;
        if (midptorient == FUNNEL_EXTERNAL) //only occurs with reflex apexes
          internalpt = funnel.mApexPos  + -(midpt - funnel.mApexPos);
        funnel.ComputeOpcode(internalpt);
      }
      else //new pt is external to the funnel
      {
        // now we need to detect if the new point is the case "b crossover a" or if it widens the funnel
        float dotval1 = funnel.mPerpVecA.DotProduct(funnel.mFeelerBPos - funnel.mApexPos);
        float dotval2 = funnel.mPerpVecA.DotProduct(extend_vtx_radiusexp - funnel.mApexPos);

        if ((dotval1 >= 0.0f && dotval2 < 0.0f) || (dotval1 < 0.0f && dotval2 >= 0.0f)) //points on both sides of a, crossover
        {
#ifdef LOG_PATHFINDER
          LOG("New apex due to b (handle: %d) crossing a (handle: %d)!", vtxB->mHandle, funnel.mFeelerAVtx ? funnel.mFeelerAVtx->mHandle : -1);
#endif
          funnel.mApexVtx = funnel.mFeelerAVtx;
          funnel.mApexPos = funnel.mFeelerAPos;
          portalidx = sideA_portal - 1; //compensate for ++portalidx
          needreinitfunnel = restartedfromSideJ = true;
          if (pathdbg)
            pathdbg->AddFeelerB(extend_vtx_radiusexp, vtxB->mHandle, false);
        }
        else if (pathdbg) //else widen
          pathdbg->AddFeelerB(extend_vtx_radiusexp, vtxB->mHandle, false);
      }
    }
    else if (last_tested_sideB && vtxBh == last_tested_sideB->mHandle) //if last_tested_sideB == nullptr, then the last tested vtx is the end
    {
      //try to extend feeler a
      last_tested_sideA = vtxA;
      Vector3 extend_vtx_radiusexp = getGrownVtxPositionSideJ(portalidx);
      
      FUNNEL_ORIENTATION orient = funnel.ClassifyPoint(extend_vtx_radiusexp);
      if (orient == FUNNEL_INTERNAL)
      {
        //update new feeler A
        funnel.mFeelerAVtx = vtxA;
        funnel.mFeelerAPos = extend_vtx_radiusexp;
        funnel.ComputeSideA();

        sideA_portal = portalidx;
        if (pathdbg)
          pathdbg->AddFeelerA(extend_vtx_radiusexp, vtxA->mHandle, true);
        
        Vector3 midpt = (funnel.mFeelerAPos + funnel.mFeelerBPos) * 0.5f;
        FUNNEL_ORIENTATION midptorient = funnel.ClassifyPoint(midpt);
        Vector3 internalpt = midpt;
        if (midptorient == FUNNEL_EXTERNAL) //only occurs with reflex apexes
          internalpt = funnel.mApexPos + -(midpt - funnel.mApexPos);
        funnel.ComputeOpcode(internalpt);
      }
      else //new pt is external to the funnel
      {
        // now we need to detect if the new point is the case "a crossover b" or if it widens the funnel
        float dotval1 = funnel.mPerpVecB.DotProduct(funnel.mFeelerAPos - funnel.mApexPos);
        float dotval2 = funnel.mPerpVecB.DotProduct(extend_vtx_radiusexp - funnel.mApexPos);

        if ((dotval1 >= 0.0f && dotval2 < 0.0f) || (dotval1 < 0.0f && dotval2 >= 0.0f))
        {
          //new apex using endpt of feeler A, one line crosses the other
#ifdef LOG_PATHFINDER
          LOG("New apex due to a (handle: %d) crossing b (handle: %d)!", vtxA->mHandle, funnel.mFeelerBVtx ? funnel.mFeelerBVtx->mHandle : -1);
#endif
          funnel.mApexVtx = funnel.mFeelerBVtx;
          funnel.mApexPos = funnel.mFeelerBPos;
          portalidx = sideB_portal - 1; //compensate for ++portalidx
          needreinitfunnel = restartedfromSideK = true;
          if (pathdbg)
            pathdbg->AddFeelerA(extend_vtx_radiusexp, vtxA->mHandle, false);
        }
        else if (pathdbg) //else widen
          pathdbg->AddFeelerA(extend_vtx_radiusexp, vtxB->mHandle, false);
      }
    }
  }

  //extend feeler a and feeler b to endpoint
  //only need to check for feeler stepovers, because narrowing the funnel just yields the endpt
  { 
    FUNNEL_ORIENTATION orient = funnel.ClassifyPoint(end);
    if (orient == FUNNEL_EXTERNAL)
    {
      if (funnel.mFeelerBVtx)
      {
        //move feeler A to endpt
        float dotval1 = funnel.mPerpVecB.DotProduct(funnel.mFeelerAPos - funnel.mApexPos);
        float dotval2 = funnel.mPerpVecB.DotProduct(end - funnel.mApexPos);
        if ((dotval1 >= 0.0f && dotval2 < 0.0f) || (dotval1 < 0.0f && dotval2 >= 0.0f))
        {
#ifdef LOG_PATHFINDER
          LOG("(final vtx)New apex due to a (ENDPOINT) crossing b (handle: %d)!", funnel.mFeelerBVtx ? funnel.mFeelerBVtx->mHandle : -1);
#endif
          //new apex using endpt of feeler A, one line crosses the other
          funnel.mApexVtx = funnel.mFeelerBVtx;
          funnel.mApexPos = funnel.mFeelerBPos;
          portalidx = sideB_portal;
          needreinitfunnel = restartedfromSideK = true;
          goto FunnelMain;
        }
      }
      if (funnel.mFeelerAVtx)
      {
        //move feeler B to endpt
        float dotval1 = funnel.mPerpVecA.DotProduct(funnel.mFeelerBPos - funnel.mApexPos);
        float dotval2 = funnel.mPerpVecA.DotProduct(end - funnel.mApexPos);
        if ((dotval1 >= 0.0f && dotval2 < 0.0f) || (dotval1 < 0.0f && dotval2 >= 0.0f))
        {
#ifdef LOG_PATHFINDER
          LOG("(final vtx)New apex due to b (ENDPOINT) crossing a (handle: %d)!", funnel.mFeelerAVtx ? funnel.mFeelerAVtx->mHandle : -1);
#endif
          //new apex using endpt of feeler A, one line crosses the other
          funnel.mApexVtx = funnel.mFeelerAVtx;
          funnel.mApexPos = funnel.mFeelerAPos;
          portalidx = sideA_portal;
          needreinitfunnel = restartedfromSideJ = true;
          goto FunnelMain;
        }
      }
    }
  }
  
  pathpoints.push_back(end);
}

void cPathfinder::DoPathStraightening(std::vector<Vector3>& pathpoints, float radius)
{
  auto capsuleIntersectsCSEdge = [](const Capsule2D& capsule) -> bool
  {
    //construct box
    Vector3 start(capsule.mStart.x, 0, capsule.mStart.y);
    Vector3 end(capsule.mEnd.x, 0, capsule.mEnd.y);
    AABB3D box(start, end);
    box.AutoCorrectMinMax();
    Vector3 halfsize = box.GetHalfSize();
    halfsize.x += capsule.mRadius * 0.5f;
    halfsize.z += capsule.mRadius * 0.5f;
    box.SetHalfSize(halfsize);

    std::vector<sSectorCoords> sectorstosearch;
    auto navmesh = Systems::mRTSSimulation->mNavMesh;
    navmesh->mSectorGrid.LocateSectorsOfBox(sectorstosearch, box);

    for (auto sector : sectorstosearch)
    {
      sSector* s = navmesh->mSectorGrid.GetSector(sector);
      for (auto edgeh : s->mEdgesInSector)
      {
        sNavEdge* e = navmesh->GetEdge(edgeh);
        if (!e || !e->IsConstrainted())
          continue;
        sNavVertex* v1 = navmesh->GetVertex(e->mOriginVertexIdx);
        sNavVertex* v2 = navmesh->GetVertex(e->mDestVertexIdx);
        LineSegment2D ls(v1->mPosition.x, v1->mPosition.z, v2->mPosition.x, v2->mPosition.z);

        if (capsule.Intersects(ls))
          return true;
      }
    }
    return false;
  };

  for (uint i = 0; i < pathpoints.size(); )
  {
    Vector3 startpt = pathpoints[i];
    bool elim_waypt = false;
    uint elim_waypt_idx = (uint)-1;
    
    for (uint j = i + 2; j < pathpoints.size(); ++j)
    {
      Vector3 endpt = pathpoints[j];
      Capsule2D c(startpt.x, startpt.z, endpt.x, endpt.z, radius * 0.95f);

      bool intersects = capsuleIntersectsCSEdge(c);

      if (!intersects) //no constrainted edges in between
      {
        elim_waypt = true;
        elim_waypt_idx = j - 1; //eliminate point in between
#ifdef LOG_PATHFINDER
        LOG("Waypoint eliminated: %d", elim_waypt_idx);
#endif
        break;
      }
      else
      {
        //cannot eliminate 100%, so see if we can shorten it
        Vector3 midpt = pathpoints[j - 1];
        Vector3 startpt_to_mid = midpt - startpt;
        float param = GetClosestParamOnBiInfLineToPoint(Vector2(startpt.x, startpt.z), Vector2(midpt.x, midpt.z), 
          Vector2(endpt.x, endpt.z));
        if (param < 0.0f)
        {
          //behind the 1st portion, see if we can do a shortcut
          float startparam = GetClosestParamOnBiInfLineToPoint(Vector2(midpt.x, midpt.z), Vector2(endpt.x, endpt.z),
            Vector2(startpt.x, startpt.z));

          Vector3 shortcutpt = midpt + startparam * (endpt - midpt);
          Capsule2D shortcutpath(startpt.x, startpt.z, shortcutpt.x, shortcutpt.z, radius * 0.95f);
          
          bool shortcutblocked = capsuleIntersectsCSEdge(shortcutpath);
          if (!shortcutblocked)
          {
            pathpoints[j - 1] = shortcutpt;
#ifdef LOG_PATHFINDER
            LOGC("****************Shortcut found!***************");
#endif
          }
        }
      }
    } 

    if (elim_waypt)
      pathpoints.erase(pathpoints.begin() + elim_waypt_idx);
    else
      ++i;
  }
}

void cPathfinder::ModifyEndpointNoIntersection(std::vector<Vector3>& pathpoints, float radius)
{
  if (pathpoints.size() <= 1)
    return;
  //modify adjustedend_out to not connect to the closest edge
  Vector3& lastpt = pathpoints.back();
  Vector3 secondlastpt = pathpoints[pathpoints.size() - 2];
  Capsule2D c(Vector2(secondlastpt.x, secondlastpt.z), Vector2(lastpt.x, lastpt.z), radius);

  //custom intersection function. use a line segment to intersect with nav edges, and get the closest edge

  {
    Vector3 start(c.mStart.x, 0, c.mStart.y);
    Vector3 end(c.mEnd.x, 0, c.mEnd.y);
    AABB3D box(start, end);
    box.AutoCorrectMinMax();
    Vector3 halfsize = box.GetHalfSize();
    halfsize.x += c.mRadius * 0.5f;
    halfsize.z += c.mRadius * 0.5f;
    box.SetHalfSize(halfsize);

    float smallestlineparam = FLT_MAX;
    sNavEdge* closestedge = nullptr;

    std::vector<sSectorCoords> sectorstosearch;
    auto navmesh = Systems::mRTSSimulation->mNavMesh;
    navmesh->mSectorGrid.LocateSectorsOfBox(sectorstosearch, box);

    for (auto sector : sectorstosearch)
    {
      sSector* s = navmesh->mSectorGrid.GetSector(sector);
      for (auto edgeh : s->mEdgesInSector)
      {
        sNavEdge* e = navmesh->GetEdge(edgeh);
        if (!e || !e->IsConstrainted())
          continue;
        sNavVertex* v1 = navmesh->GetVertex(e->mOriginVertexIdx);
        sNavVertex* v2 = navmesh->GetVertex(e->mDestVertexIdx);
        LineSegment2D ls(v1->mPosition.x, v1->mPosition.z, v2->mPosition.x, v2->mPosition.z);

        float lineparam1 = 0.0f, lineparam2 = 0.0f;
        GetLineIntersectionParams(c.mStart, c.mEnd, ls.mStart, ls.mEnd, lineparam1, lineparam2);
        if (lineparam1 >= -0.05f && lineparam1 <= 1.05f && lineparam2 >= 0.0f && lineparam2 <= 1.0f)
        {
          if (smallestlineparam > lineparam1)
          {
            smallestlineparam = lineparam1;
            closestedge = e;
          }
        }
      }
    }

    if (closestedge)
    {
      auto navmesh = Systems::mRTSSimulation->mNavMesh;
      
      //based on the distance moved, either set endpt to lineparam or use the penetration values
      sNavVertex* v1 = navmesh->GetVertex(closestedge->mOriginVertexIdx);
      sNavVertex* v2 = navmesh->GetVertex(closestedge->mDestVertexIdx);

      Vector2 start(v1->mPosition.x, v1->mPosition.z);
      Vector2 end(v2->mPosition.x, v2->mPosition.z);
      
      
      Vector2 intersectionTangent(0, 0), reducedpt(0,0);
      float capsule_toi = 0.0f, reducedpt_to_end_distsq = 0.0f;
      bool res = GetTOICapsuleLineSegment(c.mStart, c.mEnd, c.mRadius, start, end, capsule_toi, intersectionTangent, 0.05f);
      if (res)
      {
        reducedpt = c.mStart + capsule_toi * (c.mEnd - c.mStart);
        reducedpt_to_end_distsq = reducedpt.SquaredDistance(c.mEnd);
        //LOG("TOI: %f", capsule_toi);
      }
      else
      {
        reducedpt = c.mStart;
        reducedpt_to_end_distsq = reducedpt.SquaredDistance(c.mEnd);
      }

      //compute penetration
      Vector2 pen_vec = c.mEnd - reducedpt;
      Vector2 linevec_perp = (end - start).Perpendicular();
      linevec_perp.Normalise();

      float projlength = linevec_perp.DotProduct(pen_vec);
      if (projlength >= 0.0f)
        linevec_perp = -linevec_perp;

      projlength = fabs(projlength);

      
      if (reducedpt_to_end_distsq <= (projlength * projlength))
        lastpt.Set(reducedpt.x, 0.0f, reducedpt.y);
      else
      {
        Vector2 e = c.mEnd + projlength * linevec_perp;
        lastpt.Set(e.x, 0.0f, e.y);
      }
    }
  }

}

bool cPathfinderDebug::mDrawFeelers = false;

void cPathfinderDebug::ClearAll()
{
  mApexes.clear();
  mFeelersA.clear();
  mFeelersB.clear();
}

void cPathfinderDebug::DrawFunnelAlgo(float radius)
{
  auto getColor = [](uint idx) -> Color4
  {
    uint i = idx % 5;
    if (i == 0)
      return Color4::RED;
    else if (i == 1)
      return Color4::GREEN;
    else if (i == 2)
      return Color4::BLUE;
    else if (i == 3)
      return Color4::YELLOW;
    else if (i == 4)
      return Color4::GRAY;
    return Color4::RED;
  };

  Vector3 offset(0, NAVMESH_DBG_YOFFSET, 0);
  for (uint i = 0; i < mApexes.size(); ++i)
  {
    float radius_reduction = 0.025f * i;
    cMeshQuickDraw::DrawWorldCircle(Cylinder(mApexes[i] + offset, radius, 0.1f), Color4::BLACK, 8);
    cMeshQuickDraw::DrawWorldCircle(Cylinder(mApexes[i] + offset, 0.15f, 0.1f), Color4::BLACK, 8);

    if (!mDrawFeelers)
      continue;

    auto& f_a = mFeelersA[i];
    for (uint j = 0; j < f_a.size(); ++j)
    {
      Vector3 pos = f_a[j].mPos + offset;
      Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(pos);
      char buf[32] = { 0 };
      snprintf(buf, 32, "FeelerA_%d, passed:%d", f_a[j].mVtxHandle, f_a[j].mPassed ? 1 : 0);
      cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);

      cMeshQuickDraw::DrawWorldCircle(Cylinder(pos, radius - radius_reduction, 0.1f), getColor(i));
      cMeshQuickDraw::DrawWorldCircle(Cylinder(pos, 0.15f, 0.1f), getColor(i), 8);
    }

    auto& f_b = mFeelersB[i];
    for (uint j = 0; j < f_b.size(); ++j)
    {
      Vector3 pos = f_b[j].mPos + offset;
      Vector2 screenpos = Systems::mManagedRenderer->WorldPosToNormalizedScreenPos(pos);
      char buf[32] = { 0 };
      snprintf(buf, 32, "FeelerB_%d, passed:%d", f_b[j].mVtxHandle, f_b[j].mPassed ? 1 : 0);
      cTextQuickDraw::DrawQuickText(buf, screenpos, CENTER_ALIGNMENT, 16);

      cMeshQuickDraw::DrawWorldCircle(Cylinder(pos, radius - radius_reduction, 0.1f), getColor(i));
      cMeshQuickDraw::DrawWorldCircle(Cylinder(pos, 0.15f, 0.1f), getColor(i), 8);
    }
  }
}

void cPathfinderDebug::AddApex(const Vector3& apexpos)
{
  mApexes.push_back(apexpos);
  mFeelersA.emplace_back();
  mFeelersB.emplace_back();
}

void cPathfinderDebug::AddFeelerA(const Vector3& feelerpos, ObjectHandle h, bool passed)
{
  auto& feelersA_thisfunnel = mFeelersA.back();
  feelersA_thisfunnel.emplace_back(feelerpos, h, passed);
}

void cPathfinderDebug::AddFeelerB(const Vector3& feelerpos, ObjectHandle h, bool passed)
{
  auto& feelersB_thisfunnel = mFeelersB.back();
  feelersB_thisfunnel.emplace_back(feelerpos, h, passed);
}

END_NSP

