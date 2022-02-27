using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Text;
using System.Threading.Tasks;
using Unity.Assertions;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Mathematics;

namespace NSS.Data
{

    /// <summary>
    /// used to query the quadtree 
    /// </summary>
    unsafe public struct NativeQuadTree
    {
        public int root_mx, root_my, root_sx, root_sy, offset_x, offset_y;

        // Stores all the elements in the quadtree.
        // lft, top, rght, bottom & element id 
        public ConstantIndexIntList elements;

        // Stores all the element nodes in the quadtree.
        // - the elements in the leafs
        public ConstantIndexIntList element_nodes;

        // Stores all the nodes in the quadtree. The first node in this
        // sequence is always the root.
        // - the regions making up the tree
        public ConstantIndexIntList nodes;

        #region Query helpers 
        internal void push_node(ref ConstantIndexIntList list, int nd_index, int nd_depth, int nd_mx, int nd_my, int nd_sx, int nd_sy)
        {
            int back_idx = list.PushBack();
            list.Set(back_idx, QuadTree.nd_idx_mx, nd_mx);
            list.Set(back_idx, QuadTree.nd_idx_my, nd_my);
            list.Set(back_idx, QuadTree.nd_idx_sx, nd_sx);
            list.Set(back_idx, QuadTree.nd_idx_sy, nd_sy);
            list.Set(back_idx, QuadTree.nd_idx_index, nd_index);
            list.Set(back_idx, QuadTree.nd_idx_depth, nd_depth);
        }

        /// <summary>
        /// lookup the element index using the itemid on the element 
        /// !! VERY SLOW FULL LINEAR SCAN !!
        /// 
        /// commonly leaf are 5-6 nodes deep. not really worth a full scan... should never use
        /// </summary>
        /// <param name="actor_index"></param>
        /// <returns></returns>
        public int FindElementIndex(int itemid)
        {
            for(int i = 0; i < elements.Size; i++)
            {
                if (elements.Get(i, QuadTree.elt_idx_id) == itemid)
                {
                    return i; 
                }
            }
            return -1; 
        }

        internal int2 GetCenter(int elementid)
        {
            int l = elements.Get(elementid, QuadTree.elt_idx_lft);
            int r = elements.Get(elementid, QuadTree.elt_idx_rgt);
            int t = elements.Get(elementid, QuadTree.elt_idx_top);
            int b = elements.Get(elementid, QuadTree.elt_idx_btm);

            return new int2(l + (r - l) / 2, t + (b - t) / 2); 
        }
        internal int2 GetSize(int elementid)
        {
            return new int2(
                elements.Get(elementid, QuadTree.elt_idx_rgt) - elements.Get(elementid, QuadTree.elt_idx_lft),
                elements.Get(elementid, QuadTree.elt_idx_btm) - elements.Get(elementid, QuadTree.elt_idx_top));
        }

        internal int GetLeafIndex(int l, int t, int r, int b, Allocator temp_allocator = Allocator.Temp)
        {
            /*
            if(l > r)
            {
                int e = l;
                l = r;
                r = e;
            }
            if(t > b)
            {
                int e = t;
                t = b;
                b = e;
            } */
            return GetLeafIndex(l + (r - l) / 2 , t + (b - t) / 2, temp_allocator); 
        }

        internal int GetLeafIndex(int x, int y, Allocator temp_allocator = Allocator.Temp)
        {
            int* stack = stackalloc int[QuadTree.stack_size];
            ConstantIndexIntList leaves = ConstantIndexIntList.Create(QuadTree.nd_num, temp_allocator, stack, QuadTree.stack_size);

            find_leaves(ref leaves, x, y, x, y, temp_allocator);

            // should find 1 leaf for center 
            int index = -1; 
            if(leaves.Size == 1)
            {
                index = leaves.Get(0, QuadTree.nd_idx_index); 
            }
            else
            {
#if UNITY_EDITOR
                UnityEngine.Debug.LogError($"should find 1 leaf, found {leaves.Size} for location x: {x}, y: {y}"); 
#endif
            }

            leaves.Destroy();

            return index; 
        }


        /// <summary>
        /// update element region, checks if moving outside bounds of node, returns false if so
        /// 
        /// - this can be much cheaper when updating then rebuilding the whole tree.
        /// </summary>
        /// <param name="elementid"></param>
        /// <param name="l"></param>
        /// <param name="r"></param>
        /// <param name="t"></param>
        /// <param name="b"></param>
        /// <returns>true if updated, false if update needs to be defferred</returns>
        public bool UpdateElementRegion(int elementid, int l, int r, int t, int b, Allocator temp_allocator = Allocator.Temp)
        {
            int2 center = GetCenter(elementid);

            int index_old = GetLeafIndex(center.x, center.y, temp_allocator);
            if (index_old < 0) return false;

            int index_new = GetLeafIndex(l + (r - l) / 2, t + (b - t) / 2, temp_allocator);
            if (index_new < 0) return false; 

            if(index_new == index_old)
            {
                elements.Set(elementid, QuadTree.elt_idx_lft, l);
                elements.Set(elementid, QuadTree.elt_idx_rgt, r);
                elements.Set(elementid, QuadTree.elt_idx_top, t);
                elements.Set(elementid, QuadTree.elt_idx_btm, b);
                return true; 
            }

            return false; 
        }


        /// <summary>
        /// allocates elementsize/32 bytes of memory needs it for each thread...
        /// 
        /// -> internal buffer on thread index?
        /// 
        /// </summary>
        /// <param name="item_element_id"></param>
        /// <param name="item_mx"></param>
        /// <param name="item_my"></param>
        /// <param name="range"></param>
        /// <param name="allocator"></param>
        /// <returns></returns>

        public int FindClosestInRange(int item_element_id, int item_mx, int item_my, int range, Allocator allocator = Allocator.Temp)
        {
            int closest;
            
            /// NATIVELY POOL THIS BUFFER     OR   PUT ON STACK 
            /// 
            /// on 1M items its 32kb large?
            /// 
            /// - what would be limit of stack memory??
            /// 
            
            uint* temp = (uint*)UnsafeUtility.Malloc(elements.Size >> 5, 1, allocator);


            UnsafeUtility.MemClear(temp, elements.Size);
            {               
                closest = FindClosestInRange(temp, item_element_id, item_mx, item_my, range, allocator);
            }
            UnsafeUtility.Free(temp, allocator);

            return closest; 
        }

 



        /// <summary>
        /// check if the nth bit is set in the memory pointed to 
        /// </summary>
        /// <param name="temp">some memory</param>
        /// <param name="index">index in bits</param>
        /// <returns></returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool bit_is_set(uint* temp, uint index)
        {
            uint mask = (uint)1 << (byte)(index & 0b_11111); 
            return (temp[(index >> 5)] & mask) == mask;
        }

        /// <summary>
        /// set nth bit of memory to value
        /// </summary>
        /// <param name="temp">some memory</param>
        /// <param name="index">index in bits</param>
        /// <param name="value"></param>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void set_bit(uint* temp, uint index, uint value)
        {
            uint mask = (uint)1 << (byte)(index & 0b_11111);
 
            uint tempindex = index >> 5;

            uint t = temp[tempindex];

            temp[tempindex] = math.select(t & ~mask, t | mask, value != 0); 
         }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="working_memory">temp bit array of elements.size to mark elements searched, could do without.... </param>
        /// <param name="enode_idx_elt">element idx to exclude</param>
        /// <param name="item_mx"></param>
        /// <param name="item_my"></param>
        /// <param name="range"></param>
        /// <param name="allocator"></param>
        /// <returns></returns>
        public int FindClosestInRange(uint* working_memory, int enode_idx_elt, int item_mx, int item_my, int range, Allocator allocator = Allocator.Temp)
        {
            int j = 0;

            set_bit(working_memory, (uint)enode_idx_elt, 1); 

            // get leaves intersecting with item + range
            int* stack = stackalloc int[QuadTree.stack_size];
            ConstantIndexIntList leaves = ConstantIndexIntList.Create(QuadTree.nd_num, allocator, stack, QuadTree.stack_size);

            // bouding rect  
            int x1 = item_mx - range;
            int x2 = item_mx + range;
            int y1 = item_my - range;
            int y2 = item_my + range;

            // For each leaf node, look for elements that intersect.
            find_leaves(ref leaves, 0, 0, root_mx, root_my, root_sx, root_sy, x1, y1, x2, y2, allocator);

            int closest_id = -1;
            int closest_distance_sq = range * range;

            for (j = 0; j < leaves.Size; j++)
            {
                int nd_index = leaves.Get(j, QuadTree.nd_idx_index);

                // Walk the list and add elements that intersect.
                int elt_node_index = nodes.Get(nd_index, QuadTree.node_idx_fc);
                while (elt_node_index != -1)
                {
                    int element = element_nodes.Get(elt_node_index, QuadTree.enode_idx_elt);
                    int lft = elements.Get(element, QuadTree.elt_idx_lft);
                    int top = elements.Get(element, QuadTree.elt_idx_top);
                    int rgt = elements.Get(element, QuadTree.elt_idx_rgt);
                    int btm = elements.Get(element, QuadTree.elt_idx_btm);

                    if (!bit_is_set(working_memory, (uint)element))
                    {
                        int x = (lft + rgt) >> 1;
                        int y = (top + btm) >> 1;

                        int xd = x - item_mx;
                        int yd = y - item_my;

                        int distance_sq = xd * xd + yd * yd;

                        if(distance_sq < closest_distance_sq)
                        {
                            closest_id = element;
                            closest_distance_sq = distance_sq;
                        }
                        set_bit(working_memory, (uint)element, 1);
                    }
                    elt_node_index = element_nodes.Get(elt_node_index, QuadTree.enode_idx_next);
                }
            }
            leaves.Destroy();
            return closest_id;
        }


        /// <summary>
        /// find all leaves intersecting a rectangular region starting from root node
        /// </summary>
        /// <param name="leaves">preallocated output list, can expand on request</param>
        /// <param name="lft">intersection region, left</param>
        /// <param name="top">intersection region, top</param>
        /// <param name="rgt">intersection region, right</param>
        /// <param name="btm">intersection region, bottom</param>
        /// <param name="allocator">allocater for additional memory that might be needed on top of stack memory in large trees</param>
        public void find_leaves(ref ConstantIndexIntList leaves, int lft, int top, int rgt, int btm, Allocator allocator)
        {
            find_leaves(ref leaves, 0, 0, root_mx, root_my, root_sx, root_sy, lft, top, rgt, btm, allocator); 
        }


        /// <summary>
        /// find all leaves intersecting with rectangular region starting at the given node
        /// </summary>
        /// <param name="leaves">preallocated output list, can expand on request</param>
        /// <param name="node">node to start searching from</param>
        /// <param name="depth">depth of node that started search</param>
        /// <param name="mx">center x of node starting search</param>
        /// <param name="my">center y of node starting search</param>
        /// <param name="sx">size / width of node</param>
        /// <param name="sy">size / height of node</param>
        /// <param name="lft">intersection region, left</param>
        /// <param name="top">intersection region, top</param>
        /// <param name="rgt">intersection region, right</param>
        /// <param name="btm">intersection region, bottom</param>
        /// <param name="allocator">allocater for additional memory that might be needed on top of stack memory in large trees</param>
        internal void find_leaves(
            ref ConstantIndexIntList leaves,
            int node, int depth,
            int mx, int my, int sx, int sy,
            int lft, int top, int rgt, int btm, 
            Allocator allocator)
        {
            int* stack = stackalloc int[QuadTree.stack_size];
            ConstantIndexIntList work = ConstantIndexIntList.Create(QuadTree.nd_num, allocator, stack, QuadTree.stack_size);

            push_node(ref work, node, depth, mx, my, sx, sy);

            while (work.Size > 0)
            {
                int back_idx = work.Size - 1;
                int nd_mx = work.Get(back_idx, QuadTree.nd_idx_mx);
                int nd_my = work.Get(back_idx, QuadTree.nd_idx_my);
                int nd_sx = work.Get(back_idx, QuadTree.nd_idx_sx);
                int nd_sy = work.Get(back_idx, QuadTree.nd_idx_sy);
                int nd_index = work.Get(back_idx, QuadTree.nd_idx_index);
                int nd_depth = work.Get(back_idx, QuadTree.nd_idx_depth);
                work.PopBack();

                // If this node is a leaf, insert it to the list.
                if (nodes.Get(nd_index, QuadTree.node_idx_num) != -1)
                {
                    push_node(ref leaves, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
                }
                else
                {
                    // Otherwise push the children that intersect the rectangle.
                    int fc = nodes.Get(nd_index, QuadTree.node_idx_fc);
                    int hx = nd_sx >> 1, hy = nd_sy >> 1;
                    int l = nd_mx - hx, t = nd_my - hy, r = nd_mx + hx, b = nd_my + hy;

                    if (top <= nd_my)
                    {
                        if (lft <= nd_mx)
                        {
                            push_node(ref work, fc + 0, nd_depth + 1, l, t, hx, hy);
                        }
                        if (rgt > nd_mx)
                        {
                            push_node(ref work, fc + 1, nd_depth + 1, r, t, hx, hy);
                        }
                    }
                    if (btm > nd_my)
                    {
                        if (lft <= nd_mx)
                        {
                            push_node(ref work, fc + 2, nd_depth + 1, l, b, hx, hy);
                        }
                        if (rgt > nd_mx)
                        {
                            push_node(ref work, fc + 3, nd_depth + 1, r, b, hx, hy);
                        }
                    }
                }
            }
            work.Destroy();
        }

        #endregion 

        public void Query(ref ConstantIndexIntList output, int qlft, int qtop, int qrgt, int qbtm, int omit_element, Allocator allocator)
        {
            // Find the leaves that intersect the specified query rectangle.
            int j = 0;

            int* stack = stackalloc int[QuadTree.stack_size];
            ConstantIndexIntList leaves = ConstantIndexIntList.Create(QuadTree.nd_num, allocator, stack, QuadTree.stack_size);

            ///  update to use a stack based list..
            ///  TODO 
            ///  bit array list . 
            byte* temp = (byte*)UnsafeUtility.Malloc(elements.Size, 1, allocator);
            UnsafeUtility.MemClear(temp, elements.Size);

            // For each leaf node, look for elements that intersect.
            find_leaves(ref leaves, 0, 0, root_mx, root_my, root_sx, root_sy, qlft, qtop, qrgt, qbtm, allocator);

            output.Clear();
            for (j = 0; j < leaves.Size; j++)
            {
                int nd_index = leaves.Get(j, QuadTree.nd_idx_index);

                // Walk the list and add elements that intersect.
                int elt_node_index = nodes.Get(nd_index, QuadTree.node_idx_fc);
                while (elt_node_index != -1)
                {
                    int element = element_nodes.Get(elt_node_index, QuadTree.enode_idx_elt);
                    int lft = elements.Get(element, QuadTree.elt_idx_lft);
                    int top = elements.Get(element, QuadTree.elt_idx_top);
                    int rgt = elements.Get(element, QuadTree.elt_idx_rgt);
                    int btm = elements.Get(element, QuadTree.elt_idx_btm);
                    if (temp[element] == 0 && element != omit_element && QuadTree.intersect(qlft, qtop, qrgt, qbtm, lft, top, rgt, btm))
                    {
                        output.Set(output.PushBack(), 0, element);
                        temp[element] = 1;
                    }
                    elt_node_index = element_nodes.Get(elt_node_index, QuadTree.enode_idx_next);                
                }
            }
            leaves.Destroy();

            // IF DONT OWN TEMP BUFFER: Unmark the elements that were inserted.
            //  for (j = 0; j < output.Size; ++j)
            //{
            //    temp[output.Get(j, 0)] = 0;
            //}
            UnsafeUtility.Free(temp, allocator);
        }

 
    }


    /// <summary>
    /// integer based quadtree with constant halving of segments 
    /// 
    /// https://stackoverflow.com/questions/41946007/efficient-and-well-explained-implementation-of-a-quadtree-for-2d-collision-det
    /// https://github.com/Antymon/quadtree
    /// </summary>
    unsafe public class QuadTree : IDisposable
    {
        internal const int stack_size = 512;
        internal object removal_and_insertion_lock = new object(); 

        public delegate void NodeDelegate(IntPtr user_data, int node, int depth, int mx, int my, int sx, int sy);

        #region element indices 
        // Element node fields:
        internal const int enode_num = 2;

        // Points to the next element in the leaf node. A value of -1
        // indicates the end of the list.
        internal const int enode_idx_next = 0;

        // Stores the element index.
        internal const int enode_idx_elt = 1;

        // Element fields:
        internal const int elt_num = 5;

        // Stores the rectangle encompassing the element.
        internal const int elt_idx_lft = 0, elt_idx_top = 1, elt_idx_rgt = 2, elt_idx_btm = 3;

        // Stores the ID of the element.
        internal const int elt_idx_id = 4;

        // Node fields:
        internal const int node_num = 2;

        // Points to the first child if this node is a branch or the first element
        // if this node is a leaf.
        internal const int node_idx_fc = 0;

        // Stores the number of elements in the node or -1 if it is not a leaf.
        internal const int node_idx_num = 1;

        // Node data fields:
        internal const int nd_num = 6;

        // Stores the extents of the node using a centered rectangle and half-size.
        internal const int nd_idx_mx = 0, nd_idx_my = 1, nd_idx_sx = 2, nd_idx_sy = 3;

        // Stores the index of the node.
        internal const int nd_idx_index = 4;

        // Stores the depth of the node.
        internal const int nd_idx_depth = 5;
        #endregion 

        Allocator temp_allocator = Allocator.Temp;

        protected NativeQuadTree* data;

        // Stores all the elements in the quadtree.
        //public ConstantIndexIntList elements;

        // Stores all the element nodes in the quadtree.
        //public ConstantIndexIntList element_nodes;

        // Stores all the nodes in the quadtree. The first node in this
        // sequence is always the root.
        //public ConstantIndexIntList nodes;

        // Stores the quadtree extents.
        int root_mx => data->root_mx;
        int root_my => data->root_my;
        int root_sx => data->root_sx;
        int root_sy => data->root_sy;

        // Stores the first free node in the quadtree to be reclaimed as 4
        // contiguous nodes at once. A value of -1 indicates that the free
        // list is empty, at which point we simply insert 4 nodes to the
        // back of the nodes array.
        int free_node;

        // Maximum allowed elements in a leaf before the leaf is subdivided/split unless
        // the leaf is at the maximum allowed tree depth.
        int max_elements;

        // Stores the maximum depth allowed for the quadtree.
        int max_depth;

        public int MaxDepth => max_depth;
        public int MaxElementCountInLeafs => max_elements;
        public int ElementCount => data->elements.Size;
        public int NodeCount => data->nodes.Size;
        public int OffsetX { get { return data->offset_x; } protected set { data->offset_x = value; } }
        public int OffsetY { get { return data->offset_y; } protected set { data->offset_y = value; } }
        
        /// <summary>
        /// if offset, we could use seperate quadtrees for extremely large regions also
        /// todo: we can use smaller data size for rectangles (short / int) (half / float), saves 8 bytes for each actor (is it even needed to store).. ??
        /// </summary>
        public bool IsOffset => OffsetX != 0 || OffsetY != 0;
              
        public long EstimateCurrentMemoryUse()
        {
            long total = data->elements.Capacity + data->element_nodes.Capacity + data->nodes.Capacity;
            return total * sizeof(int); 
        }

        /// <summary>
        /// get a pointer to the internal node structures, so we can query the thing from anywhere
        /// </summary>
        public NativeQuadTree* NativeData { get { return data; } }


        public QuadTree(int x1, int y1, int x2, int y2, int start_max_elements = 4, int start_max_depth = 12, Allocator temporary_allocator = Allocator.Temp, Allocator persistant_allocator = Allocator.Persistent)
        {
            temp_allocator = temporary_allocator;
            data = (NativeQuadTree*)UnsafeUtility.Malloc(sizeof(NativeQuadTree), 8, Allocator.Persistent); 

            data->elements = ConstantIndexIntList.Create(elt_num, persistant_allocator);
            data->element_nodes = ConstantIndexIntList.Create(enode_num, persistant_allocator);
            data->nodes = ConstantIndexIntList.Create(node_num, persistant_allocator);

            max_elements = start_max_elements;
            max_depth = start_max_depth;

            // Insert the root node to the qt.
            data->nodes.Insert();
            data->nodes.Set(0, node_idx_fc, -1);
            data->nodes.Set(0, node_idx_num, 0);

            int hw = (x2 - x1) >> 1;
            int hh = (y2 - y1) >> 1;

            // set size and center 
            data->root_sx = hw; 
            data->root_sy = hh;
            data->root_mx = x1 + hw; 
            data->root_my = y1 + hh;

            // offset 0 for now, todo: use this and shorten datatype for rects (use 2 ints, x >> 16 = x1, x & (2^16)-1 = x2
            data->offset_x = 0;
            data->offset_y = 0;
        }


        public void Dispose()
        {
            if (data != null)
            {
                data->elements.Destroy();
                data->element_nodes.Destroy();
                data->nodes.Destroy();
                UnsafeUtility.Free(data, Allocator.Persistent);
            }
        }

        public float2 CalculateSmallestCellSize()
        {
            return new float2(
                (root_sx * 2) * math.pow(0.5f, max_depth),
                (root_sy * 2) * math.pow(0.5f, max_depth));
        }

 
        
        public int Insert(int id, float x1, float y1, float x2, float y2)
        {
            return Insert(id, (int)x1, (int)y1, (int)x2, (int)y2);
        }

        /// <summary>
        /// insert a new id with given extents 
        /// </summary>
        /// <param name="id">data reference</param>
        /// <param name="x1">left</param>
        /// <param name="y1">top</param>
        /// <param name="x2">right</param>
        /// <param name="y2">bottom</param>
        /// <returns>node index</returns>
        public int Insert(int id, int x1, int y1, int x2, int y2)
        {
            lock (removal_and_insertion_lock)
            {
                int new_element = data->elements.Insert();

                // Set the fields of the new element.
                data->elements.Set(new_element, elt_idx_lft, x1);
                data->elements.Set(new_element, elt_idx_top, y1);
                data->elements.Set(new_element, elt_idx_rgt, x2);
                data->elements.Set(new_element, elt_idx_btm, y2);
                data->elements.Set(new_element, elt_idx_id, id);

                // Insert the element to the appropriate leaf node(s).
                node_insert(0, 0, root_mx, root_my, root_sx, root_sy, new_element);
                return new_element;
            }
        }

        unsafe private void node_insert(int index, int depth, int mx, int my, int sx, int sy, int element)
        {
            // Find the leaves and insert the element to all the leaves found.
            int j = 0;

            int* stack = stackalloc int[stack_size];
            ConstantIndexIntList leaves = ConstantIndexIntList.Create(nd_num, temp_allocator, stack, stack_size);

            int lft = data->elements.Get(element, elt_idx_lft);
            int top = data->elements.Get(element, elt_idx_top);
            int rgt = data->elements.Get(element, elt_idx_rgt);
            int btm = data->elements.Get(element, elt_idx_btm);
            
            data->find_leaves(ref leaves, index, depth, mx, my, sx, sy, lft, top, rgt, btm, temp_allocator);

            for (j = 0; j < leaves.Size; j++)
            {
                int nd_mx =    leaves.Get(j, nd_idx_mx);
                int nd_my =    leaves.Get(j, nd_idx_my);
                int nd_sx =    leaves.Get(j, nd_idx_sx);
                int nd_sy =    leaves.Get(j, nd_idx_sy);
                int nd_index = leaves.Get(j, nd_idx_index);
                int nd_depth = leaves.Get(j, nd_idx_depth);
                leaf_insert(nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy, element);
            }

            leaves.Destroy();  
        }

        unsafe private void leaf_insert(int node, int depth, int mx, int my, int sx, int sy, int element)
        {
            // Insert the element node to the leaf.
            int nd_fc = data->nodes.Get(node, node_idx_fc);
            data->nodes.Set(node, node_idx_fc, data->element_nodes.Insert());
            data->element_nodes.Set(data->nodes.Get(node, node_idx_fc), enode_idx_next, nd_fc);
            data->element_nodes.Set(data->nodes.Get(node, node_idx_fc), enode_idx_elt, element);

            // If the leaf is full, split it.
            if (data->nodes.Get(node, node_idx_num) == max_elements && depth < max_depth)
            {
                int fc = 0, j = 0;

                int* stack = stackalloc int[64]; 
                ConstantIndexIntList elts = ConstantIndexIntList.Create(1, temp_allocator, stack, 64); 

                // Transfer elements from the leaf node to a list of elements.
                while (data->nodes.Get(node, node_idx_fc) != -1)
                {
                    int index = data->nodes.Get(node, node_idx_fc);
                    int next_index = data->element_nodes.Get(index, enode_idx_next);
                    int elt = data->element_nodes.Get(index, enode_idx_elt);

                    // Pop off the element node from the leaf and remove it from the qt.
                    data->nodes.Set(node, node_idx_fc, next_index);
                    data->element_nodes.Erase(index);

                    // Insert element to the list.
                    elts.Set(elts.PushBack(), 0, elt);
                }

                // Start by allocating 4 child nodes.
                fc = data->nodes.Insert();
                data->nodes.Insert();
                data->nodes.Insert();
                data->nodes.Insert();
                data->nodes.Set(node, node_idx_fc, fc);

                // Initialize the new child nodes.
                for (j = 0; j < 4; ++j)
                {
                    data->nodes.Set(fc + j, node_idx_fc, -1);
                    data->nodes.Set(fc + j, node_idx_num, 0);
                }

                // Transfer the elements in the former leaf node to its new children.
                data->nodes.Set(node, node_idx_num, -1);
                for (j = 0; j < elts.Size; j++)
                {
                    node_insert(node, depth, mx, my, sx, sy, elts.Get(j, 0));
                }
                elts.Destroy(); 
            }
            else
            {
                // Increment the leaf element count.
                data->nodes.Set(node, node_idx_num, data->nodes.Get(node, node_idx_num) + 1);
            }
        }

        void push_node(ref ConstantIndexIntList list, int nd_index, int nd_depth, int nd_mx, int nd_my, int nd_sx, int nd_sy)
        {
            int back_idx = list.PushBack();
            list.Set(back_idx, nd_idx_mx, nd_mx);
            list.Set(back_idx, nd_idx_my, nd_my);
            list.Set(back_idx, nd_idx_sx, nd_sx);
            list.Set(back_idx, nd_idx_sy, nd_sy);
            list.Set(back_idx, nd_idx_index, nd_index);
            list.Set(back_idx, nd_idx_depth, nd_depth);
        }
 
        unsafe public void Remove(int element)
        {
            lock (removal_and_insertion_lock)
            {
                // Find the leaves.
                int j = 0;

                int* stack = stackalloc int[stack_size];
                ConstantIndexIntList leaves = ConstantIndexIntList.Create(nd_num, temp_allocator, stack, stack_size);

                int lft = data->elements.Get(element, elt_idx_lft);
                int top = data->elements.Get(element, elt_idx_top);
                int rgt = data->elements.Get(element, elt_idx_rgt);
                int btm = data->elements.Get(element, elt_idx_btm);

                data->find_leaves(ref leaves, 0, 0, root_mx, root_my, root_sx, root_sy, lft, top, rgt, btm, temp_allocator);

                // For each leaf node, remove the element node.
                for (j = 0; j < leaves.Size; j++)
                {
                    int nd_index = leaves.Get(j, nd_idx_index);

                    // Walk the list until we find the element node.
                    int node_index = data->nodes.Get(nd_index, node_idx_fc);
                    int prev_index = -1;
                    while (node_index != -1 && data->element_nodes.Get(node_index, enode_idx_elt) != element)
                    {
                        prev_index = node_index;
                        node_index = data->element_nodes.Get(node_index, enode_idx_next);
                    }

                    if (node_index != -1)
                    {
                        // Remove the element node.
                        int next_index = data->element_nodes.Get(node_index, enode_idx_next);
                        if (prev_index == -1) data->nodes.Set(nd_index, node_idx_fc, next_index);
                        else data->element_nodes.Set(prev_index, enode_idx_next, next_index);

                        data->element_nodes.Erase(node_index);

                        // Decrement the leaf element count.
                        data->nodes.Set(nd_index, node_idx_num, data->nodes.Get(nd_index, node_idx_num) - 1);
                    }
                }

                leaves.Destroy();

                // Remove the element.
                data->elements.Erase(element);
            }
        }


        /// <summary>
        /// deferred re-balancing of the quadtree, should only be done once per frame instead of 
        /// once every change. 
        /// 
        /// - after 1 iteration the balancing may not be perfect, this is done
        /// so we dont have unpredictable timings when a lot changes 
        /// </summary>
        /// <param name="allocator">allocator to use for worklist allocations </param>
        unsafe public void Cleanup(Allocator allocator = Allocator.Temp)
        {
            lock (removal_and_insertion_lock)
            {
                int* stack = stackalloc int[stack_size];
                ConstantIndexIntList work = ConstantIndexIntList.Create(1, temp_allocator, stack, stack_size);

                // Only process the root if it's not a leaf.
                if (data->nodes.Get(0, node_idx_num) == -1)
                {
                    // Push the root index to the stack.
                    work.Set(work.PushBack(), 0, 0);
                }

                while (work.Size > 0)
                {
                    // Pop a node from the stack.
                    int node = work.Get(work.Size - 1, 0);
                    int fc = data->nodes.Get(node, node_idx_fc);
                    int num_empty_leaves = 0;
                    int j = 0;

                    work.PopBack();

                    // Loop through the children.
                    for (j = 0; j < 4; ++j)
                    {
                        int child = fc + j;

                        // Increment empty leaf count if the child is an empty
                        // leaf. Otherwise if the child is a branch, add it to
                        // the stack to be processed in the next iteration.
                        if (data->nodes.Get(child, node_idx_num) == 0)
                        {
                            ++num_empty_leaves;
                        }
                        else if (data->nodes.Get(child, node_idx_num) == -1)
                        {
                            // Push the child index to the stack.
                            work.Set(work.PushBack(), 0, child);
                        }
                    }

                    // If all the children were empty leaves, remove them and
                    // make this node the new empty leaf.
                    if (num_empty_leaves == 4)
                    {
                        // Remove all 4 children in reverse order so that they
                        // can be reclaimed on subsequent insertions in proper
                        // order.
                        data->nodes.Erase(fc + 3);
                        data->nodes.Erase(fc + 2);
                        data->nodes.Erase(fc + 1);
                        data->nodes.Erase(fc + 0);

                        // Make this node the new empty leaf.
                        data->nodes.Set(node, node_idx_fc, -1);
                        data->nodes.Set(node, node_idx_num, 0);
                    }
                }

                work.Destroy();
            }
        }

        static internal bool intersect(in int l1, in int t1, in int r1, in int b1,
                     in int l2, in int t2,in int r2,in int b2)
        {
            return l2 <= r1 && r2 >= l1 && t2 <= b1 && b2 >= t1;
        }

        unsafe public void Traverse(IntPtr user_data, NodeDelegate branch, NodeDelegate leaf)
        {
            int* stack = stackalloc int[stack_size];
            ConstantIndexIntList work = ConstantIndexIntList.Create(nd_num, temp_allocator, stack, stack_size); 

            push_node(ref work, 0, 0, root_mx, root_my, root_sx, root_sy);

            while (work.Size > 0)
            {
                int back_idx = work.Size - 1;
                int nd_mx =    work.Get(back_idx, nd_idx_mx);
                int nd_my =    work.Get(back_idx, nd_idx_my);
                int nd_sx =    work.Get(back_idx, nd_idx_sx);
                int nd_sy =    work.Get(back_idx, nd_idx_sy);
                int nd_index = work.Get(back_idx, nd_idx_index);
                int nd_depth = work.Get(back_idx, nd_idx_depth);
                int fc = data->nodes.Get(nd_index, node_idx_fc);
                
                work.PopBack();

                if (data->nodes.Get(nd_index, node_idx_num) == -1)
                {
                    // Push the children of the branch to the stack.
                    int hx = nd_sx >> 1, hy = nd_sy >> 1;
                    int l = nd_mx - hx, t = nd_my - hy, r = nd_mx + hx, b = nd_my + hy;
                    push_node(ref work, fc + 0, nd_depth + 1, l, t, hx, hy);
                    push_node(ref work, fc + 1, nd_depth + 1, r, t, hx, hy);
                    push_node(ref work, fc + 2, nd_depth + 1, l, b, hx, hy);
                    push_node(ref work, fc + 3, nd_depth + 1, r, b, hx, hy);
                    if (branch != null)
                    {
                        branch(user_data, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
                    }
                }
                else if (leaf != null)
                {
                    leaf(user_data, nd_index, nd_depth, nd_mx, nd_my, nd_sx, nd_sy);
                }
            }
            work.Destroy(); 
        }


        /// <summary>
        /// get leaf element ids, normally there are 4 but its possible to have a leaf 
        /// at max tree depth that has n nodes
        /// </summary>
        /// <param name="node"></param>
        /// <param name="allocator"></param>
        /// <param name="stack"></param>
        /// <param name="stacksize"></param>
        /// <returns></returns>
        unsafe public NativeArray<int> GetLeafElementIds(int node, Allocator allocator = Allocator.Temp)
        {
            int current = data->nodes.Get(node, node_idx_fc);
            int count = data->nodes.Get(node, node_idx_num); 

            NativeArray<int> a = new NativeArray<int>(count, allocator, NativeArrayOptions.UninitializedMemory);

            int i = 0; 
            while (current >= 0)
            {
                int next = data->element_nodes.Get(current, enode_idx_next);
                int element_id = data->element_nodes.Get(current, enode_idx_elt);

                a[i] = data->elements.Get(element_id, elt_idx_id);
                i = i + 1; 

                current = next;
            }
                                    
            return a; 
        }

        /// <summary>
        /// lookup element id for given item id 
        /// </summary>
        /// <param name="itemid"></param>
        /// <returns>-1 if not found</returns>
        public int FindElementIndex(int itemid)
        {
            return data->FindElementIndex(itemid); 
        }

    }
}
