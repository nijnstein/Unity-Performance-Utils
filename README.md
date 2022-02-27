# Unity-Performance-Utils
Collection of utility classes for use in Unity with Burst  


### ConstantIntIndexList 

A fast freelist with constant indexes while removing and/or adding items. Can be based on function stack and grows into the heap if necessary using the provided allocator. 

```csharp

    const int stack_size = 1024;
    int* stack = stackalloc int[stack_size];

    // init list with initial capacity of 1024 integers, tuple size = 3 integers 
    ConstantIndexIntList list = ConstantIndexIntList.Create(3, Allocator.Temp, stack, stack_size);

    // use the list


    // destroy the list, freeing any possible allocation 
    list.Destroy();

```

### Threadsafe Bit Set

Set a bit in a map from multiple threads ensuring threadsafety using interlocked ops. 


### Implementation of a quadtree

Partition regions on the xz plane efficiently, uses integers only. Data in native memory and search/find burstable.
