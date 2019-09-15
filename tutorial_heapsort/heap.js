/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
  var elntIdx = heap.length;
  var prntIdx = Math.floor( (elntIdx - 1)/ 2);

  heap.push(new_element);

    // STENCIL: implement your min binary heap insert operation
    var heaped = (elntIdx <= 0) || (heap[prntIdx] <= heap[elntIdx]);
    while (!heaped) {
      // Swap element and parent
      var tmp = heap[prntIdx];
      heap[prntIdx] = heap[elntIdx];
      heap[elntIdx] = tmp;

      // Update element and parent index
      elntIdx = prntIdx;
      prntIdx = Math.floor( (elntIdx -1 ) / 2 );

      // Re-evaluate heap condition
      heaped = (elntIdx <= 0) || (heap[prntIdx] <= heap[elntIdx]);
    }
}

// assign insert function within minheaper object
      minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {

    // // STENCIL: implement your min binary heap extract operation
    if (heap.length == 1) {
       return heap.pop();
    }
    // Extract root of heap, replace with farthest right node
    var lastHeap = heap.pop();
    var minHeap = heap[0];
    heap[0] = lastHeap;
    
    // Heapify
    var elntIdx = 0;
    var heaped = 0; 

    while (!heaped) {
      leftHeap = 2 * elntIdx + 1;
      rightHeap = 2 * elntIdx + 2;
      var tmp = heap[elntIdx];
      if (leftHeap >= heap.length) {
        break;
      } else if (((rightHeap >= heap.length) && (heap[elntIdx] > heap[leftHeap])) || ((rightHeap < heap.length) && (heap[elntIdx] > heap[leftHeap]) && (heap[leftHeap] <= heap[rightHeap]))) {
        heap[elntIdx] = heap[leftHeap];
        heap[leftHeap] = tmp;
        elntIdx = leftHeap;
      } else if ((rightHeap  < heap.length) && (heap[elntIdx] > heap[rightHeap]) && (heap[leftHeap] > heap[rightHeap])) {
        heap[elntIdx] = heap[rightHeap];
        heap[rightHeap] = tmp;
        elntIdx = rightHeap;
      } else {
        break;
      }
    }
  return minHeap;
}

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
      minheaper.extract = minheap_extract;





