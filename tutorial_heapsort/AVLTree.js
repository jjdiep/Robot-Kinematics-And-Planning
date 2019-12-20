/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    AVL Tree | JavaScript support functions

    Justin Diep
    ECE Robotics
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/
<html>

<body onload="init()">

function init() {
    var test = [1,2,4,5,3];
    var test_tree = avltree_init(test.pop());
    for (var i; i < test.length; i++) {
        avltree_insert = test[i]
    }
}

// // create empty object 
// avltree = {}; 
function avltree_build(new_element) {
    avltree_init(new_element);
    avltree_search(tree,new_element);
}
avltree.build = avltree_build;

// define init function for AVL tree
function avltree_init(root) {
    var tree = {};
    tree.vertices[0] = {};
    tree.vertices[0].vertex = root;
    tree.vertices[0].left_child = [];
    tree.vertices[0].right_child = [];
    tree.vertices[0].height = 0;
    tree.vertices[0].idx = 0;
    return tree;
}

avltree.init = avltree_init;

// define search function for AVL tree
function avltree_insert(tree, new_element) {
    var idx = 0;
    while ((tree.vertices[idx].left_child.length != 0) || (tree.vertices[idx].right_child.length != 0)) {
        if (new_element > tree.vertices[idx]) {
            if (tree.vertices[idx].right_child.length != 0) {
                tree.vertices[idx].right_child.idx = idx;
            } else {
                var h = Math.floor( (idx - 1)/ 2) + 1;
                node_insert(tree, new_element, h);
                tree.vertices[idx].right_child.push(tree.vertices[tree.newest]);
            }
        } else if (new_element < tree.vertices[idx]) {
            if (tree.vertices[idx].left_child.length != 0) {
                tree.vertices[idx].left_child.idx = idx;
            } else {
                var h = Math.floor( (idx - 1)/ 2) + 1;
                node_insert(tree, new_element, h);
                tree.vertices[idx].left_child.push(tree.vertices[tree.newest]);
            }
        }
    }
}

avltree.insert = avltree_insert;

// assign insert function within avltree object
function node_insert(tree, new_element, h) {
	var new_vertex = {};
    new_vertex.vertex = new_element;
	new_vertex.left_child = [];
	new_vertex.right_child = [];
	new_vertex.height = h;
    new_vertex.idx = idx;
	tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}
// assign insert function within avltree object
avltree.node_insert = node_insert;
/* Note: because the avltree_insert function is an object, we can assign 
      a reference to the function within the avltree object, which can be called
      as avltree.insert
*/
// define balance function for AVL tree
function avltree_balance(tree) {
    switch(type) {
      case: rightright
      case: leftleft
      case: leftright
      case: rightleft
    }
}

// define extract function for AVL tree
function avltree_extract(tree) {

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
      avltree.extract = avltree_extract;





