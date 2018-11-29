/* Singly Linked List of Blocks */
// This data structure should be used only for the GCoptimization class implementation
// because it lucks some important general functions for general list, like remove_item()
// The head block may be not full 
// For regular 2D grids, it's better to set GCLL_BLOCK_SIZE to 2
// For other graphs, it should be set to the average expected number of neighbors
// Data in linked list for the neighborhood system is allocated in blocks of size GCLL_BLOCK_SIZE 

#ifndef __LINKEDBLOCKLIST_H__
#define __LINKEDBLOCKLIST_H__

#define GCLL_BLOCK_SIZE 4  
// GCLL_BLOCKSIZE should "fit" into the type BlockType. That is 
// if GCLL_BLOCKSIZE is larger than 255 but smaller than largest short integer
// then  BlockType should be set to short
typedef char BlockType;

//The type of data stored in the linked list
typedef void * ListType;

class LinkedBlockList{

public: 
	void addFront(ListType item);
	inline bool isEmpty(){if (m_head == 0) return(true); else return(false);};
	inline LinkedBlockList(){m_head = 0; m_head_block_size = GCLL_BLOCK_SIZE;}; 
	~LinkedBlockList();

	// Next three functins are for the linked list traversal
	inline void setCursorFront(){m_cursor = m_head; m_cursor_ind = 0;};
	ListType next();
	bool hasNext();

private:
	typedef struct LLBlockStruct{
		ListType m_item[GCLL_BLOCK_SIZE];
		struct LLBlockStruct *m_next;
	} LLBlock;

	LLBlock *m_head;
	// Remembers the number of elements in the head block, since it may not be full
	BlockType m_head_block_size;
	// For block traversal, points to current element in the current block
	BlockType m_cursor_ind;
	// For block traversal, points to current block in the linked list
	LLBlock *m_cursor;
};

#endif

