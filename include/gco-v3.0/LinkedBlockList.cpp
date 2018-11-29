#include <gco-v3.0/LinkedBlockList.h>
#include <stdio.h>
#include <stdlib.h>

/*********************************************************************/

void LinkedBlockList::addFront(ListType item) {

	if ( m_head_block_size == GCLL_BLOCK_SIZE )
	{
		LLBlock *tmp      = (LLBlock *) new LLBlock;
		if ( !tmp ) {printf("\nOut of memory");exit(1);}
		tmp -> m_next     = m_head;
		m_head            = tmp;
		m_head_block_size = 0;
	}
	
	m_head ->m_item[m_head_block_size] = item;
	m_head_block_size++;
}

/*********************************************************************/

ListType LinkedBlockList::next()
{
	ListType toReturn = m_cursor -> m_item[m_cursor_ind];

	m_cursor_ind++;

	if ( m_cursor == m_head && m_cursor_ind >= m_head_block_size )
	{
		m_cursor     = m_cursor ->m_next;
		m_cursor_ind = 0;
	}
	else if ( m_cursor_ind == GCLL_BLOCK_SIZE )
	{
		m_cursor = m_cursor ->m_next;
		m_cursor_ind = 0;
	}
	return(toReturn);
}

/*********************************************************************/

bool LinkedBlockList::hasNext()
{
	if ( m_cursor != 0 ) return (true);
	else return(false);
}


/*********************************************************************/

LinkedBlockList::~LinkedBlockList()
{
	LLBlock *tmp;

	while ( m_head != 0 ) 
	{
		tmp = m_head;
		m_head = m_head->m_next;
		delete tmp;
	}
};

/*********************************************************************/

