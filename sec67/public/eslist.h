/*************************************************************************************************/
/*
Copyright:		QingDao Eastsoft Communication Technology Co.,Ltd.
File Name:		list.h
Description:	list template for hole system
Author:			ch
Version:		v1.0
Date:			2015/06/02
History:
*/
/************************************************************************************************/
#ifndef __ESLIST_H__
#define __ESLIST_H__

#define LIST_CONCAT2(s1, s2) s1##s2
#define LIST_CONCAT(s1, s2) LIST_CONCAT2(s1, s2)


#define LIST(name) \
		 static void *LIST_CONCAT(name,_list) = NULL; \
		 static list_t name = (list_t)&LIST_CONCAT(name,_list)

/**
 * The linked list type.
 *
 */
typedef void ** list_t;

void eslist_init(list_t list);

void *eslist_head(list_t list);

void *eslist_tail(list_t list);

void *eslist_pop (list_t list);

void eslist_push(list_t list, void *item);

void *eslist_chop(list_t list);

void eslist_add(list_t list, void *item);

void eslist_remove(list_t list, void *item);

int	eslist_length(list_t list);

void eslist_copy(list_t dest, list_t src);

void eslist_insert(list_t list, void *previtem, void *newitem);

bool eslist_search(list_t list, void *item);

#endif
