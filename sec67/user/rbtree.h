/*
 * Copyright: (c) 2006-2007, 2008 Triductor Technology, Inc.
 * All Rights Reserved.
 *
 * File:        rbtree.h
 * Purpose:     Red black tree generic manipulation structures.
 * History:     07/15/2007, created by jetmotor
 */

#ifndef _RBTREE_H
#define _RBTREE_H


#include "types.h"
#include "list.h"


/* tree_traverse - Traverse a binary tree in an iterative way.
 * @type: binary tree type
 * @tree: tree pointer, could be null
 * @n: tree node pointer
 * @statement: actions to be taken
 */
#define tree_traverse(type, tree, n, statement) do {	\
	list_head_t que;							\
										\
	INIT_LIST_HEAD(&que);							\
	n = (tree)->root.type##_node;						\
	if (n) {								\
		list_add_tail(&(n)->link, &que);				\
										\
		while (!list_empty(&que)) {					\
			n = list_entry(que.next, struct type##_node, link);	\
			list_del(que.next);					\
										\
			{statement};						\
										\
			if ((n)->type##_left)					\
				list_add_tail(&(n)->type##_left->link, &que);	\
			if ((n)->type##_right)					\
				list_add_tail(&(n)->type##_right->link, &que);	\
		}								\
	}									\
} while (0)


/* left-child-right-sibling representation of a tree with unbounded branching
 */
struct lcrs_node {
	list_head_t link;
	struct lcrs_node *lcrs_parent;
	struct lcrs_node *lcrs_left;
	struct lcrs_node *lcrs_right;
};

struct lcrs_root {
	struct lcrs_node *lcrs_node;
};

typedef struct lcrs_tree_s {
	uint32_t nr;
	struct lcrs_root root;
} lcrs_tree_t;

#define lcrs_entry(ptr, type, member)	\
	((type *)(uint32_t)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))
#define lcrs_adjacent(node)		(node)->left
#define lcrs_next(node)			(node)->right

extern void lcrs_tree_init(lcrs_tree_t *tree);
extern void lcrs_tree_insert(lcrs_tree_t *tree, struct lcrs_node *n, struct lcrs_node *parent, struct lcrs_node **p);


struct rb_node{
	list_head_t link;
	struct rb_node *rb_parent;
	int rb_color;
#define RB_RED          1
#define RB_BLACK        2
	struct rb_node *rb_right;
	struct rb_node *rb_left;
};
#define rb_entry(ptr, type, member) \
        ((type *)(uint32_t)((char *)(ptr)-(unsigned long)(&((type *)0)->member)))

#define rb_node_init(node) do{ \
	(node).rb_parent = (node).rb_right = (node).rb_left = NULL; \
	(node).rb_color = RB_RED; \
}while (0)

#define RB_NODE_INIT() {{0, 0}, 0, RB_RED, 0, 0}


struct rb_root{
	struct rb_node *rb_node;
};


struct rb_tree_s;

/**
 * rb_treenode_cmp_fp - Function pointer of the function that compare the two node.
 * @na:		the node in tree
 * @key:	the new-coming node's key
 * @return: 
 *		< 0, if na < nb 
 *		== 0, if na == nb
 *		> 0, if na > nb
 */
typedef int32_t (*rb_treenode_cmp_fp)(struct rb_node *na, void *key);

/**
 * rb_tree_find_fp - Function pointer of the function that find the right place in the 
 *	tree for the new-coming node.
 * @tree:	the tree
 * @n:		the new-coming node's key
 * @return:
 *		*(@return) points to the right place for inserting the new-coming node,	if cannot find one
 *		*(@return) points to the node in tree those key equals the new_coming node's one, if can find one
 */
typedef struct rb_node ** (*rb_tree_find_fp)(struct rb_tree_s *tree, void *key, struct rb_node **parent);

/**
 * rb_tree_insert_fp - Function pointer of the function that insert a new node into the 
 *	exact postion determined by the operation `find' of the tree.
 * @return:
 */
typedef void (*rb_tree_insert_fp)(struct rb_tree_s *tree, struct rb_node *n, struct rb_node *parent, struct rb_node **p);

/**
 * rb_tree_erase_fp - Function pointer of the function that delete a node from the tree.
 */
typedef void (*rb_tree_erase_fp)(struct rb_tree_s *tree, struct rb_node *n);

/**
 * rb_treenode_del_fp - Function pointer of the function that delete a tree node of any type.
 */
typedef void (*rb_treenode_del_fp)(struct rb_node *n);

/**
 * rb_tree_finalize_fp - Function pointer of the function that free all the nodes on the tree.
 */
typedef void (*rb_tree_finalize_fp)(struct rb_tree_s *tree);

typedef struct rb_tree_s{
	uint32_t nr;
	struct rb_root root;
	struct rb_node *first;
	
	rb_treenode_cmp_fp	cmp;
	rb_tree_find_fp		find;
	rb_tree_insert_fp	insert;
	rb_tree_erase_fp	erase;
	rb_treenode_del_fp	del;
	rb_tree_finalize_fp	finalize;
}rb_tree_t;

#define RB_TREE_INIT() {0, {0}, 0, 0, 0, 0, 0, 0, 0}

extern void rb_tree_init(rb_tree_t *tree,
			 rb_treenode_cmp_fp cmp,	/* user provide, it's a must */
			 rb_tree_find_fp find,
			 rb_tree_insert_fp insert,
			 rb_tree_erase_fp erase,
			 rb_treenode_del_fp del,	/* user provide, it's not a must */
			 rb_tree_finalize_fp finalize);

extern struct rb_node ** rb_tree_find_default(rb_tree_t *tree, void *key, struct rb_node **parent);
extern void rb_tree_insert_default(rb_tree_t *tree, struct rb_node *n, struct rb_node *parent, struct rb_node **p);
extern void rb_tree_erase_default(rb_tree_t *tree, struct rb_node *n);
extern void rb_tree_finalize_default(rb_tree_t *tree);
extern void _rb_tree_finalize_default(struct rb_node **p, rb_treenode_del_fp del);

extern struct rb_node * rb_first(struct rb_root *root);
extern struct rb_node * rb_last(struct rb_root *root);
extern struct rb_node * rb_next(struct rb_node *node);
extern struct rb_node * rb_prev(struct rb_node *node);


#endif	/* end of _RBTREE_H */
