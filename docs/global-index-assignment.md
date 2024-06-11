# Global index assignment

Once we have multiple rigid bodies, each with their own collections of nodes, we need a way to combine them before building the matrices.

Instead of creating new collections of nodes, use use a class to assign sequential ids to the existing nodes.

These id's may not match the order in which nodes appear in their own sets but instead will act to label where we should put them in any global operations which span multiple rigid bodies.