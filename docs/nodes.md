# Nodes

We have a introduced dedicated classes to represent quadrature nodes and force nodes.

`QuadratureNode` and  `ForceNode`.

There are also collections of these, so we can define any methods applying to the collection.

Although the only information these contain is a position, the existence of the class clarifies what the data refers to.

We hope to reduce bugs and aid clarity by labelling the purpose of these positions.