#
#  Copyright (c) 2025 Composiv.ai, Eteration A.S. and others
#
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# and Eclipse Distribution License v1.0 which accompany this distribution.
#
# The Eclipse Public License is available at
#    http://www.eclipse.org/legal/epl-v10.html
# and the Eclipse Distribution License is available at
#   http://www.eclipse.org/org/documents/edl-v10.php.
#
# Contributors:
#    Composiv.ai, Eteration A.S. - initial API and implementation
#

import ast
import operator as op

class SafeEvaluator:
    """
    A safe evaluator for simple expressions used in conditions.
    Supports basic comparisons and logical operations.
    """
    operators = {
        ast.Eq: op.eq,
        ast.NotEq: op.ne,
        ast.Lt: op.lt,
        ast.LtE: op.le,
        ast.Gt: op.gt,
        ast.GtE: op.ge,
        ast.And: op.and_,
        ast.Or: op.or_,
        ast.Not: op.not_,
    }

    def __init__(self, context):
        self.context = context

    def eval_expr(self, expr):
        """
        Evaluate an expression in the given context.
        """
        try:
            expr_ast = ast.parse(expr, mode='eval').body
            return self._eval(expr_ast)
        except Exception as e:
            raise ValueError(f"Invalid condition expression '{expr}': {e}")

    def _eval(self, node):
        if isinstance(node, ast.BoolOp):
            if isinstance(node.op, ast.And):
                return all(self._eval(value) for value in node.values)
            elif isinstance(node.op, ast.Or):
                return any(self._eval(value) for value in node.values)
        elif isinstance(node, ast.UnaryOp):
            if isinstance(node.op, ast.Not):
                return not self._eval(node.operand)
        elif isinstance(node, ast.Compare):
            left = self._eval(node.left)
            for op_, right in zip(node.ops, node.comparators):
                right_val = self._eval(right)
                oper = self.operators[type(op_)]
                if not oper(left, right_val):
                    return False
                left = right_val
            return True
        elif isinstance(node, ast.Name):
            return self.context.get(node.id, False)
        elif isinstance(node, ast.Attribute):
            value = self._eval(node.value)
            return getattr(value, node.attr, False)
        elif isinstance(node, ast.Constant):
            return node.value
        else:
            raise TypeError(f"Unsupported expression: {ast.dump(node)}")