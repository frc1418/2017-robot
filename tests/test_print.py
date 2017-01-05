import ast
from collections import deque
import sys
import pytest

class FuncCallVisitor(ast.NodeVisitor):
    def __init__(self):
        self._name = deque()

    @property
    def name(self):
        return '.'.join(self._name)

    @name.deleter
    def name(self):
        self._name.clear()

    def visit_Name(self, node):
        self._name.appendleft(node.id)

    def visit_Attribute(self, node):
        try:
            self._name.appendleft(node.attr)
            self._name.appendleft(node.value.id)
        except AttributeError:
            self.generic_visit(node)


def get_func_calls(tree):
    func_calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            callvisitor = FuncCallVisitor()
            callvisitor.visit(node.func)
            func_calls.append(callvisitor.name)
            func_calls.append(node.func.lineno)

    return func_calls

if __name__ == '__main__':
    tree = ast.parse(open(sys.argv[1]).read())
    calls = get_func_calls(tree)
    assert 'print' not in calls, 'Found print at line {0} of {1}'.format(calls[calls.index('print') + 1], sys.argv[1])
