"""금지 사항 정적 점검 (Python ast 기반, src/ 전체).

R1. 상태 대입: qpos/qvel/qacc/act 속성에 대한 대입(=, +=, 슬라이스 대입), np.copyto(..qpos..),
    .qpos.fill()/put()/__setitem__(), 상태 배열의 별칭 바인딩(x = data.qpos, x = data.qpos[a:b]).
    허용:
      (a) model.py의 apply_initial_keyframe() 함수 범위 안(대입문 개수와 무관)
      (b) planning.py에서 대입 대상의 소유 객체 식별자가 plan_data인 경우(self.plan_data.qpos[...] = ...)
    별칭 바인딩은 어디서든 위반이다.
R2. 금지 호출: mj_resetData, mj_resetDataKeyframe, mj_resetDataDebug, mj_setState, mj_copyData — src/ 어디서도 금지.
R3. 주입 분리: src/에서 fault_inject를 import하거나 문자열로 참조(importlib, __import__ 등)하지 않는다.

사용법: scripts/run.sh scripts/check_forbidden.py
"""

import ast
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
STATE_ATTRS = {"qpos", "qvel", "qacc", "act"}
FORBIDDEN_CALLS = {"mj_resetData", "mj_resetDataKeyframe", "mj_resetDataDebug", "mj_setState", "mj_copyData"}
MUTATING_METHODS = {"fill", "put", "__setitem__", "itemset", "resize", "sort"}


def state_owner(node):
    """node가 <owner>.qpos 또는 <owner>.qpos[...] 형태면 (owner 식별자, 속성명)을, 아니면 None."""
    while isinstance(node, ast.Subscript):
        node = node.value
    if isinstance(node, ast.Attribute) and node.attr in STATE_ATTRS:
        owner = node.value
        if isinstance(owner, ast.Attribute):
            return owner.attr, node.attr
        if isinstance(owner, ast.Name):
            return owner.id, node.attr
        return "<expr>", node.attr
    return None


def is_alias_value(node):
    """별칭(뷰) 바인딩: data.qpos 또는 data.qpos[a:b]"""
    if isinstance(node, ast.Attribute) and node.attr in STATE_ATTRS:
        return True
    if isinstance(node, ast.Subscript) and isinstance(node.slice, ast.Slice):
        return state_owner(node) is not None
    return False


class Checker(ast.NodeVisitor):
    def __init__(self, path):
        self.path = path
        self.rel = path.relative_to(ROOT)
        self.funcs = []
        self.violations = []
        self.allowed = []

    def _where(self, node):
        return f"{self.rel}:{node.lineno}"

    def _func(self):
        return self.funcs[-1] if self.funcs else "<module>"

    def visit_FunctionDef(self, node):
        self.funcs.append(node.name)
        self.generic_visit(node)
        self.funcs.pop()

    visit_AsyncFunctionDef = visit_FunctionDef

    def _state_write(self, node, owner_attr, how):
        owner, attr = owner_attr
        if self.path.name == "model.py" and self._func() == "apply_initial_keyframe":
            self.allowed.append(f"{self._where(node)} R1-a {owner}.{attr} {how} (apply_initial_keyframe)")
        elif self.path.name == "planning.py" and owner == "plan_data":
            self.allowed.append(f"{self._where(node)} R1-b {owner}.{attr} {how} ({self._func()})")
        else:
            self.violations.append(f"{self._where(node)} R1 {owner}.{attr} {how} in {self._func()}()")

    def _targets(self, t):
        if isinstance(t, (ast.Tuple, ast.List)):
            for e in t.elts:
                yield from self._targets(e)
        else:
            yield t

    def visit_Assign(self, node):
        for t in node.targets:
            for tt in self._targets(t):
                oa = state_owner(tt)
                if oa:
                    self._state_write(node, oa, "대입")
        if is_alias_value(node.value):
            self.violations.append(f"{self._where(node)} R1 상태 배열 별칭 바인딩 in {self._func()}()")
        self.generic_visit(node)

    def visit_AugAssign(self, node):
        oa = state_owner(node.target)
        if oa:
            self._state_write(node, oa, "복합대입")
        self.generic_visit(node)

    def visit_AnnAssign(self, node):
        oa = state_owner(node.target)
        if oa:
            self._state_write(node, oa, "대입")
        self.generic_visit(node)

    def visit_Call(self, node):
        f = node.func
        name = f.attr if isinstance(f, ast.Attribute) else (f.id if isinstance(f, ast.Name) else None)
        if name in FORBIDDEN_CALLS:
            self.violations.append(f"{self._where(node)} R2 금지 호출 {name}()")
        if name == "copyto" and node.args:
            oa = state_owner(node.args[0])
            if oa:
                self._state_write(node, oa, "np.copyto")
        if isinstance(f, ast.Attribute) and f.attr in MUTATING_METHODS:
            oa = state_owner(f.value)
            if oa:
                self._state_write(node, oa, f".{f.attr}()")
        self.generic_visit(node)

    def visit_Import(self, node):
        for a in node.names:
            if "fault_inject" in a.name:
                self.violations.append(f"{self._where(node)} R3 import {a.name}")

    def visit_ImportFrom(self, node):
        mod = node.module or ""
        if "fault_inject" in mod or any("fault_inject" in a.name for a in node.names):
            self.violations.append(f"{self._where(node)} R3 from {mod} import ...")

    def visit_Constant(self, node):
        if isinstance(node.value, str) and "fault_inject" in node.value:
            self.violations.append(f"{self._where(node)} R3 문자열 참조 '{node.value[:40]}'")


def main():
    files = sorted(SRC.rglob("*.py"))
    violations, allowed = [], []
    for p in files:
        c = Checker(p)
        c.visit(ast.parse(p.read_text(), filename=str(p)))
        violations += c.violations
        allowed += c.allowed
    print(f"== 검사 대상: src/ 아래 {len(files)}개 파일")
    print("== 허용 지점")
    for a in allowed:
        print(f"  {a}")
    has_a = any(" R1-a " in a for a in allowed)
    print("== 위반")
    for v in violations:
        print(f"  {v}")
    if not violations:
        print("  없음")
    ok = not violations and has_a
    print(f"== 결과: {'PASS' if ok else 'FAIL'}  (위반 {len(violations)}, 허용 지점 {len(allowed)}: "
          f"R1-a {sum(' R1-a ' in a for a in allowed)}, R1-b {sum(' R1-b ' in a for a in allowed)})")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
