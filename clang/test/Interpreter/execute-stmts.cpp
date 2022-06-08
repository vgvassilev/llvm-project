// REQUIRES: host-supports-jit
// UNSUPPORTED: system-aix
// RUN: cat %s | clang-repl -Xcc -Xclang -Xcc  -verify | FileCheck %s
// RUN: %clang_cc1 -fsyntax-only -verify -fincremental-extensions %s

// expected-no-diagnostics

extern "C" int printf(const char*,...);

template <typename T> T call() { printf("called\n"); return T(); }
call<int>();
// CHECK: called

int i = 1;
++i;
printf("i = %d\n", i);
// CHECK: i = 2

namespace Ns { void f(){ i++; } }
Ns::f();

void g() { ++i; }
g();
::g();

printf("i = %d\n", i);
// CHECK-NEXT: i = 5

for (; i > 4; --i) printf("i = %d\n", i);
// CHECK-NEXT: i = 5

int j = i; printf("j = %d\n", j);
// CHECK-NEXT: j = 4
