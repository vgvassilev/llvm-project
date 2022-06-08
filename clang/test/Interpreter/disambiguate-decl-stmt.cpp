// REQUIRES: host-supports-jit
// UNSUPPORTED: system-aix
// RUN: cat %s | clang-repl -Xcc -Xclang -Xcc -verify | FileCheck %s
// RUN: %clang_cc1 -fsyntax-only -verify -fincremental-extensions %s

// expected-no-diagnostics

extern "C" int printf(const char*,...);

// Decls which are hard to disambiguate

// Operators.
struct S1 { operator int(); };
S1::operator int() { return 0; }

// Dtors
using I = int;
I x = 10;
x.I::~I();
x = 20;

// Ctors

// Deduction guide
template<typename T> struct A { A(); A(T); };
A() -> A<int>;

struct S2 { S2(); };
S2::S2() = default;

namespace N { struct S { S(); }; }
N::S::S() { printf("N::S::S()\n"); }
N::S s;
// CHECK: N::S::S()

namespace Ns {namespace Ns { void Ns(); void Fs();}}
void Ns::Ns::Ns() { printf("void Ns::Ns::Ns()\n"); }
void Ns::Ns::Fs() {}

Ns::Ns::Fs();
Ns::Ns::Ns();
// CHECK-NEXT: void Ns::Ns::Ns()

struct Attrs1 { Attrs1(); };
Attrs1::Attrs1() __attribute((pure)) = default;

struct Attrs2 { Attrs2(); };
__attribute((pure)) Attrs2::Attrs2() = default;

// Extra semicolon
namespace N {};
