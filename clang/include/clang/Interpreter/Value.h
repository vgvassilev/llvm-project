//===--- Interpreter.h - Incremental Compiation and Execution---*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the class that used to represent a value in incremental
// C++.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_INTERPRETER_VALUE_H
#define LLVM_CLANG_INTERPRETER_VALUE_H

#include "llvm/Support/Compiler.h"
#include <cassert>
#include <cstdint>

namespace llvm {
class raw_ostream;

} // namespace llvm

namespace clang {

class ASTContext;
class Interpreter;

#if __has_attribute(visibility) &&                                             \
    (!(defined(_WIN32) || defined(__CYGWIN__)) ||                              \
     (defined(__MINGW32__) && defined(__clang__)))
#if defined(LLVM_BUILD_LLVM_DYLIB) || defined(LLVM_BUILD_SHARED_LIBS)
#define REPL_EXTERNAL_VISIBILITY __attribute__((visibility("default")))
#else
#define REPL_EXTERNAL_VISIBILITY
#endif
#else
#if defined(_WIN32)
#define REPL_EXTERNAL_VISIBILITY __declspec(dllexport)
#endif
#endif

class Interpreter;
class QualType;

#define REPL_BUILTIN_TYPES                                                     \
  X(bool, Bool)                                                                \
  X(char, Char_S)                                                              \
  X(signed char, SChar)                                                        \
  X(unsigned char, UChar)                                                      \
  X(short, Short)                                                              \
  X(unsigned short, UShort)                                                    \
  X(int, Int)                                                                  \
  X(unsigned int, UInt)                                                        \
  X(long, Long)                                                                \
  X(unsigned long, ULong)                                                      \
  X(long long, LongLong)                                                       \
  X(unsigned long long, ULongLong)                                             \
  X(float, Float)                                                              \
  X(double, Double)                                                            \
  X(long double, LongDouble)

class REPL_EXTERNAL_VISIBILITY Value {
  union Storage {
#define X(type, name) type m_##name;
    REPL_BUILTIN_TYPES
#undef X
    void *m_Ptr;
  };

public:
  enum Kind {
#define X(type, name) K_##name,
    REPL_BUILTIN_TYPES
#undef X

        K_Void,
    K_PtrOrObj,
    K_Unspecified
  };

  Value() = default;
  Value(void /*Interpreter*/ *In, void /*QualType*/ *Ty);
  Value(const Value &RHS);
  Value(Value &&RHS) noexcept;
  Value &operator=(const Value &RHS);
  Value &operator=(Value &&RHS) noexcept;
  ~Value();

  void printType(llvm::raw_ostream &Out) const;
  void printData(llvm::raw_ostream &Out) const;
  void print(llvm::raw_ostream &Out) const;
  void dump() const;

  ASTContext &getASTContext() const;
  QualType getType() const;
  Interpreter &getInterpreter() const;

  bool isValid() const { return ValueKind != K_Unspecified; }
  bool isVoid() const { return ValueKind == K_Void; }
  bool isManuallyAlloc() const { return IsManuallyAlloc; }
  Kind getKind() const { return ValueKind; }
  void setKind(Kind K) { ValueKind = K; }
  void setOpaqueType(void *Ty) { OpaqueType = Ty; }

  void setPtr(void *Ptr) { Data.m_Ptr = Ptr; }

  void *getPtr() const {
    assert(ValueKind == K_PtrOrObj);
    return Data.m_Ptr;
  }

  bool isPointerOrObjectType() const { return ValueKind == K_PtrOrObj; }

#define X(type, name)                                                          \
  void set##name(type Val) { Data.m_##name = Val; }                            \
  type get##name() const { return Data.m_##name; }
  REPL_BUILTIN_TYPES
#undef X

  // Allow castAs to be partially specialized.
  template <typename T> struct CastFwd {
    static T cast(const Value &V) {
      if (V.isPointerOrObjectType())
        return (T)(uintptr_t)V.getAs<void *>();
      if (!V.isValid() || V.isVoid()) {
        return T();
      }
      return V.getAs<T>();
    }
  };

  template <typename T> struct CastFwd<T *> {
    static T *cast(const Value &V) {
      if (V.isPointerOrObjectType())
        return (T *)(uintptr_t)V.getAs<void *>();
      return nullptr;
    }
  };

  /// \brief Get the value with cast.
  //
  /// Get the value cast to T. This is similar to reinterpret_cast<T>(value),
  /// casting the value of builtins (except void), enums and pointers.
  /// Values referencing an object are treated as pointers to the object.
  template <typename T> T castAs() const { return CastFwd<T>::cast(*this); }

  /// \brief Get to the value with type checking casting the underlying
  /// stored value to T.
  template <typename T> T getAs() const {
    switch (ValueKind) {
    default:
      return T();
#define X(type, name)                                                          \
  case Value::K_##name:                                                        \
    return (T)Data.m_##name;
      REPL_BUILTIN_TYPES
#undef X
    }
  }

private:
  // Interpreter, QualType are stored as void* to reduce dependencies.
  void *Interp = nullptr;
  void *OpaqueType = nullptr;
  Storage Data;
  Kind ValueKind = K_Unspecified;
  bool IsManuallyAlloc = false;
};

template <> inline void *Value::getAs() const {
  if (isPointerOrObjectType())
    return Data.m_Ptr;
  return (void *)getAs<uintptr_t>();
}

} // namespace clang
#endif
